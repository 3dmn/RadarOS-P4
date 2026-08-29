#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"

#include "wifi_manager.h"

#define NVS_NAMESPACE           "radar_cfg"

#define WIFI_SSID_DEFAULT       ""
#define WIFI_PASS_DEFAULT       ""
#define STATION_NAME_DEFAULT    "RADAR-STATION"
#define RADAR_LAT_DEFAULT       52.1657f
#define RADAR_LON_DEFAULT       20.9671f
#define BRIGHTNESS_DEFAULT      100
#define BRIGHTNESS_MIN          10
#define BRIGHTNESS_MAX          100
#define DEFAULT_RANGE_DEFAULT   250
#define AIR_MODE_DEFAULT        0
#define APTS_MODE_DEFAULT       1
#define HIDE_GROUND_DEFAULT     1
#define MAP_ENABLED_DEFAULT     1
#define SQUAWK_ALERT_DEFAULT    1
#define LANG_DEFAULT            0
#define TRAIL_LEN_DEFAULT       30
#define MAX_AIRCRAFT_DEFAULT    64
#define MAX_AIRCRAFT_MIN        10
#define MAX_AIRCRAFT_MAX        200

#define AP_SSID                 "RadarADSB-Setup"
#define AP_CHANNEL               1
#define WIFI_CONNECT_TIMEOUT_MS  10000

static const char *TAG = "WIFI_MGR";
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT      BIT0

static httpd_handle_t g_web_server = NULL;
// Sledzi, czy esp_netif_create_default_wifi_sta() zostalo juz wywolane -
// wywolanie go dwukrotnie jest niebezpieczne (kolizja klucza netif).
static bool s_sta_netif_created = false;

float g_radar_lat = RADAR_LAT_DEFAULT;
float g_radar_lon = RADAR_LON_DEFAULT;
char g_station_name[STATION_NAME_LEN] = STATION_NAME_DEFAULT;
char g_wifi_ssid[WIFI_SSID_MAX_LEN] = WIFI_SSID_DEFAULT;
char g_wifi_pass[WIFI_PASS_MAX_LEN] = WIFI_PASS_DEFAULT;
static uint8_t g_brightness = BRIGHTNESS_DEFAULT;
static uint16_t g_default_range = DEFAULT_RANGE_DEFAULT;
static uint8_t g_air_mode = AIR_MODE_DEFAULT;
static uint8_t g_apts_mode = APTS_MODE_DEFAULT;
static uint8_t g_hide_ground = HIDE_GROUND_DEFAULT;
static uint8_t g_map_enabled = MAP_ENABLED_DEFAULT;
static uint8_t g_squawk_alert_enabled = SQUAWK_ALERT_DEFAULT;
static uint8_t g_lang = LANG_DEFAULT;
static uint8_t g_trail_len = TRAIL_LEN_DEFAULT;
static uint16_t g_max_aircraft = MAX_AIRCRAFT_DEFAULT;

static bool is_valid_trail_len(uint8_t len) {
    return len == 0 || len == 15 || len == 30 || len == 60 || len == 120;
}

static uint16_t clamp_max_aircraft(uint16_t v) {
    if (v > MAX_AIRCRAFT_MAX) v = MAX_AIRCRAFT_MAX;
    if (v < MAX_AIRCRAFT_MIN) v = MAX_AIRCRAFT_MIN;
    return v;
}

// ================= NVS =================
static void load_settings_from_nvs(void) {
    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) {
        ESP_LOGI(TAG, "Brak zapisanej konfiguracji, uzywam wartosci domyslnych.");
        return;
    }

    size_t len = sizeof(g_wifi_ssid);
    nvs_get_str(my_handle, "ssid", g_wifi_ssid, &len);
    len = sizeof(g_wifi_pass);
    nvs_get_str(my_handle, "pass", g_wifi_pass, &len);
    len = sizeof(g_station_name);
    nvs_get_str(my_handle, "name", g_station_name, &len);

    size_t flen = sizeof(float);
    nvs_get_blob(my_handle, "lat", &g_radar_lat, &flen);
    flen = sizeof(float);
    nvs_get_blob(my_handle, "lon", &g_radar_lon, &flen);

    nvs_get_u8(my_handle, "brightness", &g_brightness);
    if (g_brightness < BRIGHTNESS_MIN || g_brightness > BRIGHTNESS_MAX) {
        g_brightness = BRIGHTNESS_DEFAULT;
    }
    nvs_get_u16(my_handle, "default_rng", &g_default_range);
    {
        static const uint16_t valid_ranges[] = {10, 20, 30, 50, 100, 150, 200, 250};
        bool valid_rng = false;
        for (size_t i = 0; i < sizeof(valid_ranges) / sizeof(valid_ranges[0]); i++) {
            if (valid_ranges[i] == g_default_range) { valid_rng = true; break; }
        }
        if (!valid_rng) g_default_range = DEFAULT_RANGE_DEFAULT;
    }
    nvs_get_u8(my_handle, "air_mode", &g_air_mode);
    if (g_air_mode > 2) {
        g_air_mode = AIR_MODE_DEFAULT;
    }
    nvs_get_u8(my_handle, "apts_mode", &g_apts_mode);
    if (g_apts_mode > 1) {
        g_apts_mode = APTS_MODE_DEFAULT;
    }
    nvs_get_u8(my_handle, "hide_ground", &g_hide_ground);
    if (g_hide_ground > 1) {
        g_hide_ground = HIDE_GROUND_DEFAULT;
    }
    nvs_get_u8(my_handle, "map_en", &g_map_enabled);
    if (g_map_enabled > 1) {
        g_map_enabled = MAP_ENABLED_DEFAULT;
    }
    nvs_get_u8(my_handle, "sqk_alert", &g_squawk_alert_enabled);
    if (g_squawk_alert_enabled > 1) {
        g_squawk_alert_enabled = SQUAWK_ALERT_DEFAULT;
    }
    nvs_get_u8(my_handle, "lang", &g_lang);
    if (g_lang > 1) {
        g_lang = LANG_DEFAULT;
    }
    i18n_set_lang((app_lang_t)g_lang);
    nvs_get_u8(my_handle, "trail_len", &g_trail_len);
    if (!is_valid_trail_len(g_trail_len)) {
        g_trail_len = TRAIL_LEN_DEFAULT;
    }
    nvs_get_u16(my_handle, "max_aircraft", &g_max_aircraft);
    g_max_aircraft = clamp_max_aircraft(g_max_aircraft);

    nvs_close(my_handle);
    ESP_LOGI(TAG, "Wczytano z NVS: SSID='%s' stacja='%s' (%.6f, %.6f)",
             g_wifi_ssid, g_station_name, g_radar_lat, g_radar_lon);
}

static void save_settings_to_nvs(void) {
    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) != ESP_OK) {
        return;
    }
    nvs_set_str(my_handle, "ssid", g_wifi_ssid);
    nvs_set_str(my_handle, "pass", g_wifi_pass);
    nvs_set_str(my_handle, "name", g_station_name);
    nvs_set_blob(my_handle, "lat", &g_radar_lat, sizeof(float));
    nvs_set_blob(my_handle, "lon", &g_radar_lon, sizeof(float));
    nvs_set_u8(my_handle, "brightness", g_brightness);
    nvs_set_u16(my_handle, "default_rng", g_default_range);
    nvs_set_u8(my_handle, "air_mode", g_air_mode);
    nvs_set_u8(my_handle, "apts_mode", g_apts_mode);
    nvs_set_u8(my_handle, "hide_ground", g_hide_ground);
    nvs_set_u8(my_handle, "map_en", g_map_enabled);
    nvs_set_u8(my_handle, "sqk_alert", g_squawk_alert_enabled);
    nvs_set_u8(my_handle, "lang", g_lang);
    nvs_set_u8(my_handle, "trail_len", g_trail_len);
    nvs_set_u16(my_handle, "max_aircraft", g_max_aircraft);
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

uint8_t wifi_mgr_get_brightness(void) {
    return g_brightness;
}

int wifi_mgr_get_default_range(void) {
    return g_default_range;
}

uint8_t wifi_mgr_get_default_air_mode(void) {
    return g_air_mode;
}

uint8_t wifi_mgr_get_default_apts_mode(void) {
    return g_apts_mode;
}

bool wifi_mgr_get_hide_ground(void) {
    return g_hide_ground == 1;
}

bool wifi_mgr_get_map_enabled(void) {
    return g_map_enabled == 1;
}

bool wifi_mgr_get_squawk_alert_enabled(void) {
    return g_squawk_alert_enabled == 1;
}

app_lang_t wifi_mgr_get_lang(void) {
    return (app_lang_t)g_lang;
}

void wifi_mgr_set_lang(app_lang_t lang) {
    g_lang = (lang == LANG_PL) ? LANG_PL : LANG_EN;
    i18n_set_lang((app_lang_t)g_lang);
}

uint8_t wifi_mgr_get_trail_len(void) {
    return g_trail_len;
}

void wifi_mgr_set_trail_len(uint8_t len) {
    g_trail_len = is_valid_trail_len(len) ? len : TRAIL_LEN_DEFAULT;
}

uint16_t wifi_mgr_get_max_aircraft(void) {
    return g_max_aircraft;
}

void wifi_mgr_set_max_aircraft(uint16_t max_val) {
    g_max_aircraft = clamp_max_aircraft(max_val);
}

// ================= PANEL WWW =================
static void restart_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static void url_decode(char *s) {
    char *o = s;
    while (*s) {
        if (*s == '+') {
            *o++ = ' ';
            s++;
        } else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
            char hex[3] = {s[1], s[2], '\0'};
            *o++ = (char)strtol(hex, NULL, 16);
            s += 3;
        } else {
            *o++ = *s++;
        }
    }
    *o = '\0';
}

// ================= BUDOWA STRONY WWW =================
// Cala strona budowana jest do jednego bufora (malloc, zwalniany na koncu
// handlera) przez sekwencje hb_append() zamiast dziesiatek osobnych
// static const fragmentow + snprintf-buforow na stosie jak poprzednio -
// znacznie mniej podatne na bledy przy dalszej rozbudowie i nizsze zuzycie
// stosu tasku httpd (jeden bufor zamiast ~10 osobnych, licza sie razem
// kilka KB na kazde wywolanie handlera).
typedef struct {
    char *buf;
    size_t cap;
    size_t pos;
} html_builder_t;

static void hb_init(html_builder_t *hb, char *buf, size_t cap) {
    hb->buf = buf;
    hb->cap = cap;
    hb->pos = 0;
    if (cap > 0) buf[0] = '\0';
}

static void hb_append(html_builder_t *hb, const char *fmt, ...) {
    if (hb->pos >= hb->cap) return;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(hb->buf + hb->pos, hb->cap - hb->pos, fmt, args);
    va_end(args);
    if (n <= 0) return;
    hb->pos += (size_t)n;
    if (hb->pos > hb->cap) hb->pos = hb->cap;
}

#define HTML_PAGE_BUF_SIZE (12 * 1024)

static esp_err_t root_get_handler(httpd_req_t *req) {
    char *page = malloc(HTML_PAGE_BUF_SIZE);
    if (!page) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    html_builder_t hb;
    hb_init(&hb, page, HTML_PAGE_BUF_SIZE);

    hb_append(&hb,
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>%s</title>"
        "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>"
        "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>"
        "<style>"
        "body{background:#0a0f1d;color:#e6f7ee;font-family:-apple-system,system-ui,sans-serif;"
        "max-width:540px;margin:0 auto;padding:24px 16px;}"
        "h1{color:#00ff88;font-size:1.3em;text-align:center;margin-bottom:20px;}"
        ".card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:16px;margin-bottom:16px;}"
        ".card h2{margin:0;font-size:1em;color:#00ff88;}"
        "label{display:block;margin-top:10px;margin-bottom:4px;font-size:0.85em;color:#8fb;}"
        "input,select{width:100%%;box-sizing:border-box;height:42px;padding:0 10px;border-radius:6px;"
        "border:1px solid #30363d;background:#0d1117;color:#fff;font-size:0.95em;}"
        "input[type=range]{height:auto;padding:0;margin-top:6px;}"
        ".grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;}"
        ".scanrow{display:flex;gap:8px;align-items:center;}"
        ".scanrow>input{flex:1;}"
        ".scanrow>button{width:auto;white-space:nowrap;padding:0 14px;height:42px;margin:0;}"
        ".geobtn{width:100%%;margin-top:10px;padding:10px 6px;background:#0d1b2a;color:#8fb;"
        "border:1px solid #145;border-radius:6px;font-size:0.85em;cursor:pointer;}"
        ".geobtn:active{background:#145;}"
        ".geobtn:disabled{opacity:0.6;cursor:default;}"
        ".btn-scan{background:#0d1b2a;color:#8fb;border:1px solid #145;border-radius:6px;"
        "font-size:0.85em;cursor:pointer;}"
        ".btn-scan:active{background:#145;}"
        ".btn-scan:disabled{opacity:0.6;cursor:default;}"
        "#wifi-select{cursor:pointer;}"
        "#map{height:280px;border-radius:8px;margin-top:10px;display:none;border:1px solid #145;overflow:hidden;}"
        "small{display:block;color:#8b949e;margin-top:6px;font-size:0.8em;}"
        ".tip{color:#7a889b;font-size:0.85em;cursor:help;margin-left:4px;}"
        ".btn-save{width:100%%;padding:14px;background:#238636;color:#fff;font-weight:bold;"
        "border:none;border-radius:8px;font-size:1.05em;cursor:pointer;margin-top:6px;transition:background .15s;}"
        ".btn-save:hover{background:#00c853;}"
        ".btn-save:disabled{opacity:0.6;cursor:default;}"
        "p#status{text-align:center;margin-top:14px;font-size:0.95em;}"
        "</style></head><body>"
        "<h1>%s</h1>"
        "<form id='cfgForm' onsubmit='return saveConfig(event)'>",
        T(STR_WEB_PAGE_TITLE), T(STR_WEB_PAGE_TITLE));

    // KARTA 1: jezyk + identyfikacja stacji
    hb_append(&hb, "<div class='card'><h2>\xF0\x9F\x8C\x90 %s</h2>", T(STR_WEB_CARD_LANG));
    hb_append(&hb,
        "<label>%s</label>"
        "<select name='lang'>"
        "<option value='0' %s>English (Default)</option>"
        "<option value='1' %s>Polski</option>"
        "</select>",
        T(STR_WEB_LANGUAGE), g_lang == 0 ? "selected" : "", g_lang == 1 ? "selected" : "");
    hb_append(&hb, "<label>%s</label><input type='text' name='name' value='%s' maxlength='31'>",
              T(STR_WEB_STATION_NAME), g_station_name);
    hb_append(&hb, "</div>");

    // KARTA 2: Wi-Fi
    hb_append(&hb, "<div class='card'><h2>\xF0\x9F\x93\xB6 %s</h2>", T(STR_WEB_WIFI_SECTION));
    hb_append(&hb,
        "<label>%s</label>"
        "<div class='scanrow'>"
        "<input type='text' id='ssid-input' name='ssid' value='%s' maxlength='32' required>"
        "<button type='button' class='btn-scan' id='btn-scan' onclick='scanWifi()'>\xF0\x9F\x94\x8D %s</button>"
        "</div>"
        "<select id='wifi-select' style='display:none;margin-top:8px;width:100%%;'></select>",
        T(STR_WEB_SSID), g_wifi_ssid, T(STR_WEB_SCAN_NETWORKS));
    hb_append(&hb, "<label>%s</label><input type='password' name='pass' value='' maxlength='64' placeholder='%s'>",
              T(STR_WEB_PASSWORD), T(STR_WEB_PASS_PLACEHOLDER));
    hb_append(&hb, "</div>");

    // KARTA 3: lokalizacja radaru
    hb_append(&hb, "<div class='card'><h2>\xF0\x9F\x93\x8D %s</h2>", T(STR_WEB_CARD_LOCATION));
    hb_append(&hb,
        "<div class='grid2'>"
        "<div><label>%s</label><input type='text' id='lat' name='lat' value='%.6f'></div>"
        "<div><label>%s</label><input type='text' id='lon' name='lon' value='%.6f'></div>"
        "</div>"
        "<button type='button' class='geobtn' onclick='toggleMap()'>\xF0\x9F\x97\xBA\xEF\xB8\x8F %s</button>"
        "<div id='map'></div>",
        T(STR_WEB_LATITUDE), (double)g_radar_lat, T(STR_WEB_LONGITUDE), (double)g_radar_lon,
        T(STR_WEB_SELECT_ON_MAP));
    hb_append(&hb, "</div>");

    // KARTA 4: parametry wyswietlacza i radaru
    hb_append(&hb, "<div class='card'><h2>\xF0\x9F\x8E\x9B\xEF\xB8\x8F %s</h2>", T(STR_WEB_CARD_DISPLAY));
    hb_append(&hb,
        "<label>%s: <span id='bval'>%u</span>%%</label>"
        "<input type='range' name='brightness' id='brightness' min='10' max='100' step='5' "
        "oninput=\"document.getElementById('bval').innerText=this.value; "
        "fetch('/set_brightness?val=' + this.value);\" value='%u'>",
        T(STR_WEB_BRIGHTNESS), g_brightness, g_brightness);

    hb_append(&hb, "<div class='grid2'>");

    hb_append(&hb,
        "<div><label>%s</label><select name='rng'>"
        "<option value='10' %s>10 km</option><option value='20' %s>20 km</option>"
        "<option value='30' %s>30 km</option><option value='50' %s>50 km</option>"
        "<option value='100' %s>100 km</option><option value='150' %s>150 km</option>"
        "<option value='200' %s>200 km</option><option value='250' %s>250 km</option>"
        "</select></div>",
        T(STR_WEB_DEFAULT_RANGE),
        g_default_range == 10 ? "selected" : "", g_default_range == 20 ? "selected" : "",
        g_default_range == 30 ? "selected" : "", g_default_range == 50 ? "selected" : "",
        g_default_range == 100 ? "selected" : "", g_default_range == 150 ? "selected" : "",
        g_default_range == 200 ? "selected" : "", g_default_range == 250 ? "selected" : "");

    hb_append(&hb,
        "<div><label>%s<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></label>"
        "<input type=\"number\" name=\"max_aircraft\" min=\"10\" max=\"200\" value=\"%u\" "
        "oninput=\"if(this.value>200){alert('MAX limit is 200 aircraft!'); this.value=200;} "
        "if(this.value<10 && this.value!=''){this.value=10;}\">"
        "</div>",
        T(STR_WEB_MAX_AIRCRAFT), T(STR_WEB_MAX_AIRCRAFT_HELP), g_max_aircraft);

    hb_append(&hb,
        "<div><label>%s<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></label><select name='air_mode'>"
        "<option value='0' %s>%s</option><option value='1' %s>%s</option><option value='2' %s>%s</option>"
        "</select></div>",
        T(STR_WEB_AIR_FILTER), T(STR_WEB_AIR_FILTER_HINT),
        g_air_mode == 0 ? "selected" : "", T(STR_WEB_AIR_ALL),
        g_air_mode == 1 ? "selected" : "", T(STR_WEB_AIR_CIVIL),
        g_air_mode == 2 ? "selected" : "", T(STR_WEB_AIR_MIL));

    hb_append(&hb,
        "<div><label>%s</label><select name='hide_ground'>"
        "<option value='1' %s>%s</option><option value='0' %s>%s</option>"
        "</select></div>",
        T(STR_WEB_GND),
        g_hide_ground == 1 ? "selected" : "", T(STR_WEB_GND_HIDE),
        g_hide_ground == 0 ? "selected" : "", T(STR_WEB_GND_SHOW));

    hb_append(&hb,
        "<div><label>%s</label><select name='map_en'>"
        "<option value='1' %s>%s</option><option value='0' %s>%s</option>"
        "</select></div>",
        T(STR_WEB_MAP),
        g_map_enabled == 1 ? "selected" : "", T(STR_WEB_MAP_ON),
        g_map_enabled == 0 ? "selected" : "", T(STR_WEB_MAP_OFF));

    hb_append(&hb,
        "<div><label>%s<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></label><select name='sqk_alert'>"
        "<option value='1' %s>%s</option><option value='0' %s>%s</option>"
        "</select></div>",
        T(STR_WEB_SQUAWK_ALERT), T(STR_WEB_SQUAWK_ALERT_HINT),
        g_squawk_alert_enabled == 1 ? "selected" : "", T(STR_WEB_SQUAWK_ON),
        g_squawk_alert_enabled == 0 ? "selected" : "", T(STR_WEB_SQUAWK_OFF));

    hb_append(&hb,
        "<div><label>%s</label><select name='apts'>"
        "<option value='1' %s>%s</option><option value='0' %s>%s</option>"
        "</select></div>",
        T(STR_WEB_APTS),
        g_apts_mode == 1 ? "selected" : "", T(STR_WEB_APTS_ON),
        g_apts_mode == 0 ? "selected" : "", T(STR_WEB_APTS_OFF));

    hb_append(&hb,
        "<div><label>%s</label><select name='trail_len'>"
        "<option value='0' %s>%s</option><option value='15' %s>%s</option>"
        "<option value='30' %s>%s</option><option value='60' %s>%s</option>"
        "<option value='120' %s>%s</option>"
        "</select></div>",
        T(STR_WEB_TRAIL_LEN),
        g_trail_len == 0 ? "selected" : "", T(STR_WEB_TRAIL_0),
        g_trail_len == 15 ? "selected" : "", T(STR_WEB_TRAIL_15),
        g_trail_len == 30 ? "selected" : "", T(STR_WEB_TRAIL_30),
        g_trail_len == 60 ? "selected" : "", T(STR_WEB_TRAIL_60),
        g_trail_len == 120 ? "selected" : "", T(STR_WEB_TRAIL_120));

    hb_append(&hb, "</div>"); // .grid2
    hb_append(&hb, "</div>"); // karta 4

    hb_append(&hb,
        "<button type='submit' id='btn' class='btn-save'>\xF0\x9F\x92\xBE %s</button>"
        "<p id='status'></p>"
        "</form>",
        T(STR_WEB_SAVE_REBOOT));

    hb_append(&hb,
        "<script>"
        "var I18N={scanning:'%s',scanFoundPrefix:'%s',scanFoundSuffix:'%s',scanNone:'%s',"
        "saving:'%s',rebooting:'%s',saveError:'%s'};"
        "var __map=null,__marker=null;"
        "function toggleMap(){"
        "var m=document.getElementById('map');"
        "var show=(m.style.display!=='block');"
        "m.style.display=show?'block':'none';"
        "if(show&&!__map){"
        "var la=parseFloat(document.getElementById('lat').value)||52.0;"
        "var lo=parseFloat(document.getElementById('lon').value)||19.0;"
        "__map=L.map('map').setView([la,lo],9);"
        "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{attribution:'&copy; OpenStreetMap'}).addTo(__map);"
        "__marker=L.marker([la,lo]).addTo(__map);"
        "__map.on('click',function(e){"
        "__marker.setLatLng(e.latlng);"
        "document.getElementById('lat').value=e.latlng.lat.toFixed(6);"
        "document.getElementById('lon').value=e.latlng.lng.toFixed(6);"
        "});"
        "}else if(show){"
        "setTimeout(function(){__map.invalidateSize();},150);"
        "}"
        "}"
        "function scanWifi(){"
        "var b=document.getElementById('btn-scan');"
        "var sel=document.getElementById('wifi-select');"
        "b.disabled=true;b.textContent=I18N.scanning;"
        "fetch('/scan').then(function(r){return r.json();}).then(function(list){"
        "sel.innerHTML='';"
        "if(list&&list.length>0){"
        "list.forEach(function(n){"
        "var o=document.createElement('option');"
        "o.value=n.ssid;"
        "o.textContent=n.ssid+' ('+n.rssi+' dBm)';"
        "sel.appendChild(o);"
        "});"
        "sel.style.display='block';"
        "sel.onchange=function(){document.getElementById('ssid-input').value=sel.value;};"
        "b.textContent=I18N.scanFoundPrefix+list.length+I18N.scanFoundSuffix;"
        "}else{"
        "sel.style.display='none';"
        "b.textContent=I18N.scanNone;"
        "}"
        "b.disabled=false;"
        "}).catch(function(){"
        "sel.style.display='none';"
        "b.textContent=I18N.scanNone;"
        "b.disabled=false;"
        "});"
        "}"
        "function saveConfig(e){"
        "e.preventDefault();"
        "var f=document.getElementById('cfgForm');"
        "var btn=document.getElementById('btn');"
        "var st=document.getElementById('status');"
        "btn.disabled=true;st.textContent=I18N.saving;"
        "var body=new URLSearchParams(new FormData(f)).toString();"
        "fetch('/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body})"
        ".then(function(r){st.textContent=I18N.rebooting;})"
        ".catch(function(){st.textContent=I18N.saveError;btn.disabled=false;});"
        "return false;"
        "}"
        "</script>"
        "</body></html>",
        T(STR_WEB_SCANNING), T(STR_WEB_SCAN_FOUND_PREFIX), T(STR_WEB_SCAN_FOUND_SUFFIX), T(STR_WEB_SCAN_NONE),
        T(STR_WEB_SAVING_MSG), T(STR_WEB_REBOOTING_MSG), T(STR_WEB_SAVE_ERROR));

    if (hb.pos >= hb.cap - 1) {
        ESP_LOGE(TAG, "Strona WWW przekroczyla bufor %d B - odpowiedz ucieta!", HTML_PAGE_BUF_SIZE);
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, page, hb.pos);
    free(page);
    return ESP_OK;
}

#define SAVE_BODY_MAX_LEN 512

static esp_err_t save_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > SAVE_BODY_MAX_LEN) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *buf = malloc(total_len + 1);
    if (buf == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            free(buf);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[received] = '\0';

    char param[96];
    if (httpd_query_key_value(buf, "ssid", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", param);
    }
    if (httpd_query_key_value(buf, "pass", param, sizeof(param)) == ESP_OK && strlen(param) > 0) {
        url_decode(param);
        snprintf(g_wifi_pass, sizeof(g_wifi_pass), "%s", param);
    }
    if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        snprintf(g_station_name, sizeof(g_station_name), "%s", param);
    }
    if (httpd_query_key_value(buf, "lat", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        g_radar_lat = atof(param);
    }
    if (httpd_query_key_value(buf, "lon", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        g_radar_lon = atof(param);
    }
    if (httpd_query_key_value(buf, "brightness", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        int val = atoi(param);
        if (val < BRIGHTNESS_MIN) val = BRIGHTNESS_MIN;
        if (val > BRIGHTNESS_MAX) val = BRIGHTNESS_MAX;
        g_brightness = (uint8_t)val;
    }
    if (httpd_query_key_value(buf, "rng", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        int v = atoi(param);
        static const uint16_t valid_ranges[] = {10, 20, 30, 50, 100, 150, 200, 250};
        bool valid = false;
        for (size_t i = 0; i < sizeof(valid_ranges) / sizeof(valid_ranges[0]); i++) {
            if (valid_ranges[i] == v) { valid = true; break; }
        }
        g_default_range = valid ? (uint16_t)v : DEFAULT_RANGE_DEFAULT;
    }
    if (httpd_query_key_value(buf, "air_mode", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        int v = atoi(param);
        g_air_mode = (v >= 0 && v <= 2) ? (uint8_t)v : AIR_MODE_DEFAULT;
    }
    if (httpd_query_key_value(buf, "apts", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        g_apts_mode = (atoi(param) == 1) ? 1 : 0;
    }
    if (httpd_query_key_value(buf, "hide_ground", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        g_hide_ground = (atoi(param) == 1) ? 1 : 0;
    }
    if (httpd_query_key_value(buf, "map_en", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        g_map_enabled = (atoi(param) == 1) ? 1 : 0;
    }
    if (httpd_query_key_value(buf, "sqk_alert", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        g_squawk_alert_enabled = (atoi(param) == 1) ? 1 : 0;
    }
    if (httpd_query_key_value(buf, "lang", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        wifi_mgr_set_lang((app_lang_t)atoi(param));
    }
    if (httpd_query_key_value(buf, "trail_len", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        wifi_mgr_set_trail_len((uint8_t)atoi(param));
    }
    if (httpd_query_key_value(buf, "max_aircraft", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        int max_val = atoi(param);
        if (max_val > 200) max_val = 200;
        if (max_val < 10)  max_val = 10;
        wifi_mgr_set_max_aircraft((uint16_t)max_val);
    }
    free(buf);

    save_settings_to_nvs();
    ESP_LOGI(TAG, "Zapisano konfiguracje z panelu WWW: SSID='%s' stacja='%s' (%.6f, %.6f)",
             g_wifi_ssid, g_station_name, g_radar_lat, g_radar_lon);

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);

    xTaskCreate(restart_task, "wifi_restart", 2048, NULL, 5, NULL);
    return ESP_OK;
}

// Wpisuje SSID do bufora JSON, eskejpujac '"' i '\\' oraz pomijajac znaki
// kontrolne (SSID moze teoretycznie zawierac dowolne bajty).
static void json_escape_ssid(const char *ssid, char *out, size_t out_size) {
    size_t o = 0;
    for (size_t i = 0; ssid[i] != '\0' && o < out_size - 1; i++) {
        unsigned char c = (unsigned char)ssid[i];
        if (c == '"' || c == '\\') {
            if (o >= out_size - 2) break;
            out[o++] = '\\';
            out[o++] = (char)c;
        } else if (c < 0x20) {
            continue;
        } else {
            out[o++] = (char)c;
        }
    }
    out[o] = '\0';
}

#define SCAN_MAX_APS 20

// Skanuje dostepne sieci Wi-Fi i zwraca tablice JSON z nazwami SSID.
// Jesli urzadzenie jest w trybie czystego SoftAP (pierwsza konfiguracja bez
// zapisanej sieci), na czas skanowania przelacza sie na APSTA - sam SoftAP
// (i polaczeni klienci obslugujacy ten panel) pozostaje aktywny.
static esp_err_t scan_get_handler(httpd_req_t *req) {
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);

    bool switched_to_apsta = false;
    if (mode == WIFI_MODE_AP) {
        if (!s_sta_netif_created) {
            esp_netif_create_default_wifi_sta();
            s_sta_netif_created = true;
        }
        switched_to_apsta = (esp_wifi_set_mode(WIFI_MODE_APSTA) == ESP_OK);
    }

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 250,
    };
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);

    httpd_resp_set_type(req, "application/json");

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Skanowanie Wi-Fi nieudane: %s", esp_err_to_name(err));
        httpd_resp_sendstr(req, "[]");
    } else {
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count > SCAN_MAX_APS) ap_count = SCAN_MAX_APS;

        wifi_ap_record_t *records = (ap_count > 0) ? calloc(ap_count, sizeof(wifi_ap_record_t)) : NULL;
        if (records) {
            esp_wifi_scan_get_ap_records(&ap_count, records);
        } else {
            ap_count = 0;
        }

        ESP_LOGI(TAG, "Skanowanie zakonczone: znaleziono %d sieci Wi-Fi", ap_count);

        size_t json_size = (size_t)ap_count * 56 + 8;
        char *json = malloc(json_size);
        if (json) {
            int pos = snprintf(json, json_size, "[");
            for (int i = 0; i < ap_count && pos < (int)json_size; i++) {
                char esc_ssid[40];
                json_escape_ssid((const char *)records[i].ssid, esc_ssid, sizeof(esc_ssid));
                pos += snprintf(json + pos, json_size - (size_t)pos, "%s{\"ssid\":\"%s\",\"rssi\":%d}",
                                 i > 0 ? "," : "", esc_ssid, (int)records[i].rssi);
            }
            if (pos < (int)json_size) snprintf(json + pos, json_size - (size_t)pos, "]");
            httpd_resp_sendstr(req, json);
            free(json);
        } else {
            httpd_resp_sendstr(req, "[]");
        }
        if (records) free(records);
    }

    esp_wifi_scan_stop();
    if (switched_to_apsta) {
        esp_wifi_set_mode(WIFI_MODE_AP);
    }

    return ESP_OK;
}

static esp_err_t set_brightness_get_handler(httpd_req_t *req) {
    char query[32];
    char param[8];
    int val = BRIGHTNESS_DEFAULT;

    if (httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "val", param, sizeof(param)) == ESP_OK) {
        val = atoi(param);
    }
    if (val < BRIGHTNESS_MIN) val = BRIGHTNESS_MIN;
    if (val > BRIGHTNESS_MAX) val = BRIGHTNESS_MAX;

    bsp_display_brightness_set(val);
    g_brightness = (uint8_t)val;
    save_settings_to_nvs();

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void start_web_server(void) {
    if (g_web_server != NULL) {
        return;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;
    httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
    httpd_uri_t save_uri = {.uri = "/save", .method = HTTP_POST, .handler = save_post_handler};
    httpd_uri_t brightness_uri = {.uri = "/set_brightness", .method = HTTP_GET, .handler = set_brightness_get_handler};
    httpd_uri_t scan_uri = {.uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler};

    if (httpd_start(&g_web_server, &config) == ESP_OK) {
        httpd_register_uri_handler(g_web_server, &root_uri);
        httpd_register_uri_handler(g_web_server, &save_uri);
        httpd_register_uri_handler(g_web_server, &brightness_uri);
        httpd_register_uri_handler(g_web_server, &scan_uri);
        ESP_LOGI(TAG, "Panel WWW konfiguracji radaru uruchomiony na porcie 80!");
    }
}

// ================= OBSLUGA ZDARZEN WI-FI =================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi rozlaczone, ponawianie polaczenia...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Polaczono z Wi-Fi. Uzyskano adres IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        start_web_server();
    }
}

// Probuje polaczyc sie w trybie STA z zapisana siecia. Zwraca true w razie sukcesu.
static bool wifi_try_connect_sta(void) {
    if (strlen(g_wifi_ssid) == 0) {
        ESP_LOGW(TAG, "Brak skonfigurowanego SSID.");
        return false;
    }

    esp_netif_create_default_wifi_sta();
    s_sta_netif_created = true;

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { .sta = { .threshold.authmode = WIFI_AUTH_OPEN } };
    strlcpy((char *)wifi_config.sta.ssid, g_wifi_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, g_wifi_pass, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Laczenie z siecia '%s' (timeout %d s)...", g_wifi_ssid, WIFI_CONNECT_TIMEOUT_MS / 1000);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                                            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT) {
        return true;
    }

    ESP_LOGW(TAG, "Nie udalo sie polaczyc z '%s' - przechodze w tryb SoftAP.", g_wifi_ssid);
    esp_wifi_stop();
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    return false;
}

static void start_ap_mode(void) {
    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_config = {
        .ap = {
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };
    strlcpy((char *)ap_config.ap.ssid, AP_SSID, sizeof(ap_config.ap.ssid));

    esp_err_t err;
    bool ok = true;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) nieudane: %s", esp_err_to_name(err));
        ok = false;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) nieudane: %s", esp_err_to_name(err));
        ok = false;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start() (SoftAP) nieudane: %s", esp_err_to_name(err));
        ok = false;
    }

    if (!ok) {
        ESP_LOGE(TAG, "Nie udalo sie uruchomic SoftAP '%s' - panel WWW bedzie niedostepny.", AP_SSID);
        return;
    }

    ESP_LOGI(TAG, "SoftAP uruchomiony: SSID='%s' (bez hasla). Polacz sie i wejdz na http://192.168.4.1/", AP_SSID);
    start_web_server();
}

void wifi_manager_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_settings_from_nvs();

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (!wifi_try_connect_sta()) {
        start_ap_mode();
    }
}

void wifi_manager_wait_connected(void) {
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}
