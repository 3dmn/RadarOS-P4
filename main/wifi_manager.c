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
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"

#include "wifi_manager.h"
#include "version.h"
#include "mqtt_service.h"
#include "ota_update_service.h"
#include "radar_ui.h"

#define NVS_NAMESPACE           "radar_cfg"

#define WIFI_SSID_DEFAULT       ""
#define WIFI_PASS_DEFAULT       ""
#define STATION_NAME_DEFAULT    "RADAR-STATION"
// Default demo coordinates - Warsaw Chopin Airport (EPWA). Only used until
// the user sets their own station location on the Location tab; not a
// real deployment's coordinates.
#define RADAR_LAT_DEFAULT       52.1657f
#define RADAR_LON_DEFAULT       20.9671f
#define BRIGHTNESS_DEFAULT      100
#define BRIGHTNESS_MIN          10
#define BRIGHTNESS_MAX          100
#define DEFAULT_RANGE_DEFAULT   400
#define AIR_MODE_DEFAULT        0
#define APTS_MODE_DEFAULT       1
#define APT_FILTER_MASK_DEFAULT APT_TYPE_ALL
#define HIDE_GROUND_DEFAULT     1
#define MAP_ENABLED_DEFAULT     1
#define SQUAWK_ALERT_DEFAULT    1
#define LANG_DEFAULT            0
#define TRAIL_LEN_DEFAULT       30
#define MAX_AIRCRAFT_DEFAULT    64
#define MAX_AIRCRAFT_MIN        10
#define MAX_AIRCRAFT_MAX        200
#define MQTT_ENABLED_DEFAULT    0
#define MQTT_PORT_DEFAULT       1883
#define MQTT_HA_DISCOVERY_DEFAULT 1
#define MQTT_DEVICE_ID_DEFAULT  "radaros_p4"

#define AP_SSID                 "RadarADSB-Setup"
#define AP_CHANNEL               1
#define WIFI_CONNECT_TIMEOUT_MS  10000

static const char *TAG = "WIFI_MGR";
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT      BIT0

static httpd_handle_t g_web_server = NULL;
// Tracks whether esp_netif_create_default_wifi_sta() has already been
// called - calling it twice is unsafe (netif key collision).
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
static uint8_t g_apt_filter_mask = APT_FILTER_MASK_DEFAULT;
static uint8_t g_hide_ground = HIDE_GROUND_DEFAULT;
static uint8_t g_map_enabled = MAP_ENABLED_DEFAULT;
static uint8_t g_squawk_alert_enabled = SQUAWK_ALERT_DEFAULT;
static uint8_t g_lang = LANG_DEFAULT;
static uint8_t g_trail_len = TRAIL_LEN_DEFAULT;
static uint16_t g_max_aircraft = MAX_AIRCRAFT_DEFAULT;
static uint8_t g_mqtt_enabled = MQTT_ENABLED_DEFAULT;
static uint8_t g_mqtt_ha_discovery = MQTT_HA_DISCOVERY_DEFAULT;
char g_mqtt_host[MQTT_HOST_LEN] = "";
uint16_t g_mqtt_port = MQTT_PORT_DEFAULT;
char g_mqtt_user[MQTT_USER_LEN] = "";
char g_mqtt_pass[MQTT_PASS_LEN] = "";
char g_mqtt_device_id[MQTT_DEVICE_ID_LEN] = MQTT_DEVICE_ID_DEFAULT;
SemaphoreHandle_t g_https_mutex = NULL;

static bool is_valid_trail_len(uint8_t len) {
    return len == 0 || len == 15 || len == 30 || len == 60 || len == 120;
}

// Radar range steps in km - must stay in sync with range_steps[] in
// aircraft_types.h (HUD cycling order), the MQTT Discovery select options
// in mqtt_service.c, and the map zoom lookup table in map_tile_service.c.
static const uint16_t RADAR_RANGE_STEPS_KM[] = {25, 50, 100, 200, 400};
#define RADAR_RANGE_STEPS_COUNT (sizeof(RADAR_RANGE_STEPS_KM) / sizeof(RADAR_RANGE_STEPS_KM[0]))

static bool is_valid_range_km(uint16_t km) {
    for (size_t i = 0; i < RADAR_RANGE_STEPS_COUNT; i++) {
        if (RADAR_RANGE_STEPS_KM[i] == km) return true;
    }
    return false;
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
        ESP_LOGI(TAG, "No saved configuration, using default values.");
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
    if (!is_valid_range_km(g_default_range)) {
        g_default_range = DEFAULT_RANGE_DEFAULT;
    }
    nvs_get_u8(my_handle, "air_mode", &g_air_mode);
    if (g_air_mode > 2) {
        g_air_mode = AIR_MODE_DEFAULT;
    }
    nvs_get_u8(my_handle, "apts_mode", &g_apts_mode);
    if (g_apts_mode > 1) {
        g_apts_mode = APTS_MODE_DEFAULT;
    }
    nvs_get_u8(my_handle, "apt_fmask", &g_apt_filter_mask);
    if ((g_apt_filter_mask & ~APT_TYPE_ALL) != 0) {
        g_apt_filter_mask = APT_FILTER_MASK_DEFAULT;
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

    nvs_get_u8(my_handle, "mqtt_en", &g_mqtt_enabled);
    if (g_mqtt_enabled > 1) g_mqtt_enabled = MQTT_ENABLED_DEFAULT;
    nvs_get_u8(my_handle, "mqtt_ha", &g_mqtt_ha_discovery);
    if (g_mqtt_ha_discovery > 1) g_mqtt_ha_discovery = MQTT_HA_DISCOVERY_DEFAULT;
    len = sizeof(g_mqtt_host);
    nvs_get_str(my_handle, "mqtt_host", g_mqtt_host, &len);
    nvs_get_u16(my_handle, "mqtt_port", &g_mqtt_port);
    if (g_mqtt_port == 0) g_mqtt_port = MQTT_PORT_DEFAULT;
    len = sizeof(g_mqtt_user);
    nvs_get_str(my_handle, "mqtt_user", g_mqtt_user, &len);
    len = sizeof(g_mqtt_pass);
    nvs_get_str(my_handle, "mqtt_pass", g_mqtt_pass, &len);
    len = sizeof(g_mqtt_device_id);
    nvs_get_str(my_handle, "mqtt_devid", g_mqtt_device_id, &len);
    if (g_mqtt_device_id[0] == '\0') {
        snprintf(g_mqtt_device_id, sizeof(g_mqtt_device_id), "%s", MQTT_DEVICE_ID_DEFAULT);
    }
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Loaded from NVS: SSID='%s' station='%s' (%.6f, %.6f)",
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
    nvs_set_u8(my_handle, "apt_fmask", g_apt_filter_mask);
    nvs_set_u8(my_handle, "hide_ground", g_hide_ground);
    nvs_set_u8(my_handle, "map_en", g_map_enabled);
    nvs_set_u8(my_handle, "sqk_alert", g_squawk_alert_enabled);
    nvs_set_u8(my_handle, "lang", g_lang);
    nvs_set_u8(my_handle, "trail_len", g_trail_len);
    nvs_set_u16(my_handle, "max_aircraft", g_max_aircraft);
    nvs_set_u8(my_handle, "mqtt_en", g_mqtt_enabled);
    nvs_set_u8(my_handle, "mqtt_ha", g_mqtt_ha_discovery);
    nvs_set_str(my_handle, "mqtt_host", g_mqtt_host);
    nvs_set_u16(my_handle, "mqtt_port", g_mqtt_port);
    nvs_set_str(my_handle, "mqtt_user", g_mqtt_user);
    nvs_set_str(my_handle, "mqtt_pass", g_mqtt_pass);
    nvs_set_str(my_handle, "mqtt_devid", g_mqtt_device_id);
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

uint8_t wifi_mgr_get_brightness(void) {
    return g_brightness;
}

void wifi_mgr_set_brightness(uint8_t pct) {
    if (pct < BRIGHTNESS_MIN) pct = BRIGHTNESS_MIN;
    if (pct > BRIGHTNESS_MAX) pct = BRIGHTNESS_MAX;
    bsp_display_brightness_set(pct);
    g_brightness = pct;
    save_settings_to_nvs();
}

int wifi_mgr_get_default_range(void) {
    return g_default_range;
}

void wifi_mgr_set_default_range(uint16_t range_km) {
    g_default_range = is_valid_range_km(range_km) ? range_km : DEFAULT_RANGE_DEFAULT;
    save_settings_to_nvs();
}

uint8_t wifi_mgr_get_default_air_mode(void) {
    return g_air_mode;
}

void wifi_mgr_set_default_air_mode(uint8_t mode) {
    g_air_mode = (mode <= 2) ? mode : AIR_MODE_DEFAULT;
    save_settings_to_nvs();
}

uint8_t wifi_mgr_get_default_apts_mode(void) {
    return g_apts_mode;
}

void wifi_mgr_set_default_apts_mode(bool on) {
    g_apts_mode = on ? 1 : 0;
    save_settings_to_nvs();
}

uint8_t wifi_mgr_get_apt_filter_mask(void) {
    return g_apt_filter_mask;
}

void wifi_mgr_set_apt_filter_mask(uint8_t mask) {
    g_apt_filter_mask = mask & APT_TYPE_ALL;
    save_settings_to_nvs();
}

bool wifi_mgr_get_hide_ground(void) {
    return g_hide_ground == 1;
}

void wifi_mgr_set_hide_ground(bool hide) {
    g_hide_ground = hide ? 1 : 0;
    save_settings_to_nvs();
}

bool wifi_mgr_get_map_enabled(void) {
    return g_map_enabled == 1;
}

void wifi_mgr_set_map_enabled(bool on) {
    g_map_enabled = on ? 1 : 0;
    save_settings_to_nvs();
}

bool wifi_mgr_get_squawk_alert_enabled(void) {
    return g_squawk_alert_enabled == 1;
}

void wifi_mgr_set_squawk_alert_enabled(bool on) {
    g_squawk_alert_enabled = on ? 1 : 0;
    save_settings_to_nvs();
}

app_lang_t wifi_mgr_get_lang(void) {
    return (app_lang_t)g_lang;
}

void wifi_mgr_set_lang(app_lang_t lang) {
    g_lang = (lang == LANG_PL) ? LANG_PL : LANG_EN;
    i18n_set_lang((app_lang_t)g_lang);
    save_settings_to_nvs();
}

uint8_t wifi_mgr_get_trail_len(void) {
    return g_trail_len;
}

void wifi_mgr_set_trail_len(uint8_t len) {
    g_trail_len = is_valid_trail_len(len) ? len : TRAIL_LEN_DEFAULT;
    save_settings_to_nvs();
}

uint16_t wifi_mgr_get_max_aircraft(void) {
    return g_max_aircraft;
}

void wifi_mgr_set_max_aircraft(uint16_t max_val) {
    g_max_aircraft = clamp_max_aircraft(max_val);
}

bool wifi_mgr_get_mqtt_enabled(void) {
    return g_mqtt_enabled == 1;
}

bool wifi_mgr_get_mqtt_ha_discovery(void) {
    return g_mqtt_ha_discovery == 1;
}

// ================= WEB PANEL =================
// arg: delay in ms (passed as a pointer value, not an address).
static void restart_task(void *arg) {
    uint32_t delay_ms = (uint32_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
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

// ================= WEB PAGE BUILDING =================
// The whole page is built into a single buffer (malloc, freed at the end of
// the handler) through a sequence of hb_append() calls instead of the dozens
// of separate static const fragments + stack snprintf buffers used
// previously - far less error-prone as the page grows further, and lower
// httpd task stack usage (one buffer instead of ~10 separate ones, together
// several KB per handler call).
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

#define HTML_PAGE_BUF_SIZE (64 * 1024)

// Current connection status (STA connected / AP-setup) and IP address of the
// interface actually serving the web panel - for the "Wi-Fi & Network" tab.
static void get_wifi_status_info(bool *out_sta_connected, char *ip_buf, size_t ip_len) {
    bool sta_connected = (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(sta_connected ? "WIFI_STA_DEF" : "WIFI_AP_DEF");
    esp_netif_ip_info_t ip_info = {0};
    if (netif) {
        esp_netif_get_ip_info(netif, &ip_info);
    }
    snprintf(ip_buf, ip_len, IPSTR, IP2STR(&ip_info.ip));
    *out_sta_connected = sta_connected;
}

// Strips a leading 'v'/'V' from a string (e.g. git describe sometimes
// already returns "v..."), so appending our own "v" prefix in
// FW_VERSION/commit doesn't produce "vv".
static const char *strip_v_prefix(const char *s) {
    if (s && (s[0] == 'v' || s[0] == 'V') && s[1] != '\0') return s + 1;
    return s ? s : "";
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    // The page buffer is built in PSRAM (not on the httpd task stack or the
    // tight internal heap) - at 64 KB a plain malloc() would also land in
    // SPIRAM (CONFIG_SPIRAM_USE_MALLOC), but heap_caps_malloc forces this
    // explicitly regardless of the allocator's auto-routing threshold.
    char *page = heap_caps_malloc(HTML_PAGE_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!page) {
        page = malloc(HTML_PAGE_BUF_SIZE);
    }
    if (!page) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    html_builder_t hb;
    hb_init(&hb, page, HTML_PAGE_BUF_SIZE);

    bool sta_connected = false;
    char ip_str[16] = "0.0.0.0";
    get_wifi_status_info(&sta_connected, ip_str, sizeof(ip_str));
    const esp_app_desc_t *app_desc = esp_app_get_description();
    // Full version string ("RadarOS P4 v1.0.0 . commit 948d2d3 (IDF v5.3.5)")
    // for the System tab, and a shorter one for the fixed footer - the
    // commit hash comes from esp_app_desc_t.version (auto git describe from
    // the ESP-IDF build system), strip_v_prefix prevents a double "vv" if
    // that string already had a "v".
    char fw_full[112];
    snprintf(fw_full, sizeof(fw_full), "%s %s \xC2\xB7 commit %s (IDF %s)",
             FW_NAME, FW_VERSION, strip_v_prefix(app_desc->version), app_desc->idf_ver);
    char fw_footer[80];
    snprintf(fw_footer, sizeof(fw_footer), "%s %s (IDF %s)", FW_NAME, FW_VERSION, app_desc->idf_ver);
    // A device without an STA connection serves the panel from its SoftAP
    // (setup mode) - in that state the user almost always came here to set
    // up Wi-Fi, so the default active tab is "Wi-Fi & Network" instead of
    // "Radar".
    bool ap_mode = !sta_connected;

    const char *mqtt_status_label = T(STR_WEB_MQTT_STATUS_DISABLED);
    const char *mqtt_led_color = "#64748b";
    switch (mqtt_service_get_status()) {
        case MQTT_STATUS_CONNECTED:
            mqtt_status_label = T(STR_WEB_MQTT_STATUS_CONNECTED);
            mqtt_led_color = "#22c55e";
            break;
        case MQTT_STATUS_CONNECTING:
            mqtt_status_label = T(STR_WEB_MQTT_STATUS_CONNECTING);
            mqtt_led_color = "#eab308";
            break;
        case MQTT_STATUS_ERROR:
            mqtt_status_label = T(STR_WEB_MQTT_STATUS_ERROR);
            mqtt_led_color = "#ef4444";
            break;
        default:
            break;
    }

    hb_append(&hb,
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>%s</title>"
        "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>"
        "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>"
        "<style>"
        "body{background:#0a0f1d;color:#e6f7ee;font-family:-apple-system,system-ui,sans-serif;"
        "max-width:820px;width:94%%;margin:0 auto;padding:24px 16px;}"
        "h1{color:#00ff88;font-size:1.3em;text-align:center;margin-bottom:20px;}"
        ".card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:16px;margin-bottom:16px;}"
        ".card h2{margin:0;font-size:1em;color:#00ff88;}"
        ".tabbar{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin-bottom:20px;}"
        ".tabbtn{padding:8px 14px;border-radius:8px;border:1px solid #30363d;white-space:nowrap;"
        "background:#1e293b;color:#c9d6e3;font-size:0.9rem;font-weight:600;cursor:pointer;"
        "transition:background .15s,color .15s;}"
        ".tabbtn:hover{background:#26313f;}"
        ".tabbtn.active{background:#10b981;border-color:#10b981;color:#04150c;}"
        ".statrow{display:flex;justify-content:space-between;align-items:center;margin-top:10px;"
        "padding:10px 12px;background:#0d1117;border:1px solid #30363d;border-radius:6px;font-size:0.85em;}"
        ".statrow span:first-child{color:#8fb;}"
        "@keyframes ledpulse{0%%,100%%{opacity:1;transform:scale(1);}50%%{opacity:0.55;transform:scale(0.8);}}"
        ".led{width:10px;height:10px;border-radius:50%%;display:inline-block;margin-right:8px;"
        "background:#64748b;animation:ledpulse 1.6s ease-in-out infinite;flex-shrink:0;}"
        "label{display:block;margin-top:10px;margin-bottom:4px;font-size:0.85em;color:#8fb;}"
        "input,select{width:100%%;box-sizing:border-box;height:42px;padding:0 10px;border-radius:6px;"
        "border:1px solid #30363d;background:#0d1117;color:#fff;font-size:0.95em;}"
        "input[type=range]{height:auto;padding:0;margin-top:6px;}"
        ".grid2{display:grid;grid-template-columns:1fr 1fr;gap:16px;}"
        "@media (max-width:600px){.grid2{grid-template-columns:1fr;}}"
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
        "#map{width:100%%;aspect-ratio:1/1;border-radius:8px;margin-top:12px;display:none;"
        "border:1px solid #145;overflow:hidden;}"
        "small{display:block;color:#8b949e;margin-top:6px;font-size:0.8em;}"
        ".tip{color:#7a889b;font-size:0.85em;cursor:help;margin-left:4px;}"
        ".setting-row{display:flex;justify-content:space-between;align-items:center;"
        "margin-top:10px;gap:10px;}"
        ".setting-label{font-size:0.9em;color:#e6f7ee;}"
        ".switch{position:relative;display:inline-block;width:46px;height:26px;flex-shrink:0;margin:0;}"
        ".switch input{opacity:0;width:0;height:0;}"
        ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;"
        "background-color:#334155;transition:.25s ease-in-out;border-radius:26px;}"
        ".slider:before{position:absolute;content:'';height:20px;width:20px;left:3px;bottom:3px;"
        "background-color:#ffffff;transition:.25s ease-in-out;border-radius:50%%;"
        "box-shadow:0 2px 4px rgba(0,0,0,0.25);}"
        "input:checked + .slider{background-color:#22c55e;}"
        "input:checked + .slider:before{transform:translateX(20px);}"
        ".btn-save{width:100%%;padding:14px;background:#238636;color:#fff;font-weight:bold;"
        "border:none;border-radius:8px;font-size:1.05em;cursor:pointer;margin-top:6px;transition:background .15s;}"
        ".btn-save:hover{background:#00c853;}"
        ".btn-save:disabled{opacity:0.6;cursor:default;}"
        "p#status{text-align:center;margin-top:14px;font-size:0.95em;}"
        ".fw-footer{font-size:0.8rem;color:#64748b;text-align:center;margin:14px 0 10px 0;"
        "font-family:monospace;}"
        ".filerow{display:flex;gap:8px;align-items:center;margin-top:6px;}"
        ".filerow>input[type=file]{flex:1;height:auto;padding:8px;font-size:0.8em;}"
        ".filerow>button{width:auto;white-space:nowrap;padding:0 14px;height:42px;margin:0;"
        "background:#0d1b2a;color:#8fb;border:1px solid #145;border-radius:6px;"
        "font-size:0.85em;cursor:pointer;}"
        ".filerow>button:disabled{opacity:0.6;cursor:default;}"
        ".progress-wrap{background:#0d1117;border:1px solid #30363d;border-radius:6px;"
        "height:20px;overflow:hidden;margin-top:10px;}"
        ".progress-bar{height:100%%;width:0%%;background:#10b981;transition:width .2s;}"
        "p.hint{color:#8b949e;font-size:0.8em;margin-top:8px;}"
        "</style></head><body>"
        "<h1>%s</h1>"
        "<form id='cfgForm' onsubmit='return saveConfig(event)'>",
        T(STR_WEB_PAGE_TITLE), T(STR_WEB_PAGE_TITLE));

    // Tab bar (top tabs) - view switching via plain JS
    // (element.style.display), see showTab() in <script> below.
    hb_append(&hb,
        "<div class='tabbar'>"
        "<button type='button' class='tabbtn%s' data-tab='display' onclick=\"showTab('display')\">\xF0\x9F\x93\x9F %s</button>"
        "<button type='button' class='tabbtn' data-tab='location' onclick=\"showTab('location')\">\xF0\x9F\x93\x8D %s</button>"
        "<button type='button' class='tabbtn%s' data-tab='wifi' onclick=\"showTab('wifi')\">\xF0\x9F\x93\xB6 %s</button>"
        "<button type='button' class='tabbtn' data-tab='system' onclick=\"showTab('system')\">\xE2\x9A\x99\xEF\xB8\x8F %s</button>"
        "<button type='button' class='tabbtn' data-tab='mqtt' onclick=\"showTab('mqtt')\">\xF0\x9F\x93\xB6 %s</button>"
        "</div>",
        ap_mode ? "" : " active", T(STR_WEB_TAB_DISPLAY), T(STR_WEB_TAB_LOCATION),
        ap_mode ? " active" : "", T(STR_WEB_TAB_WIFI), T(STR_WEB_TAB_SYSTEM), T(STR_WEB_TAB_MQTT));

    // TAB: Radar & Display (default in STA mode)
    hb_append(&hb, "<div class='card tabpanel' id='tab-display' style='display:%s'><h2>\xF0\x9F\x8E\x9B\xEF\xB8\x8F %s</h2>",
              ap_mode ? "none" : "block", T(STR_WEB_CARD_DISPLAY));
    hb_append(&hb,
        "<label>%s: <span id='bval'>%u</span>%%</label>"
        "<input type='range' name='brightness' id='brightness' min='10' max='100' step='5' "
        "oninput=\"document.getElementById('bval').innerText=this.value; "
        "fetch('/set_brightness?val=' + this.value);\" value='%u'>",
        T(STR_WEB_BRIGHTNESS), g_brightness, g_brightness);

    hb_append(&hb, "<div class='grid2'>");

    hb_append(&hb,
        "<div><label>%s</label><select name='rng'>"
        "<option value='25' %s>25 km</option><option value='50' %s>50 km</option>"
        "<option value='100' %s>100 km</option><option value='200' %s>200 km</option>"
        "<option value='400' %s>400 km</option>"
        "</select></div>",
        T(STR_WEB_DEFAULT_RANGE),
        g_default_range == 25 ? "selected" : "", g_default_range == 50 ? "selected" : "",
        g_default_range == 100 ? "selected" : "", g_default_range == 200 ? "selected" : "",
        g_default_range == 400 ? "selected" : "");

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
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"gnd_show\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>",
        T(STR_WEB_GND), g_hide_ground == 0 ? "checked" : "");

    hb_append(&hb,
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"map_en\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>",
        T(STR_WEB_MAP), g_map_enabled == 1 ? "checked" : "");

    hb_append(&hb,
        "<div class=\"setting-row\"><span class=\"setting-label\">%s"
        "<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"sqk_alert\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>",
        T(STR_WEB_SQUAWK_ALERT), T(STR_WEB_SQUAWK_ALERT_HINT),
        g_squawk_alert_enabled == 1 ? "checked" : "");

    hb_append(&hb,
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"apts\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>",
        T(STR_WEB_APTS), g_apts_mode == 1 ? "checked" : "");

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

    hb_append(&hb,
        "<div style=\"margin-top:10px;\">"
        "<label>%s<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></label>"
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"apt_civil\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>"
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"apt_mil\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>"
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"apt_ga\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>"
        "</div>",
        T(STR_WEB_APT_TYPES), T(STR_WEB_APT_TYPES_HINT),
        T(STR_WEB_APT_CIVIL), (g_apt_filter_mask & APT_TYPE_CIVIL) ? "checked" : "",
        T(STR_WEB_APT_MIL), (g_apt_filter_mask & APT_TYPE_MIL) ? "checked" : "",
        T(STR_WEB_APT_GA), (g_apt_filter_mask & APT_TYPE_GA) ? "checked" : "");

    hb_append(&hb, "</div>"); // #tab-display

    // TAB: Location
    hb_append(&hb, "<div class='card tabpanel' id='tab-location' style='display:none'><h2>\xF0\x9F\x93\x8D %s</h2>", T(STR_WEB_CARD_LOCATION));
    hb_append(&hb,
        "<div class='grid2'>"
        "<div><label>%s</label><input type='text' id='lat' name='lat' value='%.6f'></div>"
        "<div><label>%s</label><input type='text' id='lon' name='lon' value='%.6f'></div>"
        "</div>"
        "<button type='button' class='geobtn' onclick='toggleMap()'>\xF0\x9F\x97\xBA\xEF\xB8\x8F %s</button>"
        "<div id='map'></div>",
        T(STR_WEB_LATITUDE), (double)g_radar_lat, T(STR_WEB_LONGITUDE), (double)g_radar_lon,
        T(STR_WEB_SELECT_ON_MAP));
    hb_append(&hb, "</div>"); // #tab-location

    // TAB: Wi-Fi & Network (default in AP/setup mode)
    hb_append(&hb, "<div class='card tabpanel' id='tab-wifi' style='display:%s'><h2>\xF0\x9F\x93\xB6 %s</h2>",
              ap_mode ? "block" : "none", T(STR_WEB_WIFI_SECTION));
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
    hb_append(&hb,
        "<div class='statrow'><span>%s</span><span>%s</span></div>"
        "<div class='statrow'><span>%s</span><span>%s</span></div>",
        T(STR_WEB_CONN_STATUS), sta_connected ? T(STR_WEB_CONN_STA) : T(STR_WEB_CONN_AP),
        T(STR_WEB_IP_ADDRESS), ip_str);
    hb_append(&hb, "</div>"); // #tab-wifi

    // TAB: System
    hb_append(&hb, "<div class='card tabpanel' id='tab-system' style='display:none'><h2>\xE2\x9A\x99\xEF\xB8\x8F %s</h2>", T(STR_WEB_TAB_SYSTEM));
    hb_append(&hb,
        "<label>%s</label>"
        "<select name='lang'>"
        "<option value='0' %s>English (Default)</option>"
        "<option value='1' %s>Polski</option>"
        "</select>",
        T(STR_WEB_LANGUAGE), g_lang == 0 ? "selected" : "", g_lang == 1 ? "selected" : "");
    hb_append(&hb, "<label>%s</label><input type='text' name='name' value='%s' maxlength='31'>",
              T(STR_WEB_STATION_NAME), g_station_name);
    hb_append(&hb,
        "<div class='statrow'><span>%s</span><span>%s</span></div>",
        T(STR_WEB_FIRMWARE_INFO), fw_full);

    // Configuration backup/restore - the export does not include the Wi-Fi
    // password (security), import works outside the form (fetches JSON
    // directly to /import_config), so it never touches the form fields.
    hb_append(&hb,
        "<div style='margin-top:18px;border-top:1px solid #30363d;padding-top:14px;'>"
        "<label style='color:#00ff88;font-weight:bold;'>\xF0\x9F\x93\xA6 %s</label>"
        "<button type='button' class='geobtn' onclick='exportConfig()'>\xF0\x9F\x92\xBE %s</button>"
        "<div class='filerow'>"
        "<input type='file' id='import-file' accept='.json'>"
        "<button type='button' onclick='importConfig()'>%s</button>"
        "</div>"
        "<label style='margin-top:4px;'>%s</label>"
        "<p id='import-status' class='hint'></p>"
        "</div>",
        T(STR_WEB_BACKUP_SECTION), T(STR_WEB_EXPORT_BTN), T(STR_WEB_IMPORT_BTN), T(STR_WEB_IMPORT_LABEL));

    // Browser-based firmware update (raw .bin bytes in the POST /update
    // body, progress computed from xhr.upload.onprogress in JS).
    hb_append(&hb,
        "<div style='margin-top:18px;border-top:1px solid #30363d;padding-top:14px;'>"
        "<label style='color:#00ff88;font-weight:bold;'>\xE2\x9A\xA1 %s</label>"
        "<label style='margin-top:4px;'>%s</label>"
        "<div class='filerow'>"
        "<input type='file' id='ota-file' accept='.bin'>"
        "<button type='button' id='ota-btn' onclick='flashFirmware()'>%s</button>"
        "</div>"
        "<div class='progress-wrap'><div id='ota-progress' class='progress-bar'></div></div>"
        "<p id='ota-status' class='hint'></p>"
        "</div>",
        T(STR_WEB_OTA_SECTION), T(STR_WEB_OTA_FILE_LABEL), T(STR_WEB_OTA_BTN));

    // Firmware update check (version.json manifest) - always active, fetched
    // from the hardcoded OTA_VERSION_CHECK_URL.
    {
        bool upd_avail = ota_update_is_available();
        const char *latest = ota_update_get_latest_version();
        char latest_line[80];
        if (latest[0] == '\0') {
            snprintf(latest_line, sizeof(latest_line), "%s", T(STR_WEB_FWUPD_UNKNOWN));
        } else {
            snprintf(latest_line, sizeof(latest_line), "v%s (%s)", latest,
                      upd_avail ? T(STR_WEB_FWUPD_AVAILABLE) : T(STR_WEB_FWUPD_UP_TO_DATE));
        }
        char download_btn_label[80];
        snprintf(download_btn_label, sizeof(download_btn_label), "%s%s%s",
                 T(STR_WEB_FWUPD_DOWNLOAD_BTN_PREFIX), latest[0] ? latest : "?", T(STR_WEB_FWUPD_DOWNLOAD_BTN_SUFFIX));
        const char *release_url = ota_update_get_release_url();
        hb_append(&hb,
            "<div style='margin-top:18px;border-top:1px solid #30363d;padding-top:14px;'>"
            "<label style='color:#00ff88;font-weight:bold;'>\xF0\x9F\x94\x84 %s</label>"
            "<div class='statrow'><span>%s</span><span>v%s</span></div>"
            "<div class='statrow'><span>%s</span><span id='latest_version_label'>%s</span></div>"
            "<button type='button' class='geobtn' style='margin-top:10px;' id='btn_check_update' onclick='checkOtaUpdate(event)'>%s</button>"
            "<p id='check-update-status' class='hint'></p>"
            "<a id='btn_download_update' href='%s' target='_blank' rel='noopener' class='geobtn' "
            "style='display:%s;background:#2563eb;color:#fff;text-align:center;text-decoration:none;font-weight:bold;box-sizing:border-box;'>%s</a>"
            "<p id='download-update-hint' class='hint' style='display:%s;'>%s</p>"
            "</div>",
            T(STR_WEB_FWUPD_SECTION),
            T(STR_WEB_FWUPD_CURRENT), ota_update_get_installed_version(),
            T(STR_WEB_FWUPD_LATEST), latest_line,
            T(STR_WEB_FWUPD_CHECK_BTN),
            release_url, upd_avail ? "block" : "none", download_btn_label,
            upd_avail ? "block" : "none", T(STR_WEB_FWUPD_DOWNLOAD_HINT));
    }

    hb_append(&hb, "</div>"); // #tab-system

    // TAB: MQTT & Home Assistant
    hb_append(&hb, "<div class='card tabpanel' id='tab-mqtt' style='display:none'><h2>\xF0\x9F\x93\xB6 %s</h2>", T(STR_WEB_MQTT_SECTION));
    hb_append(&hb,
        "<div class=\"setting-row\"><span class=\"setting-label\">%s</span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"mqtt_en\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>",
        T(STR_WEB_MQTT_ENABLE), g_mqtt_enabled ? "checked" : "");
    hb_append(&hb,
        "<div class='grid2'>"
        "<div><label>%s</label><input type='text' name='mqtt_host' value='%s' maxlength='63' placeholder='192.168.1.100'></div>"
        "<div><label>%s</label><input type='number' name='mqtt_port' min='1' max='65535' value='%u'></div>"
        "</div>",
        T(STR_WEB_MQTT_HOST), g_mqtt_host, T(STR_WEB_MQTT_PORT), (unsigned)g_mqtt_port);
    hb_append(&hb,
        "<div class='grid2'>"
        "<div><label>%s</label><input type='text' name='mqtt_user' value='%s' maxlength='31' autocomplete='off'></div>"
        "<div><label>%s</label><input type='password' name='mqtt_pass' value='' maxlength='63' placeholder='%s'></div>"
        "</div>",
        T(STR_WEB_MQTT_USER), g_mqtt_user, T(STR_WEB_MQTT_PASSWORD), T(STR_WEB_PASS_PLACEHOLDER));
    hb_append(&hb,
        "<label>%s<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></label>"
        "<input type=\"text\" name=\"mqtt_devid\" value=\"%s\" maxlength='31'>",
        T(STR_WEB_MQTT_DEVICE_ID), T(STR_WEB_MQTT_DEVICE_ID_HINT), g_mqtt_device_id);
    hb_append(&hb,
        "<div class=\"setting-row\" style='margin-top:10px;'><span class=\"setting-label\">%s"
        "<span class=\"tip\" title=\"%s\">\xE2\x93\x98</span></span>"
        "<label class=\"switch\"><input type=\"checkbox\" name=\"mqtt_ha\" value=\"1\" %s>"
        "<span class=\"slider\"></span></label></div>",
        T(STR_WEB_MQTT_HA_DISCOVERY), T(STR_WEB_MQTT_HA_DISCOVERY_HINT),
        g_mqtt_ha_discovery ? "checked" : "");
    hb_append(&hb,
        "<div class='statrow'><span>%s</span>"
        "<span style='display:flex;align-items:center;'>"
        "<span id='mqtt-led' class='led' style='background:%s;'></span>"
        "<span id='mqtt-status-val'>%s</span></span></div>",
        T(STR_WEB_MQTT_STATUS), mqtt_led_color, mqtt_status_label);
    hb_append(&hb, "</div>"); // #tab-mqtt

    // Fixed version footer - outside the tab panels (visible on every tab),
    // right above the save button.
    hb_append(&hb,
        "<div class='fw-footer'><span class='fw-label'>%s</span> <span class='fw-val'>%s</span></div>",
        T(STR_WEB_FW_FOOTER_LABEL), fw_footer);

    hb_append(&hb,
        "<button type='submit' id='btn' class='btn-save'>\xF0\x9F\x92\xBE %s</button>"
        "<p id='status'></p>"
        "</form>",
        T(STR_WEB_SAVE_REBOOT));

    hb_append(&hb,
        "<script>"
        "var IS_AP_MODE=%s;"
        "var I18N={scanning:'%s',scanFoundPrefix:'%s',scanFoundSuffix:'%s',scanNone:'%s',"
        "saving:'%s',rebooting:'%s',saveError:'%s',"
        "importOk:'%s',importError:'%s',importSelectFile:'%s',"
        "otaUploading:'%s',otaOk:'%s',otaError:'%s',otaSelectFile:'%s',"
        "mqttConnected:'%s',mqttConnecting:'%s',mqttError:'%s',mqttDisabled:'%s',"
        "fwChecking:'%s',fwCheckError:'%s',fwAvailable:'%s',fwUpToDate:'%s',fwUnknown:'%s',"
        "fwDownloadPrefix:'%s',fwDownloadSuffix:'%s'};"
        "var __map=null,__marker=null;"
        "function showTab(name){"
        "['display','location','wifi','system','mqtt'].forEach(function(k){"
        "var el=document.getElementById('tab-'+k);"
        "if(el)el.style.display=(k===name)?'block':'none';"
        "});"
        "document.querySelectorAll('.tabbtn').forEach(function(b){"
        "b.className='tabbtn'+(b.dataset.tab===name?' active':'');"
        "});"
        "if(name==='location'&&__map){setTimeout(function(){__map.invalidateSize();},150);}"
        "try{localStorage.setItem('active_tab',name);}catch(e){}"
        "if(window.location.hash!=='#'+name){"
        "try{history.replaceState(null,'','#'+name);}catch(e){window.location.hash=name;}"
        "}"
        "}"
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
        "setTimeout(function(){__map.invalidateSize();},100);"
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
        "function exportConfig(){window.location.href='/export_config';}"
        "function importConfig(){"
        "var f=document.getElementById('import-file');"
        "var st=document.getElementById('import-status');"
        "if(!f.files||f.files.length===0){st.textContent=I18N.importSelectFile;return;}"
        "var reader=new FileReader();"
        "reader.onload=function(){"
        "fetch('/import_config',{method:'POST',headers:{'Content-Type':'application/json'},body:reader.result})"
        ".then(function(r){"
        "if(!r.ok)throw new Error('bad');"
        "st.textContent=I18N.importOk+' '+I18N.rebooting;"
        "})"
        ".catch(function(){st.textContent=I18N.importError;});"
        "};"
        "reader.readAsText(f.files[0]);"
        "}"
        "function flashFirmware(){"
        "var f=document.getElementById('ota-file');"
        "var st=document.getElementById('ota-status');"
        "var bar=document.getElementById('ota-progress');"
        "var btn=document.getElementById('ota-btn');"
        "if(!f.files||f.files.length===0){st.textContent=I18N.otaSelectFile;return;}"
        "btn.disabled=true;st.textContent=I18N.otaUploading;bar.style.width='0%%';"
        "var xhr=new XMLHttpRequest();"
        "xhr.open('POST','/update',true);"
        "xhr.setRequestHeader('Content-Type','application/octet-stream');"
        "xhr.upload.onprogress=function(e){"
        "if(e.lengthComputable){"
        "var pct=Math.round(e.loaded/e.total*100);"
        "bar.style.width=pct+'%%';"
        "st.textContent=I18N.otaUploading+' '+pct+'%%';"
        "}"
        "};"
        "xhr.onload=function(){"
        "if(xhr.status===200){"
        "bar.style.width='100%%';"
        "st.textContent=I18N.otaOk+' '+I18N.rebooting;"
        "}else{"
        "st.textContent=I18N.otaError+(xhr.responseText?(' - '+xhr.responseText):'');"
        "btn.disabled=false;"
        "}"
        "};"
        "xhr.onerror=function(){st.textContent=I18N.otaError;btn.disabled=false;};"
        "xhr.send(f.files[0]);"
        "}"
        "function checkOtaUpdate(e){"
        "if(e){e.preventDefault();e.stopPropagation();}"
        "var st=document.getElementById('check-update-status');"
        "var lbl=document.getElementById('latest_version_label');"
        "var btn=document.getElementById('btn_check_update');"
        "if(btn)btn.disabled=true;"
        "st.textContent=I18N.fwChecking;"
        "fetch('/check_update',{method:'POST'}).then(function(r){return r.json();}).then(function(d){"
        "if(btn)btn.disabled=false;"
        "st.textContent='';"
        "if(lbl){"
        "lbl.textContent=d.latest_version?('v'+d.latest_version+' ('+(d.update_available?I18N.fwAvailable:I18N.fwUpToDate)+')'):I18N.fwUnknown;"
        "}"
        "var dlBtn=document.getElementById('btn_download_update');"
        "var dlHint=document.getElementById('download-update-hint');"
        "if(dlBtn){"
        "if(d.update_available&&d.release_url){"
        "dlBtn.href=d.release_url;"
        "dlBtn.style.display='block';"
        "dlBtn.textContent=I18N.fwDownloadPrefix+d.latest_version+I18N.fwDownloadSuffix;"
        "if(dlHint)dlHint.style.display='block';"
        "}else{"
        "dlBtn.style.display='none';"
        "if(dlHint)dlHint.style.display='none';"
        "}"
        "}"
        "}).catch(function(){if(btn)btn.disabled=false;st.textContent=I18N.fwCheckError;});"
        "return false;"
        "}"
        "function pollMqttStatus(){"
        "var el=document.getElementById('mqtt-status-val');"
        "var led=document.getElementById('mqtt-led');"
        "if(!el)return;"
        "fetch('/mqtt_status').then(function(r){return r.text();}).then(function(s){"
        "var color='#64748b',label=I18N.mqttDisabled;"
        "if(s==='connected'){color='#22c55e';label=I18N.mqttConnected;}"
        "else if(s==='connecting'){color='#eab308';label=I18N.mqttConnecting;}"
        "else if(s==='error'){color='#ef4444';label=I18N.mqttError;}"
        "el.textContent=label;"
        "if(led)led.style.background=color;"
        "}).catch(function(){});"
        "}"
        "document.addEventListener('DOMContentLoaded',function(){"
        "var isAP=IS_AP_MODE||(window.location.hostname==='192.168.4.1');"
        "if(isAP){showTab('wifi');scanWifi();}"
        "else{"
        "var hashTab=window.location.hash.replace('#','');"
        "var savedTab=null;"
        "try{savedTab=localStorage.getItem('active_tab');}catch(e){}"
        "showTab(hashTab||savedTab||'display');"
        "}"
        "pollMqttStatus();"
        "setInterval(pollMqttStatus,4000);"
        "});"
        "</script>"
        "</body></html>",
        ap_mode ? "true" : "false",
        T(STR_WEB_SCANNING), T(STR_WEB_SCAN_FOUND_PREFIX), T(STR_WEB_SCAN_FOUND_SUFFIX), T(STR_WEB_SCAN_NONE),
        T(STR_WEB_SAVING_MSG), T(STR_WEB_REBOOTING_MSG), T(STR_WEB_SAVE_ERROR),
        T(STR_WEB_IMPORT_OK), T(STR_WEB_IMPORT_ERROR), T(STR_WEB_IMPORT_SELECT_FILE),
        T(STR_WEB_OTA_UPLOADING), T(STR_WEB_OTA_OK), T(STR_WEB_OTA_ERROR), T(STR_WEB_OTA_SELECT_FILE),
        T(STR_WEB_MQTT_STATUS_CONNECTED), T(STR_WEB_MQTT_STATUS_CONNECTING),
        T(STR_WEB_MQTT_STATUS_ERROR), T(STR_WEB_MQTT_STATUS_DISABLED),
        T(STR_WEB_FWUPD_CHECKING), T(STR_WEB_FWUPD_CHECK_ERROR),
        T(STR_WEB_FWUPD_AVAILABLE), T(STR_WEB_FWUPD_UP_TO_DATE), T(STR_WEB_FWUPD_UNKNOWN),
        T(STR_WEB_FWUPD_DOWNLOAD_BTN_PREFIX), T(STR_WEB_FWUPD_DOWNLOAD_BTN_SUFFIX));

    if (hb.pos >= hb.cap - 1) {
        ESP_LOGE(TAG, "Web page exceeded buffer (%d B) - response truncated!", HTML_PAGE_BUF_SIZE);
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, page, hb.pos);
    free(page);
    return ESP_OK;
}

#define SAVE_BODY_MAX_LEN 1024

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

    char param[WEB_FORM_PARAM_LEN];
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
        g_default_range = is_valid_range_km((uint16_t)v) ? (uint16_t)v : DEFAULT_RANGE_DEFAULT;
    }
    if (httpd_query_key_value(buf, "air_mode", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        int v = atoi(param);
        g_air_mode = (v >= 0 && v <= 2) ? (uint8_t)v : AIR_MODE_DEFAULT;
    }
    // Toggle switches are rendered as checkboxes now - unchecked ones are
    // not sent in the form at all, so their state is derived purely from
    // whether the key is present, not from its value.
    g_apts_mode = (httpd_query_key_value(buf, "apts", param, sizeof(param)) == ESP_OK) ? 1 : 0;
    {
        uint8_t mask = 0;
        if (httpd_query_key_value(buf, "apt_civil", param, sizeof(param)) == ESP_OK) mask |= APT_TYPE_CIVIL;
        if (httpd_query_key_value(buf, "apt_mil", param, sizeof(param)) == ESP_OK) mask |= APT_TYPE_MIL;
        if (httpd_query_key_value(buf, "apt_ga", param, sizeof(param)) == ESP_OK) mask |= APT_TYPE_GA;
        g_apt_filter_mask = mask;
    }
    // The "gnd_show" checkbox is checked when ground traffic should be
    // visible, which is the inverse of g_hide_ground.
    g_hide_ground = (httpd_query_key_value(buf, "gnd_show", param, sizeof(param)) == ESP_OK) ? 0 : 1;
    g_map_enabled = (httpd_query_key_value(buf, "map_en", param, sizeof(param)) == ESP_OK) ? 1 : 0;
    g_squawk_alert_enabled = (httpd_query_key_value(buf, "sqk_alert", param, sizeof(param)) == ESP_OK) ? 1 : 0;
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
    g_mqtt_enabled = (httpd_query_key_value(buf, "mqtt_en", param, sizeof(param)) == ESP_OK) ? 1 : 0;
    g_mqtt_ha_discovery = (httpd_query_key_value(buf, "mqtt_ha", param, sizeof(param)) == ESP_OK) ? 1 : 0;
    if (httpd_query_key_value(buf, "mqtt_host", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        snprintf(g_mqtt_host, sizeof(g_mqtt_host), "%s", param);
    }
    if (httpd_query_key_value(buf, "mqtt_port", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        int port = atoi(param);
        g_mqtt_port = (port > 0 && port <= 65535) ? (uint16_t)port : MQTT_PORT_DEFAULT;
    }
    if (httpd_query_key_value(buf, "mqtt_user", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        snprintf(g_mqtt_user, sizeof(g_mqtt_user), "%s", param);
    }
    if (httpd_query_key_value(buf, "mqtt_pass", param, sizeof(param)) == ESP_OK && strlen(param) > 0) {
        url_decode(param);
        snprintf(g_mqtt_pass, sizeof(g_mqtt_pass), "%s", param);
    }
    if (httpd_query_key_value(buf, "mqtt_devid", param, sizeof(param)) == ESP_OK) {
        url_decode(param);
        mqtt_sanitize_topic_id(param, g_mqtt_device_id, sizeof(g_mqtt_device_id));
    }
    free(buf);

    save_settings_to_nvs();
    ESP_LOGI(TAG, "Saved configuration from web panel: SSID='%s' station='%s' (%.6f, %.6f)",
             g_wifi_ssid, g_station_name, g_radar_lat, g_radar_lon);

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);

    xTaskCreate(restart_task, "wifi_restart", 2048, (void *)(uintptr_t)1500, 5, NULL);
    return ESP_OK;
}

// Exports the current configuration as a JSON file for the browser to
// download. The Wi-Fi and MQTT passwords are deliberately omitted
// (security) - the export is meant for backing up radar settings, not for
// carrying credentials.
static esp_err_t export_config_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "station_name", g_station_name);
    cJSON_AddStringToObject(root, "wifi_ssid", g_wifi_ssid);
    cJSON_AddNumberToObject(root, "lat", (double)g_radar_lat);
    cJSON_AddNumberToObject(root, "lon", (double)g_radar_lon);
    cJSON_AddNumberToObject(root, "brightness", g_brightness);
    cJSON_AddNumberToObject(root, "default_range", g_default_range);
    cJSON_AddNumberToObject(root, "air_mode", g_air_mode);
    cJSON_AddNumberToObject(root, "apts_mode", g_apts_mode);
    cJSON_AddNumberToObject(root, "apt_filter_mask", g_apt_filter_mask);
    cJSON_AddNumberToObject(root, "hide_ground", g_hide_ground);
    cJSON_AddNumberToObject(root, "map_enabled", g_map_enabled);
    cJSON_AddNumberToObject(root, "squawk_alert", g_squawk_alert_enabled);
    cJSON_AddNumberToObject(root, "lang", g_lang);
    cJSON_AddNumberToObject(root, "trail_len", g_trail_len);
    cJSON_AddNumberToObject(root, "max_aircraft", g_max_aircraft);
    cJSON_AddNumberToObject(root, "mqtt_enabled", g_mqtt_enabled);
    cJSON_AddStringToObject(root, "mqtt_host", g_mqtt_host);
    cJSON_AddNumberToObject(root, "mqtt_port", g_mqtt_port);
    cJSON_AddStringToObject(root, "mqtt_user", g_mqtt_user);
    cJSON_AddStringToObject(root, "mqtt_device_id", g_mqtt_device_id);
    cJSON_AddNumberToObject(root, "mqtt_ha_discovery", g_mqtt_ha_discovery);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"radar_config.json\"");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return ESP_OK;
}

#define IMPORT_BODY_MAX_LEN 2048

// Imports the configuration from a JSON file (see export_config_get_handler
// for the field schema) - every field is optional and validated the same
// way as in save_post_handler; unknown/missing keys are simply skipped.
static esp_err_t import_config_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > IMPORT_BODY_MAX_LEN) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *buf = malloc(total_len + 1);
    if (!buf) {
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

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *item;
    if ((item = cJSON_GetObjectItem(root, "station_name")) && cJSON_IsString(item)) {
        snprintf(g_station_name, sizeof(g_station_name), "%s", item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(root, "wifi_ssid")) && cJSON_IsString(item)) {
        snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(root, "lat")) && cJSON_IsNumber(item)) {
        g_radar_lat = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(root, "lon")) && cJSON_IsNumber(item)) {
        g_radar_lon = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(root, "brightness")) && cJSON_IsNumber(item)) {
        int v = item->valueint;
        if (v < BRIGHTNESS_MIN) v = BRIGHTNESS_MIN;
        if (v > BRIGHTNESS_MAX) v = BRIGHTNESS_MAX;
        g_brightness = (uint8_t)v;
    }
    if ((item = cJSON_GetObjectItem(root, "default_range")) && cJSON_IsNumber(item)) {
        int v = item->valueint;
        g_default_range = is_valid_range_km((uint16_t)v) ? (uint16_t)v : DEFAULT_RANGE_DEFAULT;
    }
    if ((item = cJSON_GetObjectItem(root, "air_mode")) && cJSON_IsNumber(item)) {
        int v = item->valueint;
        g_air_mode = (v >= 0 && v <= 2) ? (uint8_t)v : AIR_MODE_DEFAULT;
    }
    if ((item = cJSON_GetObjectItem(root, "apts_mode")) && cJSON_IsNumber(item)) {
        g_apts_mode = (item->valueint == 1) ? 1 : 0;
    }
    if ((item = cJSON_GetObjectItem(root, "apt_filter_mask")) && cJSON_IsNumber(item)) {
        int v = item->valueint;
        g_apt_filter_mask = (v >= 0 && (v & ~APT_TYPE_ALL) == 0) ? (uint8_t)v : APT_FILTER_MASK_DEFAULT;
    }
    if ((item = cJSON_GetObjectItem(root, "hide_ground")) && cJSON_IsNumber(item)) {
        g_hide_ground = (item->valueint == 1) ? 1 : 0;
    }
    if ((item = cJSON_GetObjectItem(root, "map_enabled")) && cJSON_IsNumber(item)) {
        g_map_enabled = (item->valueint == 1) ? 1 : 0;
    }
    if ((item = cJSON_GetObjectItem(root, "squawk_alert")) && cJSON_IsNumber(item)) {
        g_squawk_alert_enabled = (item->valueint == 1) ? 1 : 0;
    }
    if ((item = cJSON_GetObjectItem(root, "lang")) && cJSON_IsNumber(item)) {
        wifi_mgr_set_lang((app_lang_t)item->valueint);
    }
    if ((item = cJSON_GetObjectItem(root, "trail_len")) && cJSON_IsNumber(item)) {
        wifi_mgr_set_trail_len((uint8_t)item->valueint);
    }
    if ((item = cJSON_GetObjectItem(root, "max_aircraft")) && cJSON_IsNumber(item)) {
        wifi_mgr_set_max_aircraft((uint16_t)item->valueint);
    }
    if ((item = cJSON_GetObjectItem(root, "mqtt_enabled")) && cJSON_IsNumber(item)) {
        g_mqtt_enabled = (item->valueint == 1) ? 1 : 0;
    }
    if ((item = cJSON_GetObjectItem(root, "mqtt_host")) && cJSON_IsString(item)) {
        snprintf(g_mqtt_host, sizeof(g_mqtt_host), "%s", item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(root, "mqtt_port")) && cJSON_IsNumber(item)) {
        int port = item->valueint;
        g_mqtt_port = (port > 0 && port <= 65535) ? (uint16_t)port : MQTT_PORT_DEFAULT;
    }
    if ((item = cJSON_GetObjectItem(root, "mqtt_user")) && cJSON_IsString(item)) {
        snprintf(g_mqtt_user, sizeof(g_mqtt_user), "%s", item->valuestring);
    }
    if ((item = cJSON_GetObjectItem(root, "mqtt_device_id")) && cJSON_IsString(item)) {
        mqtt_sanitize_topic_id(item->valuestring, g_mqtt_device_id, sizeof(g_mqtt_device_id));
    }
    if ((item = cJSON_GetObjectItem(root, "mqtt_ha_discovery")) && cJSON_IsNumber(item)) {
        g_mqtt_ha_discovery = (item->valueint == 1) ? 1 : 0;
    }
    cJSON_Delete(root);

    save_settings_to_nvs();
    ESP_LOGI(TAG, "Imported configuration from JSON file, restarting device...");

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);

    xTaskCreate(restart_task, "wifi_restart", 2048, (void *)(uintptr_t)1500, 5, NULL);
    return ESP_OK;
}

#define OTA_RECV_CHUNK_SIZE 4096

// Browser-based firmware update: the POST /update body is the raw .bin file
// bytes (no multipart), streamed straight into the OTA partition. On
// success the restart happens after a ~2s delay (see restart_task), so the
// browser has time to show the message/progress bar at 100%.
// ESP-IDF app images always start with this magic byte. Checked before any
// OTA partition is touched, so a malformed upload (most commonly a browser
// sending multipart/form-data instead of the raw .bin body, which starts
// with '-' from "------WebKitFormBoundary...") is rejected outright instead
// of writing garbage into the OTA partition and bricking the device on the
// next boot (invalid header / bootloop).
#define ESP_IMAGE_MAGIC_BYTE 0xE9

static esp_err_t update_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *buf = malloc(OTA_RECV_CHUNK_SIZE);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read the first chunk (and validate the magic byte) before calling
    // esp_ota_begin(), so a rejected upload never touches the OTA partition.
    int received = httpd_req_recv(req, buf, OTA_RECV_CHUNK_SIZE);
    if (received <= 0) {
        free(buf);
        ESP_LOGE(TAG, "OTA: failed to receive firmware header");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    if ((uint8_t)buf[0] != ESP_IMAGE_MAGIC_BYTE) {
        free(buf);
        ESP_LOGE(TAG, "Invalid binary format: multipart boundary detected instead of raw .bin");
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "Invalid firmware file: expected a raw .bin image (magic byte 0xE9)");
        return ESP_OK;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        free(buf);
        ESP_LOGE(TAG, "OTA: no free update partition");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // OTA_WITH_SEQUENTIAL_WRITES erases flash sectors as esp_ota_write()
    // reaches them instead of erasing the whole (8 MB) partition up front,
    // so the first write below lands quickly instead of blocking behind a
    // multi-second bulk erase.
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        free(buf);
        ESP_LOGE(TAG, "OTA: esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    // Critical: the first chunk was already consumed from the socket above
    // (to peek at the magic byte) - it must still be written to flash here,
    // otherwise the OTA partition starts with erased 0xFF bytes instead of
    // the image header and the bootloader rejects it on next boot with
    // "invalid header: 0xffffffff".
    bool ota_failed = (esp_ota_write(ota_handle, buf, received) != ESP_OK);
    int remaining = total_len - received;

    while (!ota_failed && remaining > 0) {
        int to_read = remaining < OTA_RECV_CHUNK_SIZE ? remaining : OTA_RECV_CHUNK_SIZE;
        int recv_len = httpd_req_recv(req, buf, to_read);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                // Transient socket read timeout - the client may still be
                // sending data, retry instead of aborting the whole upload.
                continue;
            }
            ota_failed = true;
            break;
        }
        if (esp_ota_write(ota_handle, buf, recv_len) != ESP_OK) {
            ota_failed = true;
            break;
        }
        remaining -= recv_len;

        // The httpd task is not registered with the Task Watchdog Timer, so
        // esp_task_wdt_reset() here would fail with "task not found" - the
        // real risk is starving the IDLE1 task (which IS watched by TWDT)
        // on this core during a long, tight receive/write loop. Yielding
        // briefly after every write lets IDLE1 run and feed its own
        // watchdog entry, so a multi-MB upload completes without tripping
        // "IDLE1 watchdog triggered".
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free(buf);

    // Only a fully-received image (remaining == 0, i.e. exactly
    // req->content_len bytes written) is allowed to proceed - a short read
    // must never reach esp_ota_end()/esp_ota_set_boot_partition().
    if (ota_failed || remaining != 0) {
        esp_ota_abort(ota_handle);
        ESP_LOGE(TAG, "OTA: firmware receive/write failed (%d/%d bytes received)",
                 total_len - remaining, total_len);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_end failed (corrupt image?): %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA: firmware written successfully (%d B), restarting in 2s...", total_len);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);

    xTaskCreate(restart_task, "ota_restart", 2048, (void *)(uintptr_t)2000, 5, NULL);
    return ESP_OK;
}

// Writes the SSID into a JSON buffer, escaping '"' and '\\' and skipping
// control characters (an SSID can theoretically contain arbitrary bytes).
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

// Scans available Wi-Fi networks and returns a JSON array of SSID names. If
// the device is in pure SoftAP mode (first-time setup with no saved
// network), it switches to APSTA for the duration of the scan - the SoftAP
// itself (and clients connected to this panel) stays active.
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
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(err));
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

        ESP_LOGI(TAG, "Scan complete: found %d Wi-Fi networks", ap_count);

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
    wifi_mgr_set_brightness((uint8_t)val);
    mqtt_service_publish_state();

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Polled by the MQTT tab's JS every few seconds for a near-real-time
// connection status readout, without a full page reload.
static esp_err_t mqtt_status_get_handler(httpd_req_t *req) {
    const char *status_str = "disabled";
    switch (mqtt_service_get_status()) {
        case MQTT_STATUS_CONNECTED:  status_str = "connected"; break;
        case MQTT_STATUS_CONNECTING: status_str = "connecting"; break;
        case MQTT_STATUS_ERROR:      status_str = "error"; break;
        default:                     status_str = "disabled"; break;
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr(req, status_str);
    return ESP_OK;
}

// Performs a blocking, out-of-cycle firmware version check against the
// hardcoded OTA_VERSION_CHECK_URL manifest. Responds with the fresh version
// state as JSON so the browser can update the System tab in place instead
// of reloading the page (which would also reset the currently open tab).
static esp_err_t check_update_post_handler(httpd_req_t *req) {
    ota_update_check_now(); // blocking - see doc comment in ota_update_service.h

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "installed_version", ota_update_get_installed_version());
    cJSON_AddStringToObject(root, "latest_version", ota_update_get_latest_version());
    cJSON_AddBoolToObject(root, "update_available", ota_update_is_available());
    cJSON_AddStringToObject(root, "release_url", ota_update_get_release_url());
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, payload ? payload : "{}");
    free(payload);
    return ESP_OK;
}

static void start_web_server(void) {
    if (g_web_server != NULL) {
        return;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;
    config.max_uri_handlers = 12;
    httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
    httpd_uri_t save_uri = {.uri = "/save", .method = HTTP_POST, .handler = save_post_handler};
    httpd_uri_t brightness_uri = {.uri = "/set_brightness", .method = HTTP_GET, .handler = set_brightness_get_handler};
    httpd_uri_t scan_uri = {.uri = "/scan", .method = HTTP_GET, .handler = scan_get_handler};
    httpd_uri_t export_uri = {.uri = "/export_config", .method = HTTP_GET, .handler = export_config_get_handler};
    httpd_uri_t import_uri = {.uri = "/import_config", .method = HTTP_POST, .handler = import_config_post_handler};
    httpd_uri_t update_uri = {.uri = "/update", .method = HTTP_POST, .handler = update_post_handler};
    httpd_uri_t mqtt_status_uri = {.uri = "/mqtt_status", .method = HTTP_GET, .handler = mqtt_status_get_handler};
    httpd_uri_t check_update_uri = {.uri = "/check_update", .method = HTTP_POST, .handler = check_update_post_handler};

    if (httpd_start(&g_web_server, &config) == ESP_OK) {
        httpd_register_uri_handler(g_web_server, &root_uri);
        httpd_register_uri_handler(g_web_server, &save_uri);
        httpd_register_uri_handler(g_web_server, &brightness_uri);
        httpd_register_uri_handler(g_web_server, &scan_uri);
        httpd_register_uri_handler(g_web_server, &export_uri);
        httpd_register_uri_handler(g_web_server, &import_uri);
        httpd_register_uri_handler(g_web_server, &update_uri);
        httpd_register_uri_handler(g_web_server, &mqtt_status_uri);
        httpd_register_uri_handler(g_web_server, &check_update_uri);
        ESP_LOGI(TAG, "Radar configuration web panel started on port 80!");
    }
}

// ================= WI-FI EVENT HANDLING =================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        radar_ui_wifi_notify_connecting(g_wifi_ssid);
        radar_ui_update_wifi_status(WIFI_STATUS_CONNECTING);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying connection...");
        esp_wifi_connect();
        radar_ui_wifi_notify_connecting(g_wifi_ssid);
        radar_ui_update_wifi_status(WIFI_STATUS_ERROR);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected to Wi-Fi. Obtained IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        start_web_server();
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        radar_ui_wifi_notify_connected(ip_str);
        radar_ui_update_wifi_status(WIFI_STATUS_CONNECTED);
    }
}

// Attempts to connect in STA mode to the saved network. Returns true on success.
static bool wifi_try_connect_sta(void) {
    if (strlen(g_wifi_ssid) == 0) {
        ESP_LOGW(TAG, "No SSID configured.");
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

    ESP_LOGI(TAG, "Connecting to network '%s' (timeout %d s)...", g_wifi_ssid, WIFI_CONNECT_TIMEOUT_MS / 1000);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                                            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT) {
        return true;
    }

    ESP_LOGW(TAG, "Failed to connect to '%s' - switching to SoftAP mode.", g_wifi_ssid);
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
        ESP_LOGE(TAG, "esp_wifi_set_mode(AP) failed: %s", esp_err_to_name(err));
        ok = false;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) failed: %s", esp_err_to_name(err));
        ok = false;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start() (SoftAP) failed: %s", esp_err_to_name(err));
        ok = false;
    }

    if (!ok) {
        ESP_LOGE(TAG, "Failed to start SoftAP '%s' - the web panel will be unavailable.", AP_SSID);
        return;
    }

    ESP_LOGI(TAG, "SoftAP started: SSID='%s' (no password). Connect and go to http://192.168.4.1/", AP_SSID);
    radar_ui_wifi_notify_ap_mode(AP_SSID, "");
    radar_ui_update_wifi_status(WIFI_STATUS_CONNECTING);
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

    g_https_mutex = xSemaphoreCreateMutex();

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

bool wifi_mgr_is_connected(void) {
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}
