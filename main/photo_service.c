#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"

#include "lvgl.h"

// stb_image allocates in PSRAM (JPEG images don't reasonably fit in DRAM).
// free() works correctly here for pointers from heap_caps_malloc - in
// ESP-IDF the standard libc allocator and heap_caps share the same multi_heap.
#define STBI_MALLOC(sz)           heap_caps_malloc(sz, MALLOC_CAP_SPIRAM)
#define STBI_REALLOC(p, newsz)    heap_caps_realloc(p, newsz, MALLOC_CAP_SPIRAM)
#define STBI_FREE(p)              free(p)
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include "stb_image.h"

#include "wifi_manager.h"
#include "radar_ui.h"
#include "photo_service.h"

static const char *TAG = "PHOTO_SVC";

#define PHOTO_JSON_BUF_SIZE   (8 * 1024)
#define PHOTO_JPEG_BUF_SIZE   (256 * 1024)

static QueueHandle_t s_req_queue;

#define PHOTO_HTTP_USER_AGENT "Mozilla/5.0 (Windows NT 10.0; Win64; x64) RadarADSB/1.0"

static bool http_get_to_buffer_ua(const char *url, uint8_t *buf, int buf_size, int *out_len, int *out_status,
                                   const char *user_agent) {
    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 8000,
        .buffer_size = 4096,
        .max_redirection_count = 5,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return false;
    esp_http_client_set_header(client, "User-Agent", user_agent);

    bool ok = false;
    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        *out_status = esp_http_client_get_status_code(client);

        int total = 0, r;
        while ((r = esp_http_client_read(client, (char *)(buf + total), buf_size - total - 1)) > 0) {
            total += r;
            if (total >= buf_size - 1) break;
        }
        *out_len = total;
        ok = (*out_status == 200 && total > 0);
        esp_http_client_close(client);
        ESP_LOGI(TAG, "HTTP GET %s -> status %d, %d B", url, *out_status, total);
    } else {
        ESP_LOGW(TAG, "HTTP open failed: %s (%s)", esp_err_to_name(err), url);
    }
    esp_http_client_cleanup(client);
    return ok;
}

static bool http_get_to_buffer(const char *url, uint8_t *buf, int buf_size, int *out_len, int *out_status) {
    return http_get_to_buffer_ua(url, buf, buf_size, out_len, out_status, PHOTO_HTTP_USER_AGENT);
}

// Extracts a single "key":"value" text field without a full JSON parser -
// consistent with the approach in adsb_service.c (parse_adsb_json). Reverses
// "\/" -> "/" escapes that some servers insert into URLs.
static bool extract_string_field(const char *json, const char *key, char *out, size_t out_size) {
    const char *p = strstr(json, key);
    if (!p) return false;
    p += strlen(key);

    const char *end = strchr(p, '"');
    if (!end) return false;

    size_t len = end - p;
    if (len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';

    char *w = out, *r = out;
    while (*r) {
        if (r[0] == '\\' && r[1] == '/') { *w++ = '/'; r += 2; }
        else *w++ = *r++;
    }
    *w = '\0';
    return true;
}

static bool extract_photo_url(const char *json, char *url_out, size_t url_out_size) {
    if (extract_string_field(json, "\"thumbnail_large\":{\"src\":\"", url_out, url_out_size)) return true;
    return extract_string_field(json, "\"thumbnail\":{\"src\":\"", url_out, url_out_size);
}

// STEP 1/2 of the fallback chain: Planespotters.net by hex or by
// registration (same JSON response format for both endpoints).
static bool fetch_planespotters(const char *url, char *photo_url_out, size_t photo_url_size,
                                 char *photographer_out, size_t photographer_size) {
    uint8_t *json_buf = heap_caps_malloc(PHOTO_JSON_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!json_buf) return false;

    int len = 0, status = 0;
    bool ok = http_get_to_buffer(url, json_buf, PHOTO_JSON_BUF_SIZE, &len, &status);
    if (ok) json_buf[len < PHOTO_JSON_BUF_SIZE ? len : PHOTO_JSON_BUF_SIZE - 1] = '\0';

    bool have_url = ok && extract_photo_url((char *)json_buf, photo_url_out, photo_url_size);
    if (have_url) {
        extract_string_field((char *)json_buf, "\"photographer\":\"", photographer_out, photographer_size);
    }
    heap_caps_free(json_buf);

    if (have_url) {
        ESP_LOGI(TAG, "Planespotters: photo found -> %s", photo_url_out);
    } else {
        ESP_LOGI(TAG, "Planespotters: no photo (%s, status %d)", url, status);
    }
    return have_url;
}

// STEP 3 of the fallback chain: Airport-Data.com (backup source when
// Planespotters has no photo by hex or by registration). param_key is "m"
// (hex) or "r" (registration) - matching the airport-data.com API.
static bool fetch_airport_data(const char *param_key, const char *param_value,
                                char *photo_url_out, size_t photo_url_size,
                                char *photographer_out, size_t photographer_size) {
    char url[160];
    snprintf(url, sizeof(url), "https://airport-data.com/api/ac_thumb.json?%s=%s&n=1", param_key, param_value);

    uint8_t *json_buf = heap_caps_malloc(PHOTO_JSON_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!json_buf) return false;

    int len = 0, status = 0;
    bool ok = http_get_to_buffer(url, json_buf, PHOTO_JSON_BUF_SIZE, &len, &status);
    if (ok) json_buf[len < PHOTO_JSON_BUF_SIZE ? len : PHOTO_JSON_BUF_SIZE - 1] = '\0';

    bool have_url = false;
    if (ok) {
        have_url = extract_string_field((char *)json_buf, "\"image\":\"", photo_url_out, photo_url_size);
        if (!have_url) {
            have_url = extract_string_field((char *)json_buf, "\"thumbnail\":\"", photo_url_out, photo_url_size);
        }
        if (have_url) {
            extract_string_field((char *)json_buf, "\"photographer\":\"", photographer_out, photographer_size);
        }
    }
    heap_caps_free(json_buf);

    if (have_url) {
        ESP_LOGI(TAG, "Airport-Data: photo found -> %s", photo_url_out);
    } else {
        ESP_LOGI(TAG, "Airport-Data: no photo (%s, status %d)", url, status);
    }
    return have_url;
}

// STEP 4 of the fallback chain (a representative photo for the type, not the
// specific airframe): maps common ICAO codes to the exact Wikipedia article
// title, used directly with the REST Summary API (no search-engine
// guessing). Codes outside the table fall back to the type_code itself as
// the title - only works if it happens to match a real page.
typedef struct {
    const char *icao;
    const char *wiki_title;
} icao_wiki_map_t;

static const icao_wiki_map_t ICAO_WIKI_MAP[] = {
    {"C208", "Cessna_208_Caravan"},
    {"C172", "Cessna_172_Skyhawk"},
    {"C152", "Cessna_152"},
    {"C182", "Cessna_182_Skylane"},
    {"P208", "Tecnam_P2008"},
    {"P210", "Tecnam_P2010"},
    {"P06T", "Tecnam_P2006T"},
    {"AS50", "Eurocopter_AS350_\xC3\x89cureuil"},
    {"EC35", "Eurocopter_EC135"},
    {"EC45", "Eurocopter_EC145"},
    {"H145", "Airbus_Helicopters_H145"},
    {"H135", "Airbus_Helicopters_H135"},
    {"H125", "Eurocopter_AS350_\xC3\x89cureuil"},
    {"R44",  "Robinson_R44"},
    {"R22",  "Robinson_R22"},
    {"R66",  "Robinson_R66"},
    {"B738", "Boeing_737_Next_Generation"},
    {"B38M", "Boeing_737_MAX"},
    {"B737", "Boeing_737"},
    {"B77W", "Boeing_777"},
    {"B788", "Boeing_787_Dreamliner"},
    {"B789", "Boeing_787_Dreamliner"},
    {"A320", "Airbus_A320_family"},
    {"A20N", "Airbus_A320neo_family"},
    {"A321", "Airbus_A321"},
    {"A21N", "Airbus_A320neo_family"},
    {"A332", "Airbus_A330"},
    {"A333", "Airbus_A330"},
    {"A359", "Airbus_A350"},
    {"E190", "Embraer_E-Jet_family"},
    {"E195", "Embraer_E-Jet_family"},
    {"E295", "Embraer_E-Jets_E2_family"},
    {"AT76", "ATR_72"},
    {"AT46", "ATR_42"},
    {"DH8D", "De_Havilland_Canada_Dash_8"},
    {NULL, NULL}
};

static const char *lookup_wiki_title(const char *type_code) {
    for (size_t i = 0; ICAO_WIKI_MAP[i].icao != NULL; i++) {
        if (strcmp(type_code, ICAO_WIKI_MAP[i].icao) == 0) return ICAO_WIKI_MAP[i].wiki_title;
    }
    return type_code;
}

// Percent-encodes the page title for the URL (including non-ASCII UTF-8
// bytes, e.g. in "Écureuil") - MediaWiki expects underscores instead of
// spaces (already handled in the table), so we only encode characters
// outside the safe set.
static void url_encode_title(const char *title, char *out, size_t out_size) {
    static const char hex[] = "0123456789ABCDEF";
    size_t o = 0;
    for (size_t i = 0; title[i] != '\0' && o < out_size - 4; i++) {
        unsigned char c = (unsigned char)title[i];
        bool safe = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '-' || c == '.' || c == '~';
        if (safe) {
            out[o++] = (char)c;
        } else {
            out[o++] = '%';
            out[o++] = hex[(c >> 4) & 0xF];
            out[o++] = hex[c & 0xF];
        }
    }
    out[o] = '\0';
}

#define WIKIPEDIA_USER_AGENT "RadarADSB/1.0 (ESP32-P4 Flight Radar; https://github.com/espressif)"

// REST Summary API - returns page metadata (including
// "thumbnail":{"source":...}) for the exact title, no search needed.
static bool fetch_wikipedia_type_photo(const char *type_code, char *photo_url_out, size_t photo_url_size) {
    const char *wiki_title = lookup_wiki_title(type_code);

    char encoded_title[96];
    url_encode_title(wiki_title, encoded_title, sizeof(encoded_title));

    char url[220];
    snprintf(url, sizeof(url), "https://en.wikipedia.org/api/rest_v1/page/summary/%s", encoded_title);

    uint8_t *json_buf = heap_caps_malloc(PHOTO_JSON_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!json_buf) return false;

    int len = 0, status = 0;
    bool ok = http_get_to_buffer_ua(url, json_buf, PHOTO_JSON_BUF_SIZE, &len, &status, WIKIPEDIA_USER_AGENT);
    if (ok) json_buf[len < PHOTO_JSON_BUF_SIZE ? len : PHOTO_JSON_BUF_SIZE - 1] = '\0';

    bool have_url = ok && extract_string_field((char *)json_buf, "\"thumbnail\":{\"source\":\"",
                                                photo_url_out, photo_url_size);
    heap_caps_free(json_buf);

    if (have_url) {
        ESP_LOGI(TAG, "Wikipedia REST: found photo for type %s (%s) -> %s", type_code, wiki_title, photo_url_out);
    } else {
        ESP_LOGI(TAG, "Wikipedia REST: no thumbnail for type %s (%s), status %d", type_code, wiki_title, status);
    }
    return have_url;
}

// Decodes JPEG (including progressive - handled natively by stb_image) into
// an RGB565 buffer in PSRAM, sized to the image's actual dimensions. The
// intermediate RGB888 buffer from stbi_load_from_memory is freed right
// after conversion.
static lv_image_dsc_t *decode_jpeg(const uint8_t *jpeg_buf, size_t jpeg_len) {
    int w = 0, h = 0, src_channels = 0;
    stbi_set_flip_vertically_on_load(0);
    unsigned char *rgb_data = stbi_load_from_memory((const stbi_uc *)jpeg_buf, (int)jpeg_len,
                                                      &w, &h, &src_channels, 3);
    if (!rgb_data || w <= 0 || h <= 0) {
        if (rgb_data) stbi_image_free(rgb_data);
        return NULL;
    }

    uint8_t *pixel_buf = heap_caps_malloc((size_t)w * h * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pixel_buf) {
        stbi_image_free(rgb_data);
        return NULL;
    }

    uint16_t *dst = (uint16_t *)pixel_buf;
    for (int y = 0; y < h; y++) {
        int src_y = (h - 1) - y; // read rows bottom-to-top (flip Y)
        const unsigned char *src_row = rgb_data + (size_t)src_y * w * 3;
        uint16_t *dst_row = dst + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            uint8_t r = src_row[x * 3 + 0];
            uint8_t g = src_row[x * 3 + 1];
            uint8_t b = src_row[x * 3 + 2];
            dst_row[x] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        }
    }
    stbi_image_free(rgb_data);

    lv_image_dsc_t *dsc = heap_caps_malloc(sizeof(lv_image_dsc_t), MALLOC_CAP_SPIRAM);
    if (!dsc) {
        heap_caps_free(pixel_buf);
        return NULL;
    }
    memset(dsc, 0, sizeof(*dsc));
    dsc->header.cf = LV_COLOR_FORMAT_RGB565;
    dsc->header.magic = LV_IMAGE_HEADER_MAGIC;
    dsc->header.w = (uint32_t)w;
    dsc->header.h = (uint32_t)h;
    dsc->data_size = (uint32_t)w * h * 2;
    dsc->data = pixel_buf;
    return dsc;
}

typedef struct {
    char hex[8];
    char reg[16];
    char type_code[8];
} photo_request_t;

// Fallback chain: Planespotters/hex -> Planespotters/reg -> Airport-Data.com
// -> Wikipedia (representative photo of the type). Stops at the first
// source that returns a photo URL.
static void process_request(const char *hex, const char *reg, const char *type_code) {
    char photo_url[256] = "";
    char photographer[64] = "";
    bool have_reg = reg && strlen(reg) > 1;
    bool have_type = type_code && strlen(type_code) >= 2;
    bool is_type_fallback = false;

    char url[160];
    snprintf(url, sizeof(url), "https://api.planespotters.net/pub/photos/hex/%s", hex);
    bool have_url = fetch_planespotters(url, photo_url, sizeof(photo_url), photographer, sizeof(photographer));

    if (!have_url && have_reg) {
        snprintf(url, sizeof(url), "https://api.planespotters.net/pub/photos/reg/%s", reg);
        have_url = fetch_planespotters(url, photo_url, sizeof(photo_url), photographer, sizeof(photographer));
    }

    if (!have_url) {
        if (have_reg) {
            have_url = fetch_airport_data("r", reg, photo_url, sizeof(photo_url), photographer, sizeof(photographer));
        } else {
            have_url = fetch_airport_data("m", hex, photo_url, sizeof(photo_url), photographer, sizeof(photographer));
        }
    }

    if (!have_url && have_type) {
        have_url = fetch_wikipedia_type_photo(type_code, photo_url, sizeof(photo_url));
        if (have_url) {
            snprintf(photographer, sizeof(photographer), "[Model] %s", type_code);
            is_type_fallback = true;
        }
    }

    if (!have_url) {
        ESP_LOGI(TAG, "No photo in any database for hex=%s reg=%s type=%s",
                 hex, have_reg ? reg : "-", have_type ? type_code : "-");
        radar_ui_set_aircraft_photo(NULL, NULL, hex, false);
        return;
    }

    uint8_t *jpeg_buf = heap_caps_malloc(PHOTO_JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!jpeg_buf) {
        radar_ui_set_aircraft_photo(NULL, NULL, hex, false);
        return;
    }

    int jpeg_len = 0, status = 0;
    bool ok = http_get_to_buffer(photo_url, jpeg_buf, PHOTO_JPEG_BUF_SIZE, &jpeg_len, &status);
    if (!ok || jpeg_len <= 0) {
        ESP_LOGW(TAG, "JPEG fetch failed for %s (status %d)", hex, status);
        heap_caps_free(jpeg_buf);
        radar_ui_set_aircraft_photo(NULL, NULL, hex, false);
        return;
    }

    lv_image_dsc_t *dsc = decode_jpeg(jpeg_buf, (size_t)jpeg_len);
    heap_caps_free(jpeg_buf);

    if (!dsc) {
        ESP_LOGW(TAG, "JPEG decode failed for %s", hex);
        radar_ui_set_aircraft_photo(NULL, NULL, hex, false);
        return;
    }

    ESP_LOGI(TAG, "Photo %s decoded: %" PRIu32 "x%" PRIu32, hex, dsc->header.w, dsc->header.h);
    radar_ui_set_aircraft_photo(dsc, photographer, hex, is_type_fallback);
}

static void photo_worker_task(void *arg) {
    (void)arg;
    wifi_manager_wait_connected();

    photo_request_t req;
    while (1) {
        if (xQueueReceive(s_req_queue, &req, portMAX_DELAY) == pdTRUE) {
            process_request(req.hex, req.reg, req.type_code);
        }
    }
}

void photo_service_start(void) {
    s_req_queue = xQueueCreate(1, sizeof(photo_request_t));
    xTaskCreatePinnedToCore(photo_worker_task, "photo_worker", 8192, NULL, 2, NULL, 1);
}

void photo_service_request(const char *hex, const char *registration, const char *type_code) {
    if (!s_req_queue || !hex || !hex[0]) return;
    photo_request_t req = { 0 };
    snprintf(req.hex, sizeof(req.hex), "%s", hex);
    if (registration) snprintf(req.reg, sizeof(req.reg), "%s", registration);
    if (type_code) snprintf(req.type_code, sizeof(req.type_code), "%s", type_code);
    xQueueOverwrite(s_req_queue, &req);
}
