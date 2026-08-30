#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"

#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"
#include "lvgl.h"

// stbi_load_from_memory() is already implemented (STB_IMAGE_IMPLEMENTATION)
// in photo_service.c, which links STBI_MALLOC to PSRAM - only declarations
// here, no re-implementation.
#include "stb_image.h"

#include "aircraft_types.h"
#include "wifi_manager.h"
#include "map_tile_service.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "MAP_TILE_SVC";

#define TILE_SIZE               256
#define MAP_TILE_PNG_BUF_SIZE   (64 * 1024)
#define MAP_TILE_USER_AGENT     "ESP32P4-ADSB-Radar/1.0 (ESP32-P4 Radar Client)"
#define MAP_MIN_ZOOM             2
#define MAP_MAX_ZOOM            18
#define MAP_BRIGHTNESS_PCT      58

typedef struct {
    float lat;
    float lon;
    float range_km;
} map_reload_req_t;

static QueueHandle_t s_req_queue;
static lv_obj_t *s_canvas = NULL;
static uint8_t *s_canvas_buf = NULL;
static bool s_enabled = true;

static void latlon_to_tilef(double lat, double lon, int zoom, double *xtile, double *ytile) {
    double lat_rad = lat * M_PI / 180.0;
    double n = pow(2.0, zoom);
    *xtile = (lon + 180.0) / 360.0 * n;
    *ytile = (1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * n;
}

// Picks a zoom level so the tile resolution (m/px) approximately matches
// the radar scale: range_km over RADAR_MAX_RADIUS pixels.
static int compute_zoom_for_range(float range_km, float lat) {
    double lat_rad = (double)lat * M_PI / 180.0;
    double meters_per_px_target = ((double)range_km * 1000.0) / (double)RADAR_MAX_RADIUS;
    if (meters_per_px_target < 0.1) meters_per_px_target = 0.1;
    double z = log2(156543.03392 * cos(lat_rad) / meters_per_px_target);
    int zoom = (int)lround(z);
    if (zoom < MAP_MIN_ZOOM) zoom = MAP_MIN_ZOOM;
    if (zoom > MAP_MAX_ZOOM) zoom = MAP_MAX_ZOOM;
    return zoom;
}

// g_https_mutex serializes this against adsb_service.c's TLS fetches - see
// the comment on g_https_mutex in wifi_manager.h. Each tile is a separate
// short-lived HTTPS connection (no keep-alive - the next tile is usually a
// different host-relative path anyway), so only the actual open/read/close
// section holds the mutex, not PNG decoding or blitting.
static bool http_get_tile(const char *url, uint8_t *buf, int buf_size, int *out_len, int *out_status) {
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
    esp_http_client_set_header(client, "User-Agent", MAP_TILE_USER_AGENT);

    bool ok = false;
    xSemaphoreTake(g_https_mutex, portMAX_DELAY);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        *out_status = esp_http_client_get_status_code(client);

        int total = 0, r;
        while ((r = esp_http_client_read(client, (char *)(buf + total), buf_size - total)) > 0) {
            total += r;
            if (total >= buf_size) break;
        }
        *out_len = total;
        ok = (*out_status == 200 && total > 0);
        esp_http_client_close(client);
    } else {
        ESP_LOGW(TAG, "HTTP open failed: %s (%s)", esp_err_to_name(err), url);
    }
    xSemaphoreGive(g_https_mutex);
    esp_http_client_cleanup(client);
    return ok;
}

// Converts an RGB888 tile to luminance, inverts it (negative) and darkens
// it to a uniform, neutral shade of gray/graphite (no tint - r=g=b), then
// writes the result (converted to RGB565) into the map canvas buffer
// (MAP_SIZE x MAP_SIZE) 1:1, clipped to its bounds.
static void darken_and_blit_tile(const uint8_t *rgb, int tile_w, int tile_h, int dst_x0, int dst_y0) {
    uint16_t *dst = (uint16_t *)s_canvas_buf;
    for (int y = 0; y < tile_h; y++) {
        int dy = dst_y0 + (tile_h - 1 - y);
        if (dy < 0 || dy >= MAP_SIZE) continue;
        const uint8_t *src_row = rgb + (size_t)y * tile_w * 3;
        uint16_t *dst_row = dst + (size_t)dy * MAP_SIZE;
        for (int x = 0; x < tile_w; x++) {
            int dx = dst_x0 + x;
            if (dx < 0 || dx >= MAP_SIZE) continue;
            uint8_t src_r = src_row[x * 3 + 0];
            uint8_t src_g = src_row[x * 3 + 1];
            uint8_t src_b = src_row[x * 3 + 2];
            uint32_t lum = ((uint32_t)src_r * 77 + (uint32_t)src_g * 150 + (uint32_t)src_b * 29) >> 8;
            uint32_t inv = 255 - lum;
            uint8_t gray = (uint8_t)((inv * MAP_BRIGHTNESS_PCT) / 100);
            uint8_t r = gray;
            uint8_t g = gray;
            uint8_t b = gray;
            dst_row[dx] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        }
    }
}

static void process_reload(float lat, float lon, float range_km) {
    int zoom = compute_zoom_for_range(range_km, lat);
    int n = 1 << zoom;

    double xtile_f, ytile_f;
    latlon_to_tilef(lat, lon, zoom, &xtile_f, &ytile_f);

    int tx_min = (int)floor(xtile_f - (double)RADAR_CENTER_X / TILE_SIZE) - 1;
    int tx_max = (int)floor(xtile_f + (double)(MAP_SIZE - RADAR_CENTER_X) / TILE_SIZE) + 1;
    int ty_min = (int)floor(ytile_f - (double)RADAR_CENTER_Y / TILE_SIZE) - 1;
    int ty_max = (int)floor(ytile_f + (double)(MAP_SIZE - RADAR_CENTER_Y) / TILE_SIZE) + 1;

    uint8_t *png_buf = heap_caps_malloc(MAP_TILE_PNG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!png_buf) {
        ESP_LOGE(TAG, "Failed to allocate tile PNG buffer in PSRAM!");
        return;
    }

    memset(s_canvas_buf, 0, (size_t)MAP_SIZE * MAP_SIZE * 2);

    for (int ty = ty_min; ty <= ty_max; ty++) {
        if (ty < 0 || ty >= n) continue;
        for (int tx_raw = tx_min; tx_raw <= tx_max; tx_raw++) {
            int tx = ((tx_raw % n) + n) % n;
            char url[96];
            snprintf(url, sizeof(url), "https://tile.openstreetmap.org/%d/%d/%d.png", zoom, tx, ty);

            int len = 0, status = 0;
            if (!http_get_tile(url, png_buf, MAP_TILE_PNG_BUF_SIZE, &len, &status)) {
                ESP_LOGW(TAG, "Tile fetch failed: %s (status %d)", url, status);
                continue;
            }

            int w = 0, h = 0, ch = 0;
            unsigned char *rgb = stbi_load_from_memory(png_buf, len, &w, &h, &ch, 3);
            if (!rgb || w <= 0 || h <= 0) {
                if (rgb) stbi_image_free(rgb);
                ESP_LOGW(TAG, "PNG decode failed for tile %d/%d/%d", zoom, tx, ty);
                continue;
            }

            int dst_x0 = (int)lround((tx_raw - xtile_f) * TILE_SIZE) + RADAR_CENTER_X;
            int dst_y0 = (int)lround((ty - ytile_f) * TILE_SIZE) + RADAR_CENTER_Y;
            darken_and_blit_tile(rgb, w, h, dst_x0, dst_y0);
            stbi_image_free(rgb);
        }
    }

    heap_caps_free(png_buf);

    if (s_canvas) {
        bsp_display_lock(0);
        lv_obj_invalidate(s_canvas);
        bsp_display_unlock();
    }
}

static void map_worker_task(void *arg) {
    (void)arg;
    wifi_manager_wait_connected();

    map_reload_req_t req;
    while (1) {
        if (xQueueReceive(s_req_queue, &req, portMAX_DELAY) == pdTRUE) {
            process_reload(req.lat, req.lon, req.range_km);
        }
    }
}

bool map_tile_service_init(void) {
    s_enabled = wifi_mgr_get_map_enabled();

    s_canvas_buf = heap_caps_malloc((size_t)MAP_SIZE * MAP_SIZE * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_canvas_buf) {
        ESP_LOGE(TAG, "Failed to allocate map canvas buffer in PSRAM!");
        return false;
    }
    memset(s_canvas_buf, 0, (size_t)MAP_SIZE * MAP_SIZE * 2);

    s_req_queue = xQueueCreate(1, sizeof(map_reload_req_t));
    if (!s_req_queue) {
        ESP_LOGE(TAG, "Failed to create map_tile_service queue!");
        return false;
    }

    xTaskCreatePinnedToCore(map_worker_task, "map_worker", 12288, NULL, 2, NULL, 1);
    return true;
}

void map_tile_service_set_canvas_parent(lv_obj_t *radar_area) {
    if (!s_canvas_buf) return;

    s_canvas = lv_canvas_create(radar_area);
    lv_canvas_set_buffer(s_canvas, s_canvas_buf, MAP_SIZE, MAP_SIZE, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(s_canvas, 0, 0);
    lv_obj_clear_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    if (!s_enabled) {
        lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
    }
}

void map_tile_service_request_reload(float center_lat, float center_lon, float range_km) {
    if (!s_req_queue) return;
    map_reload_req_t req = { .lat = center_lat, .lon = center_lon, .range_km = range_km };
    xQueueOverwrite(s_req_queue, &req);
}

void map_tile_service_set_enabled(bool on) {
    s_enabled = on;
    if (s_canvas) {
        if (on) {
            lv_obj_clear_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

bool map_tile_service_is_enabled(void) {
    return s_enabled;
}
