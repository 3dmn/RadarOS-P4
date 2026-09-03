#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"
#include "lvgl.h"

// Declarations only - stb_image implementation is linked from photo_service.c
#include "stb_image.h"

#include "aircraft_types.h"
#include "wifi_manager.h"
#include "map_tile_service.h"
#include "adsb_service.h"
#include "radar_ui.h"
#include "net_lock.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "MAP_TILE_SVC";

#define TILE_SIZE                256
#define MAP_TILE_PNG_BUF_SIZE    (64 * 1024)
#define MAP_TILE_USER_AGENT      "RadarOS-P4/1.0 (+https://github.com/vmisiek/RadarOS-P4)"
#define MAP_MIN_ZOOM              2
#define MAP_MAX_ZOOM             18
#define MAP_BRIGHTNESS_PCT       58
// Coalesces rapid successive map_tile_service_request_reload() calls (e.g.
// quick RNG clicks) into a single tile grid fetch, started this long after
// the last call - see s_debounce_timer.
#define MAP_RELOAD_DEBOUNCE_US   (1500 * 1000)

typedef struct {
    float lat;
    float lon;
    float range_km;
} map_reload_req_t;

static QueueHandle_t s_req_queue;
static esp_timer_handle_t s_debounce_timer = NULL;
static map_reload_req_t s_pending_req;
static lv_obj_t *s_canvas = NULL;
static uint8_t *s_canvas_buf = NULL;
static bool s_enabled = true;
static volatile bool s_paused = false;
// Bumped on every map_tile_service_request_reload() call - lets an
// in-flight process_reload() for an older RNG/zoom request notice it has
// been superseded and abort instead of continuing to fetch a now-stale
// grid of tiles.
static volatile uint32_t s_generation = 0;
// Zoom level of the currently displayed tile grid, updated once a reload
// for a different zoom actually starts drawing - -1 means "none yet" so
// the very first reload always counts as a zoom change. volatile: read
// from other tasks via map_tile_service_get_current_zoom().
static volatile int s_current_zoom = -1;
// True for the duration of an active tile grid fetch - see
// map_tile_is_downloading() and the sequential map-then-ADS-B fetch in
// process_reload() below. Starts true so adsb_service.c's worker task (which
// starts paused too) never races the very first boot-time tile grid for the
// shared HTTPS mutex/SDIO bandwidth before process_reload() has even run
// once.
static volatile bool s_is_downloading = true;

// Standard Slippy Map (Web Mercator) projection. tile_y grows southward -
// a more northern latitude (larger lat_deg) always yields a *smaller*
// tile_y, and dst_y0 in process_reload() below must add (never subtract)
// (ty - center_tile_y) to place tiles in the right N/S order on the grid.
// See self_check_tile_y_orientation() for a runtime check of this part.
// Whether the rows *within* each tile need mirroring on top of that is a
// separate, display-pipeline-specific concern - see the comment in
// darken_and_blit_tile().
static void latlon_to_tilef(double lat, double lon, int zoom, double *xtile, double *ytile) {
    double lat_rad = lat * M_PI / 180.0;
    double n = pow(2.0, zoom);
    *xtile = (lon + 180.0) / 360.0 * n;
    *ytile = (1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * n;
}

// Regression guard for the lat/lon -> tile_y projection direction only
// (not the physical-panel row mirroring done in darken_and_blit_tile() -
// that is a separate, deliberate compensation and is unaffected by this
// check). Run once at startup: Warsaw (~52N) must resolve to a smaller
// tile_y than Madrid (~40N) at the same zoom. If this ever fails, dst_y0's
// sign in process_reload() was flipped and tiles will be arranged in the
// wrong N/S order - logged loudly instead of asserting so a build issue
// here cannot brick the rest of the radar.
static void self_check_tile_y_orientation(void) {
    double warsaw_x, warsaw_y, madrid_x, madrid_y;
    latlon_to_tilef(52.0, 21.0, 10, &warsaw_x, &warsaw_y);
    latlon_to_tilef(40.0, -3.7, 10, &madrid_x, &madrid_y);
    if (warsaw_y >= madrid_y) {
        ESP_LOGE(TAG, "Tile Y-axis orientation check FAILED: Warsaw tile_y=%.3f should be < Madrid tile_y=%.3f - map tiles will render upside down!",
                 warsaw_y, madrid_y);
    } else {
        ESP_LOGI(TAG, "Tile Y-axis orientation check passed (Warsaw tile_y=%.3f < Madrid tile_y=%.3f)", warsaw_y, madrid_y);
    }
}

// Threshold-based range -> zoom mapping (radar_ui.c's range_steps[] in
// aircraft_types.h, kept in sync with RADAR_RANGE_STEPS_KM in
// wifi_manager.c and RANGE_OPTIONS in mqtt_service.c) - a >= ladder instead
// of an exact-value table match, so a range never silently fails to match
// (float rounding noise, a future in-between step, etc.) and falls through
// to the wrong zoom. Locks the tile background to a whole OSM zoom level
// per range step - no latitude-dependent interpolation - so the map is
// always crisp at every RNG setting instead of a blurry in-between zoom.
static int compute_zoom_for_range(float range_km) {
    int zoom;
    if (range_km >= 350.0f) zoom = 6;       // 400 km
    else if (range_km >= 150.0f) zoom = 7;  // 200 km
    else if (range_km >= 75.0f) zoom = 8;   // 100 km
    else if (range_km >= 35.0f) zoom = 9;   // 50 km
    else zoom = 10;                         // 25 km

    if (zoom < MAP_MIN_ZOOM) zoom = MAP_MIN_ZOOM;
    if (zoom > MAP_MAX_ZOOM) zoom = MAP_MAX_ZOOM;
    return zoom;
}

// Written only from within http_get_tile() below, itself only ever called
// sequentially from map_worker_task - no concurrent access, so a plain
// module-static (instead of passing state through esp_http_client's
// event_handler user_data, which is fixed at esp_http_client_init() time and
// cannot be changed per-request) is safe.
static uint8_t *s_tile_dl_buf = NULL;
static int s_tile_dl_buf_size = 0;
static int s_tile_dl_len = 0;

static esp_err_t tile_http_event_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        // OTA update started mid-transfer - abort this tile's read
        // immediately instead of finishing it (esp_http_client_perform()
        // returns an error, which http_get_tile() below reports as a
        // failed fetch).
        if (s_paused) return ESP_FAIL;
        if (s_tile_dl_buf && !esp_http_client_is_chunked_response(evt->client)) {
            int copy_len = evt->data_len;
            if (s_tile_dl_len + copy_len > s_tile_dl_buf_size) {
                copy_len = s_tile_dl_buf_size - s_tile_dl_len;
            }
            if (copy_len > 0) {
                memcpy(s_tile_dl_buf + s_tile_dl_len, evt->data, copy_len);
                s_tile_dl_len += copy_len;
            }
        }
    }
    return ESP_OK;
}

// Fetches one tile over the given client - reused across the whole grid
// (see process_reload()) via esp_http_client_set_url()/perform() so a
// keep-alive connection to tile.openstreetmap.org survives between tiles
// instead of renegotiating a fresh TLS session for every single PNG.
static bool http_get_tile(esp_http_client_handle_t client, const char *url, uint8_t *buf, int buf_size,
                           int *out_len, int *out_status) {
    if (s_paused) return false;

    if (!net_http_lock(portMAX_DELAY)) {
        ESP_LOGE(TAG, "Failed to acquire the global HTTP/HTTPS network lock");
        return false;
    }
    if (s_paused) {
        net_http_unlock();
        return false;
    }

    s_tile_dl_buf = buf;
    s_tile_dl_buf_size = buf_size;
    s_tile_dl_len = 0;

    esp_http_client_set_url(client, url);
    esp_err_t err = esp_http_client_perform(client);

    bool ok = false;
    if (err == ESP_OK) {
        *out_status = esp_http_client_get_status_code(client);
        *out_len = s_tile_dl_len;
        ok = (*out_status == 200 && s_tile_dl_len > 0);
        if (!ok) {
            ESP_LOGW(TAG, "HTTP %d for tile fetch (%s)", *out_status, url);
        }
    } else {
        ESP_LOGW(TAG, "HTTP perform failed: %s (%s)", esp_err_to_name(err), url);
        *out_status = 0;
        *out_len = 0;
    }

    s_tile_dl_buf = NULL;
    net_http_unlock();
    return ok;
}

static void darken_and_blit_tile(const uint8_t *rgb, int tile_w, int tile_h, int dst_x0, int dst_y0) {
    if (!s_canvas_buf || !rgb) return;
    uint16_t *dst = (uint16_t *)s_canvas_buf;

    for (int y = 0; y < tile_h; y++) {
        int src_y = y; // HARDWARE VERIFIED: direct top-to-bottom. NEVER change to tile_h - 1 - y!
        int canvas_y = dst_y0 + y;
        if (canvas_y < 0 || canvas_y >= MAP_SIZE) continue;

        const uint8_t *src_row = rgb + (size_t)src_y * tile_w * 3;
        uint16_t *dst_row = dst + (size_t)canvas_y * MAP_SIZE;

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

static void process_reload(float lat, float lon, float range_km, uint32_t my_generation) {
    int zoom = compute_zoom_for_range(range_km);
    int n = 1 << zoom;

    double xtile_f, ytile_f;
    latlon_to_tilef(lat, lon, zoom, &xtile_f, &ytile_f);

    // Logged on every reload (RNG/zoom change included) so the chosen zoom
    // level can be verified against the requested range on real hardware,
    // instead of guessing from on-screen appearance alone.
    ESP_LOGI(TAG, "Tile reload: range=%.0f km -> zoom=%d, center tile=(%.3f, %.3f)",
             (double)range_km, zoom, xtile_f, ytile_f);

    int tx_min = (int)floor(xtile_f - (double)RADAR_CENTER_X / TILE_SIZE) - 1;
    int tx_max = (int)floor(xtile_f + (double)(MAP_SIZE - RADAR_CENTER_X) / TILE_SIZE) + 1;
    int ty_min = (int)floor(ytile_f - (double)RADAR_CENTER_Y / TILE_SIZE) - 1;
    int ty_max = (int)floor(ytile_f + (double)(MAP_SIZE - RADAR_CENTER_Y) / TILE_SIZE) + 1;

    memset(s_canvas_buf, 0, (size_t)MAP_SIZE * MAP_SIZE * 2);

    if (zoom != s_current_zoom) {
        // Push the now-blank buffer to the screen immediately, instead of
        // waiting for the whole (multi-second, 250 ms/tile-paced) grid to
        // finish - otherwise the previous zoom's tiles stay visible on
        // screen for that whole time, which reads as "the zoom did not
        // change" even though it already did internally.
        ESP_LOGI(TAG, "Zoom changed %d -> %d, clearing tile canvas immediately", s_current_zoom, zoom);
        s_current_zoom = zoom;
        if (s_canvas) {
            bsp_display_lock(0);
            lv_obj_invalidate(s_canvas);
            bsp_display_unlock();
        }
    }

    int total_tiles = (tx_max - tx_min + 1) * (ty_max - ty_min + 1);
    int tile_count = 0;

    // Sequential map-tiles-then-ADS-B fetch: pause adsb_service.c's polling
    // for the whole grid download (it shares the global net_http_lock, so
    // interleaving the two just adds pointless TLS handshake contention) and
    // show a progress bubble instead of two silent fetches racing each
    // other. See the completion handling below for how adsb_service.c is
    // handed back control.
    s_is_downloading = true;
    adsb_service_pause();
    radar_ui_show_map_loading(0, total_tiles);

    // One esp_http_client handle for the whole grid, kept alive across every
    // tile (keep_alive_enable + esp_http_client_set_url()/perform() below) -
    // opening/closing a brand new TLS session per tile flooded the SDIO
    // driver with handshake traffic and exhausted its DMA mempool
    // ("mempool_alloc" assert in esp_hosted). Cleaned up once after the loop
    // below, on every exit path (normal completion, superseded, or paused).
    esp_http_client_config_t tile_client_config = {
        .url = "https://tile.openstreetmap.org/0/0/0.png", // placeholder, overwritten before first use
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 8000,
        // Deliberately small: a bigger internal read/TCP-window buffer lets
        // the server push a whole flood of packets before esp_http_client
        // ever drains them, which is what overran the SDIO driver's DMA
        // mempool ("sdio_rx_get_buffer"/transport_drv_sta_tx assert). A
        // smaller buffer_size caps the effective TCP receive window, forcing
        // the server to send in smaller, ACK-paced bursts instead.
        .buffer_size = 2048,
        .buffer_size_tx = 1024,
        .max_redirection_count = 5,
        .keep_alive_enable = true,
        .user_agent = MAP_TILE_USER_AGENT,
        .event_handler = tile_http_event_handler,
    };
    esp_http_client_handle_t tile_client = esp_http_client_init(&tile_client_config);
    if (!tile_client) {
        ESP_LOGE(TAG, "Failed to initialize the map tile HTTP client - aborting grid reload");
        s_is_downloading = false;
        radar_ui_hide_loading();
        adsb_service_resume();
        adsb_service_request_immediate_fetch();
        return;
    }
    esp_http_client_set_header(tile_client, "User-Agent", MAP_TILE_USER_AGENT);

    for (int ty = ty_min; ty <= ty_max; ty++) {
        if (s_paused) {
            ESP_LOGW(TAG, "Tile reload aborted: OTA update in progress");
            break;
        }
        if (my_generation != s_generation) {
            ESP_LOGI(TAG, "Tile reload cancelled: superseded by a newer RNG/zoom request");
            break;
        }
        if (ty < 0 || ty >= n) continue;

        for (int tx_raw = tx_min; tx_raw <= tx_max; tx_raw++) {
            if (s_paused) break;
            if (my_generation != s_generation) break;
            tile_count++;
            radar_ui_update_map_loading(tile_count, total_tiles);
            int tx = ((tx_raw % n) + n) % n;
            char url[96];
            snprintf(url, sizeof(url), "https://tile.openstreetmap.org/%d/%d/%d.png", zoom, tx, ty);
            ESP_LOGI(TAG, "Downloading tile z=%d x=%d y=%d from %s", zoom, tx, ty, url);

            uint8_t *png_buf = heap_caps_malloc(MAP_TILE_PNG_BUF_SIZE, MALLOC_CAP_SPIRAM);
            if (!png_buf) {
                ESP_LOGE(TAG, "Failed to allocate tile PNG buffer in PSRAM!");
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            int len = 0, status = 0;
            bool got_tile = http_get_tile(tile_client, url, png_buf, MAP_TILE_PNG_BUF_SIZE, &len, &status);
            ESP_LOGI(TAG, "Tile z=%d x=%d y=%d -> HTTP %d (%s)", zoom, tx, ty, status, got_tile ? "OK" : "FAILED");

            if (!got_tile) {
                ESP_LOGW(TAG, "Tile fetch failed: %s (status %d)", url, status);
                heap_caps_free(png_buf);
                // Paces even a failed fetch, on the same reused keep-alive
                // connection - gives the SDIO driver and lwIP time to ACK
                // and release DMA mempool buffers before the next request.
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            if (my_generation != s_generation) {
                // A newer RNG/zoom request arrived while this tile was in
                // flight - discard it and stop fetching the now-stale grid.
                heap_caps_free(png_buf);
                ESP_LOGI(TAG, "Tile reload cancelled: superseded by a newer RNG/zoom request");
                break;
            }

            // Lets the SDIO driver task and lwIP's tcpip_thread finish
            // processing already-received packets and return their buffers
            // to the DMA mempool before this task pins the CPU decoding the
            // PNG below - otherwise the buffers accumulate faster than the
            // pipeline can drain them, still exhausting the same mempool.
            vTaskDelay(pdMS_TO_TICKS(80));

            // CRITICAL: Always force normal top-down orientation before decoding
            stbi_set_flip_vertically_on_load(0);

            int w = 0, h = 0, ch = 0;
            unsigned char *rgb = stbi_load_from_memory(png_buf, len, &w, &h, &ch, 3);
            heap_caps_free(png_buf);

            if (!rgb || w <= 0 || h <= 0) {
                if (rgb) stbi_image_free(rgb);
                ESP_LOGW(TAG, "PNG decode failed for tile %d/%d/%d", zoom, tx, ty);
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            // Plus sign only, never minus: tile_y and screen Y both grow
            // southward, so a tile north of center (smaller ty than
            // ytile_f) must get a negative offset here and land above
            // RADAR_CENTER_Y - see self_check_tile_y_orientation() above.
            // tx_raw (not the longitude-wrapped tx) keeps tiles correctly
            // placed across the antimeridian.
            int dst_x0 = RADAR_CENTER_X + (int)roundf((float)(tx_raw - xtile_f) * TILE_SIZE);
            int dst_y0 = RADAR_CENTER_Y + (int)roundf((float)(ty - ytile_f) * TILE_SIZE);
            darken_and_blit_tile(rgb, w, h, dst_x0, dst_y0);
            stbi_image_free(rgb);

            // Pacing after fetch+render, on the same reused keep-alive
            // connection - gives the SDIO driver and lwIP stack time to ACK
            // and release DMA mempool buffers before the next tile's request.
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }

    // The keep-alive connection is only ever used sequentially within this
    // function - safe to tear down here regardless of which exit path above
    // was taken (normal completion, superseded generation, or paused).
    esp_http_client_cleanup(tile_client);

    s_is_downloading = false;

    if (my_generation != s_generation) {
        // Superseded mid-reload - the canvas holds a partial/stale grid;
        // skip the redraw, and leave adsb_service.c paused and the loading
        // bubble as-is, since the newer generation's process_reload() call
        // already re-armed both (it called adsb_service_pause() and
        // radar_ui_show_map_loading() again at its own start) and will run
        // this same completion sequence itself once it finishes. This
        // generation's keep-alive HTTP client was already torn down above
        // (esp_http_client_cleanup(tile_client)), so there is no lingering
        // open connection to worry about - just let any in-flight TCP
        // packets drain off the SDIO bus before the newer generation's grid
        // starts fetching.
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }

    if (s_paused) {
        // Aborted by ota_update_service.c, not superseded - it manages
        // adsb_service.c's pause/resume itself while flashing, so leave
        // that alone here. Just clear the now-stuck "downloading map"
        // bubble instead of leaving it on screen for the OTA's duration.
        radar_ui_hide_loading();
        if (s_canvas) {
            bsp_display_lock(0);
            lv_obj_invalidate(s_canvas);
            bsp_display_unlock();
        }
        return;
    }

    // Tile grid finished normally - hand off to adsb_service.c for the
    // first (immediate) ADS-B fetch. It hides the bubble itself once that
    // fetch succeeds and radar_ui_refresh() has run - see adsb_worker_task().
    radar_ui_show_adsb_loading();
    adsb_service_resume();
    adsb_service_request_immediate_fetch();

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
        if (s_paused) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (xQueueReceive(s_req_queue, &req, pdMS_TO_TICKS(500)) == pdTRUE) {
            process_reload(req.lat, req.lon, req.range_km, s_generation);
        }
    }
}

// Fires MAP_RELOAD_DEBOUNCE_US after the last map_tile_service_request_
// reload() call with no further call in between - this is what actually
// enqueues the fetch for map_worker_task, so a burst of rapid RNG/zoom
// changes only ever triggers one tile grid download.
static void debounce_timer_cb(void *arg) {
    (void)arg;
    if (!s_req_queue) return;
    // See s_generation - lets an in-flight process_reload() for a
    // previous request (e.g. before the user changed RNG/zoom) notice it
    // is now stale and abort instead of finishing a wasted tile grid.
    s_generation++;
    xQueueOverwrite(s_req_queue, &s_pending_req);
}

bool map_tile_service_init(void) {
    self_check_tile_y_orientation();

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

    const esp_timer_create_args_t debounce_timer_args = {
        .callback = debounce_timer_cb,
        .name = "map_reload_debounce",
    };
    if (esp_timer_create(&debounce_timer_args, &s_debounce_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create map reload debounce timer!");
        return false;
    }

    // 12 KB stack - with CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y (see
    // sdkconfig.defaults), FreeRTOS places this in external PSRAM instead of
    // the internal DMA-capable SRAM the Wi-Fi SDIO driver needs, so this
    // task's stack never competes with it.
    // Pinned to core 0 (System & Network) - PNG fetch and decode both run
    // here, on the same core as esp_hosted's SDIO threads; core 1 is
    // reserved for LVGL/UI (see main.c).
    xTaskCreatePinnedToCore(map_worker_task, "map_worker", 12288, NULL, 2, NULL, 0);
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
    if (!s_req_queue || !s_debounce_timer) return;
    s_pending_req.lat = center_lat;
    s_pending_req.lon = center_lon;
    s_pending_req.range_km = range_km;
    esp_timer_stop(s_debounce_timer); // ESP_ERR_INVALID_STATE if not running - fine, ignored
    esp_timer_start_once(s_debounce_timer, MAP_RELOAD_DEBOUNCE_US);
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

int map_tile_service_get_current_zoom(void) {
    return s_current_zoom;
}

bool map_tile_is_downloading(void) {
    return s_is_downloading;
}

void map_tile_service_pause(void) {
    s_paused = true;
    ESP_LOGI(TAG, "Tile fetching paused (OTA update in progress)");
}

void map_tile_service_resume(void) {
    s_paused = false;
    ESP_LOGI(TAG, "Tile fetching resumed");
}