#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "cJSON.h"

#include "adsb_service.h"
#include "wifi_manager.h"
#include "radar_ui.h"
#include "map_tile_service.h"
#include "net_lock.h"

static const char *TAG = "ADSB_SVC";

static SemaphoreHandle_t g_data_mutex = NULL;

static AircraftTrackHistory *track_db = NULL;
static int track_db_count = 0;

AircraftData *live_fleet = NULL;
static AircraftData *temp_fleet = NULL;
int total_aircraft_in_zone = 0;

TracePoint *g_trace_points = NULL;
int g_trace_point_count = 0;
char g_trace_hex[8] = "";

// Guarded by g_data_mutex (adsb_service_lock/unlock), same as g_trace_*
// above. s_trace_generation lets the fetch task tell a superseded request
// apart from the one it is currently servicing once the HTTP round-trip
// completes.
static char s_trace_pending_hex[8] = "";
static uint32_t s_trace_generation = 0;
static TaskHandle_t s_trace_task_handle = NULL;

// How many polling cycles (each API_FETCH_SEC seconds) between memory telemetry logs.
#define MEM_TELEMETRY_EVERY_N_CYCLES  15

// Spoof a real desktop browser UA - some dump1090-derived APIs sit behind a
// WAF that returns HTTP 403 for a custom/non-browser User-Agent.
#define ADSB_USER_AGENT "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/128.0.0.0 Safari/537.36"

uint32_t get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Converts the current HUD range to a query radius in nautical miles (both
// APIs take "dist" in NM) - querying only as far as the radar actually
// displays keeps the JSON response (and the PSRAM parse buffer) small even
// over dense metro areas, instead of always requesting a fixed 140 NM.
// Clamped to a sane minimum/maximum regardless of the configured range - the
// upper bound covers the widest radar_ui.c range step (400 km = ~216 NM)
// with headroom, so widening the range ladder never silently under-queries
// the API relative to what the radar visually displays.
static uint32_t compute_dist_nm(float range_km) {
    uint32_t nm = (uint32_t)ceilf(range_km * 0.539957f);
    if (nm < 5) nm = 5;
    if (nm > 230) nm = 230;
    return nm;
}

void calculate_coords(float lat, float lon, float *out_dist_km, float *out_bearing_deg) {
    float dlat = (lat - g_radar_lat) * 111.132f;
    float dlon = (lon - g_radar_lon) * 111.132f * cosf(g_radar_lat * DEG_TO_RAD);
    *out_dist_km = sqrtf(dlat * dlat + dlon * dlon);
    float angle = atan2f(dlon, dlat) * RAD_TO_DEG;
    if(angle < 0.0f) angle += 360.0f;
    *out_bearing_deg = angle;
}

AircraftTrackHistory* find_aircraft_track(const char *hex) {
    if (!hex || hex[0] == '\0' || !track_db) return NULL;
    for (int i = 0; i < track_db_count; i++) {
        if (strcmp(track_db[i].hex, hex) == 0) {
            return &track_db[i];
        }
    }
    return NULL;
}

static AircraftTrackHistory* record_aircraft_track(const char *hex, float lat, float lon, int alt, uint32_t now) {
    if (!track_db) return NULL;
    AircraftTrackHistory *th = NULL;
    for (int i = 0; i < track_db_count; i++) {
        if ((now - track_db[i].last_seen_ms) > PLANE_TIMEOUT_MS) {
            track_db[i] = track_db[track_db_count - 1];
            track_db_count--;
            i--;
        }
    }
    for (int i = 0; i < track_db_count; i++) {
        if (strcmp(track_db[i].hex, hex) == 0) {
            th = &track_db[i];
            break;
        }
    }
    if (!th && track_db_count < MAX_SEEN_DB) {
        th = &track_db[track_db_count++];
        memset(th, 0, sizeof(AircraftTrackHistory));
        snprintf(th->hex, sizeof(th->hex), "%s", hex);
        th->first_seen_ms = now;
        th->pt_count = 0;
    }
    if (th) {
        th->last_seen_ms = now;
        uint32_t max_age_ms = 300 * 1000;
        int valid_idx = 0;
        for (int p = 0; p < th->pt_count; p++) {
            if ((now - th->times[p]) <= max_age_ms) {
                valid_idx = p;
                break;
            }
        }
        if (valid_idx > 0) {
            int remaining = th->pt_count - valid_idx;
            for (int p = 0; p < remaining; p++) {
                th->lats[p] = th->lats[valid_idx + p];
                th->lons[p] = th->lons[valid_idx + p];
                th->alts[p] = th->alts[valid_idx + p];
                th->times[p] = th->times[valid_idx + p];
            }
            th->pt_count = remaining;
        }
        if (th->pt_count == 0) {
            th->lats[0] = lat;
            th->lons[0] = lon;
            th->alts[0] = alt;
            th->times[0] = now;
            th->pt_count = 1;
        } else {
            float dlat = lat - th->lats[th->pt_count - 1];
            float dlon = lon - th->lons[th->pt_count - 1];
            if ((dlat * dlat + dlon * dlon) > 0.00000010f) {
                if (th->pt_count < MAX_TRACK_HISTORY_PTS) {
                    th->lats[th->pt_count] = lat;
                    th->lons[th->pt_count] = lon;
                    th->alts[th->pt_count] = alt;
                    th->times[th->pt_count] = now;
                    th->pt_count++;
                } else {
                    for (int k = 0; k < MAX_TRACK_HISTORY_PTS - 1; k++) {
                        th->lats[k] = th->lats[k + 1];
                        th->lons[k] = th->lons[k + 1];
                        th->alts[k] = th->alts[k + 1];
                        th->times[k] = th->times[k + 1];
                    }
                    th->lats[MAX_TRACK_HISTORY_PTS - 1] = lat;
                    th->lons[MAX_TRACK_HISTORY_PTS - 1] = lon;
                    th->alts[MAX_TRACK_HISTORY_PTS - 1] = alt;
                    th->times[MAX_TRACK_HISTORY_PTS - 1] = now;
                }
            }
        }
    }
    return th;
}

// ================= ADS-B JSON PARSER =================
// Search bounded to [block, end) - never crosses into the next aircraft
// JSON object (e.g. the next "{\"hex\":..." entry in the "ac" array).
static const char *bounded_strstr(const char *block, const char *end, const char *needle) {
    if (!block || !end || !needle) return NULL;
    size_t needle_len = strlen(needle);
    if (needle_len == 0 || end < block) return NULL;
    size_t hay_len = (size_t)(end - block);
    if (needle_len > hay_len) return NULL;
    for (size_t i = 0; i + needle_len <= hay_len; i++) {
        if (memcmp(block + i, needle, needle_len) == 0) return block + i;
    }
    return NULL;
}

static const char *json_get_field(const char *block, const char *end, const char *key) {
    if (!block || !end || !key) return NULL;
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = bounded_strstr(block, end, search);
    if (!p) return NULL;
    return p + strlen(search);
}

static bool json_get_str(const char *block, const char *end, const char *key, char *out_str, int max_len) {
    if (out_str && max_len > 0) out_str[0] = '\0';
    if (!block || !end || !key || !out_str || max_len <= 0) return false;
    const char *p = json_get_field(block, end, key);
    if (!p) return false;
    while (p < end && (*p == ' ' || *p == '\t')) p++;
    if (p < end && *p == '"') p++;
    int i = 0;
    while (p < end && *p != '\0' && *p != '"' && i < max_len - 1) {
        out_str[i++] = *p++;
    }
    out_str[i] = '\0';
    return true;
}

static bool json_get_float(const char *block, const char *end, const char *key, float *out_val) {
    if (out_val) *out_val = 0.0f;
    if (!block || !end || !key || !out_val) return false;
    const char *p = json_get_field(block, end, key);
    if (!p) return false;
    while (p < end && (*p == ' ' || *p == '\t' || *p == '"')) p++;
    if (p >= end) return false;
    *out_val = (float)atof(p);
    return true;
}

static bool json_get_int(const char *block, const char *end, const char *key, int *out_val) {
    if (out_val) *out_val = 0;
    if (!block || !end || !key || !out_val) return false;
    const char *p = json_get_field(block, end, key);
    if (!p) return false;
    while (p < end && (*p == ' ' || *p == '\t' || *p == '"')) p++;
    if (p >= end) return false;
    *out_val = atoi(p);
    return true;
}

static const char *military_callsign_prefixes[] = {
    "PLF", "RCH", "NATO", "VIP", "GAF", "IAM", "BAF", "ASY",
    "FORTE", "HOMER", "LAGR", "DUKE", "NAF", "HAF",
    "RRR", "BOXER", "REDEYE", "NCHO", "JAKE", "MMF", "FAF", "CFC", "HKY"
};
#define NUM_MIL_PREFIXES (sizeof(military_callsign_prefixes) / sizeof(military_callsign_prefixes[0]))

static bool detect_military(const char *block, const char *end, const char *callsign) {
    if (block && end) {
        int db_flags = 0;
        if (json_get_int(block, end, "dbFlags", &db_flags) && (db_flags & 1)) return true;
    }

    if (callsign == NULL || callsign[0] == '\0') return false;

    for (size_t i = 0; i < NUM_MIL_PREFIXES; i++) {
        size_t len = strlen(military_callsign_prefixes[i]);
        if (strncmp(callsign, military_callsign_prefixes[i], len) == 0) return true;
    }
    return false;
}

// Strips 6-bit ADS-B padding ('@' = code 0x00) and other non-printable
// characters from the callsign, then trims leading/trailing spaces. If
// nothing is left after cleanup (pure padding/spaces/empty string), falls
// back to the readable ICAO hex address (uppercased) instead of an empty or
// garbled identifier.
static void sanitize_callsign(char *cs, size_t cs_size, const char *hex) {
    for (size_t i = 0; cs[i] != '\0'; i++) {
        unsigned char c = (unsigned char)cs[i];
        if (c == '@' || c < 0x20 || c > 0x7E) cs[i] = ' ';
    }

    size_t len = strlen(cs);
    size_t start = 0;
    while (start < len && cs[start] == ' ') start++;
    size_t end = len;
    while (end > start && cs[end - 1] == ' ') end--;
    size_t out_len = end - start;

    if (out_len > 0) {
        memmove(cs, cs + start, out_len);
    }
    cs[out_len] = '\0';

    if (cs[0] == '\0' && hex) {
        size_t i = 0;
        for (; i < cs_size - 1 && hex[i] != '\0'; i++) {
            cs[i] = (char)toupper((unsigned char)hex[i]);
        }
        cs[i] = '\0';
    }
}

// Sort context for the qsort comparators below - safe because both calls in
// apply_priority_limit() happen back to back, synchronously, on the single
// adsb_worker task; nothing else touches this pointer concurrently.
static const AircraftData *s_priority_sort_planes;

static int priority_dist_cmp(const void *a, const void *b) {
    float da = s_priority_sort_planes[*(const int *)a].distance_km;
    float db = s_priority_sort_planes[*(const int *)b].distance_km;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

static int int_asc_cmp(const void *a, const void *b) {
    return *(const int *)a - *(const int *)b;
}

// Enforces wifi_mgr_get_max_aircraft() without dropping military/NATO
// contacts ahead of civilians when priority is enabled (web/MQTT toggle,
// wifi_mgr_get_mil_priority_enabled()): the farthest civilian is dropped
// first, and a military aircraft is only ever dropped once the number of
// military aircraft alone exceeds max_planes. When priority is disabled,
// every aircraft (military or not) competes purely on distance. Compacts
// the kept aircraft to out_planes[0..return value) in place - only
// fixed-size MAX_AIRCRAFT_CAPACITY index arrays on the stack (~2.4 KB
// total), no heap allocation, so this is safe to call every fetch cycle.
static int apply_priority_limit(AircraftData *planes, int count, int max_planes) {
    if (count <= max_planes) return count;

    bool priority = wifi_mgr_get_mil_priority_enabled();
    int mil_idx[MAX_AIRCRAFT_CAPACITY];
    int civ_idx[MAX_AIRCRAFT_CAPACITY];
    int mil_count = 0, civ_count = 0;
    for (int i = 0; i < count; i++) {
        if (priority && planes[i].is_military) mil_idx[mil_count++] = i;
        else civ_idx[civ_count++] = i;
    }

    s_priority_sort_planes = planes;
    qsort(mil_idx, mil_count, sizeof(int), priority_dist_cmp);
    qsort(civ_idx, civ_count, sizeof(int), priority_dist_cmp);

    int keep_mil = mil_count < max_planes ? mil_count : max_planes;
    int keep_civ = max_planes - keep_mil;
    if (keep_civ > civ_count) keep_civ = civ_count;

    int keep_idx[MAX_AIRCRAFT_CAPACITY];
    int keep_count = 0;
    for (int i = 0; i < keep_mil; i++) keep_idx[keep_count++] = mil_idx[i];
    for (int i = 0; i < keep_civ; i++) keep_idx[keep_count++] = civ_idx[i];
    qsort(keep_idx, keep_count, sizeof(int), int_asc_cmp);

    // In-place stable compaction: keep_idx is ascending and distinct, so
    // keep_idx[i] >= i always - each write target has either already been
    // consumed or never held a kept element, never one still needed later.
    int w = 0;
    for (int i = 0; i < keep_count; i++) {
        int r = keep_idx[i];
        if (r != w) planes[w] = planes[r];
        w++;
    }

    ESP_LOGI(TAG, "Aircraft limit %d reached (%d in zone) - kept %d military + %d civilian (priority %s)",
             max_planes, count, keep_mil, keep_civ, priority ? "ON" : "OFF");
    return keep_count;
}

static int parse_adsb_json(const char *json, AircraftData *out_planes, int max_planes) {
    if (!json || !out_planes) return 0;
    int count = 0;
    const char *p = strstr(json, "\"ac\":");
    if (!p) p = strstr(json, "\"aircraft\":");
    if (!p) p = json;

    uint32_t now = get_time_ms();

    // Parses every aircraft the response offers, up to the hard PSRAM buffer
    // ceiling - NOT max_planes - so apply_priority_limit() below can see the
    // full picture (every distance/is_military value) before deciding which
    // max_planes to keep. Stopping the scan early at max_planes, like this
    // used to, would silently favor whatever order the API happens to
    // return and could never protect military contacts.
    while ((p = strstr(p, "{\"hex\":")) != NULL && count < MAX_AIRCRAFT_CAPACITY) {
        const char *end = strchr(p, '}');
        if (!end) break;

        AircraftData *plane = &out_planes[count];
        memset(plane, 0, sizeof(AircraftData));

        json_get_str(p, end, "hex", plane->hex, sizeof(plane->hex));
        json_get_str(p, end, "flight", plane->callsign, sizeof(plane->callsign));
        sanitize_callsign(plane->callsign, sizeof(plane->callsign), plane->hex);

        json_get_str(p, end, "r", plane->registration, sizeof(plane->registration));
        json_get_str(p, end, "t", plane->model, sizeof(plane->model));
        json_get_str(p, end, "squawk", plane->squawk, sizeof(plane->squawk));

        char type_str[16] = {0};
        if (json_get_str(p, end, "t", type_str, sizeof(type_str))) {
            if (is_helicopter(type_str)) {
                plane->type = AC_HELI;
            } else if (strstr(type_str, "B77") || strstr(type_str, "A35") || strstr(type_str, "B74")) {
                plane->type = AC_LARGE;
            } else if (strstr(type_str, "C172") || strstr(type_str, "P28")) {
                plane->type = AC_SMALL;
            } else {
                plane->type = AC_NORMAL;
            }
        } else {
            plane->type = AC_NORMAL;
        }

        plane->is_military = detect_military(p, end, plane->callsign);

        // dump1090-derived APIs (adsb.fi, adsb.lol) flag an aircraft on
        // the ground with the text value "ground" in alt_baro (instead of a number).
        plane->on_ground = bounded_strstr(p, end, "\"alt_baro\":\"ground\"") != NULL;

        json_get_float(p, end, "lat", &plane->lat);
        json_get_float(p, end, "lon", &plane->lon);
        json_get_int(p, end, "alt_baro", &plane->altitude_ft);
        json_get_int(p, end, "track", &plane->heading_deg);
        json_get_int(p, end, "gs", &plane->speed_kt);

        int vsi_val = 0;
        if (json_get_int(p, end, "baro_rate", &vsi_val)) {
            plane->vsi_fpm = vsi_val;
            if (vsi_val > 64) snprintf(plane->vsi_str, sizeof(plane->vsi_str), "+%d", vsi_val);
            else if (vsi_val < -64) snprintf(plane->vsi_str, sizeof(plane->vsi_str), "%d", vsi_val);
            else strcpy(plane->vsi_str, "LEVEL");
        } else {
            plane->vsi_fpm = 0;
            strcpy(plane->vsi_str, "LEVEL");
        }

        if (strlen(plane->hex) > 0 && plane->lat != 0.0f && plane->lon != 0.0f) {
            calculate_coords(plane->lat, plane->lon, &plane->distance_km, &plane->bearing_deg);
            plane->last_seen_ms = now;
            plane->active = true;
            record_aircraft_track(plane->hex, plane->lat, plane->lon, plane->altitude_ft, now);
            count++;
        }
        p = end + 1;
    }

    if (count == MAX_AIRCRAFT_CAPACITY && p != NULL && strstr(p, "{\"hex\":") != NULL) {
        ESP_LOGW(TAG, "Hard aircraft buffer limit (%d) reached - excess aircraft in API response discarded", MAX_AIRCRAFT_CAPACITY);
    }

    return apply_priority_limit(out_planes, count, max_planes);
}

// ================= MEMORY TELEMETRY =================
static void log_memory_telemetry(void) {
    ESP_LOGI(TAG, "Heap: free=%" PRIu32 "B min=%" PRIu32 "B | PSRAM free=%u B | adsb_worker stack: %u B free",
             (uint32_t)esp_get_free_heap_size(),
             (uint32_t)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
}

// ================= ADS-B NETWORK TASK =================
bool adsb_service_lock(uint32_t timeout_ms) {
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout (%" PRIu32 "ms) waiting for g_data_mutex", timeout_ms);
        return false;
    }
    return true;
}

void adsb_service_unlock(void) {
    xSemaphoreGive(g_data_mutex);
}

void adsb_service_request_trace(const char *hex) {
    if (!hex || hex[0] == '\0') return;
    if (adsb_service_lock(100)) {
        snprintf(s_trace_pending_hex, sizeof(s_trace_pending_hex), "%s", hex);
        for (char *p = s_trace_pending_hex; *p != '\0'; p++) *p = (char)tolower((unsigned char)*p);
        s_trace_generation++;
        adsb_service_unlock();
    }
    if (s_trace_task_handle) {
        xTaskNotifyGive(s_trace_task_handle);
    }
}

void adsb_service_clear_trace(void) {
    if (adsb_service_lock(100)) {
        s_trace_pending_hex[0] = '\0';
        s_trace_generation++;
        g_trace_point_count = 0;
        g_trace_hex[0] = '\0';
        adsb_service_unlock();
    }
}

// Starts paused - the very first request must never race map_tile_service.c
// for the shared HTTPS mutex/SDIO bandwidth at boot. adsb_service_resume()
// (called by map_tile_service.c once the initial tile grid finishes) is
// what unblocks the first fetch.
static volatile bool s_adsb_paused = true;
static TaskHandle_t s_adsb_task_handle = NULL;
// Set by adsb_service_request_immediate_fetch() - the next successful fetch
// hides the map/ADS-B loading bubble (radar_ui_hide_loading()) and clears
// this, so a normal periodic fetch never touches the bubble.
static volatile bool s_first_fetch_pending = false;

void adsb_service_pause(void) {
    s_adsb_paused = true;
    // Currently only called by map_tile_service.c's sequential map-then-
    // ADS-B fetch (see process_reload()) - update this text if a second
    // caller with a different reason (e.g. an OTA update) is wired up.
    ESP_LOGI(TAG, "ADS-B polling paused (map tile download in progress)");
}

void adsb_service_resume(void) {
    s_adsb_paused = false;
    ESP_LOGI(TAG, "ADS-B polling resumed");
}

void adsb_service_request_immediate_fetch(void) {
    s_first_fetch_pending = true;
    if (s_adsb_task_handle) {
        xTaskNotifyGive(s_adsb_task_handle);
    }
}

#define TRACE_HTTP_BUFFER_SIZE (128 * 1024)

// Fetches one URL into resp_buf under the global net_http_lock (serialized
// against every other esp_http_client session in the app - see net_lock.h).
// referer must be the scheme+host of url (e.g. "https://globe.adsb.fi/") -
// confirmed by curl testing (see the comment on trace_fetch_task below) that
// the Cloudflare-fronted globe.* trace endpoints return 403 Forbidden unless
// the Referer header's host matches the one being requested, on top of the
// existing browser-like ADSB_USER_AGENT. Returns true on HTTP 200 with a
// non-empty body.
static bool trace_fetch_url(esp_http_client_handle_t client, const char *url, const char *referer,
                             char *resp_buf, int buf_size) {
    esp_http_client_set_url(client, url);
    esp_http_client_set_header(client, "Referer", referer);

    // Defer entirely to an in-progress map tile grid download instead of
    // just waiting on net_http_lock() below - an on-demand trace fetch
    // landing between two tile fetches (each already paced to let the SDIO
    // driver recover) was enough to interleave an extra TLS session and
    // exhaust its DMA buffer pool ("sdio_rx_get_buffer" assert). A trace
    // fetch is user-triggered but not latency-critical, so blocking here
    // until the grid finishes is preferable to racing it.
    while (map_tile_is_downloading()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!net_http_lock(portMAX_DELAY)) {
        ESP_LOGE(TAG, "Failed to acquire the global HTTP/HTTPS network lock");
        return false;
    }
    esp_err_t err = esp_http_client_open(client, 0);
    int status = 0;
    int total_read = 0;

    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
        if (status == 200) {
            int read_len;
            while ((read_len = esp_http_client_read(client, resp_buf + total_read, buf_size - total_read - 1)) > 0) {
                total_read += read_len;
            }
            resp_buf[total_read] = '\0';
        } else {
            ESP_LOGW(TAG, "HTTP response %d from flight trace API (%s)", status, url);
        }
        esp_http_client_close(client);
    } else {
        ESP_LOGW(TAG, "Flight trace HTTPS request failed: %s", esp_err_to_name(err));
    }
    net_http_unlock();

    return err == ESP_OK && status == 200 && total_read > 0;
}

// Parses the tar1090/readsb "trace_recent_<hex>.json" response -
// {"trace":[[t, lat, lon, alt, ...], ...]} - into scratch_buf (PSRAM,
// MAX_TRACE_POINTS capacity), evenly downsampling if the API returned more
// points than that. Points with a missing/non-numeric lat or lon are skipped
// (e.g. a gap in coverage); a non-numeric altitude (the string "ground") is
// stored as 0.
static int trace_parse_json(const char *json, TracePoint *scratch_buf) {
    cJSON *root = cJSON_Parse(json);
    if (!root) return 0;

    cJSON *trace = cJSON_GetObjectItem(root, "trace");
    int out_count = 0;
    if (cJSON_IsArray(trace)) {
        int n = cJSON_GetArraySize(trace);
        int step = (n > MAX_TRACE_POINTS) ? (n + MAX_TRACE_POINTS - 1) / MAX_TRACE_POINTS : 1;
        for (int i = 0; i < n && out_count < MAX_TRACE_POINTS; i += step) {
            cJSON *pt = cJSON_GetArrayItem(trace, i);
            if (!cJSON_IsArray(pt) || cJSON_GetArraySize(pt) < 3) continue;
            cJSON *jlat = cJSON_GetArrayItem(pt, 1);
            cJSON *jlon = cJSON_GetArrayItem(pt, 2);
            if (!cJSON_IsNumber(jlat) || !cJSON_IsNumber(jlon)) continue;
            if (jlat->valuedouble == 0.0 && jlon->valuedouble == 0.0) continue;

            scratch_buf[out_count].lat = (float)jlat->valuedouble;
            scratch_buf[out_count].lon = (float)jlon->valuedouble;
            cJSON *jalt = cJSON_GetArrayItem(pt, 3);
            scratch_buf[out_count].alt_ft = cJSON_IsNumber(jalt) ? jalt->valueint : 0;
            out_count++;
        }
    }

    cJSON_Delete(root);
    return out_count;
}

// One-shot-per-request task servicing adsb_service_request_trace() - stays
// blocked on ulTaskNotifyTake() the rest of the time, so it costs nothing
// while the "Flight Trace (API)" click action is not in use.
static void trace_fetch_task(void *pvParameters) {
    wifi_manager_wait_connected();

    char *resp_buf = heap_caps_malloc(TRACE_HTTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    TracePoint *scratch_buf = heap_caps_malloc(MAX_TRACE_POINTS * sizeof(TracePoint), MALLOC_CAP_SPIRAM);
    if (!resp_buf || !scratch_buf) {
        ESP_LOGE(TAG, "Failed to allocate flight trace buffers in PSRAM!");
        vTaskDelete(NULL);
        return;
    }

    esp_http_client_config_t config = {
        .url = "https://globe.adsb.fi/",
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        // Capped like the map tile client (map_tile_service.c) - a smaller
        // buffer_size limits the effective TCP receive window so the server
        // cannot flood the SDIO driver's DMA mempool with a burst of
        // packets faster than lwIP can drain them.
        .buffer_size = 4096,
        .buffer_size_tx = 1024,
        .max_redirection_count = 3,
        .user_agent = ADSB_USER_AGENT,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize flight trace HTTP client!");
        heap_caps_free(resp_buf);
        heap_caps_free(scratch_buf);
        vTaskDelete(NULL);
        return;
    }
    esp_http_client_set_header(client, "User-Agent", ADSB_USER_AGENT);
    esp_http_client_set_header(client, "Accept", "application/json");

    while (1) {
        // This is the only task that ever calls trace_fetch_url() - loop
        // iterations run strictly one at a time, so at most one flight
        // trace fetch is ever in flight app-wide, by construction.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char hex[8];
        uint32_t my_generation;
        if (!adsb_service_lock(1000)) continue;
        snprintf(hex, sizeof(hex), "%s", s_trace_pending_hex);
        my_generation = s_trace_generation;
        adsb_service_unlock();
        if (hex[0] == '\0') continue;

        // Debounce: a rapid string of clicks (switching the selected
        // aircraft, or selecting/deselecting quickly) would otherwise open a
        // new HTTPS/TLS session per click, piling more load onto the SDIO
        // driver's DMA buffer pool for requests whose results are about to
        // be discarded anyway. Wait here, before opening any socket, and
        // bail out without ever connecting if a newer request (or a clear)
        // superseded this one in the meantime.
        vTaskDelay(pdMS_TO_TICKS(300));
        if (!adsb_service_lock(1000)) continue;
        bool still_current = (s_trace_generation == my_generation) && (strcmp(s_trace_pending_hex, hex) == 0);
        adsb_service_unlock();
        if (!still_current) {
            ESP_LOGI(TAG, "Flight trace request for %s superseded during debounce - skipping fetch", hex);
            continue;
        }

        // tar1090/readsb trace files are sharded into subdirectories keyed
        // by the last 2 hex characters of the ICAO address (e.g. "4406fe"
        // lives under ".../traces/fe/trace_recent_4406fe.json").
        size_t hex_len = strlen(hex);
        const char *subfolder = hex_len >= 2 ? hex + hex_len - 2 : hex;

        // Confirmed by curl testing against the live endpoints: both
        // globe.adsb.fi and globe.airplanes.live are Cloudflare-fronted and
        // return 403 Forbidden unless the Referer header's host matches the
        // one being requested (in addition to the existing browser-like
        // ADSB_USER_AGENT - a generic UA is 403'd even with a correct
        // Referer). globe.adsb.lol was dropped as a fallback: it always
        // 302-redirects to the bare adsb.lol host, which then answers with
        // Content-Encoding: gzip unconditionally - esp_http_client has no
        // gzip decoder, so that response is unusable here.
        char url[160];
        snprintf(url, sizeof(url), "https://globe.adsb.fi/data/traces/%s/trace_recent_%s.json", subfolder, hex);
        bool ok = trace_fetch_url(client, url, "https://globe.adsb.fi/", resp_buf, TRACE_HTTP_BUFFER_SIZE);
        if (!ok) {
            snprintf(url, sizeof(url), "https://globe.airplanes.live/data/traces/%s/trace_recent_%s.json", subfolder, hex);
            ok = trace_fetch_url(client, url, "https://globe.airplanes.live/", resp_buf, TRACE_HTTP_BUFFER_SIZE);
        }

        int parsed_count = ok ? trace_parse_json(resp_buf, scratch_buf) : 0;

        if (!adsb_service_lock(1000)) continue;
        // A newer request, or a cleared selection, superseded this fetch
        // while it was in flight - discard the (possibly stale) result.
        if (s_trace_generation == my_generation && strcmp(s_trace_pending_hex, hex) == 0) {
            if (parsed_count > 0) {
                memcpy(g_trace_points, scratch_buf, parsed_count * sizeof(TracePoint));
                g_trace_point_count = parsed_count;
                snprintf(g_trace_hex, sizeof(g_trace_hex), "%s", hex);
                ESP_LOGI(TAG, "Fetched flight trace for %s: %d points", hex, parsed_count);
            } else {
                g_trace_point_count = 0;
                g_trace_hex[0] = '\0';
                ESP_LOGW(TAG, "Flight trace fetch failed or empty for %s", hex);
            }
        }
        adsb_service_unlock();

        radar_ui_refresh();
    }
}

// Appends the selected aircraft's current position to g_trace_points, once
// its historical trace has been fetched (g_trace_hex matches - see
// adsb_service.h). Called under g_data_mutex, right after live_fleet is
// refreshed each polling cycle, so the flight trace keeps growing with live
// breadcrumbs instead of jumping straight from the last history point to the
// aircraft's current position when it turns. FIFO-evicts the oldest point
// once MAX_TRACE_POINTS is reached so the trace can grow indefinitely
// without unbounded PSRAM use.
static void trace_append_live_point(void) {
    if (g_trace_hex[0] == '\0') return;

    AircraftData *ac = NULL;
    for (int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
        if (live_fleet[i].active && strcmp(live_fleet[i].hex, g_trace_hex) == 0) {
            ac = &live_fleet[i];
            break;
        }
    }
    if (!ac) return;

    if (g_trace_point_count > 0) {
        const TracePoint *last = &g_trace_points[g_trace_point_count - 1];
        // Skip if the aircraft hasn't moved since the last recorded point
        // (parked/holding) - avoids piling up duplicate points.
        if (fabsf(last->lat - ac->lat) < 1e-5f && fabsf(last->lon - ac->lon) < 1e-5f) {
            return;
        }
    }

    if (g_trace_point_count >= MAX_TRACE_POINTS) {
        memmove(&g_trace_points[0], &g_trace_points[1], (MAX_TRACE_POINTS - 1) * sizeof(TracePoint));
        g_trace_point_count = MAX_TRACE_POINTS - 1;
    }
    g_trace_points[g_trace_point_count].lat = ac->lat;
    g_trace_points[g_trace_point_count].lon = ac->lon;
    g_trace_points[g_trace_point_count].alt_ft = ac->altitude_ft;
    g_trace_point_count++;
}

static void adsb_worker_task(void *pvParameters) {
    wifi_manager_wait_connected();
    ESP_LOGI(TAG, "Starting cyclic ADS-B polling task...");

    char *resp_buf = heap_caps_malloc(HTTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (!resp_buf) {
        ESP_LOGE(TAG, "Failed to allocate ADS-B buffer in PSRAM!");
        vTaskDelete(NULL);
        return;
    }

    // api.airplanes.live removed - its Cloudflare WAF blocks the ESP32's
    // MbedTLS TLS fingerprint outright (HTTP 403), unrelated to User-Agent.
    static const char *api_hosts[] = {
        "https://opendata.adsb.fi/api/v2/lat/%.4f/lon/%.4f/dist/%u",
        "https://api.adsb.lol/v2/point/%.4f/%.4f/%u"
    };
#define NUM_HOSTS (sizeof(api_hosts) / sizeof(api_hosts[0]))
    // opendata.adsb.fi is the default/preferred host (index 0) - only falls
    // over to adsb.lol after a failed request, see the rotation below.
    int s_current_host_idx = 0;
    int cycle_count = 0;

    char url[160];
    snprintf(url, sizeof(url), api_hosts[s_current_host_idx], g_radar_lat, g_radar_lon, compute_dist_nm(radar_ui_get_range_km()));

    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 12000,
        .buffer_size = 8192,
        .buffer_size_tx = 1024,
        .user_agent = ADSB_USER_AGENT,
        // Keeps the TCP/TLS session open between cycles when the API host
        // stays the same, avoiding a full mbedTLS handshake (and its
        // internal DMA-capable RAM allocation) every API_FETCH_SEC seconds.
        .keep_alive_enable = true,
    };
    // The HTTP client handle is created once and reused for the task's whole
    // lifetime (only esp_http_client_set_url() between cycles) - this avoids
    // repeated allocation/free of client structures every API_FETCH_SEC
    // seconds and limits DRAM heap fragmentation over long-running operation.
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client!");
        heap_caps_free(resp_buf);
        vTaskDelete(NULL);
        return;
    }
    // Set explicitly on top of config.user_agent above, since it must
    // survive across esp_http_client_set_url() calls on this reused handle.
    esp_http_client_set_header(client, "User-Agent", ADSB_USER_AGENT);
    esp_http_client_set_header(client, "Accept", "application/json");

    while (1) {
        if (s_adsb_paused || map_tile_is_downloading()) {
            // Blocks indefinitely - no periodic poll, no fetch is ever
            // attempted here. adsb_service_request_immediate_fetch() wakes
            // us the instant map_tile_service.c's sequential map fetch
            // finishes and calls adsb_service_resume().
            ESP_LOGI(TAG, "ADS-B worker waiting for map tiles completion...");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        // Recomputed every cycle - the user can change the HUD range live,
        // and a query radius wider than what is actually displayed only
        // wastes bandwidth/PSRAM on aircraft that never get drawn.
        uint32_t dist_nm = compute_dist_nm(radar_ui_get_range_km());
        snprintf(url, sizeof(url), api_hosts[s_current_host_idx], g_radar_lat, g_radar_lon, dist_nm);
        esp_http_client_set_url(client, url);

        // net_http_lock() serializes this against every other esp_http_client
        // session in the app - see net_lock.h. Note: this handle is
        // deliberately kept alive across cycles (keep_alive_enable above,
        // reused via esp_http_client_set_url()) instead of being cleaned up
        // per request like the one-shot fetches elsewhere - an idle
        // established TLS session costs no DMA-capable SDIO buffers, while
        // tearing it down and renegotiating a full TLS handshake every
        // API_FETCH_SEC seconds would only add to the driver's buffer
        // pressure, working against the very problem this lock exists to fix.
        if (!net_http_lock(portMAX_DELAY)) {
            ESP_LOGE(TAG, "Failed to acquire the global HTTP/HTTPS network lock");
            vTaskDelay(pdMS_TO_TICKS(API_FETCH_SEC * 1000));
            continue;
        }
        esp_err_t err = esp_http_client_open(client, 0);
        int status = 0;
        int total_read = 0;

        if (err == ESP_OK) {
            esp_http_client_fetch_headers(client);
            status = esp_http_client_get_status_code(client);

            if (status == 200) {
                int read_len = 0;
                while ((read_len = esp_http_client_read(client, resp_buf + total_read, HTTP_BUFFER_SIZE - total_read - 1)) > 0) {
                    total_read += read_len;
                }
                resp_buf[total_read] = '\0';
                // Connection intentionally left open for keep-alive reuse on
                // the next cycle (see .keep_alive_enable above).
            } else {
                ESP_LOGW(TAG, "HTTP response %d from ADS-B API (%s)", status, url);
                esp_http_client_close(client);
            }
        } else {
            ESP_LOGW(TAG, "ADS-B HTTPS request failed: %s", esp_err_to_name(err));
        }
        net_http_unlock();

        if (err == ESP_OK && status == 200 && total_read > 200) {
            if (total_read >= HTTP_BUFFER_SIZE - 1) {
                ESP_LOGW(TAG, "HTTP buffer (%d B) full - API response may have been truncated", HTTP_BUFFER_SIZE);
            }

            int max_planes = wifi_mgr_get_max_aircraft();
            if (max_planes > MAX_AIRCRAFT_CAPACITY) max_planes = MAX_AIRCRAFT_CAPACITY;

            memset(temp_fleet, 0, sizeof(AircraftData) * MAX_AIRCRAFT_CAPACITY);
            int parsed = parse_adsb_json(resp_buf, temp_fleet, max_planes);

            if (parsed > 0 && adsb_service_lock(100)) {
                total_aircraft_in_zone = parsed;
                for (int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
                    if (i < parsed) live_fleet[i] = temp_fleet[i];
                    else live_fleet[i].active = false;
                }
                trace_append_live_point();
                adsb_service_unlock();
                ESP_LOGI(TAG, "Fetched %d aircraft from %s", parsed, url);

                radar_ui_refresh();

                if (s_first_fetch_pending) {
                    s_first_fetch_pending = false;
                    radar_ui_hide_loading();
                }
            }
        } else {
            // Any non-200 response, transport error, or suspiciously short
            // body rotates to the next host unconditionally, so a failing
            // host is never retried forever - the next cycle always tries
            // the other API.
            s_current_host_idx = (s_current_host_idx + 1) % NUM_HOSTS;
        }

        cycle_count++;
        if (cycle_count >= MEM_TELEMETRY_EVERY_N_CYCLES) {
            cycle_count = 0;
            log_memory_telemetry();
        }

        vTaskDelay(pdMS_TO_TICKS(API_FETCH_SEC * 1000));
    }
}

bool adsb_service_init(void) {
    g_data_mutex = xSemaphoreCreateMutex();

    track_db = (AircraftTrackHistory *)heap_caps_calloc(MAX_SEEN_DB, sizeof(AircraftTrackHistory), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    live_fleet = (AircraftData *)heap_caps_calloc(MAX_AIRCRAFT_CAPACITY, sizeof(AircraftData), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    temp_fleet = (AircraftData *)heap_caps_calloc(MAX_AIRCRAFT_CAPACITY, sizeof(AircraftData), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_trace_points = (TracePoint *)heap_caps_calloc(MAX_TRACE_POINTS, sizeof(TracePoint), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!g_data_mutex || !track_db || !live_fleet || !temp_fleet || !g_trace_points) {
        ESP_LOGE(TAG, "Failed to allocate ADS-B data structures in PSRAM!");
        return false;
    }
    return true;
}

void adsb_service_start(void) {
    // 16 KB stack - with CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y (see
    // sdkconfig.defaults), FreeRTOS places this in external PSRAM instead of
    // the internal DMA-capable SRAM the Wi-Fi SDIO driver needs, so this
    // task's stack never competes with it.
    // Pinned to core 0 (System & Network), alongside lwIP's tcpip_thread
    // (CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0, sdkconfig.defaults) - core 1
    // is reserved for LVGL/UI (see main.c).
    xTaskCreatePinnedToCore(adsb_worker_task, "adsb_worker", 16384, NULL, 3, &s_adsb_task_handle, 0);
    xTaskCreatePinnedToCore(trace_fetch_task, "trace_fetch", 12288, NULL, 2, &s_trace_task_handle, 0);
}
