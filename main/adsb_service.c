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

#include "adsb_service.h"
#include "wifi_manager.h"
#include "radar_ui.h"

static const char *TAG = "ADSB_SVC";

static SemaphoreHandle_t g_data_mutex = NULL;

static AircraftTrackHistory *track_db = NULL;
static int track_db_count = 0;

AircraftData *live_fleet = NULL;
static AircraftData *temp_fleet = NULL;
int total_aircraft_in_zone = 0;

// Co ile cykli odpytywania (co API_FETCH_SEC sekund) logowana jest telemetria pamieci.
#define MEM_TELEMETRY_EVERY_N_CYCLES  15

uint32_t get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
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

// ================= PARSER JSON ADS-B =================
// Wyszukiwanie ograniczone do [block, end) - nie wychodzi poza pojedynczy
// obiekt JSON samolotu (np. na kolejny "{\"hex\":..." w tablicy "ac").
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

// Czysci callsign z paddingu 6-bit ADS-B ('@' = kod 0x00) i innych znakow
// niedrukowalnych, po czym obcina wiodace/koncowe spacje. Jesli po czyszczeniu
// nic nie zostanie (samo padding/spacje/pusty string), podstawia czytelny
// adres ICAO Hex (wielkimi literami) zamiast pustego/smieciowego identyfikatora.
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

static int parse_adsb_json(const char *json, AircraftData *out_planes, int max_planes) {
    if (!json || !out_planes) return 0;
    int count = 0;
    const char *p = strstr(json, "\"ac\":");
    if (!p) p = strstr(json, "\"aircraft\":");
    if (!p) p = json;

    uint32_t now = get_time_ms();

    while ((p = strstr(p, "{\"hex\":")) != NULL && count < max_planes) {
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

        // dump1090-pochodne API (adsb.fi, airplanes.live) sygnalizuja samolot
        // na ziemi wartoscia tekstowa "ground" w polu alt_baro (zamiast liczby).
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

    if (count == max_planes && p != NULL && strstr(p, "{\"hex\":") != NULL) {
        ESP_LOGW(TAG, "Limit max_aircraft (%d) osiagniety - nadmiarowe samoloty w odpowiedzi API zostaly odrzucone", max_planes);
    }

    return count;
}

// ================= TELEMETRIA PAMIECI =================
static void log_memory_telemetry(void) {
    ESP_LOGI(TAG, "Heap: wolna=%" PRIu32 "B min=%" PRIu32 "B | PSRAM wolna=%u B | stos adsb_worker: %u B wolne",
             (uint32_t)esp_get_free_heap_size(),
             (uint32_t)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
}

// ================= WATEK SIECIOWY ADS-B =================
bool adsb_service_lock(uint32_t timeout_ms) {
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout (%" PRIu32 "ms) oczekiwania na g_data_mutex", timeout_ms);
        return false;
    }
    return true;
}

void adsb_service_unlock(void) {
    xSemaphoreGive(g_data_mutex);
}

static void adsb_worker_task(void *pvParameters) {
    wifi_manager_wait_connected();
    ESP_LOGI(TAG, "Start zadan cyklicznego odpytywania ADS-B...");

    char *resp_buf = heap_caps_malloc(HTTP_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (!resp_buf) {
        ESP_LOGE(TAG, "Blad alokacji bufora ADS-B w PSRAM!");
        vTaskDelete(NULL);
        return;
    }

    static const char *api_hosts[] = {
        "https://opendata.adsb.fi/api/v2/lat/%.4f/lon/%.4f/dist/140",
        "https://api.airplanes.live/v2/point/%.4f/%.4f/140"
    };
    int host_idx = 0;
    int cycle_count = 0;

    char url[160];
    snprintf(url, sizeof(url), api_hosts[host_idx], g_radar_lat, g_radar_lon);

    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 9000,
        .buffer_size = 8192,
        .buffer_size_tx = 1024,
        .user_agent = "ADSBRadarViewer/1.0",
    };
    // Uchwyt klienta HTTP jest tworzony raz i reuzywany przez caly czas zycia
    // tasku (tylko esp_http_client_set_url() miedzy cyklami) - unika to
    // powtarzalnej alokacji/zwolnienia struktur klienta co API_FETCH_SEC sekund
    // i ogranicza fragmentacje sterty DRAM przy dlugotrwalej pracy.
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Blad inicjalizacji klienta HTTP!");
        heap_caps_free(resp_buf);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        snprintf(url, sizeof(url), api_hosts[host_idx], g_radar_lat, g_radar_lon);
        esp_http_client_set_url(client, url);

        esp_err_t err = esp_http_client_open(client, 0);

        if (err == ESP_OK) {
            esp_http_client_fetch_headers(client);
            int status = esp_http_client_get_status_code(client);

            if (status == 200) {
                int total_read = 0;
                int read_len = 0;

                while ((read_len = esp_http_client_read(client, resp_buf + total_read, HTTP_BUFFER_SIZE - total_read - 1)) > 0) {
                    total_read += read_len;
                }
                resp_buf[total_read] = '\0';
                esp_http_client_close(client);

                if (total_read >= HTTP_BUFFER_SIZE - 1) {
                    ESP_LOGW(TAG, "Bufor HTTP (%d B) zapelniony - odpowiedz API mogla zostac ucieta", HTTP_BUFFER_SIZE);
                }

                if (total_read > 200) {
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
                        adsb_service_unlock();
                        ESP_LOGI(TAG, "Pobrano %d samolotow z %s", parsed, url);

                        radar_ui_refresh();
                    }
                } else {
                    host_idx = (host_idx + 1) % 2;
                }
            } else {
                ESP_LOGW(TAG, "Odpowiedz HTTP %d z API ADS-B (%s)", status, url);
                esp_http_client_close(client);
                host_idx = (host_idx + 1) % 2;
            }
        } else {
            ESP_LOGW(TAG, "Blad zapytania ADS-B HTTPS: %s", esp_err_to_name(err));
            host_idx = (host_idx + 1) % 2;
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

    if (!g_data_mutex || !track_db || !live_fleet || !temp_fleet) {
        ESP_LOGE(TAG, "Nie udalo sie zaalokowac pamieci struktur ADS-B w PSRAM!");
        return false;
    }
    return true;
}

void adsb_service_start(void) {
    xTaskCreatePinnedToCore(adsb_worker_task, "adsb_worker", 16384, NULL, 3, NULL, 1);
}
