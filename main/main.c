#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"
#include "lvgl.h"

// Grafiki wektorowe samolotów
#include "aircraft_yellow_15.c"
#include "aircraft_yellow_20.c"
#include "aircraft_yellow_25.c"
#include "helicopter_yellow_20.c"

// ================= KONFIGURACJA RADARU I SIECI =================
#define WIFI_SSID               "Mis_test"
#define WIFI_PASS               "Testowo11"

static float g_radar_lat = 52.5463f;    // Płock (N)
static float g_radar_lon = 19.7065f;    // Płock (E)
static char g_station_name[32] = "PLOCK RADAR";

#define MAX_AIRCRAFT            64
#define MAX_SEEN_DB             512
#define MAX_TRACK_POINTS        64
#define MAX_TRACK_HISTORY_PTS   64
#define MAX_HUD_SEGS            32
#define PLANE_TIMEOUT_MS        45000
#define API_FETCH_SEC           4

#define SCREEN_WIDTH            1024
#define SCREEN_HEIGHT           600
#define MAP_SIZE                580
#define RADAR_CENTER_X          290
#define RADAR_CENTER_Y          290
#define RADAR_MAX_RADIUS        260
#define NUM_SWEEP_RAYS          16

#define DEG_TO_RAD              0.017453292519943295f
#define RAD_TO_DEG              57.29577951308232f

#define HTTP_BUFFER_SIZE        (128 * 1024)
// ===============================================================

static const char *TAG = "ADSB_RADAR";
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT      BIT0

static SemaphoreHandle_t g_data_mutex = NULL;
static httpd_handle_t g_web_server = NULL;

typedef enum {
    AC_SMALL = 0,
    AC_NORMAL,
    AC_LARGE,
    AC_HELI
} AircraftType;

typedef struct {
    char hex[8];
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    int pt_count;
    float lats[MAX_TRACK_HISTORY_PTS];
    float lons[MAX_TRACK_HISTORY_PTS];
    int   alts[MAX_TRACK_HISTORY_PTS];
    uint32_t times[MAX_TRACK_HISTORY_PTS];
} AircraftTrackHistory;

typedef struct {
    char hex[8];
    char callsign[12];
    char registration[16];
    char model[8];
    char category[4];
    char squawk[8];
    char vsi_str[10];
    float lat;
    float lon;
    float distance_km;
    float bearing_deg;
    int altitude_ft;
    int speed_kt;
    int heading_deg;
    AircraftType type;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    bool active;
} AircraftData;

static AircraftTrackHistory *track_db = NULL;
static int track_db_count = 0;

static AircraftData *live_fleet = NULL;
static AircraftData *temp_fleet = NULL;
static int total_aircraft_in_zone = 0;

static const float range_steps[] = {250.0f, 200.0f, 150.0f, 100.0f, 50.0f, 30.0f, 20.0f, 10.0f};
#define NUM_RANGE_STEPS (sizeof(range_steps) / sizeof(range_steps[0]))
static int current_range_idx = 0;
static bool hide_ground_planes = false;
static int radar_mode = 0;
static bool show_airports = true;

typedef struct {
    const char *icao;
    const char *name;
    float lat;
    float lon;
} AirportInfo;

static const AirportInfo nearby_airports[] = {
    {"EPWA", "Warszawa",      52.1657f, 20.9671f},
    {"EPMO", "Modlin",        52.4511f, 20.6518f},
    {"EPLL", "Lodz",          51.7219f, 19.3981f},
    {"EPBY", "Bydgoszcz",     53.0968f, 17.9777f},
    {"EPPO", "Poznan",        52.4210f, 16.8260f},
    {"EPRA", "Radom",         51.3888f, 21.2130f},
    {"EPLB", "Lublin",        51.2403f, 22.7136f},
    {"EPSY", "Szymany",       53.4819f, 20.9422f},
    {"EPGD", "Gdansk",        54.3776f, 18.4662f},
    {"EPOD", "Olsztyn",       53.7744f, 20.4194f},
    {"EPTO", "Torun",         53.0294f, 18.5436f},
    {"EPPL", "Plock",         52.5628f, 19.7211f}
};
#define NUM_AIRPORTS (sizeof(nearby_airports) / sizeof(nearby_airports[0]))

typedef struct {
    lv_obj_t *marker;
    lv_obj_t *label;
    float distance_km;
    float bearing_deg;
} AirportSlotUI;

static AirportSlotUI ui_airports[NUM_AIRPORTS];

typedef struct {
    lv_obj_t *radar_vector;
    lv_point_precise_t vec_pts[MAX_TRACK_POINTS + 2];
    lv_obj_t *radar_icon;
    lv_obj_t *radar_label;
    
    lv_obj_t *list_item;
    lv_obj_t *list_icon;
    lv_obj_t *list_lbl_cs;
    lv_obj_t *list_lbl_sub;
    lv_obj_t *list_lbl_route;
} AircraftSlotUI;

static AircraftSlotUI *ui_slots = NULL;

static lv_obj_t *radar_area;
static lv_obj_t *list_cont;
static lv_obj_t *lbl_status_count;
static lv_obj_t *lbl_range_header;
static lv_obj_t *lbl_range_scope;
static lv_obj_t *lbl_range_pill;
static lv_obj_t *btn_filter_ground;
static lv_obj_t *lbl_filter_ground;
static lv_obj_t *btn_sweep_toggle;
static lv_obj_t *lbl_sweep_toggle;
static lv_obj_t *btn_airport_toggle;
static lv_obj_t *lbl_airport_toggle;
static lv_obj_t *radar_pulse_ring;

static lv_obj_t *sweep_lines[NUM_SWEEP_RAYS];
static lv_point_precise_t sweep_pts[NUM_SWEEP_RAYS][2];
static float sweep_angle_deg = 0.0f;
static float pulse_ring_radius = 0.0f;

static lv_obj_t *hud_trail_segs[MAX_HUD_SEGS];
static lv_point_precise_t hud_seg_pts[MAX_HUD_SEGS][2];

static lv_obj_t *popup_card_cont;
static lv_obj_t *popup_top_box;
static lv_obj_t *popup_lbl_time;
static lv_obj_t *popup_lbl_date;
static lv_obj_t *popup_lbl_count;
static lv_obj_t *popup_img_preview;
static lv_obj_t *popup_lbl_credit;

static lv_obj_t *popup_hud_box;
static lv_obj_t *popup_lbl_title;
static lv_obj_t *popup_lbl_col_left;
static lv_obj_t *popup_lbl_col_right;
static lv_obj_t *popup_lbl_route;

static char selected_hex[8] = "";
static char selected_callsign[12] = "";
static char selected_reg[16] = "";
static char selected_model[8] = "";

static inline void lv_obj_set_hidden(lv_obj_t *obj, bool hidden) {
    if (!obj) return;
    if (hidden) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static uint32_t get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static lv_color_t get_fr24_altitude_color(int alt_ft) {
    if(alt_ft <= 1500)  return lv_color_hex(0xff3300);
    if(alt_ft <= 5000)  return lv_color_hex(0xffea00);
    if(alt_ft <= 12000) return lv_color_hex(0x00e676);
    if(alt_ft <= 22000) return lv_color_hex(0x00e5ff);
    if(alt_ft <= 32000) return lv_color_hex(0x2979ff);
    if(alt_ft <= 38000) return lv_color_hex(0x9d4edd);
    return lv_color_hex(0xe040fb);
}

static const lv_image_dsc_t* get_aircraft_dsc(AircraftType type) {
    if(type == AC_HELI)  return &helicopter_yellow_20;
    if(type == AC_SMALL) return &aircraft_yellow_15;
    if(type == AC_LARGE) return &aircraft_yellow_25;
    return &aircraft_yellow_20;
}

static void calculate_coords(float lat, float lon, float *out_dist_km, float *out_bearing_deg) {
    float dlat = (lat - g_radar_lat) * 111.132f;
    float dlon = (lon - g_radar_lon) * 111.132f * cosf(g_radar_lat * DEG_TO_RAD);
    *out_dist_km = sqrtf(dlat * dlat + dlon * dlon);
    float angle = atan2f(dlon, dlat) * RAD_TO_DEG;
    if(angle < 0.0f) angle += 360.0f;
    *out_bearing_deg = angle;
}

static AircraftTrackHistory* find_aircraft_track(const char *hex) {
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
static const char *json_get_field(const char *block, const char *key) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(block, search);
    if (!p) return NULL;
    return p + strlen(search);
}

static bool json_get_str(const char *block, const char *key, char *out_str, int max_len) {
    const char *p = json_get_field(block, key);
    if (!p) return false;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '"') p++;
    int i = 0;
    while (*p != '\0' && *p != '"' && i < max_len - 1) {
        out_str[i++] = *p++;
    }
    out_str[i] = '\0';
    return true;
}

static bool json_get_float(const char *block, const char *key, float *out_val) {
    const char *p = json_get_field(block, key);
    if (!p) return false;
    while (*p == ' ' || *p == '\t' || *p == '"') p++;
    *out_val = (float)atof(p);
    return true;
}

static bool json_get_int(const char *block, const char *key, int *out_val) {
    const char *p = json_get_field(block, key);
    if (!p) return false;
    while (*p == ' ' || *p == '\t' || *p == '"') p++;
    *out_val = atoi(p);
    return true;
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

        json_get_str(p, "hex", plane->hex, sizeof(plane->hex));
        json_get_str(p, "flight", plane->callsign, sizeof(plane->callsign));
        for (int s = strlen(plane->callsign) - 1; s >= 0 && plane->callsign[s] == ' '; s--) plane->callsign[s] = '\0';

        json_get_str(p, "r", plane->registration, sizeof(plane->registration));
        json_get_str(p, "t", plane->model, sizeof(plane->model));
        json_get_str(p, "squawk", plane->squawk, sizeof(plane->squawk));

        char type_str[16] = {0};
        if (json_get_str(p, "t", type_str, sizeof(type_str))) {
            if (strstr(type_str, "H") || strstr(type_str, "EC")) {
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

        json_get_float(p, "lat", &plane->lat);
        json_get_float(p, "lon", &plane->lon);
        json_get_int(p, "alt_baro", &plane->altitude_ft);
        json_get_int(p, "track", &plane->heading_deg);
        json_get_int(p, "gs", &plane->speed_kt);

        int vsi_val = 0;
        if (json_get_int(p, "baro_rate", &vsi_val)) {
            if (vsi_val > 64) snprintf(plane->vsi_str, sizeof(plane->vsi_str), "+%d", vsi_val);
            else if (vsi_val < -64) snprintf(plane->vsi_str, sizeof(plane->vsi_str), "%d", vsi_val);
            else strcpy(plane->vsi_str, "LEVEL");
        } else {
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
    return count;
}

// ================= AKTUALIZACJA UI SAMOLOTÓW =================
static void update_aircraft_ui(void) {
    if (!live_fleet || !ui_slots) return;

    bsp_display_lock(0);

    uint32_t now = get_time_ms();
    float current_range_km = range_steps[current_range_idx];

    // Pozycjonowanie lotnisk
    for (size_t i = 0; i < NUM_AIRPORTS; i++) {
        if (show_airports && (ui_airports[i].distance_km <= current_range_km)) {
            float rad = ui_airports[i].bearing_deg * DEG_TO_RAD;
            float scale = (float)RADAR_MAX_RADIUS / current_range_km;
            int px = RADAR_CENTER_X + (int)(ui_airports[i].distance_km * scale * sinf(rad));
            int py = RADAR_CENTER_Y - (int)(ui_airports[i].distance_km * scale * cosf(rad));

            if (px >= 10 && px <= (MAP_SIZE - 10) && py >= 10 && py <= (MAP_SIZE - 10)) {
                lv_obj_set_hidden(ui_airports[i].marker, false);
                lv_obj_set_hidden(ui_airports[i].label, false);
                lv_obj_set_pos(ui_airports[i].marker, px - 3, py - 3);
                lv_obj_set_pos(ui_airports[i].label, px + 6, py - 6);
            } else {
                lv_obj_set_hidden(ui_airports[i].marker, true);
                lv_obj_set_hidden(ui_airports[i].label, true);
            }
        } else {
            lv_obj_set_hidden(ui_airports[i].marker, true);
            lv_obj_set_hidden(ui_airports[i].label, true);
        }
    }

    int visible_count = 0;
    int list_y_offset = 0;
    AircraftData *selected_ac = NULL;

    for(int i = 0; i < MAX_AIRCRAFT; i++) {
        bool is_valid = live_fleet[i].active && ((now - live_fleet[i].last_seen_ms) < PLANE_TIMEOUT_MS);
        if (hide_ground_planes && live_fleet[i].altitude_ft <= 0) is_valid = false;

        if(is_valid && live_fleet[i].distance_km <= current_range_km) {
            visible_count++;
            if(selected_hex[0] != '\0' && strcmp(selected_hex, live_fleet[i].hex) == 0) {
                selected_ac = &live_fleet[i];
            }

            float rad = live_fleet[i].bearing_deg * DEG_TO_RAD;
            float scale = (float)RADAR_MAX_RADIUS / current_range_km;
            int px = RADAR_CENTER_X + (int)(live_fleet[i].distance_km * scale * sinf(rad));
            int py = RADAR_CENTER_Y - (int)(live_fleet[i].distance_km * scale * cosf(rad));

            const lv_image_dsc_t *dsc = get_aircraft_dsc(live_fleet[i].type);
            int half_w = (int)dsc->header.w / 2;
            int half_h = (int)dsc->header.h / 2;

            // Wektor śladu lotu
            AircraftTrackHistory *th = find_aircraft_track(live_fleet[i].hex);
            if(th && th->pt_count >= 1) {
                int pts_to_draw = th->pt_count > MAX_TRACK_POINTS ? MAX_TRACK_POINTS : th->pt_count;
                for(int p = 0; p < pts_to_draw; p++) {
                    float p_dist, p_bear;
                    calculate_coords(th->lats[p], th->lons[p], &p_dist, &p_bear);
                    float p_rad = p_bear * DEG_TO_RAD;
                    ui_slots[i].vec_pts[p].x = RADAR_CENTER_X + (int)(p_dist * scale * sinf(p_rad));
                    ui_slots[i].vec_pts[p].y = RADAR_CENTER_Y - (int)(p_dist * scale * cosf(p_rad));
                }
                ui_slots[i].vec_pts[pts_to_draw].x = px;
                ui_slots[i].vec_pts[pts_to_draw].y = py;

                lv_obj_set_style_line_color(ui_slots[i].radar_vector, get_fr24_altitude_color(live_fleet[i].altitude_ft), 0);
                lv_line_set_points(ui_slots[i].radar_vector, ui_slots[i].vec_pts, pts_to_draw + 1);
                lv_obj_set_hidden(ui_slots[i].radar_vector, false);
            } else {
                lv_obj_set_hidden(ui_slots[i].radar_vector, true);
            }

            lv_obj_set_hidden(ui_slots[i].radar_icon, false);
            lv_obj_set_hidden(ui_slots[i].radar_label, false);

            lv_image_set_src(ui_slots[i].radar_icon, dsc);
            lv_image_set_pivot(ui_slots[i].radar_icon, half_w, half_h);
            lv_obj_set_pos(ui_slots[i].radar_icon, px - half_w, py - half_h);
            lv_obj_set_pos(ui_slots[i].radar_label, px + half_w + 6, py - half_h);
            lv_image_set_rotation(ui_slots[i].radar_icon, live_fleet[i].heading_deg * 10);

            lv_label_set_text_fmt(ui_slots[i].radar_label, "%s\n%d ft", 
                                  live_fleet[i].callsign[0] ? live_fleet[i].callsign : live_fleet[i].hex, 
                                  live_fleet[i].altitude_ft);
            lv_obj_set_style_text_color(ui_slots[i].radar_label, get_fr24_altitude_color(live_fleet[i].altitude_ft), 0);

            lv_obj_set_hidden(ui_slots[i].list_item, false);
            lv_obj_set_pos(ui_slots[i].list_item, 0, list_y_offset);
            list_y_offset += 65;

            lv_image_set_src(ui_slots[i].list_icon, dsc);
            lv_image_set_pivot(ui_slots[i].list_icon, half_w, half_h);
            lv_image_set_rotation(ui_slots[i].list_icon, live_fleet[i].heading_deg * 10);

            lv_label_set_text(ui_slots[i].list_lbl_cs, live_fleet[i].callsign[0] ? live_fleet[i].callsign : live_fleet[i].hex);
            lv_label_set_text_fmt(ui_slots[i].list_lbl_sub, "%s  %.1fKM  %d FT  %s  %dKT", 
                                  live_fleet[i].model, live_fleet[i].distance_km, live_fleet[i].altitude_ft, 
                                  live_fleet[i].vsi_str, live_fleet[i].speed_kt);
            lv_label_set_text_fmt(ui_slots[i].list_lbl_route, "HDG: %d deg  LAT: %.2f  LON: %.2f", 
                                  live_fleet[i].heading_deg, live_fleet[i].lat, live_fleet[i].lon);
        } else {
            lv_obj_set_hidden(ui_slots[i].radar_vector, true);
            lv_obj_set_hidden(ui_slots[i].radar_icon, true);
            lv_obj_set_hidden(ui_slots[i].radar_label, true);
            lv_obj_set_hidden(ui_slots[i].list_item, true);
        }
    }

    lv_label_set_text_fmt(lbl_status_count, "TRAFFIC: %d PLANES IN %.0fKM", visible_count, current_range_km);

    // Segmenty śladu HUD
    if(selected_ac) {
        AircraftTrackHistory *sth = find_aircraft_track(selected_ac->hex);
        int drawn_segs = 0;
        if(sth && sth->pt_count >= 2) {
            float scale = (float)RADAR_MAX_RADIUS / current_range_km;
            int total_pts = sth->pt_count > MAX_TRACK_POINTS ? MAX_TRACK_POINTS : sth->pt_count;
            int max_segs = total_pts - 1;
            if(max_segs > MAX_HUD_SEGS) max_segs = MAX_HUD_SEGS;

            for(int s = 0; s < max_segs; s++) {
                float d1, b1, d2, b2;
                calculate_coords(sth->lats[s], sth->lons[s], &d1, &b1);
                calculate_coords(sth->lats[s+1], sth->lons[s+1], &d2, &b2);

                hud_seg_pts[s][0].x = RADAR_CENTER_X + (int)(d1 * scale * sinf(b1 * DEG_TO_RAD));
                hud_seg_pts[s][0].y = RADAR_CENTER_Y - (int)(d1 * scale * cosf(b1 * DEG_TO_RAD));
                hud_seg_pts[s][1].x = RADAR_CENTER_X + (int)(d2 * scale * sinf(b2 * DEG_TO_RAD));
                hud_seg_pts[s][1].y = RADAR_CENTER_Y - (int)(d2 * scale * cosf(b2 * DEG_TO_RAD));

                lv_obj_set_style_line_color(hud_trail_segs[s], get_fr24_altitude_color(sth->alts[s]), 0);
                lv_line_set_points(hud_trail_segs[s], hud_seg_pts[s], 2);
                lv_obj_set_hidden(hud_trail_segs[s], false);
                drawn_segs++;
            }
        }
        for(int s = drawn_segs; s < MAX_HUD_SEGS; s++) {
            lv_obj_set_hidden(hud_trail_segs[s], true);
        }
    } else {
        for(int s = 0; s < MAX_HUD_SEGS; s++) {
            lv_obj_set_hidden(hud_trail_segs[s], true);
        }
    }

    if(selected_ac) {
        lv_obj_set_hidden(popup_card_cont, false);

        time_t t = time(NULL);
        struct tm *tm_info = localtime(&t);
        char time_buf[16], date_buf[24];
        strftime(time_buf, sizeof(time_buf), "%H:%M", tm_info);
        strftime(date_buf, sizeof(date_buf), "%d %b %Y", tm_info);

        lv_label_set_text(popup_lbl_time, time_buf);
        lv_label_set_text(popup_lbl_date, date_buf);
        lv_label_set_text_fmt(popup_lbl_count, "%d", visible_count);

        lv_image_set_src(popup_img_preview, get_aircraft_dsc(selected_ac->type));
        lv_image_set_rotation(popup_img_preview, 0);

        lv_label_set_text_fmt(popup_lbl_title, "%s  %s  [%s]", 
                              selected_ac->callsign[0] ? selected_ac->callsign : selected_ac->hex, 
                              selected_ac->model, selected_ac->registration);

        lv_label_set_text_fmt(popup_lbl_col_left, "ALT  %d ft\nSPD  %d kt\nDIST %.1f km", 
                              selected_ac->altitude_ft, selected_ac->speed_kt, selected_ac->distance_km);

        lv_label_set_text_fmt(popup_lbl_col_right, "V/S  %s\nHDG  %03d\nSQK  %s", 
                              selected_ac->vsi_str, selected_ac->heading_deg, selected_ac->squawk);

        lv_label_set_text_fmt(popup_lbl_route, "LAT: %.2f  LON: %.2f  (%s)", 
                              selected_ac->lat, selected_ac->lon, g_station_name);
    } else {
        lv_obj_set_hidden(popup_card_cont, true);
    }

    bsp_display_unlock();
}

// ================= ZDARZENIA PRZYCISKÓW =================
static void switch_range(void) {
    current_range_idx = (current_range_idx + 1) % NUM_RANGE_STEPS;
    float current_range = range_steps[current_range_idx];

    bsp_display_lock(0);
    lv_label_set_text_fmt(lbl_range_header, "RNG: %.0fKM", current_range);
    lv_label_set_text_fmt(lbl_range_scope, "%.0fKM", current_range);
    lv_label_set_text_fmt(lbl_range_pill, "<> %.0f km", current_range);
    bsp_display_unlock();

    update_aircraft_ui();
}

static void range_click_event_cb(lv_event_t *e) {
    switch_range();
}

static void filter_click_event_cb(lv_event_t *e) {
    hide_ground_planes = !hide_ground_planes;
    bsp_display_lock(0);
    if (hide_ground_planes) {
        lv_obj_set_style_bg_color(btn_filter_ground, lv_color_hex(0x005522), 0);
        lv_obj_set_style_border_color(btn_filter_ground, lv_color_hex(0x00ff88), 0);
        lv_label_set_text(lbl_filter_ground, "AIR: AIRBORNE");
    } else {
        lv_obj_set_style_bg_color(btn_filter_ground, lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_filter_ground, lv_color_hex(0x005522), 0);
        lv_label_set_text(lbl_filter_ground, "AIR: ALL");
    }
    bsp_display_unlock();

    update_aircraft_ui();
}

static void sweep_toggle_click_event_cb(lv_event_t *e) {
    radar_mode = (radar_mode + 1) % 3;
    bsp_display_lock(0);
    if (radar_mode == 0) {
        lv_obj_set_style_bg_color(btn_sweep_toggle, lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_sweep_toggle, lv_color_hex(0x005522), 0);
        lv_label_set_text(lbl_sweep_toggle, "RADAR: SWEEP");
    } else if (radar_mode == 1) {
        lv_obj_set_style_bg_color(btn_sweep_toggle, lv_color_hex(0x004d25), 0);
        lv_obj_set_style_border_color(btn_sweep_toggle, lv_color_hex(0x00ff88), 0);
        lv_label_set_text(lbl_sweep_toggle, "RADAR: PULSE");
    } else {
        lv_obj_set_style_bg_color(btn_sweep_toggle, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_sweep_toggle, lv_color_hex(0xff5555), 0);
        lv_label_set_text(lbl_sweep_toggle, "RADAR: OFF");
    }
    bsp_display_unlock();
}

static void airport_toggle_click_event_cb(lv_event_t *e) {
    show_airports = !show_airports;
    bsp_display_lock(0);
    if (show_airports) {
        lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0x005522), 0);
        lv_label_set_text(lbl_airport_toggle, "APTS: ON");
    } else {
        lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0xff5555), 0);
        lv_label_set_text(lbl_airport_toggle, "APTS: OFF");
    }
    bsp_display_unlock();

    update_aircraft_ui();
}

static void select_aircraft_by_index(int idx) {
    if(live_fleet && idx >= 0 && idx < MAX_AIRCRAFT && live_fleet[idx].active) {
        if(strcmp(selected_hex, live_fleet[idx].hex) == 0) {
            selected_hex[0] = '\0';
            selected_callsign[0] = '\0';
            selected_reg[0] = '\0';
            selected_model[0] = '\0';
        } else {
            snprintf(selected_hex, sizeof(selected_hex), "%s", live_fleet[idx].hex);
            snprintf(selected_callsign, sizeof(selected_callsign), "%s", live_fleet[idx].callsign);
            snprintf(selected_reg, sizeof(selected_reg), "%s", live_fleet[idx].registration);
            snprintf(selected_model, sizeof(selected_model), "%s", live_fleet[idx].model);
        }
        update_aircraft_ui();
    }
}

static void aircraft_click_event_cb(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    select_aircraft_by_index(idx);
}

static void popup_close_event_cb(lv_event_t *e) {
    selected_hex[0] = '\0';
    selected_callsign[0] = '\0';
    selected_reg[0] = '\0';
    selected_model[0] = '\0';
    update_aircraft_ui();
}

static void create_radar_circle(lv_obj_t *parent, int radius) {
    lv_obj_t *circ = lv_obj_create(parent);
    lv_obj_set_size(circ, radius * 2, radius * 2);
    lv_obj_set_pos(circ, RADAR_CENTER_X - radius, RADAR_CENTER_Y - radius);
    lv_obj_set_style_radius(circ, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(circ, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(circ, lv_color_hex(0x004d25), 0);
    lv_obj_set_style_border_width(circ, 1, 0);
    lv_obj_clear_flag(circ, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
}

static void radar_ui_timer_cb(lv_timer_t *timer) {
    sweep_angle_deg += 2.0f;
    if(sweep_angle_deg >= 360.0f) sweep_angle_deg -= 360.0f;

    if (radar_mode == 0) {
        lv_obj_set_hidden(radar_pulse_ring, true);
        for(int r = 0; r < NUM_SWEEP_RAYS; r++) {
            float a = (sweep_angle_deg - (float)r * 2.0f) * DEG_TO_RAD;
            sweep_pts[r][0].x = RADAR_CENTER_X;
            sweep_pts[r][0].y = RADAR_CENTER_Y;
            sweep_pts[r][1].x = RADAR_CENTER_X + (int)(RADAR_MAX_RADIUS * sinf(a));
            sweep_pts[r][1].y = RADAR_CENTER_Y - (int)(RADAR_MAX_RADIUS * cosf(a));
            lv_line_set_points(sweep_lines[r], sweep_pts[r], 2);
            lv_obj_set_hidden(sweep_lines[r], false);

            lv_opa_t opa = (r == 0) ? LV_OPA_COVER : (lv_opa_t)(LV_OPA_70 * (NUM_SWEEP_RAYS - r) / NUM_SWEEP_RAYS);
            lv_obj_set_style_line_opa(sweep_lines[r], opa, 0);
        }
    } else if (radar_mode == 1) {
        for(int r = 0; r < NUM_SWEEP_RAYS; r++) {
            lv_obj_set_hidden(sweep_lines[r], true);
        }
        pulse_ring_radius += 3.0f;
        if (pulse_ring_radius > (float)RADAR_MAX_RADIUS) {
            pulse_ring_radius = 0.0f;
        }
        int ring_size = (int)(pulse_ring_radius * 2);
        lv_obj_set_size(radar_pulse_ring, ring_size, ring_size);
        lv_obj_set_pos(radar_pulse_ring, RADAR_CENTER_X - (int)pulse_ring_radius, RADAR_CENTER_Y - (int)pulse_ring_radius);

        lv_opa_t ring_opa = (lv_opa_t)(255 * (1.0f - (pulse_ring_radius / (float)RADAR_MAX_RADIUS)));
        lv_obj_set_style_border_color(radar_pulse_ring, lv_color_hex(0xffffff), 0);
        lv_obj_set_style_border_opa(radar_pulse_ring, ring_opa, 0);
        lv_obj_set_hidden(radar_pulse_ring, false);
    } else {
        lv_obj_set_hidden(radar_pulse_ring, true);
        for(int r = 0; r < NUM_SWEEP_RAYS; r++) {
            lv_obj_set_hidden(sweep_lines[r], true);
        }
    }
}

// ================= PANEL WWW I ZAPIS DO NVS =================
static void load_settings_from_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("radar_cfg", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t len = sizeof(g_station_name);
        nvs_get_str(my_handle, "name", g_station_name, &len);
        uint32_t lat_bits = 0, lon_bits = 0;
        if (nvs_get_u32(my_handle, "lat", &lat_bits) == ESP_OK) memcpy(&g_radar_lat, &lat_bits, 4);
        if (nvs_get_u32(my_handle, "lon", &lon_bits) == ESP_OK) memcpy(&g_radar_lon, &lon_bits, 4);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Wczytano z NVS: %s (%.4f, %.4f)", g_station_name, g_radar_lat, g_radar_lon);
    }
}

static void save_settings_to_nvs(void) {
    nvs_handle_t my_handle;
    if (nvs_open("radar_cfg", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_str(my_handle, "name", g_station_name);
        uint32_t lat_bits = 0, lon_bits = 0;
        memcpy(&lat_bits, &g_radar_lat, 4);
        memcpy(&lon_bits, &g_radar_lon, 4);
        nvs_set_u32(my_handle, "lat", lat_bits);
        nvs_set_u32(my_handle, "lon", lon_bits);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    char resp[1024];
    snprintf(resp, sizeof(resp),
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Radar ADS-B Setup</title>"
        "<style>body{background:#0a0f1d;color:#00ff88;font-family:sans-serif;text-align:center;padding:20px;}"
        "input{padding:8px;margin:5px;border-radius:4px;border:1px solid #00ff88;background:#031018;color:#fff;}"
        "button{padding:10px 20px;background:#00ff88;color:#000;font-weight:bold;border:none;border-radius:5px;cursor:pointer;}</style></head>"
        "<body><h1>RADAR STATION CONFIG</h1>"
        "<form action='/save' method='GET'>"
        "Nazwa: <br><input type='text' name='name' value='%s'><br>"
        "Szerokosc (LAT): <br><input type='text' name='lat' value='%.4f'><br>"
        "Dlugosc (LON): <br><input type='text' name='lon' value='%.4f'><br><br>"
        "<button type='submit'>ZAPISZ I PRZELADUJ</button>"
        "</form></body></html>",
        g_station_name, g_radar_lat, g_radar_lon);
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t save_get_handler(httpd_req_t *req) {
    char buf[128];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[64];
        if (httpd_query_key_value(buf, "lat", param, sizeof(param)) == ESP_OK) g_radar_lat = atof(param);
        if (httpd_query_key_value(buf, "lon", param, sizeof(param)) == ESP_OK) g_radar_lon = atof(param);
        if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) snprintf(g_station_name, sizeof(g_station_name), "%s", param);
        save_settings_to_nvs();
        ESP_LOGI(TAG, "Zaktualizowano koordynaty z panelu WWW: %s (%.4f, %.4f)", g_station_name, g_radar_lat, g_radar_lon);
    }
    const char *resp = "<script>alert('Ustawienia zapisane!');window.location.href='/';</script>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void start_web_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = root_get_handler};
    httpd_uri_t save_uri = {.uri = "/save", .method = HTTP_GET, .handler = save_get_handler};

    if (httpd_start(&g_web_server, &config) == ESP_OK) {
        httpd_register_uri_handler(g_web_server, &root_uri);
        httpd_register_uri_handler(g_web_server, &save_uri);
        ESP_LOGI(TAG, "Panel WWW konfiguracji radaru uruchomiony na porcie 80!");
    }
}

// ================= BUDOWA STRUKTURY LVGL =================
static void build_plane_radar_ui(void) {
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x020704), 0);

    radar_area = lv_obj_create(scr);
    lv_obj_set_size(radar_area, MAP_SIZE, MAP_SIZE);
    lv_obj_set_pos(radar_area, 10, 10);
    lv_obj_set_style_bg_color(radar_area, lv_color_hex(0x040d1a), 0);
    lv_obj_set_style_border_color(radar_area, lv_color_hex(0x004d25), 0);
    lv_obj_set_style_border_width(radar_area, 1, 0);
    lv_obj_set_style_pad_all(radar_area, 0, 0);
    lv_obj_clear_flag(radar_area, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *grid_h = lv_line_create(radar_area);
    static lv_point_precise_t pt_h[2] = {{0, 290}, {580, 290}};
    lv_line_set_points(grid_h, pt_h, 2);
    lv_obj_set_style_line_color(grid_h, lv_color_hex(0x0a1e12), 0);
    lv_obj_set_style_line_width(grid_h, 1, 0);

    lv_obj_t *grid_v = lv_line_create(radar_area);
    static lv_point_precise_t pt_v[2] = {{290, 0}, {290, 580}};
    lv_line_set_points(grid_v, pt_v, 2);
    lv_obj_set_style_line_color(grid_v, lv_color_hex(0x0a1e12), 0);
    lv_obj_set_style_line_width(grid_v, 1, 0);

    for(int s = 0; s < MAX_HUD_SEGS; s++) {
        hud_trail_segs[s] = lv_line_create(radar_area);
        lv_obj_set_style_line_width(hud_trail_segs[s], 3, 0);
        lv_obj_set_style_line_opa(hud_trail_segs[s], LV_OPA_COVER, 0);
        lv_obj_set_hidden(hud_trail_segs[s], true);
    }

    create_radar_circle(radar_area, 60);
    create_radar_circle(radar_area, 125);
    create_radar_circle(radar_area, 190);
    create_radar_circle(radar_area, RADAR_MAX_RADIUS);

    lbl_range_scope = lv_label_create(radar_area);
    lv_label_set_text_fmt(lbl_range_scope, "%.0fKM", range_steps[current_range_idx]);
    lv_obj_set_style_text_color(lbl_range_scope, lv_color_hex(0x00aa55), 0);
    lv_obj_set_pos(lbl_range_scope, RADAR_CENTER_X + 195, RADAR_CENTER_Y - 10);

    const char *card_names[] = {"N", "S", "E", "W"};
    lv_align_t card_aligns[] = {LV_ALIGN_TOP_MID, LV_ALIGN_BOTTOM_MID, LV_ALIGN_RIGHT_MID, LV_ALIGN_LEFT_MID};
    int card_ox[] = {0, 0, -8, 8};
    int card_oy[] = {8, -8, 0, 0};
    for(int i = 0; i < 4; i++) {
        lv_obj_t *lbl = lv_label_create(radar_area);
        lv_label_set_text(lbl, card_names[i]);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xffffff), 0);
        lv_obj_align(lbl, card_aligns[i], card_ox[i], card_oy[i]);
    }

    for (size_t i = 0; i < NUM_AIRPORTS; i++) {
        calculate_coords(nearby_airports[i].lat, nearby_airports[i].lon, 
                         &ui_airports[i].distance_km, &ui_airports[i].bearing_deg);

        ui_airports[i].marker = lv_obj_create(radar_area);
        lv_obj_set_size(ui_airports[i].marker, 6, 6);
        lv_obj_set_style_radius(ui_airports[i].marker, 1, 0);
        lv_obj_set_style_bg_color(ui_airports[i].marker, lv_color_hex(0x38bdf8), 0);
        lv_obj_set_style_border_color(ui_airports[i].marker, lv_color_hex(0xffffff), 0);
        lv_obj_set_style_border_width(ui_airports[i].marker, 1, 0);
        lv_obj_clear_flag(ui_airports[i].marker, LV_OBJ_FLAG_SCROLLABLE);

        ui_airports[i].label = lv_label_create(radar_area);
        lv_label_set_text_fmt(ui_airports[i].label, "%s\n%s", nearby_airports[i].icao, nearby_airports[i].name);
        lv_obj_set_style_text_color(ui_airports[i].label, lv_color_hex(0x38bdf8), 0);
        lv_obj_set_hidden(ui_airports[i].marker, true);
        lv_obj_set_hidden(ui_airports[i].label, true);
    }

    lv_obj_t *range_pill = lv_obj_create(radar_area);
    lv_obj_set_size(range_pill, 110, 30);
    lv_obj_align(range_pill, LV_ALIGN_BOTTOM_LEFT, 15, -15);
    lv_obj_set_style_bg_color(range_pill, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(range_pill, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_border_width(range_pill, 1, 0);
    lv_obj_set_style_pad_all(range_pill, 0, 0);
    lv_obj_set_style_radius(range_pill, 15, 0);
    lv_obj_add_flag(range_pill, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(range_pill, range_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_range_pill = lv_label_create(range_pill);
    lv_label_set_text_fmt(lbl_range_pill, "<> %.0f km", range_steps[current_range_idx]);
    lv_obj_set_style_text_color(lbl_range_pill, lv_color_hex(0x00ff88), 0);
    lv_obj_center(lbl_range_pill);

    for(int r = NUM_SWEEP_RAYS - 1; r >= 0; r--) {
        sweep_lines[r] = lv_line_create(radar_area);
        lv_obj_set_style_line_color(sweep_lines[r], lv_color_hex(0x00ff66), 0);
        lv_obj_set_style_line_width(sweep_lines[r], 2, 0);
        lv_obj_set_style_line_opa(sweep_lines[r], (r == 0) ? LV_OPA_COVER : (lv_opa_t)(LV_OPA_60 * (NUM_SWEEP_RAYS - r) / NUM_SWEEP_RAYS), 0);
    }

    radar_pulse_ring = lv_obj_create(radar_area);
    lv_obj_set_size(radar_pulse_ring, 10, 10);
    lv_obj_set_style_radius(radar_pulse_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(radar_pulse_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(radar_pulse_ring, 2, 0);
    lv_obj_set_style_border_color(radar_pulse_ring, lv_color_hex(0xffffff), 0);
    lv_obj_clear_flag(radar_pulse_ring, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_hidden(radar_pulse_ring, true);

    lv_obj_t *home = lv_obj_create(radar_area);
    lv_obj_set_size(home, 8, 8);
    lv_obj_set_pos(home, RADAR_CENTER_X - 4, RADAR_CENTER_Y - 4);
    lv_obj_set_style_radius(home, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(home, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_border_width(home, 0, 0);
    lv_obj_clear_flag(home, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *right_panel = lv_obj_create(scr);
    lv_obj_set_size(right_panel, 414, 580);
    lv_obj_set_pos(right_panel, 600, 10);
    lv_obj_set_style_bg_color(right_panel, lv_color_hex(0x020a05), 0);
    lv_obj_set_style_border_color(right_panel, lv_color_hex(0x004d25), 0);
    lv_obj_set_style_border_width(right_panel, 1, 0);
    lv_obj_set_style_pad_all(right_panel, 8, 0);
    lv_obj_clear_flag(right_panel, LV_OBJ_FLAG_SCROLLABLE);

    btn_sweep_toggle = lv_obj_create(right_panel);
    lv_obj_set_size(btn_sweep_toggle, 96, 26);
    lv_obj_align(btn_sweep_toggle, LV_ALIGN_TOP_LEFT, 4, 0);
    lv_obj_set_style_bg_color(btn_sweep_toggle, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_sweep_toggle, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_sweep_toggle, 1, 0);
    lv_obj_set_style_radius(btn_sweep_toggle, 6, 0);
    lv_obj_set_style_pad_all(btn_sweep_toggle, 0, 0);
    lv_obj_add_flag(btn_sweep_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_sweep_toggle, sweep_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_sweep_toggle = lv_label_create(btn_sweep_toggle);
    lv_label_set_text(lbl_sweep_toggle, "RADAR: SWEEP");
    lv_obj_set_style_text_color(lbl_sweep_toggle, lv_color_hex(0x00ff88), 0);
    lv_obj_center(lbl_sweep_toggle);

    btn_airport_toggle = lv_obj_create(right_panel);
    lv_obj_set_size(btn_airport_toggle, 96, 26);
    lv_obj_align(btn_airport_toggle, LV_ALIGN_TOP_LEFT, 102, 0);
    lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_airport_toggle, 1, 0);
    lv_obj_set_style_radius(btn_airport_toggle, 6, 0);
    lv_obj_set_style_pad_all(btn_airport_toggle, 0, 0);
    lv_obj_add_flag(btn_airport_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_airport_toggle, airport_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_airport_toggle = lv_label_create(btn_airport_toggle);
    lv_label_set_text(lbl_airport_toggle, "APTS: ON");
    lv_obj_set_style_text_color(lbl_airport_toggle, lv_color_hex(0x00ff88), 0);
    lv_obj_center(lbl_airport_toggle);

    btn_filter_ground = lv_obj_create(right_panel);
    lv_obj_set_size(btn_filter_ground, 96, 26);
    lv_obj_align(btn_filter_ground, LV_ALIGN_TOP_RIGHT, -102, 0);
    lv_obj_set_style_bg_color(btn_filter_ground, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_filter_ground, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_filter_ground, 1, 0);
    lv_obj_set_style_radius(btn_filter_ground, 6, 0);
    lv_obj_set_style_pad_all(btn_filter_ground, 0, 0);
    lv_obj_add_flag(btn_filter_ground, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_filter_ground, filter_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_filter_ground = lv_label_create(btn_filter_ground);
    lv_label_set_text(lbl_filter_ground, "AIR: ALL");
    lv_obj_set_style_text_color(lbl_filter_ground, lv_color_hex(0x00ff88), 0);
    lv_obj_center(lbl_filter_ground);

    lv_obj_t *btn_range = lv_obj_create(right_panel);
    lv_obj_set_size(btn_range, 94, 26);
    lv_obj_align(btn_range, LV_ALIGN_TOP_RIGHT, -4, 0);
    lv_obj_set_style_bg_color(btn_range, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_range, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_border_width(btn_range, 1, 0);
    lv_obj_set_style_radius(btn_range, 6, 0);
    lv_obj_set_style_pad_all(btn_range, 0, 0);
    lv_obj_add_flag(btn_range, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_range, range_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_range_header = lv_label_create(btn_range);
    lv_label_set_text_fmt(lbl_range_header, "RNG: %.0fKM", range_steps[current_range_idx]);
    lv_obj_set_style_text_color(lbl_range_header, lv_color_hex(0x00ff88), 0);
    lv_obj_center(lbl_range_header);

    lbl_status_count = lv_label_create(right_panel);
    lv_label_set_text(lbl_status_count, "CONNECTING TO FEED...");
    lv_obj_set_style_text_color(lbl_status_count, lv_color_hex(0x38bdf8), 0);
    lv_obj_align(lbl_status_count, LV_ALIGN_TOP_MID, 0, 34);

    list_cont = lv_obj_create(right_panel);
    lv_obj_set_size(list_cont, 396, 515);
    lv_obj_align(list_cont, LV_ALIGN_TOP_MID, 0, 55);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_cont, 0, 0);
    lv_obj_set_style_pad_all(list_cont, 0, 0);

    for(int i = 0; i < MAX_AIRCRAFT; i++) {
        ui_slots[i].radar_vector = lv_line_create(radar_area);
        lv_obj_set_style_line_width(ui_slots[i].radar_vector, 2, 0);
        lv_obj_set_style_line_opa(ui_slots[i].radar_vector, LV_OPA_90, 0);
        lv_obj_set_hidden(ui_slots[i].radar_vector, true);

        ui_slots[i].radar_icon = lv_image_create(radar_area);
        lv_image_set_src(ui_slots[i].radar_icon, &aircraft_yellow_20);
        lv_image_set_pivot(ui_slots[i].radar_icon, 10, 10);
        lv_obj_add_flag(ui_slots[i].radar_icon, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(ui_slots[i].radar_icon, aircraft_click_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_set_hidden(ui_slots[i].radar_icon, true);

        ui_slots[i].radar_label = lv_label_create(radar_area);
        lv_obj_set_hidden(ui_slots[i].radar_label, true);

        ui_slots[i].list_item = lv_obj_create(list_cont);
        lv_obj_set_size(ui_slots[i].list_item, 396, 64);
        lv_obj_set_pos(ui_slots[i].list_item, 0, i * 65);
        lv_obj_set_style_bg_opa(ui_slots[i].list_item, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(ui_slots[i].list_item, 0, 0);
        lv_obj_set_style_pad_all(ui_slots[i].list_item, 0, 0);
        lv_obj_clear_flag(ui_slots[i].list_item, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(ui_slots[i].list_item, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(ui_slots[i].list_item, aircraft_click_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_set_hidden(ui_slots[i].list_item, true);

        ui_slots[i].list_icon = lv_image_create(ui_slots[i].list_item);
        lv_image_set_src(ui_slots[i].list_icon, &aircraft_yellow_20);
        lv_image_set_pivot(ui_slots[i].list_icon, 10, 10);
        lv_obj_set_pos(ui_slots[i].list_icon, 4, 10);

        ui_slots[i].list_lbl_cs = lv_label_create(ui_slots[i].list_item);
        lv_obj_set_style_text_color(ui_slots[i].list_lbl_cs, lv_color_hex(0xffffff), 0);
        lv_obj_set_pos(ui_slots[i].list_lbl_cs, 36, 2);

        ui_slots[i].list_lbl_sub = lv_label_create(ui_slots[i].list_item);
        lv_obj_set_style_text_color(ui_slots[i].list_lbl_sub, lv_color_hex(0x6ee7b7), 0);
        lv_obj_set_pos(ui_slots[i].list_lbl_sub, 36, 22);

        ui_slots[i].list_lbl_route = lv_label_create(ui_slots[i].list_item);
        lv_obj_set_style_text_color(ui_slots[i].list_lbl_route, lv_color_hex(0x38bdf8), 0);
        lv_obj_set_pos(ui_slots[i].list_lbl_route, 36, 40);

        lv_obj_t *div = lv_obj_create(ui_slots[i].list_item);
        lv_obj_set_size(div, 396, 1);
        lv_obj_align(div, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(div, lv_color_hex(0x003318), 0);
        lv_obj_set_style_border_width(div, 0, 0);
    }

    popup_card_cont = lv_obj_create(radar_area);
    lv_obj_set_size(popup_card_cont, 340, 360);
    lv_obj_center(popup_card_cont);
    lv_obj_set_style_bg_opa(popup_card_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(popup_card_cont, 0, 0);
    lv_obj_set_style_pad_all(popup_card_cont, 0, 0);
    lv_obj_clear_flag(popup_card_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(popup_card_cont, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(popup_card_cont, popup_close_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_hidden(popup_card_cont, true);

    popup_top_box = lv_obj_create(popup_card_cont);
    lv_obj_set_size(popup_top_box, 300, 165);
    lv_obj_align(popup_top_box, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_bg_color(popup_top_box, lv_color_hex(0x04111c), 0);
    lv_obj_set_style_border_color(popup_top_box, lv_color_hex(0x38bdf8), 0);
    lv_obj_set_style_border_width(popup_top_box, 1, 0);
    lv_obj_set_style_radius(popup_top_box, 8, 0);
    lv_obj_set_style_pad_all(popup_top_box, 4, 0);
    lv_obj_clear_flag(popup_top_box, LV_OBJ_FLAG_SCROLLABLE);

    popup_lbl_count = lv_label_create(popup_top_box);
    lv_label_set_text(popup_lbl_count, "0");
    lv_obj_set_style_text_color(popup_lbl_count, lv_color_hex(0xffffff), 0);
    lv_obj_align(popup_lbl_count, LV_ALIGN_TOP_LEFT, 8, 4);

    popup_lbl_date = lv_label_create(popup_top_box);
    lv_label_set_text(popup_lbl_date, "---");
    lv_obj_set_style_text_color(popup_lbl_date, lv_color_hex(0x94a3b8), 0);
    lv_obj_align(popup_lbl_date, LV_ALIGN_TOP_MID, 0, 4);

    popup_lbl_time = lv_label_create(popup_top_box);
    lv_label_set_text(popup_lbl_time, "--:--");
    lv_obj_set_style_text_color(popup_lbl_time, lv_color_hex(0xffffff), 0);
    lv_obj_align(popup_lbl_time, LV_ALIGN_TOP_RIGHT, -8, 4);

    popup_img_preview = lv_image_create(popup_top_box);
    lv_image_set_src(popup_img_preview, &aircraft_yellow_25);
    lv_obj_align(popup_img_preview, LV_ALIGN_CENTER, 0, 6);

    popup_lbl_credit = lv_label_create(popup_top_box);
    lv_label_set_text(popup_lbl_credit, "RADAR STATION FEED");
    lv_obj_set_style_text_color(popup_lbl_credit, lv_color_hex(0x38bdf8), 0);
    lv_obj_align(popup_lbl_credit, LV_ALIGN_BOTTOM_MID, 0, -2);

    popup_hud_box = lv_obj_create(popup_card_cont);
    lv_obj_set_size(popup_hud_box, 320, 165);
    lv_obj_align(popup_hud_box, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(popup_hud_box, lv_color_hex(0x02170b), 0);
    lv_obj_set_style_bg_opa(popup_hud_box, LV_OPA_90, 0);
    lv_obj_set_style_border_color(popup_hud_box, lv_color_hex(0x00e575), 0);
    lv_obj_set_style_border_width(popup_hud_box, 2, 0);
    lv_obj_set_style_radius(popup_hud_box, 14, 0);
    lv_obj_set_style_pad_all(popup_hud_box, 10, 0);
    lv_obj_clear_flag(popup_hud_box, LV_OBJ_FLAG_SCROLLABLE);

    popup_lbl_title = lv_label_create(popup_hud_box);
    lv_label_set_text(popup_lbl_title, "TARGET DETAILS");
    lv_obj_set_style_text_color(popup_lbl_title, lv_color_hex(0xffffff), 0);
    lv_obj_align(popup_lbl_title, LV_ALIGN_TOP_LEFT, 5, 0);

    popup_lbl_col_left = lv_label_create(popup_hud_box);
    lv_label_set_text(popup_lbl_col_left, "ALT ---\nSPD ---\nDIST ---");
    lv_obj_set_style_text_color(popup_lbl_col_left, lv_color_hex(0xffffff), 0);
    lv_obj_align(popup_lbl_col_left, LV_ALIGN_TOP_LEFT, 5, 28);

    popup_lbl_col_right = lv_label_create(popup_hud_box);
    lv_label_set_text(popup_lbl_col_right, "V/S ---\nHDG ---\nSQK ----");
    lv_obj_set_style_text_color(popup_lbl_col_right, lv_color_hex(0xffffff), 0);
    lv_obj_align(popup_lbl_col_right, LV_ALIGN_TOP_RIGHT, -10, 28);

    popup_lbl_route = lv_label_create(popup_hud_box);
    lv_label_set_text_fmt(popup_lbl_route, "LAT: %.2f  LON: %.2f  (%s)", g_radar_lat, g_radar_lon, g_station_name);
    lv_obj_set_style_text_color(popup_lbl_route, lv_color_hex(0x00e575), 0);
    lv_obj_align(popup_lbl_route, LV_ALIGN_BOTTOM_LEFT, 5, -2);

    lv_timer_create(radar_ui_timer_cb, 30, NULL);

    bsp_display_unlock();
}

// ================= WĄTEK SIECIOWY ADS-B =================
static void adsb_worker_task(void *pvParameters) {
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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

    while (1) {
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

        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_err_t err = esp_http_client_open(client, 0);

        if (err == ESP_OK) {
            esp_http_client_fetch_headers(client);
            int total_read = 0;
            int read_len = 0;

            while ((read_len = esp_http_client_read(client, resp_buf + total_read, HTTP_BUFFER_SIZE - total_read - 1)) > 0) {
                total_read += read_len;
            }
            resp_buf[total_read] = '\0';
            esp_http_client_close(client);

            if (total_read > 200) {
                memset(temp_fleet, 0, sizeof(AircraftData) * MAX_AIRCRAFT);
                int parsed = parse_adsb_json(resp_buf, temp_fleet, MAX_AIRCRAFT);

                if (parsed > 0 && xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    total_aircraft_in_zone = parsed;
                    for (int i = 0; i < MAX_AIRCRAFT; i++) {
                        if (i < parsed) live_fleet[i] = temp_fleet[i];
                        else live_fleet[i].active = false;
                    }
                    xSemaphoreGive(g_data_mutex);
                    ESP_LOGI(TAG, "Pobrano %d samolotow z %s", parsed, url);

                    update_aircraft_ui();
                }
            } else {
                host_idx = (host_idx + 1) % 2;
            }
        } else {
            ESP_LOGW(TAG, "Blad zapytania ADS-B HTTPS: %s", esp_err_to_name(err));
            host_idx = (host_idx + 1) % 2;
        }

        esp_http_client_cleanup(client);
        vTaskDelay(pdMS_TO_TICKS(API_FETCH_SEC * 1000));
    }
}

// ================= OBSŁUGA ZDARZEŃ WI-FI =================
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

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_settings_from_nvs();

    g_data_mutex = xSemaphoreCreateMutex();

    track_db = (AircraftTrackHistory *)heap_caps_calloc(MAX_SEEN_DB, sizeof(AircraftTrackHistory), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    live_fleet = (AircraftData *)heap_caps_calloc(MAX_AIRCRAFT, sizeof(AircraftData), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    temp_fleet = (AircraftData *)heap_caps_calloc(MAX_AIRCRAFT, sizeof(AircraftData), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ui_slots = (AircraftSlotUI *)heap_caps_calloc(MAX_AIRCRAFT, sizeof(AircraftSlotUI), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!track_db || !live_fleet || !temp_fleet || !ui_slots) {
        ESP_LOGE(TAG, "Nie udalo sie zaalokowac pamieci struktur w PSRAM!");
        return;
    }

    lv_display_t *disp = bsp_display_start();
    if (disp == NULL) return;
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_180);
    bsp_display_backlight_on();

    build_plane_radar_ui();

    wifi_init_sta();

    xTaskCreatePinnedToCore(adsb_worker_task, "adsb_worker", 16384, NULL, 5, NULL, 1);
}