#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"
#include "lvgl.h"

#include "aircraft_types.h"
#include "airports.h"
#include "adsb_service.h"
#include "wifi_manager.h"
#include "radar_ui.h"
#include "photo_service.h"
#include "map_tile_service.h"
#include "mqtt_service.h"
#include "i18n.h"

// Vector aircraft graphics - generated LVGL files, compiled as separate
// translation units (main/CMakeLists.txt), only declared here.
extern const lv_image_dsc_t aircraft_yellow_15;
extern const lv_image_dsc_t aircraft_yellow_20;
extern const lv_image_dsc_t aircraft_yellow_25;
extern const lv_image_dsc_t helicopter_yellow_20;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "RADAR_UI";

// Projects any lat/lon to a radar-relative screen pixel using the exact
// same Web Mercator world-pixel math map_tile_service.c uses to place the
// OSM tile background (latlon_to_tilef() there, TILE_SIZE=256), at the
// zoom level the map is actually currently rendered at - not the separate
// flat-earth (equirectangular) distance/bearing approximation the
// aircraft/airport/trail drawing below uses. Kept available here as a
// shared utility so any future caller can align pixel-for-pixel with the
// map tiles instead of drifting from them at long range / high latitude.
void radar_geo_to_screen_px(double lat, double lon, int *out_x, int *out_y) {
    int zoom = map_tile_service_get_current_zoom();
    // -1 ("no reload has completed yet") would make the shift below
    // undefined behavior - fall back to a reasonable mid-range zoom.
    if (zoom < 0) zoom = 8;
    double n = (double)(1 << zoom);

    double lon_deg = lon;
    double lat_rad = lat * M_PI / 180.0;
    double center_lon_deg = g_radar_lon;
    double center_lat_rad = g_radar_lat * M_PI / 180.0;

    double world_x = (lon_deg + 180.0) / 360.0 * n * 256.0;
    double world_y = (1.0 - asinh(tan(lat_rad)) / M_PI) / 2.0 * n * 256.0;

    double center_world_x = (center_lon_deg + 180.0) / 360.0 * n * 256.0;
    double center_world_y = (1.0 - asinh(tan(center_lat_rad)) / M_PI) / 2.0 * n * 256.0;

    *out_x = RADAR_CENTER_X + (int)round(world_x - center_world_x);
    *out_y = RADAR_CENTER_Y + (int)round(world_y - center_world_y);
}

static int current_range_idx = 0;
static air_filter_mode_t air_filter_mode = AIR_FILTER_ALL;
static bool show_airports = true;
static bool hide_ground_traffic = true;

// Pool of airport UI slots - fixed size independent of the global database
// size (airports.h), to keep 60 FPS with hundreds of database entries.
#define MAX_VISIBLE_AIRPORTS 40

// Sidebar aircraft list cards - narrower than list_cont's full 396px width,
// leaving a dedicated right-hand gutter for the scrollbar (styled at
// list_cont's LV_PART_SCROLLBAR, see radar_ui_init()) so it never overlaps
// the selection border or the vertical-rate trend symbol.
#define LIST_SCROLLBAR_GUTTER 12
#define LIST_ITEM_WIDTH       (396 - LIST_SCROLLBAR_GUTTER)

// Unified aircraft detail modal (popup_card, see radar_ui_init()) - shared
// by its creation code and radar_ui_set_aircraft_photo()'s cover-crop
// target so the hero photo always fills popup_hero edge-to-edge regardless
// of which one is read first.
#define POPUP_CARD_WIDTH   332
#define POPUP_HERO_HEIGHT  165

typedef struct {
    lv_obj_t *marker;
    lv_obj_t *label;
} AirportSlotUI;

static AirportSlotUI ui_airports[MAX_VISIBLE_AIRPORTS];

typedef struct {
    lv_obj_t *radar_vector;
    lv_point_precise_t vec_pts_live[MAX_TRACK_POINTS + 2];
    lv_obj_t *radar_icon;
    lv_obj_t *radar_label;

    lv_obj_t *list_item;
    lv_obj_t *list_icon;
    lv_obj_t *list_lbl_cs;
    lv_obj_t *list_lbl_sub;
    lv_obj_t *list_lbl_route;
    lv_obj_t *list_lbl_trend;

    // Results of the "compute" phase in radar_ui_refresh() - computed under
    // adsb_service_lock, without bsp_display_lock (incl. flight trail
    // trigonometry). Applied to LVGL objects in the short "apply" phase
    // under bsp_display_lock.
    bool     calc_visible;
    bool     calc_vec_visible;
    bool     calc_is_military;
    bool     calc_is_lpr;
    bool     calc_is_selected;
    int      calc_vec_pt_count;
    lv_point_precise_t vec_pts_calc[MAX_TRACK_POINTS + 2];
    lv_color_t calc_color;
    int      calc_px, calc_py, calc_half_w, calc_half_h;
    const lv_image_dsc_t *calc_dsc;
    int      calc_heading_x10;
    int      calc_list_y;
    char     calc_id[12];
    int      calc_altitude_ft;
    char     calc_model[8];
    float    calc_distance_km;
    char     calc_vsi_str[10];
    int      calc_vsi_fpm;
    int      calc_speed_kt;
    int      calc_heading_deg;
    float    calc_lat, calc_lon;
} AircraftSlotUI;

static AircraftSlotUI *ui_slots = NULL;
// Serializes radar_ui_refresh() calls from different tasks (adsb_worker and
// click handlers in the LVGL task), because both phases write to shared
// scratch buffers in ui_slots.
static SemaphoreHandle_t g_refresh_serialize_mutex = NULL;

static lv_obj_t *radar_area;
static lv_obj_t *list_cont;
static lv_obj_t *lbl_status_count;
static lv_obj_t *lbl_range_header;
static lv_obj_t *lbl_range_scope;
static lv_obj_t *btn_filter_ground;
static lv_obj_t *lbl_filter_ground;
static lv_obj_t *btn_airport_toggle;
static lv_obj_t *lbl_airport_toggle;
static lv_obj_t *btn_gnd_toggle;
static lv_obj_t *lbl_gnd_toggle;
static lv_obj_t *btn_map_toggle;
static lv_obj_t *lbl_map_toggle;

// Unified aircraft detail modal - one card (popup_card) with the hero photo
// (popup_hero/popup_img_preview/popup_lbl_credit) flowing directly into the
// data body (popup_body and its children) below it, no gap/separate frames.
static lv_obj_t *popup_card;
static lv_obj_t *popup_hero;
static lv_obj_t *popup_img_preview;
static lv_obj_t *popup_lbl_credit;

static lv_obj_t *popup_body;
static lv_obj_t *popup_lbl_callsign;
static lv_obj_t *popup_lbl_typereg;
static lv_obj_t *popup_lbl_col_left;
static lv_obj_t *popup_lbl_col_right;
static lv_obj_t *popup_lbl_route;

static lv_obj_t *banner_alert;
static lv_obj_t *banner_lbl;
static bool banner_visible_prev = false;

// Wi-Fi status - shown inline in status_badge's WIFI chip (dot + label),
// expanding the label text horizontally to the right instead of opening a
// separate drawer/card. status_badge is LV_ALIGN_BOTTOM_LEFT with
// LV_FLEX_FLOW_ROW + LV_SIZE_CONTENT width, so lengthening wifi_led_label's
// text alone grows the whole pill rightward along the bottom edge - it can
// never creep upward into the radar area. Tapping the WIFI dot/label
// toggles expanded/collapsed - see wifi_status_click_cb().
typedef enum {
    WIFI_NOTIFY_NONE = 0,
    WIFI_NOTIFY_AP,
    WIFI_NOTIFY_CONNECTING,
    WIFI_NOTIFY_CONNECTED,
} wifi_notify_state_t;

static lv_obj_t *wifi_led_label;
static lv_timer_t *wifi_badge_collapse_timer = NULL;
// True once radar_ui_build() has created the badge widgets - notify calls
// arriving earlier (Wi-Fi connects during wifi_manager_init(), before the
// display is even started) only update the cached state below, which is
// applied to the freshly built badge at the end of radar_ui_build().
static bool wifi_badge_ready = false;
// Whether the WIFI chip currently shows the extended text (IP/SSID/etc.)
// instead of just "WIFI" - toggled by wifi_status_click_cb(), auto-cleared
// by wifi_badge_collapse_timer_cb() 5s after a connect.
static bool s_wifi_expanded = false;

static wifi_notify_state_t s_wifi_notify_state = WIFI_NOTIFY_NONE;
static char s_wifi_notify_ssid[WIFI_SSID_MAX_LEN] = "";
static char s_wifi_notify_pass[WIFI_PASS_MAX_LEN] = "";
static char s_wifi_notify_ip[32] = "";

// MQTT status - independent twin of the WIFI chip above, same inline-
// expansion mechanism. Tapping the MQTT dot/label toggles expanded/
// collapsed - see mqtt_status_click_cb().
typedef enum {
    MQTT_NOTIFY_NONE = 0,
    MQTT_NOTIFY_CONNECTING,
    MQTT_NOTIFY_CONNECTED,
    MQTT_NOTIFY_ERROR,
} mqtt_notify_state_t;

static lv_timer_t *mqtt_badge_collapse_timer = NULL;
static bool mqtt_badge_ready = false;
static bool s_mqtt_expanded = false;
static mqtt_notify_state_t s_mqtt_notify_state = MQTT_NOTIFY_NONE;

// Modal status bubble for the sequential map-tiles -> ADS-B startup/RNG-
// change fetch (see map_tile_service.c/adsb_service.c) - top-center pill,
// same dark/green visual style as status_badge below.
typedef enum {
    LOADING_NOTIFY_NONE = 0,
    LOADING_NOTIFY_MAP,
    LOADING_NOTIFY_ADSB,
} loading_notify_state_t;

static lv_obj_t *loading_card;
static lv_obj_t *loading_card_label;
static bool loading_card_ready = false;
static loading_notify_state_t s_loading_state = LOADING_NOTIFY_NONE;
static int s_loading_map_current = 0;
static int s_loading_map_total = 0;

// HUD status badge (bottom-left of radar_area) - compact, persistent LED
// indicators for Wi-Fi and (when enabled) MQTT connection health.
static lv_obj_t *status_badge;
static lv_obj_t *wifi_led;
static lv_obj_t *mqtt_led;
static lv_obj_t *mqtt_led_label;
static bool status_badge_ready = false;
static wifi_status_t s_wifi_led_status = WIFI_STATUS_CONNECTING;
static bool s_mqtt_led_enabled = false;
static mqtt_conn_status_t s_mqtt_led_status = MQTT_STATUS_DISABLED;

// Firmware update indicator - third badge chip (dot + "FW" label, built the
// same way as wifi_led/mqtt_led above), hidden unless ota_update_service.c
// has found a newer release.
static lv_obj_t *fw_led;
static lv_obj_t *fw_led_label;
static lv_obj_t *fw_update_toast;
static lv_obj_t *fw_update_toast_label;
static lv_timer_t *fw_update_toast_timer = NULL;
static bool s_fw_update_available = false;
static char s_fw_update_version[16] = "";
static char s_fw_update_notes[128] = "";

// Emergency squawk code priority: 7500 (hijack) > 7700 (general emergency) >
// 7600 (radio failure). Returns 0 when the code is not an emergency.
static int squawk_emergency_rank(const char *squawk, const char **out_label) {
    if (!squawk) return 0;
    if (strcmp(squawk, "7500") == 0) { if (out_label) *out_label = "HIJACK"; return 3; }
    if (strcmp(squawk, "7700") == 0) { if (out_label) *out_label = "EMERGENCY"; return 2; }
    if (strcmp(squawk, "7600") == 0) { if (out_label) *out_label = "RADIO FAILURE"; return 1; }
    return 0;
}

static void squawk_banner_pulse_cb(void *obj, int32_t v) {
    lv_color_t c = lv_color_mix(lv_color_hex(0xFF1E27), lv_color_hex(0x990000), (lv_opa_t)v);
    lv_obj_set_style_bg_color((lv_obj_t *)obj, c, 0);
}

static void start_banner_pulse(lv_obj_t *obj) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, squawk_banner_pulse_cb);
    lv_anim_set_values(&a, 0, 255);
    lv_anim_set_time(&a, 500);
    lv_anim_set_playback_time(&a, 500);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static void stop_banner_pulse(lv_obj_t *obj) {
    lv_anim_del(obj, squawk_banner_pulse_cb);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x990000), 0);
}

// Aircraft photo from Planespotters.net (photo_service) - RGB565 buffer in
// PSRAM, owned by radar_ui. popup_photo_hex tracks which hex it was
// requested/loaded for, so it isn't re-requested on every radar_ui_refresh().
static const lv_image_dsc_t *popup_photo_dsc = NULL;
static char popup_photo_hex[8] = "";

static void popup_photo_release(void) {
    if (popup_photo_dsc) {
        heap_caps_free((void *)popup_photo_dsc->data);
        heap_caps_free((void *)popup_photo_dsc);
        popup_photo_dsc = NULL;
    }
}

// Pulsing (blinking) "FETCHING PHOTO..." status text in yellow, for the
// duration of the photo_service request.
static void loading_pulse_anim_cb(void *obj, int32_t v) {
    lv_obj_set_style_text_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

static void start_loading_pulse(lv_obj_t *label) {
    lv_obj_set_style_text_color(label, lv_color_hex(0xfbbf24), 0);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, label);
    lv_anim_set_exec_cb(&a, loading_pulse_anim_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_30);
    lv_anim_set_time(&a, 650);
    lv_anim_set_playback_time(&a, 650);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static void stop_loading_pulse(lv_obj_t *label) {
    lv_anim_del(label, loading_pulse_anim_cb);
    lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xCCCCCC), 0);
}

static char selected_hex[8] = "";
static char selected_callsign[12] = "";
static char selected_reg[16] = "";
static char selected_model[8] = "";

// Set for exactly one radar_ui_refresh() call after the user directly
// selects a new aircraft (map icon or list row tap) - consumed and cleared
// in the apply phase below, once the selected row's list_item is
// positioned and scrolled into view. Left false on every periodic
// background refresh (ADS-B poll) so re-sorting/re-positioning the list
// while the same aircraft stays selected never yanks the view away from
// whatever the user is currently browsing.
static bool s_scroll_to_selected_pending = false;

static inline void lv_obj_set_hidden(lv_obj_t *obj, bool hidden) {
    if (!obj) return;
    if (hidden) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

// High-contrast blue for LPR (Polish Medical Air Rescue) helicopters
#define COLOR_LPR lv_color_hex(0x0096FF)

static bool is_lpr_callsign(const char *callsign) {
    return strncmp(callsign, "LPR", 3) == 0 || strncmp(callsign, "RAT", 3) == 0;
}

static lv_color_t get_fr24_altitude_color(int alt_ft) {
    if(alt_ft < 3000)  return lv_color_hex(0xFF2A6D);
    if(alt_ft < 10000) return lv_color_hex(0xFF9900);
    if(alt_ft < 25000) return lv_color_hex(0x00D2FF);
    if(alt_ft < 33000) return lv_color_hex(0x76FF03);
    return lv_color_hex(0xFFFF55);
}

static const lv_image_dsc_t* get_aircraft_dsc(AircraftType type) {
    if(type == AC_HELI)  return &helicopter_yellow_20;
    if(type == AC_SMALL) return &aircraft_yellow_15;
    if(type == AC_LARGE) return &aircraft_yellow_25;
    return &aircraft_yellow_20;
}

// Sidebar list ordering: military contacts always above civilian ones;
// within the same category, nearest first.
static int list_order_cmp(const void *a, const void *b) {
    int ia = *(const int *)a;
    int ib = *(const int *)b;
    if (wifi_mgr_get_mil_priority_enabled()) {
        bool mil_a = ui_slots[ia].calc_is_military;
        bool mil_b = ui_slots[ib].calc_is_military;
        if (mil_a != mil_b) return mil_a ? -1 : 1;
    }
    float da = ui_slots[ia].calc_distance_km;
    float db = ui_slots[ib].calc_distance_km;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

// ================= AIRCRAFT UI UPDATE =================
void radar_ui_refresh(void) {
    if (!live_fleet || !ui_slots || !g_refresh_serialize_mutex) return;
    // Bounded timeout (not portMAX_DELAY): radar_ui_refresh() is also called
    // from click handlers in the LVGL task, which already holds lvgl_mux at
    // that point - an unbounded wait here could stall it for the duration of
    // a concurrent refresh from adsb_worker_task.
    if (xSemaphoreTake(g_refresh_serialize_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

    if (!adsb_service_lock(100)) {
        xSemaphoreGive(g_refresh_serialize_mutex);
        return;
    }

    uint32_t now = get_time_ms();
    float current_range_km = range_steps[current_range_idx];
    aircraft_click_action_t click_action = (aircraft_click_action_t)wifi_mgr_get_click_action();
    uint16_t max_aircraft = wifi_mgr_get_max_aircraft();
    if (max_aircraft > MAX_AIRCRAFT_CAPACITY) max_aircraft = MAX_AIRCRAFT_CAPACITY;

    int visible_count = 0;
    AircraftData *selected_ac = NULL;

    // ---- "compute" phase: trigonometry and label data, only under
    // adsb_service_lock, without bsp_display_lock - to hold the LVGL mutex
    // for as short as possible. The loop runs over the full static buffer
    // capacity (MAX_AIRCRAFT_CAPACITY), but slots >= max_aircraft are
    // immediately rejected (is_valid=false), so the "apply" phase below can
    // correctly hide them if the limit was lowered since the last refresh. ----
    for(int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
        bool is_valid = (i < max_aircraft) && live_fleet[i].active && ((now - live_fleet[i].last_seen_ms) < PLANE_TIMEOUT_MS);
        if (hide_ground_traffic && live_fleet[i].on_ground) is_valid = false;
        if (air_filter_mode == AIR_FILTER_CIVIL && live_fleet[i].is_military) is_valid = false;
        if (air_filter_mode == AIR_FILTER_MIL && !live_fleet[i].is_military) is_valid = false;

        if(is_valid && live_fleet[i].distance_km <= current_range_km) {
            visible_count++;
            if(selected_hex[0] != '\0' && strcmp(selected_hex, live_fleet[i].hex) == 0) {
                selected_ac = &live_fleet[i];
            }

            ui_slots[i].calc_visible = true;

            // Same Web Mercator projection (and current tile zoom) the map
            // background is rendered with, so aircraft always line up with
            // the map underneath them instead of drifting from it at long
            // range or high latitude under a separate flat-earth estimate.
            int px, py;
            radar_geo_to_screen_px(live_fleet[i].lat, live_fleet[i].lon, &px, &py);
            ui_slots[i].calc_px = px;
            ui_slots[i].calc_py = py;

            const lv_image_dsc_t *dsc = get_aircraft_dsc(live_fleet[i].type);
            ui_slots[i].calc_dsc = dsc;
            ui_slots[i].calc_half_w = (int)dsc->header.w / 2;
            ui_slots[i].calc_half_h = (int)dsc->header.h / 2;
            ui_slots[i].calc_heading_x10 = live_fleet[i].heading_deg * 10;
            ui_slots[i].calc_is_military = live_fleet[i].is_military;
            ui_slots[i].calc_is_lpr = is_lpr_callsign(live_fleet[i].callsign);
            ui_slots[i].calc_color = live_fleet[i].is_military ? lv_color_hex(0xFF3333)
                                    : ui_slots[i].calc_is_lpr    ? COLOR_LPR
                                                                  : get_fr24_altitude_color(live_fleet[i].altitude_ft);

            snprintf(ui_slots[i].calc_id, sizeof(ui_slots[i].calc_id), "%s", live_fleet[i].callsign);
            ui_slots[i].calc_altitude_ft = live_fleet[i].altitude_ft;
            snprintf(ui_slots[i].calc_model, sizeof(ui_slots[i].calc_model), "%s", live_fleet[i].model);
            ui_slots[i].calc_distance_km = live_fleet[i].distance_km;
            snprintf(ui_slots[i].calc_vsi_str, sizeof(ui_slots[i].calc_vsi_str), "%s", live_fleet[i].vsi_str);
            ui_slots[i].calc_vsi_fpm = live_fleet[i].vsi_fpm;
            ui_slots[i].calc_speed_kt = live_fleet[i].speed_kt;
            ui_slots[i].calc_heading_deg = live_fleet[i].heading_deg;
            ui_slots[i].calc_lat = live_fleet[i].lat;
            ui_slots[i].calc_lon = live_fleet[i].lon;

            ui_slots[i].calc_is_selected = selected_hex[0] != '\0' && strcmp(selected_hex, live_fleet[i].hex) == 0;

            // Flight trace line (AIRCRAFT_CLICK_FLIGHT_TRACE) - only ever
            // drawn for the selected aircraft, from the asynchronously
            // fetched g_trace_points buffer (adsb_service.c). Downsampled to
            // MAX_TRACK_POINTS if the API returned more points than that.
            if (ui_slots[i].calc_is_selected && click_action == AIRCRAFT_CLICK_FLIGHT_TRACE &&
                g_trace_point_count > 0 && strcmp(g_trace_hex, live_fleet[i].hex) == 0) {
                int n = g_trace_point_count;
                int step = (n > MAX_TRACK_POINTS) ? (n + MAX_TRACK_POINTS - 1) / MAX_TRACK_POINTS : 1;
                int pts_to_draw = 0;
                for (int p = 0; p < n && pts_to_draw < MAX_TRACK_POINTS; p += step) {
                    int tp_x, tp_y;
                    radar_geo_to_screen_px(g_trace_points[p].lat, g_trace_points[p].lon, &tp_x, &tp_y);
                    ui_slots[i].vec_pts_calc[pts_to_draw].x = tp_x;
                    ui_slots[i].vec_pts_calc[pts_to_draw].y = tp_y;
                    pts_to_draw++;
                }
                ui_slots[i].vec_pts_calc[pts_to_draw].x = px;
                ui_slots[i].vec_pts_calc[pts_to_draw].y = py;
                ui_slots[i].calc_vec_pt_count = pts_to_draw + 1;
                ui_slots[i].calc_vec_visible = true;
            } else {
                ui_slots[i].calc_vec_visible = false;
            }
        } else {
            ui_slots[i].calc_visible = false;
        }
    }

    // Sidebar list order: military contacts first, then by distance ascending.
    {
        int list_order[MAX_AIRCRAFT_CAPACITY];
        int list_order_count = 0;
        for (int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
            if (ui_slots[i].calc_visible) list_order[list_order_count++] = i;
        }
        qsort(list_order, list_order_count, sizeof(int), list_order_cmp);
        for (int k = 0; k < list_order_count; k++) {
            ui_slots[list_order[k]].calc_list_y = k * 65;
        }
    }

    // Emergency squawk alert - independent of the AIR/GND/range filters,
    // computed over the whole fleet of active aircraft.
    bool emg_active = false;
    int emg_rank = 0;
    const char *emg_label = "";
    char emg_squawk[8] = "";
    char emg_id[12] = "";
    int emg_alt = 0;
    float emg_dist = 0.0f;
    for (int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
        if (!live_fleet[i].active || (now - live_fleet[i].last_seen_ms) >= PLANE_TIMEOUT_MS) continue;
        const char *lbl = NULL;
        int rank = squawk_emergency_rank(live_fleet[i].squawk, &lbl);
        if (rank > emg_rank) {
            emg_rank = rank;
            emg_active = true;
            emg_label = lbl;
            snprintf(emg_squawk, sizeof(emg_squawk), "%s", live_fleet[i].squawk);
            snprintf(emg_id, sizeof(emg_id), "%s", live_fleet[i].callsign);
            emg_alt = live_fleet[i].altitude_ft;
            emg_dist = live_fleet[i].distance_km;
        }
    }

    bool has_selection = (selected_ac != NULL);
    char sel_hex[8] = "";
    char sel_id[12] = "";
    char sel_model[8] = "";
    char sel_reg[16] = "";
    char sel_vsi[10] = "";
    char sel_squawk[8] = "";
    int sel_alt = 0, sel_spd = 0, sel_hdg = 0;
    float sel_dist = 0.0f, sel_lat = 0.0f, sel_lon = 0.0f;
    const lv_image_dsc_t *sel_dsc = NULL;

    if (has_selection) {
        snprintf(sel_hex, sizeof(sel_hex), "%s", selected_ac->hex);
        snprintf(sel_id, sizeof(sel_id), "%s", selected_ac->callsign);
        snprintf(sel_model, sizeof(sel_model), "%s", selected_ac->model);
        snprintf(sel_reg, sizeof(sel_reg), "%s", selected_ac->registration);
        snprintf(sel_vsi, sizeof(sel_vsi), "%s", selected_ac->vsi_str);
        snprintf(sel_squawk, sizeof(sel_squawk), "%s", selected_ac->squawk);
        sel_alt = selected_ac->altitude_ft;
        sel_spd = selected_ac->speed_kt;
        sel_hdg = selected_ac->heading_deg;
        sel_dist = selected_ac->distance_km;
        sel_lat = selected_ac->lat;
        sel_lon = selected_ac->lon;
        sel_dsc = get_aircraft_dsc(selected_ac->type);
    }

    adsb_service_unlock();

    // ---- "apply" phase: only LVGL setters on ready data, short bsp_display_lock. ----
    if (!bsp_display_lock(200)) {
        ESP_LOGW(TAG, "radar_ui_refresh: bsp_display_lock timeout, skipping frame");
        xSemaphoreGive(g_refresh_serialize_mutex);
        return;
    }

    // Airport positioning: first a fast bounding-box filter (lat/lon
    // deviation computed from the current radar range), only then do
    // candidates get an exact distance/bearing and pixel position
    // computed. This way the global airport database (airports.h) doesn't
    // weigh down rendering - we draw at most MAX_VISIBLE_AIRPORTS objects,
    // strictly within radar range.
    int airport_slot = 0;
    if (show_airports) {
        uint8_t apt_mask = wifi_mgr_get_apt_filter_mask();
        float dlat_max = current_range_km / 111.132f;
        float coslat = fabsf(cosf(g_radar_lat * DEG_TO_RAD));
        float dlon_max = current_range_km / (111.132f * (coslat > 0.01f ? coslat : 0.01f));

        for (size_t i = 0; i < GLOBAL_AIRPORTS_COUNT && airport_slot < MAX_VISIBLE_AIRPORTS; i++) {
            const airport_t *ap = &global_airports[i];
            if (!(ap->type & apt_mask)) continue;
            if (fabsf(ap->lat - g_radar_lat) > dlat_max) continue;
            if (fabsf(ap->lon - g_radar_lon) > dlon_max) continue;

            float dist_km, bearing_deg;
            calculate_coords(ap->lat, ap->lon, &dist_km, &bearing_deg);
            if (dist_km > current_range_km) continue;

            int px, py;
            radar_geo_to_screen_px(ap->lat, ap->lon, &px, &py);
            if (px < 10 || px > (MAP_SIZE - 10) || py < 10 || py > (MAP_SIZE - 10)) continue;

            AirportSlotUI *slot = &ui_airports[airport_slot++];
            lv_label_set_text_fmt(slot->label, "%s\n%s", ap->icao, ap->name);
            lv_obj_set_pos(slot->marker, px - 3, py - 3);
            lv_obj_set_pos(slot->label, px + 6, py - 6);
            lv_obj_set_hidden(slot->marker, false);
            lv_obj_set_hidden(slot->label, false);
        }
    }
    for (; airport_slot < MAX_VISIBLE_AIRPORTS; airport_slot++) {
        lv_obj_set_hidden(ui_airports[airport_slot].marker, true);
        lv_obj_set_hidden(ui_airports[airport_slot].label, true);
    }

    for(int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
        if (!ui_slots[i].calc_visible) {
            lv_obj_set_hidden(ui_slots[i].radar_vector, true);
            lv_obj_set_hidden(ui_slots[i].radar_icon, true);
            lv_obj_set_hidden(ui_slots[i].radar_label, true);
            lv_obj_set_hidden(ui_slots[i].list_item, true);
            continue;
        }

        if (ui_slots[i].calc_vec_visible) {
            memcpy(ui_slots[i].vec_pts_live, ui_slots[i].vec_pts_calc, sizeof(ui_slots[i].vec_pts_live));
            lv_obj_set_style_line_color(ui_slots[i].radar_vector, ui_slots[i].calc_color, 0);
            lv_line_set_points(ui_slots[i].radar_vector, ui_slots[i].vec_pts_live, ui_slots[i].calc_vec_pt_count);
            lv_obj_set_hidden(ui_slots[i].radar_vector, false);
        } else {
            lv_obj_set_hidden(ui_slots[i].radar_vector, true);
        }

        lv_obj_set_hidden(ui_slots[i].radar_icon, false);
        lv_obj_set_hidden(ui_slots[i].radar_label, false);

        lv_image_set_src(ui_slots[i].radar_icon, ui_slots[i].calc_dsc);
        lv_image_set_pivot(ui_slots[i].radar_icon, ui_slots[i].calc_half_w, ui_slots[i].calc_half_h);
        lv_obj_set_style_image_recolor(ui_slots[i].radar_icon,
            ui_slots[i].calc_is_military ? lv_color_hex(0xFF3333) : COLOR_LPR, 0);
        lv_obj_set_style_image_recolor_opa(ui_slots[i].radar_icon,
            (ui_slots[i].calc_is_military || ui_slots[i].calc_is_lpr) ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        lv_obj_set_pos(ui_slots[i].radar_icon, ui_slots[i].calc_px - ui_slots[i].calc_half_w, ui_slots[i].calc_py - ui_slots[i].calc_half_h);
        lv_obj_set_pos(ui_slots[i].radar_label, ui_slots[i].calc_px + ui_slots[i].calc_half_w + 6, ui_slots[i].calc_py - ui_slots[i].calc_half_h);
        lv_image_set_rotation(ui_slots[i].radar_icon, ui_slots[i].calc_heading_x10);

        // Second line always shows altitude; ICAO type code is appended when known
        // (aircraft->model, e.g. "C152", "F16") so map labels stay consistent.
        if (ui_slots[i].calc_model[0] != '\0') {
            lv_label_set_text_fmt(ui_slots[i].radar_label, "%s\n%d ft  %s",
                                   ui_slots[i].calc_id, ui_slots[i].calc_altitude_ft, ui_slots[i].calc_model);
        } else {
            lv_label_set_text_fmt(ui_slots[i].radar_label, "%s\n%d ft",
                                   ui_slots[i].calc_id, ui_slots[i].calc_altitude_ft);
        }
        lv_obj_set_style_text_color(ui_slots[i].radar_label, ui_slots[i].calc_color, 0);

        lv_obj_set_hidden(ui_slots[i].list_item, false);
        lv_obj_set_pos(ui_slots[i].list_item, 0, ui_slots[i].calc_list_y);

        // Selection highlight - shown for the clicked aircraft (radar icon
        // or this list row) whenever clicks are not disabled, regardless of
        // which click action (popup/trace) is active.
        if (ui_slots[i].calc_is_selected) {
            // Subtle 1px cyan outline instead of the old thick/bright 2px
            // blue border - a faint dark-green background tint keeps the
            // row readable without the heavy border doing all the work.
            lv_obj_set_style_bg_opa(ui_slots[i].list_item, LV_OPA_30, 0);
            lv_obj_set_style_bg_color(ui_slots[i].list_item, lv_color_hex(0x10241b), 0);
            lv_obj_set_style_border_width(ui_slots[i].list_item, 1, 0);
            lv_obj_set_style_border_color(ui_slots[i].list_item, lv_color_hex(0x26c6da), 0);
            lv_obj_set_style_radius(ui_slots[i].list_item, 6, 0);

            // Auto-scroll the sidebar to the just-selected aircraft - only
            // once per direct user selection (see s_scroll_to_selected_pending),
            // never on a periodic background refresh. list_item is already
            // positioned (lv_obj_set_pos above) and unhidden at this point,
            // so this is safe even if the row just got created/repositioned
            // by this same refresh pass - LVGL recalculates layout/scroll
            // extents synchronously on lv_obj_set_pos, not lazily.
            if (s_scroll_to_selected_pending) {
                lv_obj_scroll_to_view(ui_slots[i].list_item, LV_ANIM_ON);
                s_scroll_to_selected_pending = false;
            }
        } else {
            lv_obj_set_style_bg_opa(ui_slots[i].list_item, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(ui_slots[i].list_item, 0, 0);
        }

        lv_image_set_src(ui_slots[i].list_icon, ui_slots[i].calc_dsc);
        lv_image_set_pivot(ui_slots[i].list_icon, ui_slots[i].calc_half_w, ui_slots[i].calc_half_h);
        lv_image_set_rotation(ui_slots[i].list_icon, ui_slots[i].calc_heading_x10);
        lv_obj_set_style_image_recolor(ui_slots[i].list_icon,
            ui_slots[i].calc_is_military ? lv_color_hex(0xFF3333) : COLOR_LPR, 0);
        lv_obj_set_style_image_recolor_opa(ui_slots[i].list_icon,
            (ui_slots[i].calc_is_military || ui_slots[i].calc_is_lpr) ? LV_OPA_COVER : LV_OPA_TRANSP, 0);

        if (ui_slots[i].calc_is_military) {
            lv_label_set_text_fmt(ui_slots[i].list_lbl_cs, "%s [MIL]", ui_slots[i].calc_id);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_cs, lv_color_hex(0xFF3333), 0);
        } else {
            lv_label_set_text(ui_slots[i].list_lbl_cs, ui_slots[i].calc_id);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_cs, lv_color_hex(0xffffff), 0);
        }
        // Fixed 4-char field for the ICAO type code so this column never
        // shifts the KM/FT/KT fields that follow it, model unknown or not.
        const char *model_disp = ui_slots[i].calc_model[0] != '\0' ? ui_slots[i].calc_model : "----";
        lv_label_set_text_fmt(ui_slots[i].list_lbl_sub, "%-4.4s  %.1fKM  %d FT  %s  %dKT",
                              model_disp, ui_slots[i].calc_distance_km, ui_slots[i].calc_altitude_ft,
                              ui_slots[i].calc_vsi_str, ui_slots[i].calc_speed_kt);
        lv_label_set_text_fmt(ui_slots[i].list_lbl_route, "HDG: %d deg  LAT: %.2f  LON: %.2f",
                              ui_slots[i].calc_heading_deg, ui_slots[i].calc_lat, ui_slots[i].calc_lon);

        if (ui_slots[i].calc_vsi_fpm > 500) {
            lv_label_set_text(ui_slots[i].list_lbl_trend, LV_SYMBOL_UP);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_trend, lv_color_hex(0x2ecc71), 0);
        } else if (ui_slots[i].calc_vsi_fpm < -500) {
            lv_label_set_text(ui_slots[i].list_lbl_trend, LV_SYMBOL_DOWN);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_trend, lv_color_hex(0xFF9900), 0);
        } else {
            lv_label_set_text(ui_slots[i].list_lbl_trend, LV_SYMBOL_MINUS);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_trend, lv_color_hex(0x888888), 0);
        }
    }

    // Re-clamp list_cont's scroll offset to the (possibly now shorter)
    // content height - LVGL only auto-adjusts scroll position on layout
    // events, not on manual lv_obj_set_pos()/hidden-flag changes to
    // absolutely-positioned children (this list has no flex layout), so a
    // list scrolled down that then shrinks (aircraft leaving range/timeout)
    // would otherwise stay scrolled past the new bottom, leaving dead space
    // under the last card until the user manually scrolls back up.
    lv_obj_readjust_scroll(list_cont, LV_ANIM_OFF);

    // Safety net: if the selected aircraft's row was not visible this pass
    // (e.g. it dropped out of range/filter between the click and this
    // refresh, or its list_item hasn't been laid out yet), drop the pending
    // scroll instead of retrying it on a later background refresh, which
    // would yank the view out from under the user unexpectedly.
    s_scroll_to_selected_pending = false;

    lv_label_set_text_fmt(lbl_status_count, T(STR_TRAFFIC_FMT), visible_count, (int)current_range_km);

    if (has_selection && click_action == AIRCRAFT_CLICK_DETAILS_PHOTO) {
        lv_obj_set_hidden(popup_card, false);

        if (strcmp(popup_photo_hex, sel_hex) != 0) {
            popup_photo_release();
            snprintf(popup_photo_hex, sizeof(popup_photo_hex), "%s", sel_hex);

            lv_image_set_src(popup_img_preview, sel_dsc);
            lv_image_set_scale(popup_img_preview, 256);
            lv_image_set_rotation(popup_img_preview, 0);
            lv_label_set_text(popup_lbl_credit, T(STR_FETCHING_PHOTO));
            start_loading_pulse(popup_lbl_credit);

            photo_service_request(sel_hex, sel_reg, sel_model);
        }

        lv_label_set_text(popup_lbl_callsign, sel_id);
        lv_label_set_text_fmt(popup_lbl_typereg, "%s  %s", sel_model, sel_reg);

        // #hex text# spans (lv_label_set_recolor() enabled at creation) give
        // the muted-label/bright-value hierarchy without a widget per field.
        lv_label_set_text_fmt(popup_lbl_col_left,
            "#8094a5 ALT#  #ffffff %d ft#\n"
            "#8094a5 SPD#  #ffffff %d kt#\n"
            "#8094a5 DIST# #ffffff %.1f km#",
            sel_alt, sel_spd, sel_dist);

        lv_label_set_text_fmt(popup_lbl_col_right,
            "#8094a5 V/S#  #ffffff %s#\n"
            "#8094a5 HDG#  #ffffff %03d#\n"
            "#8094a5 SQK#  #ffffff %s#",
            sel_vsi, sel_hdg, sel_squawk);

        lv_label_set_text_fmt(popup_lbl_route, "LAT: %.2f  LON: %.2f  (%s)", sel_lat, sel_lon, g_station_name);
    } else {
        lv_obj_set_hidden(popup_card, true);
        popup_photo_release();
        stop_loading_pulse(popup_lbl_credit);
        popup_photo_hex[0] = '\0';
    }

    bool show_banner = emg_active && wifi_mgr_get_squawk_alert_enabled();
    if (show_banner) {
        lv_label_set_text_fmt(banner_lbl,
            "\xE2\x9A\xA0\xEF\xB8\x8F SQUAWK %s (%s): %s \xE2\x80\xA2 %d FT \xE2\x80\xA2 %.0fKM",
            emg_squawk, emg_label, emg_id, emg_alt, emg_dist);
        lv_obj_clear_flag(banner_alert, LV_OBJ_FLAG_HIDDEN);
        if (!banner_visible_prev) start_banner_pulse(banner_alert);
    } else {
        lv_obj_add_flag(banner_alert, LV_OBJ_FLAG_HIDDEN);
        if (banner_visible_prev) stop_banner_pulse(banner_alert);
    }
    banner_visible_prev = show_banner;

    bsp_display_unlock();
    xSemaphoreGive(g_refresh_serialize_mutex);
}

// ================= BUTTON EVENTS =================
static void apply_range_index(int idx) {
    current_range_idx = idx;
    float current_range = range_steps[current_range_idx];

    bsp_display_lock(0);
    lv_label_set_text_fmt(lbl_range_header, "RNG: %.0fKM", current_range);
    lv_label_set_text_fmt(lbl_range_scope, "%.0fKM", current_range);
    bsp_display_unlock();

    wifi_mgr_set_default_range((uint16_t)current_range);
    map_tile_service_request_reload(g_radar_lat, g_radar_lon, current_range);
    radar_ui_refresh();
    mqtt_service_publish_state();
}

static void switch_range(void) {
    if (map_tile_is_downloading()) {
        // Ignore the click outright instead of queuing/debouncing it - a
        // range change mid-download would supersede the in-flight tile grid
        // and restart fetching, on top of an already-active TLS session.
        ESP_LOGW(TAG, "RNG change blocked - map tiles downloading");
        return;
    }
    apply_range_index((current_range_idx + 1) % NUM_RANGE_STEPS);
}

static void range_click_event_cb(lv_event_t *e) {
    switch_range();
}

void radar_ui_set_range_km(float km) {
    int best_idx = 0;
    float best_diff = 1e9f;
    for (size_t i = 0; i < NUM_RANGE_STEPS; i++) {
        float diff = fabsf(range_steps[i] - km);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = (int)i;
        }
    }
    apply_range_index(best_idx);
}

float radar_ui_get_range_km(void) {
    return range_steps[current_range_idx];
}

static void apply_air_filter_style(void) {
    if (air_filter_mode == AIR_FILTER_MIL) {
        lv_obj_set_style_bg_color(btn_filter_ground, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_filter_ground, lv_color_hex(0xFF3333), 0);
        lv_obj_set_style_text_color(lbl_filter_ground, lv_color_hex(0xFF3333), 0);
    } else {
        lv_obj_set_style_bg_color(btn_filter_ground, air_filter_mode == AIR_FILTER_CIVIL ? lv_color_hex(0x005522) : lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_filter_ground, air_filter_mode == AIR_FILTER_CIVIL ? lv_color_hex(0x00ff88) : lv_color_hex(0x005522), 0);
        lv_obj_set_style_text_color(lbl_filter_ground, lv_color_hex(0x00ff88), 0);
    }
    switch (air_filter_mode) {
        case AIR_FILTER_CIVIL: lv_label_set_text(lbl_filter_ground, "AIR: CIVIL"); break;
        case AIR_FILTER_MIL:   lv_label_set_text(lbl_filter_ground, "AIR: MIL"); break;
        default:               lv_label_set_text(lbl_filter_ground, "AIR: ALL"); break;
    }
}

static void set_air_filter_internal(air_filter_mode_t mode) {
    air_filter_mode = mode;
    bsp_display_lock(0);
    apply_air_filter_style();
    bsp_display_unlock();

    wifi_mgr_set_default_air_mode((uint8_t)mode);
    radar_ui_refresh();
    mqtt_service_publish_state();
}

static void filter_click_event_cb(lv_event_t *e) {
    set_air_filter_internal((air_filter_mode + 1) % 3);
}

void radar_ui_set_air_filter(air_filter_mode_t mode) {
    if (mode > AIR_FILTER_MIL) mode = AIR_FILTER_ALL;
    set_air_filter_internal(mode);
}

air_filter_mode_t radar_ui_get_air_filter(void) {
    return air_filter_mode;
}

static void apply_airport_toggle_style(void) {
    if (show_airports) {
        lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0x005522), 0);
        lv_label_set_text(lbl_airport_toggle, "APTS: ON");
    } else {
        lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0xff5555), 0);
        lv_label_set_text(lbl_airport_toggle, "APTS: OFF");
    }
}

static void set_airports_enabled_internal(bool on) {
    show_airports = on;
    bsp_display_lock(0);
    apply_airport_toggle_style();
    bsp_display_unlock();

    wifi_mgr_set_default_apts_mode(on);
    radar_ui_refresh();
    mqtt_service_publish_state();
}

static void airport_toggle_click_event_cb(lv_event_t *e) {
    set_airports_enabled_internal(!show_airports);
}

void radar_ui_set_airports_enabled(bool on) {
    set_airports_enabled_internal(on);
}

bool radar_ui_get_airports_enabled(void) {
    return show_airports;
}

static void apply_gnd_filter_style(void) {
    if (hide_ground_traffic) {
        lv_obj_set_style_bg_color(btn_gnd_toggle, lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_gnd_toggle, lv_color_hex(0x005522), 0);
        lv_obj_set_style_text_color(lbl_gnd_toggle, lv_color_hex(0x00ff88), 0);
        lv_label_set_text(lbl_gnd_toggle, "GND: OFF");
    } else {
        lv_obj_set_style_bg_color(btn_gnd_toggle, lv_color_hex(0x005522), 0);
        lv_obj_set_style_border_color(btn_gnd_toggle, lv_color_hex(0x00ff88), 0);
        lv_obj_set_style_text_color(lbl_gnd_toggle, lv_color_hex(0x00ff88), 0);
        lv_label_set_text(lbl_gnd_toggle, "GND: ON");
    }
}

static void set_hide_ground_internal(bool hide) {
    hide_ground_traffic = hide;
    bsp_display_lock(0);
    apply_gnd_filter_style();
    bsp_display_unlock();

    wifi_mgr_set_hide_ground(hide);
    radar_ui_refresh();
    mqtt_service_publish_state();
}

static void gnd_toggle_click_event_cb(lv_event_t *e) {
    set_hide_ground_internal(!hide_ground_traffic);
}

void radar_ui_set_show_ground(bool show) {
    set_hide_ground_internal(!show);
}

bool radar_ui_get_show_ground(void) {
    return !hide_ground_traffic;
}

static void apply_map_toggle_style(void) {
    bool map_on = map_tile_service_is_enabled();
    if (map_on) {
        lv_obj_set_style_bg_color(btn_map_toggle, lv_color_hex(0x002b11), 0);
        lv_obj_set_style_border_color(btn_map_toggle, lv_color_hex(0x005522), 0);
        lv_obj_set_style_text_color(lbl_map_toggle, lv_color_hex(0x00ff88), 0);
        lv_label_set_text(lbl_map_toggle, "MAP: ON");
    } else {
        lv_obj_set_style_bg_color(btn_map_toggle, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_map_toggle, lv_color_hex(0xff5555), 0);
        lv_obj_set_style_text_color(lbl_map_toggle, lv_color_hex(0xff5555), 0);
        lv_label_set_text(lbl_map_toggle, "MAP: OFF");
    }
}

static void set_map_enabled_internal(bool on) {
    map_tile_service_set_enabled(on);
    bsp_display_lock(0);
    apply_map_toggle_style();
    bsp_display_unlock();

    wifi_mgr_set_map_enabled(on);
    mqtt_service_publish_state();
}

static void map_toggle_click_event_cb(lv_event_t *e) {
    set_map_enabled_internal(!map_tile_service_is_enabled());
}

void radar_ui_set_map_enabled(bool on) {
    set_map_enabled_internal(on);
}

bool radar_ui_get_map_enabled(void) {
    return map_tile_service_is_enabled();
}

static void select_aircraft_by_index(int idx) {
    aircraft_click_action_t action = (aircraft_click_action_t)wifi_mgr_get_click_action();
    if (action == AIRCRAFT_CLICK_DISABLED) return;

    if (!adsb_service_lock(100)) return;

    bool need_refresh = false;
    bool deselected = false;
    char new_hex[8] = "";
    if(live_fleet && idx >= 0 && idx < MAX_AIRCRAFT_CAPACITY && live_fleet[idx].active) {
        if(strcmp(selected_hex, live_fleet[idx].hex) == 0) {
            selected_hex[0] = '\0';
            selected_callsign[0] = '\0';
            selected_reg[0] = '\0';
            selected_model[0] = '\0';
            deselected = true;
        } else {
            snprintf(selected_hex, sizeof(selected_hex), "%s", live_fleet[idx].hex);
            snprintf(selected_callsign, sizeof(selected_callsign), "%s", live_fleet[idx].callsign);
            snprintf(selected_reg, sizeof(selected_reg), "%s", live_fleet[idx].registration);
            snprintf(selected_model, sizeof(selected_model), "%s", live_fleet[idx].model);
            snprintf(new_hex, sizeof(new_hex), "%s", live_fleet[idx].hex);
            // Direct user selection (map icon or list row tap) - scroll the
            // sidebar to this aircraft on the very next refresh, see
            // s_scroll_to_selected_pending.
            s_scroll_to_selected_pending = true;
        }
        need_refresh = true;
    }

    adsb_service_unlock();

    if (deselected) {
        adsb_service_clear_trace();
    } else if (new_hex[0] != '\0' && action == AIRCRAFT_CLICK_FLIGHT_TRACE) {
        adsb_service_request_trace(new_hex);
    }

    if (need_refresh) radar_ui_refresh();
}

static void aircraft_click_event_cb(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    select_aircraft_by_index(idx);
}

void radar_ui_set_aircraft_photo(const lv_image_dsc_t *img_dsc, const char *photographer, const char *hex, bool is_type_fallback) {
    if (!bsp_display_lock(500)) {
        ESP_LOGW(TAG, "radar_ui_set_aircraft_photo: bsp_display_lock timeout, discarding result");
        if (img_dsc) {
            heap_caps_free((void *)img_dsc->data);
            heap_caps_free((void *)img_dsc);
        }
        return;
    }

    // The user already closed the popup or selected another aircraft before
    // the photo finished fetching - the result is now stale.
    if (strcmp(popup_photo_hex, hex) != 0) {
        bsp_display_unlock();
        if (img_dsc) {
            heap_caps_free((void *)img_dsc->data);
            heap_caps_free((void *)img_dsc);
        }
        return;
    }

    popup_photo_release();
    stop_loading_pulse(popup_lbl_credit);

    if (img_dsc) {
        popup_photo_dsc = img_dsc;

        // FR24-style "cover" cropping - the photo fills the whole hero area
        // edge-to-edge (POPUP_CARD_WIDTH x POPUP_HERO_HEIGHT), popup_hero
        // (clip_corner + no padding) clips anything beyond its own bounds.
        int target_w = POPUP_CARD_WIDTH, target_h = POPUP_HERO_HEIGHT;
        uint32_t scale_w = (uint32_t)target_w * 256 / img_dsc->header.w;
        uint32_t scale_h = (uint32_t)target_h * 256 / img_dsc->header.h;
        uint32_t scale = scale_w > scale_h ? scale_w : scale_h;

        lv_image_set_src(popup_img_preview, popup_photo_dsc);
        lv_image_set_scale(popup_img_preview, scale);
        lv_image_set_rotation(popup_img_preview, 0);

        if (is_type_fallback && photographer && photographer[0]) {
            // Representative photo for the aircraft type (fallback by ICAO
            // type code, not the specific airframe) - photo_service already
            // builds the full "[Model] <code>" caption, we just display it here.
            lv_label_set_text(popup_lbl_credit, photographer);
        } else if (photographer && photographer[0]) {
            lv_label_set_text_fmt(popup_lbl_credit, "(c) %s", photographer);
        } else {
            lv_label_set_text(popup_lbl_credit, "(c) Planespotters.net");
        }
    } else {
        lv_image_set_src(popup_img_preview, &aircraft_yellow_25);
        lv_image_set_scale(popup_img_preview, 256);
        lv_image_set_rotation(popup_img_preview, 0);
        lv_label_set_text(popup_lbl_credit, T(STR_NO_PHOTO_DB));
    }

    bsp_display_unlock();
}

static void popup_close_event_cb(lv_event_t *e) {
    if (adsb_service_lock(100)) {
        selected_hex[0] = '\0';
        selected_callsign[0] = '\0';
        selected_reg[0] = '\0';
        selected_model[0] = '\0';
        adsb_service_unlock();
    }
    adsb_service_clear_trace();
    radar_ui_refresh();
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

// ================= WI-FI STATUS (inline in status_badge) =================
static void wifi_badge_collapse_timer_cb(lv_timer_t *timer) {
    (void)timer;
    s_wifi_expanded = false;
    lv_obj_set_style_text_color(wifi_led_label, lv_color_hex(0x8b949e), 0);
    lv_label_set_text(wifi_led_label, "WIFI");
    // The timer is a one-shot (repeat count 1) - LVGL deletes it right after
    // this callback returns, so drop our reference to avoid a dangling
    // lv_timer_del() call on it from apply_wifi_badge() later.
    wifi_badge_collapse_timer = NULL;
}

// Applies s_wifi_notify_state/s_wifi_expanded/ssid/pass/ip to wifi_led_label.
// Collapsed just shows "WIFI"; expanded shows the connection detail. A
// CONNECTED expansion (re)arms a 5s timer that reverts back to collapsed -
// AP mode is the exception: it stays expanded permanently (no timer, and
// s_wifi_expanded is ignored) since the SSID/IP are needed to actually join
// the setup network, and there is no "connected" event to reveal them again
// later. Growing the label text is enough to widen the whole status_badge
// pill rightward - see the LV_FLEX_FLOW_ROW/LV_SIZE_CONTENT comment on the
// state block above. Takes bsp_display_lock() itself - never call this
// while already holding it. Also invoked by wifi_status_click_cb() to
// toggle s_wifi_expanded.
static void apply_wifi_badge(void) {
    if (!wifi_badge_ready) return;

    bsp_display_lock(0);

    if (wifi_badge_collapse_timer) {
        lv_timer_del(wifi_badge_collapse_timer);
        wifi_badge_collapse_timer = NULL;
    }

    if (s_wifi_notify_state == WIFI_NOTIFY_AP) {
        lv_obj_set_style_text_color(wifi_led_label, lv_color_hex(0xffc107), 0);
        lv_label_set_text_fmt(wifi_led_label, "AP: %s (192.168.4.1)", s_wifi_notify_ssid);
        bsp_display_unlock();
        return;
    }

    if (!s_wifi_expanded || s_wifi_notify_state == WIFI_NOTIFY_NONE) {
        lv_obj_set_style_text_color(wifi_led_label, lv_color_hex(0x8b949e), 0);
        lv_label_set_text(wifi_led_label, "WIFI");
        bsp_display_unlock();
        return;
    }

    switch (s_wifi_notify_state) {
        case WIFI_NOTIFY_CONNECTING:
            lv_obj_set_style_text_color(wifi_led_label, lv_color_hex(0x00f0ff), 0);
            lv_label_set_text_fmt(wifi_led_label, T(STR_WIFI_CONNECTING_FMT), s_wifi_notify_ssid);
            break;
        case WIFI_NOTIFY_CONNECTED:
            lv_obj_set_style_text_color(wifi_led_label, lv_color_hex(0x00ff88), 0);
            lv_label_set_text_fmt(wifi_led_label, T(STR_WIFI_CONNECTED_FMT), s_wifi_notify_ip);
            break;
        default:
            break;
    }

    wifi_badge_collapse_timer = lv_timer_create(wifi_badge_collapse_timer_cb, 5000, NULL);
    lv_timer_set_repeat_count(wifi_badge_collapse_timer, 1);

    bsp_display_unlock();
}

// Tapping the WIFI dot/label in status_badge toggles s_wifi_expanded:
// collapsed -> expanded (last known state, fresh 5s auto-collapse timer),
// expanded -> collapsed immediately. No-op in AP mode - that state is always
// expanded (see apply_wifi_badge()) and must not be dismissible.
static void wifi_status_click_cb(lv_event_t *e) {
    (void)e;
    if (!wifi_badge_ready || s_wifi_notify_state == WIFI_NOTIFY_AP) return;
    s_wifi_expanded = !s_wifi_expanded;
    apply_wifi_badge();
}

void radar_ui_wifi_notify_ap_mode(const char *ap_ssid, const char *ap_password) {
    s_wifi_notify_state = WIFI_NOTIFY_AP;
    snprintf(s_wifi_notify_ssid, sizeof(s_wifi_notify_ssid), "%s", ap_ssid ? ap_ssid : "");
    snprintf(s_wifi_notify_pass, sizeof(s_wifi_notify_pass), "%s", ap_password ? ap_password : "");
    s_wifi_expanded = true;
    apply_wifi_badge();
}

void radar_ui_wifi_notify_connecting(const char *ssid) {
    s_wifi_notify_state = WIFI_NOTIFY_CONNECTING;
    snprintf(s_wifi_notify_ssid, sizeof(s_wifi_notify_ssid), "%s", ssid ? ssid : "");
    s_wifi_expanded = true;
    apply_wifi_badge();
}

void radar_ui_wifi_notify_connected(const char *ip_str) {
    s_wifi_notify_state = WIFI_NOTIFY_CONNECTED;
    snprintf(s_wifi_notify_ip, sizeof(s_wifi_notify_ip), "%s", ip_str ? ip_str : "");
    s_wifi_expanded = true;
    apply_wifi_badge();
}

// ================= MQTT STATUS (inline in status_badge) =================
static void mqtt_badge_collapse_timer_cb(lv_timer_t *timer) {
    (void)timer;
    s_mqtt_expanded = false;
    lv_obj_set_style_text_color(mqtt_led_label, lv_color_hex(0x8b949e), 0);
    lv_label_set_text(mqtt_led_label, "MQTT");
    // One-shot timer - LVGL deletes it right after this callback returns.
    mqtt_badge_collapse_timer = NULL;
}

// Applies s_mqtt_notify_state/s_mqtt_expanded to mqtt_led_label - same
// inline-expansion mechanism as apply_wifi_badge() above. Takes
// bsp_display_lock() itself - never call this while already holding it.
// Also invoked by mqtt_status_click_cb() to toggle s_mqtt_expanded.
static void apply_mqtt_badge(void) {
    if (!mqtt_badge_ready) return;

    bsp_display_lock(0);

    if (mqtt_badge_collapse_timer) {
        lv_timer_del(mqtt_badge_collapse_timer);
        mqtt_badge_collapse_timer = NULL;
    }

    if (!s_mqtt_expanded || s_mqtt_notify_state == MQTT_NOTIFY_NONE) {
        lv_obj_set_style_text_color(mqtt_led_label, lv_color_hex(0x8b949e), 0);
        lv_label_set_text(mqtt_led_label, "MQTT");
        bsp_display_unlock();
        return;
    }

    switch (s_mqtt_notify_state) {
        case MQTT_NOTIFY_CONNECTING:
            lv_obj_set_style_text_color(mqtt_led_label, lv_color_hex(0x00f0ff), 0);
            lv_label_set_text(mqtt_led_label, T(STR_MQTT_CONNECTING_MSG));
            break;
        case MQTT_NOTIFY_CONNECTED:
            lv_obj_set_style_text_color(mqtt_led_label, lv_color_hex(0x00ff88), 0);
            lv_label_set_text(mqtt_led_label, T(STR_MQTT_CONNECTED_MSG));
            break;
        case MQTT_NOTIFY_ERROR:
            lv_obj_set_style_text_color(mqtt_led_label, lv_color_hex(0xef4444), 0);
            lv_label_set_text(mqtt_led_label, T(STR_MQTT_ERROR_MSG));
            break;
        default:
            break;
    }

    mqtt_badge_collapse_timer = lv_timer_create(mqtt_badge_collapse_timer_cb, 5000, NULL);
    lv_timer_set_repeat_count(mqtt_badge_collapse_timer, 1);

    bsp_display_unlock();
}

// Tapping the MQTT dot/label in status_badge toggles s_mqtt_expanded - same
// behavior as wifi_status_click_cb() above.
static void mqtt_status_click_cb(lv_event_t *e) {
    (void)e;
    if (!mqtt_badge_ready) return;
    s_mqtt_expanded = !s_mqtt_expanded;
    apply_mqtt_badge();
}

void radar_ui_mqtt_notify_connecting(void) {
    s_mqtt_notify_state = MQTT_NOTIFY_CONNECTING;
    s_mqtt_expanded = true;
    apply_mqtt_badge();
}

void radar_ui_mqtt_notify_connected(void) {
    s_mqtt_notify_state = MQTT_NOTIFY_CONNECTED;
    s_mqtt_expanded = true;
    apply_mqtt_badge();
}

void radar_ui_mqtt_notify_error(void) {
    s_mqtt_notify_state = MQTT_NOTIFY_ERROR;
    s_mqtt_expanded = true;
    apply_mqtt_badge();
}

static void apply_loading_card(void) {
    if (!loading_card_ready) return;

    bsp_display_lock(0);
    switch (s_loading_state) {
        case LOADING_NOTIFY_MAP:
            lv_label_set_text_fmt(loading_card_label, T(STR_MAP_LOADING_FMT), s_loading_map_current, s_loading_map_total);
            lv_obj_clear_flag(loading_card, LV_OBJ_FLAG_HIDDEN);
            break;
        case LOADING_NOTIFY_ADSB:
            lv_label_set_text(loading_card_label, T(STR_ADSB_LOADING));
            lv_obj_clear_flag(loading_card, LV_OBJ_FLAG_HIDDEN);
            break;
        default:
            lv_obj_add_flag(loading_card, LV_OBJ_FLAG_HIDDEN);
            break;
    }
    bsp_display_unlock();
}

void radar_ui_show_map_loading(int current, int total) {
    s_loading_state = LOADING_NOTIFY_MAP;
    s_loading_map_current = current;
    s_loading_map_total = total;
    apply_loading_card();
}

void radar_ui_update_map_loading(int current, int total) {
    radar_ui_show_map_loading(current, total);
}

void radar_ui_show_adsb_loading(void) {
    s_loading_state = LOADING_NOTIFY_ADSB;
    apply_loading_card();
}

void radar_ui_hide_loading(void) {
    s_loading_state = LOADING_NOTIFY_NONE;
    apply_loading_card();
}

// Phase-locked opacity pulse for the status LEDs. All LEDs are driven by a
// single master lv_timer_t and compute their opacity from the absolute
// LVGL tick count (lv_tick_get()) instead of a per-LED animation start
// time. Two LEDs with the same period therefore always land on the same
// opacity on the same frame, regardless of when each one started pulsing -
// this is what keeps wifi_led/mqtt_led in phase once both reach
// LED_STATE_CONNECTED. LEDs with a different period (e.g. LED_STATE_
// CONNECTING) simply run their own cycle without disturbing the others.
#define LED_PULSE_TICK_MS 20
#define LED_PULSE_SLOT_COUNT 3

typedef struct {
    lv_obj_t *led;
    bool active;
    uint32_t period_ms;
    lv_opa_t min_opa;
    lv_opa_t max_opa;
} led_pulse_slot_t;

static led_pulse_slot_t s_led_pulse_slots[LED_PULSE_SLOT_COUNT];
static lv_timer_t *s_led_pulse_timer = NULL;

static led_pulse_slot_t *led_pulse_slot_for(lv_obj_t *led_obj) {
    for (int i = 0; i < LED_PULSE_SLOT_COUNT; i++) {
        if (s_led_pulse_slots[i].led == led_obj) {
            return &s_led_pulse_slots[i];
        }
    }
    for (int i = 0; i < LED_PULSE_SLOT_COUNT; i++) {
        if (s_led_pulse_slots[i].led == NULL) {
            s_led_pulse_slots[i].led = led_obj;
            return &s_led_pulse_slots[i];
        }
    }
    return NULL; // slot table full - should not happen with the known status LEDs
}

static void led_pulse_timer_cb(lv_timer_t *timer) {
    LV_UNUSED(timer);
    uint32_t now = lv_tick_get();
    for (int i = 0; i < LED_PULSE_SLOT_COUNT; i++) {
        led_pulse_slot_t *slot = &s_led_pulse_slots[i];
        if (!slot->active) continue;
        uint32_t phase = now % (2 * slot->period_ms);
        int32_t range = (int32_t)slot->max_opa - (int32_t)slot->min_opa;
        lv_opa_t opa = (phase < slot->period_ms)
            ? (lv_opa_t)((int32_t)slot->min_opa + (range * (int32_t)phase) / (int32_t)slot->period_ms)
            : (lv_opa_t)((int32_t)slot->max_opa - (range * (int32_t)(phase - slot->period_ms)) / (int32_t)slot->period_ms);
        lv_obj_set_style_opa(slot->led, opa, 0);
    }
}

static void start_led_pulse(lv_obj_t *led_obj, uint32_t period_ms, lv_opa_t min_opa, lv_opa_t max_opa) {
    led_pulse_slot_t *slot = led_pulse_slot_for(led_obj);
    if (!slot) return;
    slot->active = true;
    slot->period_ms = period_ms;
    slot->min_opa = min_opa;
    slot->max_opa = max_opa;
    if (!s_led_pulse_timer) {
        s_led_pulse_timer = lv_timer_create(led_pulse_timer_cb, LED_PULSE_TICK_MS, NULL);
    }
}

static void stop_led_pulse(lv_obj_t *led_obj) {
    led_pulse_slot_t *slot = led_pulse_slot_for(led_obj);
    if (slot) {
        slot->active = false;
    }
    lv_obj_set_style_opa(led_obj, LV_OPA_COVER, 0);
}

typedef enum {
    LED_STATE_CONNECTED,
    LED_STATE_CONNECTING,
    LED_STATE_ERROR,
} led_conn_state_t;

// Sets the LED color and (re)starts the animation matching the connection
// state: connecting pulses fast and wide (attention-grabbing), connected
// breathes slowly and subtly (idle "radar is alive" feel), error blinks
// quickly as a visible warning.
static void set_led_indicator(lv_obj_t *led_obj, led_conn_state_t state) {
    switch (state) {
        case LED_STATE_CONNECTED:
            lv_obj_set_style_bg_color(led_obj, lv_color_hex(0x22c55e), 0);
            start_led_pulse(led_obj, 1500, LV_OPA_40, LV_OPA_COVER);
            break;
        case LED_STATE_CONNECTING:
            lv_obj_set_style_bg_color(led_obj, lv_color_hex(0xeab308), 0);
            start_led_pulse(led_obj, 400, LV_OPA_20, LV_OPA_COVER);
            break;
        case LED_STATE_ERROR:
        default:
            lv_obj_set_style_bg_color(led_obj, lv_color_hex(0xef4444), 0);
            start_led_pulse(led_obj, 200, LV_OPA_30, LV_OPA_COVER);
            break;
    }
}

// Applies s_wifi_led_status/s_mqtt_led_enabled/s_mqtt_led_status to the
// badge LEDs. Takes bsp_display_lock() itself - never call while holding it.
static void apply_status_badge(void) {
    if (!status_badge_ready) return;

    bsp_display_lock(0);

    switch (s_wifi_led_status) {
        case WIFI_STATUS_CONNECTED:  set_led_indicator(wifi_led, LED_STATE_CONNECTED); break;
        case WIFI_STATUS_CONNECTING: set_led_indicator(wifi_led, LED_STATE_CONNECTING); break;
        case WIFI_STATUS_ERROR:      set_led_indicator(wifi_led, LED_STATE_ERROR); break;
    }

    if (!s_mqtt_led_enabled) {
        // MQTT disabled in configuration - stop the animation before
        // hiding so it doesn't keep ticking on a hidden object.
        stop_led_pulse(mqtt_led);
        lv_obj_add_flag(mqtt_led, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mqtt_led_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(mqtt_led, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mqtt_led_label, LV_OBJ_FLAG_HIDDEN);
        switch (s_mqtt_led_status) {
            case MQTT_STATUS_CONNECTED:  set_led_indicator(mqtt_led, LED_STATE_CONNECTED); break;
            case MQTT_STATUS_CONNECTING: set_led_indicator(mqtt_led, LED_STATE_CONNECTING); break;
            case MQTT_STATUS_ERROR:      set_led_indicator(mqtt_led, LED_STATE_ERROR); break;
            default:                     set_led_indicator(mqtt_led, LED_STATE_ERROR); break;
        }
    }

    if (!s_fw_update_available) {
        stop_led_pulse(fw_led);
        lv_obj_add_flag(fw_led, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(fw_led_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(fw_led, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(fw_led_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(fw_led, lv_color_hex(0xf59e0b), 0);
        start_led_pulse(fw_led, 900, LV_OPA_50, LV_OPA_COVER);
    }

    bsp_display_unlock();
}

void radar_ui_update_wifi_status(wifi_status_t status) {
    s_wifi_led_status = status;
    apply_status_badge();
}

void radar_ui_update_mqtt_status(bool enabled, mqtt_conn_status_t status) {
    s_mqtt_led_enabled = enabled;
    s_mqtt_led_status = status;
    apply_status_badge();
}

static void fw_update_toast_hide_timer_cb(lv_timer_t *timer) {
    (void)timer;
    lv_obj_add_flag(fw_update_toast, LV_OBJ_FLAG_HIDDEN);
    fw_update_toast_timer = NULL;
}

// Tapping the (pulsing) dot or "FW" label shows a brief toast with the new
// version number and release notes (when known), auto-hiding after 3s.
static void fw_led_click_cb(lv_event_t *e) {
    (void)e;
    if (!s_fw_update_available) return;

    bsp_display_lock(0);
    if (fw_update_toast_timer) {
        lv_timer_del(fw_update_toast_timer);
        fw_update_toast_timer = NULL;
    }
    char toast_text[192];
    int n = snprintf(toast_text, sizeof(toast_text), T(STR_FW_UPDATE_TOAST_FMT), s_fw_update_version);
    if (s_fw_update_notes[0] && n > 0 && (size_t)n < sizeof(toast_text)) {
        snprintf(toast_text + n, sizeof(toast_text) - (size_t)n, "\n%s", s_fw_update_notes);
    }
    lv_label_set_text(fw_update_toast_label, toast_text);
    lv_obj_clear_flag(fw_update_toast, LV_OBJ_FLAG_HIDDEN);
    fw_update_toast_timer = lv_timer_create(fw_update_toast_hide_timer_cb, 3000, NULL);
    lv_timer_set_repeat_count(fw_update_toast_timer, 1);
    bsp_display_unlock();
}

void radar_ui_update_fw_status(bool available, const char *latest_version, const char *release_notes) {
    s_fw_update_available = available;
    if (available && latest_version) {
        snprintf(s_fw_update_version, sizeof(s_fw_update_version), "%s", latest_version);
    }
    if (available && release_notes) {
        snprintf(s_fw_update_notes, sizeof(s_fw_update_notes), "%s", release_notes);
    } else if (!available) {
        s_fw_update_notes[0] = '\0';
    }
    apply_status_badge();
}

// ================= LVGL STRUCTURE BUILD =================
bool radar_ui_init(void) {
    ui_slots = (AircraftSlotUI *)heap_caps_calloc(MAX_AIRCRAFT_CAPACITY, sizeof(AircraftSlotUI), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ui_slots) {
        ESP_LOGE(TAG, "Failed to allocate UI slot memory in PSRAM!");
        return false;
    }
    g_refresh_serialize_mutex = xSemaphoreCreateMutex();
    if (!g_refresh_serialize_mutex) {
        ESP_LOGE(TAG, "Failed to create g_refresh_serialize_mutex!");
        return false;
    }

    int default_range = wifi_mgr_get_default_range();
    for (size_t i = 0; i < NUM_RANGE_STEPS; i++) {
        if ((int)range_steps[i] == default_range) {
            current_range_idx = (int)i;
            break;
        }
    }

    air_filter_mode = (air_filter_mode_t)wifi_mgr_get_default_air_mode();
    show_airports = (wifi_mgr_get_default_apts_mode() == 1);
    hide_ground_traffic = wifi_mgr_get_hide_ground();
    i18n_set_lang(wifi_mgr_get_lang());
    map_tile_service_set_enabled(wifi_mgr_get_map_enabled());

    return true;
}

void radar_ui_build(void) {
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
    lv_obj_add_flag(radar_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(radar_area, popup_close_event_cb, LV_EVENT_CLICKED, NULL);

    map_tile_service_set_canvas_parent(radar_area);

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

    for (int i = 0; i < MAX_VISIBLE_AIRPORTS; i++) {
        ui_airports[i].marker = lv_obj_create(radar_area);
        lv_obj_set_size(ui_airports[i].marker, 6, 6);
        lv_obj_set_style_radius(ui_airports[i].marker, 1, 0);
        lv_obj_set_style_bg_color(ui_airports[i].marker, lv_color_hex(0x38bdf8), 0);
        lv_obj_set_style_border_color(ui_airports[i].marker, lv_color_hex(0xffffff), 0);
        lv_obj_set_style_border_width(ui_airports[i].marker, 1, 0);
        lv_obj_clear_flag(ui_airports[i].marker, LV_OBJ_FLAG_SCROLLABLE);

        ui_airports[i].label = lv_label_create(radar_area);
        lv_obj_set_style_text_color(ui_airports[i].label, lv_color_hex(0x38bdf8), 0);
        lv_obj_set_hidden(ui_airports[i].marker, true);
        lv_obj_set_hidden(ui_airports[i].label, true);
    }

    // HUD status badge - compact Wi-Fi/MQTT LED indicator, replacing the
    // old range pill (range is already shown by the RNG button top-right
    // and by the range rings themselves).
    status_badge = lv_obj_create(radar_area);
    lv_obj_set_height(status_badge, 28);
    lv_obj_set_width(status_badge, LV_SIZE_CONTENT);
    lv_obj_align(status_badge, LV_ALIGN_BOTTOM_LEFT, 15, -15);
    lv_obj_set_style_bg_color(status_badge, lv_color_hex(0x0a0f1d), 0);
    lv_obj_set_style_bg_opa(status_badge, LV_OPA_80, 0);
    lv_obj_set_style_border_color(status_badge, lv_color_hex(0x30363d), 0);
    lv_obj_set_style_border_width(status_badge, 1, 0);
    lv_obj_set_style_radius(status_badge, 9, 0);
    lv_obj_set_style_pad_hor(status_badge, 10, 0);
    lv_obj_set_style_pad_ver(status_badge, 0, 0);
    lv_obj_set_style_pad_column(status_badge, 4, 0);
    lv_obj_set_flex_flow(status_badge, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_badge, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(status_badge, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    wifi_led = lv_obj_create(status_badge);
    lv_obj_set_size(wifi_led, 8, 8);
    lv_obj_set_style_radius(wifi_led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wifi_led, lv_color_hex(0x64748b), 0);
    lv_obj_set_style_border_width(wifi_led, 0, 0);
    lv_obj_clear_flag(wifi_led, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wifi_led, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(wifi_led, wifi_status_click_cb, LV_EVENT_CLICKED, NULL);

    wifi_led_label = lv_label_create(status_badge);
    lv_label_set_text(wifi_led_label, "WIFI");
    lv_obj_set_style_text_font(wifi_led_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(wifi_led_label, lv_color_hex(0x8b949e), 0);
    lv_obj_add_flag(wifi_led_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(wifi_led_label, wifi_status_click_cb, LV_EVENT_CLICKED, NULL);

    mqtt_led = lv_obj_create(status_badge);
    lv_obj_set_size(mqtt_led, 8, 8);
    lv_obj_set_style_radius(mqtt_led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(mqtt_led, lv_color_hex(0x64748b), 0);
    lv_obj_set_style_border_width(mqtt_led, 0, 0);
    lv_obj_set_style_pad_left(mqtt_led, 6, 0);
    lv_obj_clear_flag(mqtt_led, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(mqtt_led, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mqtt_led, mqtt_status_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(mqtt_led, LV_OBJ_FLAG_HIDDEN);

    mqtt_led_label = lv_label_create(status_badge);
    lv_label_set_text(mqtt_led_label, "MQTT");
    lv_obj_set_style_text_font(mqtt_led_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(mqtt_led_label, lv_color_hex(0x8b949e), 0);
    lv_obj_add_flag(mqtt_led_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(mqtt_led_label, mqtt_status_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(mqtt_led_label, LV_OBJ_FLAG_HIDDEN);

    // Firmware update indicator - built exactly like wifi_led/mqtt_led above
    // (dot + label, same font/color/padding) so all three badge chips read
    // as one consistent control row. Hidden unless ota_update_service.c has
    // found a newer release; tap shows a brief toast (fw_update_toast below).
    fw_led = lv_obj_create(status_badge);
    lv_obj_set_size(fw_led, 8, 8);
    lv_obj_set_style_radius(fw_led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(fw_led, lv_color_hex(0xf59e0b), 0);
    lv_obj_set_style_border_width(fw_led, 0, 0);
    lv_obj_set_style_pad_left(fw_led, 6, 0);
    lv_obj_clear_flag(fw_led, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(fw_led, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(fw_led, fw_led_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(fw_led, LV_OBJ_FLAG_HIDDEN);

    fw_led_label = lv_label_create(status_badge);
    lv_label_set_text(fw_led_label, "FW");
    lv_obj_set_style_text_font(fw_led_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(fw_led_label, lv_color_hex(0x8b949e), 0);
    lv_obj_add_flag(fw_led_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(fw_led_label, fw_led_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(fw_led_label, LV_OBJ_FLAG_HIDDEN);

    fw_update_toast = lv_obj_create(radar_area);
    lv_obj_set_height(fw_update_toast, LV_SIZE_CONTENT);
    lv_obj_set_width(fw_update_toast, LV_SIZE_CONTENT);
    // One row above status_badge - Wi-Fi/MQTT status no longer stack rows
    // above the badge (they expand status_badge itself horizontally, see
    // apply_wifi_badge()/apply_mqtt_badge()), so this is the only toast that
    // still needs a row of its own.
    lv_obj_align(fw_update_toast, LV_ALIGN_BOTTOM_LEFT, 15, -50);
    lv_obj_set_style_bg_color(fw_update_toast, lv_color_hex(0x0a0f1d), 0);
    lv_obj_set_style_bg_opa(fw_update_toast, LV_OPA_90, 0);
    lv_obj_set_style_border_color(fw_update_toast, lv_color_hex(0xf59e0b), 0);
    lv_obj_set_style_border_width(fw_update_toast, 1, 0);
    lv_obj_set_style_radius(fw_update_toast, 8, 0);
    lv_obj_set_style_pad_hor(fw_update_toast, 10, 0);
    lv_obj_set_style_pad_ver(fw_update_toast, 6, 0);
    lv_obj_clear_flag(fw_update_toast, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(fw_update_toast, LV_OBJ_FLAG_HIDDEN);

    fw_update_toast_label = lv_label_create(fw_update_toast);
    lv_obj_set_style_text_color(fw_update_toast_label, lv_color_hex(0xf59e0b), 0);
    lv_obj_set_width(fw_update_toast_label, 240);
    lv_label_set_long_mode(fw_update_toast_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(fw_update_toast_label, "");

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
    lv_obj_add_flag(right_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(right_panel, popup_close_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *top_bar = lv_obj_create(right_panel);
    lv_obj_set_size(top_bar, lv_pct(100), 26);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(top_bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(top_bar, 0, 0);
    lv_obj_set_style_pad_all(top_bar, 0, 0);
    // Clear column gap and symmetric side margins so the buttons read as
    // distinct, separated pills instead of blending into each other.
    lv_obj_set_style_pad_column(top_bar, 6, 0);
    lv_obj_set_style_pad_left(top_bar, 6, 0);
    lv_obj_set_style_pad_right(top_bar, 6, 0);
    lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(top_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    btn_airport_toggle = lv_obj_create(top_bar);
    lv_obj_set_size(btn_airport_toggle, LV_SIZE_CONTENT, 26);
    lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_airport_toggle, 1, 0);
    lv_obj_set_style_radius(btn_airport_toggle, 6, 0);
    lv_obj_set_style_pad_hor(btn_airport_toggle, 5, 0);
    lv_obj_set_style_pad_ver(btn_airport_toggle, 2, 0);
    lv_obj_add_flag(btn_airport_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_airport_toggle, airport_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_airport_toggle = lv_label_create(btn_airport_toggle);
    lv_label_set_text(lbl_airport_toggle, show_airports ? "APTS: ON" : "APTS: OFF");
    lv_obj_set_style_text_color(lbl_airport_toggle, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(lbl_airport_toggle, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_airport_toggle);
    if (!show_airports) {
        lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0xff5555), 0);
    }

    btn_filter_ground = lv_obj_create(top_bar);
    lv_obj_set_size(btn_filter_ground, LV_SIZE_CONTENT, 26);
    lv_obj_set_style_bg_color(btn_filter_ground, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_filter_ground, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_filter_ground, 1, 0);
    lv_obj_set_style_radius(btn_filter_ground, 6, 0);
    lv_obj_set_style_pad_hor(btn_filter_ground, 5, 0);
    lv_obj_set_style_pad_ver(btn_filter_ground, 2, 0);
    lv_obj_add_flag(btn_filter_ground, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_filter_ground, filter_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_filter_ground = lv_label_create(btn_filter_ground);
    lv_obj_set_style_text_font(lbl_filter_ground, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_filter_ground);
    apply_air_filter_style();

    btn_gnd_toggle = lv_obj_create(top_bar);
    lv_obj_set_size(btn_gnd_toggle, LV_SIZE_CONTENT, 26);
    lv_obj_set_style_border_width(btn_gnd_toggle, 1, 0);
    lv_obj_set_style_radius(btn_gnd_toggle, 6, 0);
    lv_obj_set_style_pad_hor(btn_gnd_toggle, 5, 0);
    lv_obj_set_style_pad_ver(btn_gnd_toggle, 2, 0);
    lv_obj_add_flag(btn_gnd_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_gnd_toggle, gnd_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_gnd_toggle = lv_label_create(btn_gnd_toggle);
    lv_obj_set_style_text_font(lbl_gnd_toggle, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_gnd_toggle);
    apply_gnd_filter_style();

    btn_map_toggle = lv_obj_create(top_bar);
    lv_obj_set_size(btn_map_toggle, LV_SIZE_CONTENT, 26);
    lv_obj_set_style_border_width(btn_map_toggle, 1, 0);
    lv_obj_set_style_radius(btn_map_toggle, 6, 0);
    lv_obj_set_style_pad_hor(btn_map_toggle, 5, 0);
    lv_obj_set_style_pad_ver(btn_map_toggle, 2, 0);
    lv_obj_add_flag(btn_map_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_map_toggle, map_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_map_toggle = lv_label_create(btn_map_toggle);
    lv_obj_set_style_text_font(lbl_map_toggle, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_map_toggle);
    apply_map_toggle_style();

    lv_obj_t *btn_range = lv_obj_create(top_bar);
    lv_obj_set_size(btn_range, LV_SIZE_CONTENT, 26);
    lv_obj_clear_flag(btn_range, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(btn_range, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_range, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_border_width(btn_range, 1, 0);
    lv_obj_set_style_radius(btn_range, 6, 0);
    lv_obj_set_style_pad_hor(btn_range, 5, 0);
    lv_obj_set_style_pad_ver(btn_range, 2, 0);
    lv_obj_add_flag(btn_range, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_range, range_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_range_header = lv_label_create(btn_range);
    lv_label_set_long_mode(lbl_range_header, LV_LABEL_LONG_CLIP);
    lv_label_set_text_fmt(lbl_range_header, "RNG: %.0fKM", range_steps[current_range_idx]);
    lv_obj_set_style_text_color(lbl_range_header, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_font(lbl_range_header, &lv_font_montserrat_12, 0);
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
    lv_obj_set_scrollbar_mode(list_cont, LV_SCROLLBAR_MODE_AUTO);
    // Reserves a gutter on the right for the scrollbar (see LIST_ITEM_WIDTH
    // below) so it never overlaps the selection border or the vertical-rate
    // trend symbol at the right edge of each card.
    lv_obj_set_style_pad_right(list_cont, LIST_SCROLLBAR_GUTTER, LV_PART_MAIN);
    lv_obj_add_flag(list_cont, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(list_cont, popup_close_event_cb, LV_EVENT_CLICKED, NULL);

    // Thin, subdued gray scrollbar - a bright green bar drew the eye away
    // from the vertical-rate trend symbols next to it, so this uses a
    // neutral, half-transparent gray instead. pad_right on LV_PART_SCROLLBAR
    // pulls it toward the outer edge of its gutter (LIST_SCROLLBAR_GUTTER
    // above), away from the cards, instead of hugging their right edge.
    lv_obj_set_style_width(list_cont, 4, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(list_cont, 2, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(list_cont, lv_color_hex(0xaaaaaa), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(list_cont, LV_OPA_40, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(list_cont, 2, LV_PART_SCROLLBAR);

    for(int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
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
        lv_obj_set_size(ui_slots[i].list_item, LIST_ITEM_WIDTH, 64);
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

        ui_slots[i].list_lbl_trend = lv_label_create(ui_slots[i].list_item);
        lv_obj_align(ui_slots[i].list_lbl_trend, LV_ALIGN_TOP_RIGHT, -10, 22);

        lv_obj_t *div = lv_obj_create(ui_slots[i].list_item);
        lv_obj_set_size(div, LIST_ITEM_WIDTH, 1);
        lv_obj_align(div, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(div, lv_color_hex(0x003318), 0);
        lv_obj_set_style_border_width(div, 0, 0);
    }

    // Unified modal card - one bordered/rounded container in a column flex
    // flow, hero photo on top flowing directly into the data body below it
    // (pad_all 0 on popup_card itself - popup_hero touches the card's top
    // edge/corners, popup_body carries its own inner padding). Replaces the
    // old two-box layout (cyan photo frame + separate green data frame with
    // a gap between them).
    popup_card = lv_obj_create(radar_area);
    lv_obj_set_size(popup_card, POPUP_CARD_WIDTH, LV_SIZE_CONTENT);
    lv_obj_center(popup_card);
    lv_obj_set_style_bg_color(popup_card, lv_color_hex(0x0f141c), 0);
    lv_obj_set_style_bg_opa(popup_card, LV_OPA_90, 0);
    lv_obj_set_style_border_color(popup_card, lv_color_hex(0x2d4255), 0);
    lv_obj_set_style_border_width(popup_card, 1, 0);
    lv_obj_set_style_radius(popup_card, 12, 0);
    lv_obj_set_style_pad_all(popup_card, 0, 0);
    lv_obj_set_style_clip_corner(popup_card, true, 0);
    lv_obj_set_flex_flow(popup_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(popup_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(popup_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(popup_card, popup_close_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_hidden(popup_card, true);

    // --- Hero image (top of the card, edge-to-edge, no separate frame) ---
    popup_hero = lv_obj_create(popup_card);
    lv_obj_set_size(popup_hero, POPUP_CARD_WIDTH, POPUP_HERO_HEIGHT);
    lv_obj_set_style_bg_color(popup_hero, lv_color_hex(0x0a0e14), 0);
    lv_obj_set_style_border_width(popup_hero, 0, 0);
    lv_obj_set_style_radius(popup_hero, 0, 0);
    lv_obj_set_style_pad_all(popup_hero, 0, 0);
    lv_obj_clear_flag(popup_hero, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_clip_corner(popup_hero, true, 0);

    popup_img_preview = lv_image_create(popup_hero);
    lv_image_set_src(popup_img_preview, &aircraft_yellow_25);
    lv_obj_set_size(popup_img_preview, POPUP_CARD_WIDTH, POPUP_HERO_HEIGHT);
    lv_image_set_inner_align(popup_img_preview, LV_IMAGE_ALIGN_CENTER);
    lv_obj_align(popup_img_preview, LV_ALIGN_CENTER, 0, 0);

    // Photo credit label (FR24-style) - semi-transparent badge in the
    // bottom-left corner of the photo; also shows status while
    // loading/when there is no photo.
    popup_lbl_credit = lv_label_create(popup_hero);
    lv_label_set_text(popup_lbl_credit, "RADAR STATION FEED");
    lv_obj_set_style_text_color(popup_lbl_credit, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_bg_color(popup_lbl_credit, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(popup_lbl_credit, LV_OPA_70, 0);
    lv_obj_set_style_pad_hor(popup_lbl_credit, 6, 0);
    lv_obj_set_style_pad_ver(popup_lbl_credit, 2, 0);
    lv_obj_set_style_radius(popup_lbl_credit, 4, 0);
    lv_obj_align(popup_lbl_credit, LV_ALIGN_BOTTOM_LEFT, 8, -8);

    // --- Data body (flows directly under the hero image, same card) ---
    popup_body = lv_obj_create(popup_card);
    lv_obj_set_size(popup_body, POPUP_CARD_WIDTH, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(popup_body, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(popup_body, 0, 0);
    lv_obj_set_style_pad_all(popup_body, 10, 0);
    lv_obj_set_style_pad_row(popup_body, 8, 0);
    lv_obj_set_flex_flow(popup_body, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(popup_body, LV_OBJ_FLAG_SCROLLABLE);

    // Title row: big white callsign + a muted pill with type/registration.
    lv_obj_t *popup_row_title = lv_obj_create(popup_body);
    lv_obj_set_size(popup_row_title, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(popup_row_title, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(popup_row_title, 0, 0);
    lv_obj_set_style_pad_all(popup_row_title, 0, 0);
    lv_obj_set_flex_flow(popup_row_title, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(popup_row_title, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(popup_row_title, LV_OBJ_FLAG_SCROLLABLE);

    popup_lbl_callsign = lv_label_create(popup_row_title);
    lv_label_set_text(popup_lbl_callsign, "---");
    lv_obj_set_style_text_font(popup_lbl_callsign, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(popup_lbl_callsign, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_pad_right(popup_lbl_callsign, 8, 0);

    popup_lbl_typereg = lv_label_create(popup_row_title);
    lv_label_set_text(popup_lbl_typereg, "---  ---");
    lv_obj_set_style_text_font(popup_lbl_typereg, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(popup_lbl_typereg, lv_color_hex(0xc7d3dc), 0);
    lv_obj_set_style_bg_color(popup_lbl_typereg, lv_color_hex(0x2a3542), 0);
    lv_obj_set_style_bg_opa(popup_lbl_typereg, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(popup_lbl_typereg, 8, 0);
    lv_obj_set_style_pad_ver(popup_lbl_typereg, 3, 0);
    lv_obj_set_style_radius(popup_lbl_typereg, 999, 0);

    // Data grid: 2 equal columns (flex_grow 1 each) - left ALT/SPD/DIST,
    // right V/S/HDG/SQK. Each label has recolor enabled so its own text can
    // mix the muted field-label color with the bright value color inline
    // (see the "#hex text#" spans in radar_ui_refresh()) instead of needing
    // a separate widget per field.
    lv_obj_t *popup_grid = lv_obj_create(popup_body);
    lv_obj_set_size(popup_grid, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(popup_grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(popup_grid, 0, 0);
    lv_obj_set_style_pad_all(popup_grid, 0, 0);
    lv_obj_set_flex_flow(popup_grid, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(popup_grid, LV_OBJ_FLAG_SCROLLABLE);

    popup_lbl_col_left = lv_label_create(popup_grid);
    lv_label_set_recolor(popup_lbl_col_left, true);
    lv_label_set_text(popup_lbl_col_left, "#8094a5 ALT#\n#8094a5 SPD#\n#8094a5 DIST#");
    lv_obj_set_flex_grow(popup_lbl_col_left, 1);

    popup_lbl_col_right = lv_label_create(popup_grid);
    lv_label_set_recolor(popup_lbl_col_right, true);
    lv_label_set_text(popup_lbl_col_right, "#8094a5 V/S#\n#8094a5 HDG#\n#8094a5 SQK#");
    lv_obj_set_flex_grow(popup_lbl_col_right, 1);

    // Footer: station coordinates, small and dimmed.
    popup_lbl_route = lv_label_create(popup_body);
    lv_label_set_text_fmt(popup_lbl_route, "LAT: %.2f  LON: %.2f  (%s)", g_radar_lat, g_radar_lon, g_station_name);
    lv_obj_set_style_text_font(popup_lbl_route, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(popup_lbl_route, lv_color_hex(0x8094a5), 0);
    lv_obj_set_style_pad_top(popup_lbl_route, 2, 0);

    // Squawk alert banner (7700/7600/7500) - child of scr (not radar_area),
    // created last so it renders on top of the whole screen.
    banner_alert = lv_obj_create(scr);
    lv_obj_set_size(banner_alert, SCREEN_WIDTH, 32);
    lv_obj_set_pos(banner_alert, 0, 0);
    lv_obj_set_style_bg_color(banner_alert, lv_color_hex(0x990000), 0);
    lv_obj_set_style_bg_opa(banner_alert, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(banner_alert, lv_color_hex(0xFF1E27), 0);
    lv_obj_set_style_border_width(banner_alert, 2, 0);
    lv_obj_set_style_border_side(banner_alert, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_radius(banner_alert, 0, 0);
    lv_obj_set_style_pad_all(banner_alert, 0, 0);
    lv_obj_clear_flag(banner_alert, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(banner_alert, LV_OBJ_FLAG_HIDDEN);

    banner_lbl = lv_label_create(banner_alert);
    lv_label_set_text(banner_lbl, "");
    lv_obj_set_style_text_color(banner_lbl, lv_color_hex(0xffffff), 0);
    lv_obj_center(banner_lbl);

    // Map/ADS-B sequential-fetch loading pill - compact badge styled like
    // status_badge (WIFI/MQTT, bottom-left), docked at the top center of
    // radar_area (not the full screen) so it stays centered over the radar
    // itself and never drifts into right_panel's top_bar buttons.
    loading_card = lv_obj_create(radar_area);
    lv_obj_set_height(loading_card, 22);
    lv_obj_set_width(loading_card, LV_SIZE_CONTENT);
    lv_obj_align(loading_card, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_bg_color(loading_card, lv_color_hex(0x0b1410), 0);
    lv_obj_set_style_bg_opa(loading_card, LV_OPA_80, 0);
    lv_obj_set_style_border_color(loading_card, lv_color_hex(0x00e676), 0);
    lv_obj_set_style_border_width(loading_card, 1, 0);
    lv_obj_set_style_radius(loading_card, 8, 0);
    lv_obj_set_style_pad_hor(loading_card, 10, 0);
    lv_obj_set_style_pad_ver(loading_card, 2, 0);
    lv_obj_clear_flag(loading_card, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(loading_card, LV_OBJ_FLAG_HIDDEN);

    loading_card_label = lv_label_create(loading_card);
    lv_obj_set_width(loading_card_label, LV_SIZE_CONTENT);
    // Clips instead of wrapping - the pill itself auto-sizes to the text
    // (LV_SIZE_CONTENT above), so this only guards against the LV_SYMBOL_
    // DOWNLOAD glyph/counter ever being cut mid-render if that changes.
    lv_label_set_long_mode(loading_card_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_font(loading_card_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(loading_card_label, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_text_align(loading_card_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(loading_card_label, "");

    bsp_display_unlock();

    wifi_badge_ready = true;
    apply_wifi_badge();
    mqtt_badge_ready = true;
    apply_mqtt_badge();
    loading_card_ready = true;
    apply_loading_card();
    status_badge_ready = true;
    apply_status_badge();

    map_tile_service_request_reload(g_radar_lat, g_radar_lon, range_steps[current_range_idx]);
}
