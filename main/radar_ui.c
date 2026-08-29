#include <stdio.h>
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
#include "adsb_service.h"
#include "wifi_manager.h"
#include "radar_ui.h"
#include "photo_service.h"
#include "i18n.h"

// Grafiki wektorowe samolotow - generowane pliki LVGL, kompilowane jako
// osobne jednostki translacji (main/CMakeLists.txt), tu tylko deklaracje.
extern const lv_image_dsc_t aircraft_yellow_15;
extern const lv_image_dsc_t aircraft_yellow_20;
extern const lv_image_dsc_t aircraft_yellow_25;
extern const lv_image_dsc_t helicopter_yellow_20;

static const char *TAG = "RADAR_UI";

static int current_range_idx = 0;
static air_filter_mode_t air_filter_mode = AIR_FILTER_ALL;
static bool show_airports = true;
static bool hide_ground_traffic = true;

typedef struct {
    lv_obj_t *marker;
    lv_obj_t *label;
    float distance_km;
    float bearing_deg;
} AirportSlotUI;

static AirportSlotUI ui_airports[NUM_AIRPORTS];

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

    // Wyniki fazy "compute" w radar_ui_refresh() - liczone pod adsb_service_lock,
    // bez bsp_display_lock (m.in. trygonometria sladu lotu). Stosowane do
    // obiektow LVGL w krotkiej fazie "apply" pod bsp_display_lock.
    bool     calc_visible;
    bool     calc_vec_visible;
    bool     calc_is_military;
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
    int      calc_speed_kt;
    int      calc_heading_deg;
    float    calc_lat, calc_lon;
} AircraftSlotUI;

static AircraftSlotUI *ui_slots = NULL;
// Serializuje wywolania radar_ui_refresh() z roznych taskow (adsb_worker
// oraz handlery klikniec w tasku LVGL), bo obie fazy pisza do wspolnych
// buforow scratch w ui_slots.
static SemaphoreHandle_t g_refresh_serialize_mutex = NULL;

static lv_obj_t *radar_area;
static lv_obj_t *list_cont;
static lv_obj_t *lbl_status_count;
static lv_obj_t *lbl_range_header;
static lv_obj_t *lbl_range_scope;
static lv_obj_t *lbl_range_pill;
static lv_obj_t *btn_filter_ground;
static lv_obj_t *lbl_filter_ground;
static lv_obj_t *btn_airport_toggle;
static lv_obj_t *lbl_airport_toggle;
static lv_obj_t *btn_gnd_toggle;
static lv_obj_t *lbl_gnd_toggle;

static lv_obj_t *hud_trail_segs[MAX_HUD_SEGS];
static lv_point_precise_t hud_seg_pts[MAX_HUD_SEGS][2];
static lv_point_precise_t hud_seg_pts_calc[MAX_HUD_SEGS][2];
static lv_color_t hud_seg_color_calc[MAX_HUD_SEGS];

static lv_obj_t *popup_card_cont;
static lv_obj_t *popup_top_box;
static lv_obj_t *popup_lbl_count;
static lv_obj_t *popup_img_preview;
static lv_obj_t *popup_lbl_credit;

static lv_obj_t *popup_hud_box;
static lv_obj_t *popup_lbl_title;
static lv_obj_t *popup_lbl_col_left;
static lv_obj_t *popup_lbl_col_right;
static lv_obj_t *popup_lbl_route;

// Zdjecie samolotu z Planespotters.net (photo_service) - bufor RGB565 w PSRAM,
// wlasnosc radar_ui. popup_photo_hex sledzi, dla ktorego hex zostalo zadane/
// zaladowane, zeby nie wysylac zapytania ponownie co kazdy radar_ui_refresh().
static const lv_image_dsc_t *popup_photo_dsc = NULL;
static char popup_photo_hex[8] = "";

static void popup_photo_release(void) {
    if (popup_photo_dsc) {
        heap_caps_free((void *)popup_photo_dsc->data);
        heap_caps_free((void *)popup_photo_dsc);
        popup_photo_dsc = NULL;
    }
}

// Pulsujacy (migajacy) tekst statusu "POBIERANIE ZDJECIA..." w kolorze
// zoltym, na czas trwania zapytania do photo_service.
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

static inline void lv_obj_set_hidden(lv_obj_t *obj, bool hidden) {
    if (!obj) return;
    if (hidden) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
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

// ================= AKTUALIZACJA UI SAMOLOTOW =================
void radar_ui_refresh(void) {
    if (!live_fleet || !ui_slots || !g_refresh_serialize_mutex) return;
    // Ograniczony timeout (nie portMAX_DELAY): radar_ui_refresh() bywa wywolywane
    // takze z handlerow klikniec w tasku LVGL, ktory w tym momencie juz trzyma
    // lvgl_mux - blokada bez limitu tutaj mogla by go zawiesic na czas trwania
    // rownoleglego refresh z adsb_worker_task.
    if (xSemaphoreTake(g_refresh_serialize_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

    if (!adsb_service_lock(100)) {
        xSemaphoreGive(g_refresh_serialize_mutex);
        return;
    }

    uint32_t now = get_time_ms();
    float current_range_km = range_steps[current_range_idx];
    float scale = (float)RADAR_MAX_RADIUS / current_range_km;
    uint8_t trail_len = wifi_mgr_get_trail_len();
    uint16_t max_aircraft = wifi_mgr_get_max_aircraft();
    if (max_aircraft > MAX_AIRCRAFT_CAPACITY) max_aircraft = MAX_AIRCRAFT_CAPACITY;

    int visible_count = 0;
    int list_y_offset = 0;
    AircraftData *selected_ac = NULL;

    // ---- Faza "compute": trygonometria i dane etykiet, tylko adsb_service_lock,
    // bez bsp_display_lock - zeby jak najkrocej trzymac mutex LVGL. Petla idzie
    // po calej statycznej pojemnosci buforow (MAX_AIRCRAFT_CAPACITY), ale
    // sloty >= max_aircraft sa od razu odrzucane (is_valid=false), zeby faza
    // "apply" ponizej mogla je poprawnie ukryc, gdy limit zostal zmniejszony
    // od ostatniego odswiezenia. ----
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

            float rad = live_fleet[i].bearing_deg * DEG_TO_RAD;
            int px = RADAR_CENTER_X + (int)(live_fleet[i].distance_km * scale * sinf(rad));
            int py = RADAR_CENTER_Y - (int)(live_fleet[i].distance_km * scale * cosf(rad));
            ui_slots[i].calc_px = px;
            ui_slots[i].calc_py = py;

            const lv_image_dsc_t *dsc = get_aircraft_dsc(live_fleet[i].type);
            ui_slots[i].calc_dsc = dsc;
            ui_slots[i].calc_half_w = (int)dsc->header.w / 2;
            ui_slots[i].calc_half_h = (int)dsc->header.h / 2;
            ui_slots[i].calc_heading_x10 = live_fleet[i].heading_deg * 10;
            ui_slots[i].calc_is_military = live_fleet[i].is_military;
            ui_slots[i].calc_color = live_fleet[i].is_military ? lv_color_hex(0xFF3333)
                                                                 : get_fr24_altitude_color(live_fleet[i].altitude_ft);

            snprintf(ui_slots[i].calc_id, sizeof(ui_slots[i].calc_id), "%s",
                     live_fleet[i].callsign[0] ? live_fleet[i].callsign : live_fleet[i].hex);
            ui_slots[i].calc_altitude_ft = live_fleet[i].altitude_ft;
            snprintf(ui_slots[i].calc_model, sizeof(ui_slots[i].calc_model), "%s", live_fleet[i].model);
            ui_slots[i].calc_distance_km = live_fleet[i].distance_km;
            snprintf(ui_slots[i].calc_vsi_str, sizeof(ui_slots[i].calc_vsi_str), "%s", live_fleet[i].vsi_str);
            ui_slots[i].calc_speed_kt = live_fleet[i].speed_kt;
            ui_slots[i].calc_heading_deg = live_fleet[i].heading_deg;
            ui_slots[i].calc_lat = live_fleet[i].lat;
            ui_slots[i].calc_lon = live_fleet[i].lon;

            ui_slots[i].calc_list_y = list_y_offset;
            list_y_offset += 65;

            // Wektor sladu lotu - liczba punktow ograniczona przez trail_len
            // z NVS (panel WWW). trail_len==0 -> slad calkowicie wylaczony.
            AircraftTrackHistory *th = (trail_len > 0) ? find_aircraft_track(live_fleet[i].hex) : NULL;
            if(th && th->pt_count >= 1) {
                int cap = trail_len;
                if (cap > MAX_TRACK_POINTS) cap = MAX_TRACK_POINTS;
                int pts_to_draw = th->pt_count > cap ? cap : th->pt_count;
                int start_idx = th->pt_count - pts_to_draw; // najnowsze pts_to_draw punktow
                for(int p = 0; p < pts_to_draw; p++) {
                    float p_dist, p_bear;
                    calculate_coords(th->lats[start_idx + p], th->lons[start_idx + p], &p_dist, &p_bear);
                    float p_rad = p_bear * DEG_TO_RAD;
                    ui_slots[i].vec_pts_calc[p].x = RADAR_CENTER_X + (int)(p_dist * scale * sinf(p_rad));
                    ui_slots[i].vec_pts_calc[p].y = RADAR_CENTER_Y - (int)(p_dist * scale * cosf(p_rad));
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

    // Segmenty sladu HUD (tylko wybrany samolot)
    int drawn_segs = 0;
    if(selected_ac) {
        AircraftTrackHistory *sth = find_aircraft_track(selected_ac->hex);
        if(sth && sth->pt_count >= 2) {
            int total_pts = sth->pt_count > MAX_TRACK_POINTS ? MAX_TRACK_POINTS : sth->pt_count;
            int max_segs = total_pts - 1;
            if(max_segs > MAX_HUD_SEGS) max_segs = MAX_HUD_SEGS;

            for(int s = 0; s < max_segs; s++) {
                float d1, b1, d2, b2;
                calculate_coords(sth->lats[s], sth->lons[s], &d1, &b1);
                calculate_coords(sth->lats[s+1], sth->lons[s+1], &d2, &b2);

                hud_seg_pts_calc[s][0].x = RADAR_CENTER_X + (int)(d1 * scale * sinf(b1 * DEG_TO_RAD));
                hud_seg_pts_calc[s][0].y = RADAR_CENTER_Y - (int)(d1 * scale * cosf(b1 * DEG_TO_RAD));
                hud_seg_pts_calc[s][1].x = RADAR_CENTER_X + (int)(d2 * scale * sinf(b2 * DEG_TO_RAD));
                hud_seg_pts_calc[s][1].y = RADAR_CENTER_Y - (int)(d2 * scale * cosf(b2 * DEG_TO_RAD));
                hud_seg_color_calc[s] = get_fr24_altitude_color(sth->alts[s]);
                drawn_segs++;
            }
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
        snprintf(sel_id, sizeof(sel_id), "%s", selected_ac->callsign[0] ? selected_ac->callsign : selected_ac->hex);
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

    // ---- Faza "apply": tylko settery LVGL na gotowych danych, krotki bsp_display_lock. ----
    if (!bsp_display_lock(200)) {
        ESP_LOGW(TAG, "radar_ui_refresh: timeout bsp_display_lock, pomijam klatke");
        xSemaphoreGive(g_refresh_serialize_mutex);
        return;
    }

    // Pozycjonowanie lotnisk (dane statyczne, nie wymagaja adsb_service_lock)
    for (size_t i = 0; i < NUM_AIRPORTS; i++) {
        if (show_airports && (ui_airports[i].distance_km <= current_range_km)) {
            float rad = ui_airports[i].bearing_deg * DEG_TO_RAD;
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
        lv_obj_set_style_image_recolor(ui_slots[i].radar_icon, lv_color_hex(0xFF3333), 0);
        lv_obj_set_style_image_recolor_opa(ui_slots[i].radar_icon, ui_slots[i].calc_is_military ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        lv_obj_set_pos(ui_slots[i].radar_icon, ui_slots[i].calc_px - ui_slots[i].calc_half_w, ui_slots[i].calc_py - ui_slots[i].calc_half_h);
        lv_obj_set_pos(ui_slots[i].radar_label, ui_slots[i].calc_px + ui_slots[i].calc_half_w + 6, ui_slots[i].calc_py - ui_slots[i].calc_half_h);
        lv_image_set_rotation(ui_slots[i].radar_icon, ui_slots[i].calc_heading_x10);

        lv_label_set_text_fmt(ui_slots[i].radar_label, "%s\n%d ft", ui_slots[i].calc_id, ui_slots[i].calc_altitude_ft);
        lv_obj_set_style_text_color(ui_slots[i].radar_label, ui_slots[i].calc_color, 0);

        lv_obj_set_hidden(ui_slots[i].list_item, false);
        lv_obj_set_pos(ui_slots[i].list_item, 0, ui_slots[i].calc_list_y);

        lv_image_set_src(ui_slots[i].list_icon, ui_slots[i].calc_dsc);
        lv_image_set_pivot(ui_slots[i].list_icon, ui_slots[i].calc_half_w, ui_slots[i].calc_half_h);
        lv_image_set_rotation(ui_slots[i].list_icon, ui_slots[i].calc_heading_x10);
        lv_obj_set_style_image_recolor(ui_slots[i].list_icon, lv_color_hex(0xFF3333), 0);
        lv_obj_set_style_image_recolor_opa(ui_slots[i].list_icon, ui_slots[i].calc_is_military ? LV_OPA_COVER : LV_OPA_TRANSP, 0);

        if (ui_slots[i].calc_is_military) {
            lv_label_set_text_fmt(ui_slots[i].list_lbl_cs, "%s [MIL]", ui_slots[i].calc_id);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_cs, lv_color_hex(0xFF3333), 0);
        } else {
            lv_label_set_text(ui_slots[i].list_lbl_cs, ui_slots[i].calc_id);
            lv_obj_set_style_text_color(ui_slots[i].list_lbl_cs, lv_color_hex(0xffffff), 0);
        }
        lv_label_set_text_fmt(ui_slots[i].list_lbl_sub, "%s  %.1fKM  %d FT  %s  %dKT",
                              ui_slots[i].calc_model, ui_slots[i].calc_distance_km, ui_slots[i].calc_altitude_ft,
                              ui_slots[i].calc_vsi_str, ui_slots[i].calc_speed_kt);
        lv_label_set_text_fmt(ui_slots[i].list_lbl_route, "HDG: %d deg  LAT: %.2f  LON: %.2f",
                              ui_slots[i].calc_heading_deg, ui_slots[i].calc_lat, ui_slots[i].calc_lon);
    }

    lv_label_set_text_fmt(lbl_status_count, T(STR_TRAFFIC_FMT), visible_count, (int)current_range_km);

    for(int s = 0; s < drawn_segs; s++) {
        memcpy(hud_seg_pts[s], hud_seg_pts_calc[s], sizeof(hud_seg_pts[s]));
        lv_obj_set_style_line_color(hud_trail_segs[s], hud_seg_color_calc[s], 0);
        lv_line_set_points(hud_trail_segs[s], hud_seg_pts[s], 2);
        lv_obj_set_hidden(hud_trail_segs[s], false);
    }
    for(int s = drawn_segs; s < MAX_HUD_SEGS; s++) {
        lv_obj_set_hidden(hud_trail_segs[s], true);
    }

    if (has_selection) {
        lv_obj_set_hidden(popup_card_cont, false);

        lv_label_set_text_fmt(popup_lbl_count, "%d", visible_count);

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

        lv_label_set_text_fmt(popup_lbl_title, "%s  %s  [%s]", sel_id, sel_model, sel_reg);

        lv_label_set_text_fmt(popup_lbl_col_left, "ALT  %d ft\nSPD  %d kt\nDIST %.1f km", sel_alt, sel_spd, sel_dist);

        lv_label_set_text_fmt(popup_lbl_col_right, "V/S  %s\nHDG  %03d\nSQK  %s", sel_vsi, sel_hdg, sel_squawk);

        lv_label_set_text_fmt(popup_lbl_route, "LAT: %.2f  LON: %.2f  (%s)", sel_lat, sel_lon, g_station_name);
    } else {
        lv_obj_set_hidden(popup_card_cont, true);
        popup_photo_release();
        stop_loading_pulse(popup_lbl_credit);
        popup_photo_hex[0] = '\0';
    }

    bsp_display_unlock();
    xSemaphoreGive(g_refresh_serialize_mutex);
}

// ================= ZDARZENIA PRZYCISKOW =================
static void switch_range(void) {
    current_range_idx = (current_range_idx + 1) % NUM_RANGE_STEPS;
    float current_range = range_steps[current_range_idx];

    bsp_display_lock(0);
    lv_label_set_text_fmt(lbl_range_header, "RNG: %.0fKM", current_range);
    lv_label_set_text_fmt(lbl_range_scope, "%.0fKM", current_range);
    lv_label_set_text_fmt(lbl_range_pill, "<> %.0f km", current_range);
    bsp_display_unlock();

    radar_ui_refresh();
}

static void range_click_event_cb(lv_event_t *e) {
    switch_range();
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

static void filter_click_event_cb(lv_event_t *e) {
    air_filter_mode = (air_filter_mode + 1) % 3;
    bsp_display_lock(0);
    apply_air_filter_style();
    bsp_display_unlock();

    radar_ui_refresh();
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

    radar_ui_refresh();
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

static void gnd_toggle_click_event_cb(lv_event_t *e) {
    hide_ground_traffic = !hide_ground_traffic;
    bsp_display_lock(0);
    apply_gnd_filter_style();
    bsp_display_unlock();

    radar_ui_refresh();
}

static void select_aircraft_by_index(int idx) {
    if (!adsb_service_lock(100)) return;

    bool need_refresh = false;
    if(live_fleet && idx >= 0 && idx < MAX_AIRCRAFT_CAPACITY && live_fleet[idx].active) {
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
        need_refresh = true;
    }

    adsb_service_unlock();

    if (need_refresh) radar_ui_refresh();
}

static void aircraft_click_event_cb(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    select_aircraft_by_index(idx);
}

void radar_ui_set_aircraft_photo(const lv_image_dsc_t *img_dsc, const char *photographer, const char *hex, bool is_type_fallback) {
    if (!bsp_display_lock(500)) {
        ESP_LOGW(TAG, "radar_ui_set_aircraft_photo: timeout bsp_display_lock, odrzucam wynik");
        if (img_dsc) {
            heap_caps_free((void *)img_dsc->data);
            heap_caps_free((void *)img_dsc);
        }
        return;
    }

    // Uzytkownik zdazyl zamknac popup albo wybrac inny samolot zanim zdjecie
    // sie pobralo - wynik jest juz nieaktualny.
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

        // Kadrowanie "cover" na wzor FR24 - zdjecie wypelnia caly kafelek
        // (300x165, wewnatrz paddingu ~292x157), popup_top_box przycina
        // nadmiar poza wlasnymi granicami.
        int target_w = 292, target_h = 157;
        uint32_t scale_w = (uint32_t)target_w * 256 / img_dsc->header.w;
        uint32_t scale_h = (uint32_t)target_h * 256 / img_dsc->header.h;
        uint32_t scale = scale_w > scale_h ? scale_w : scale_h;

        lv_image_set_src(popup_img_preview, popup_photo_dsc);
        lv_image_set_scale(popup_img_preview, scale);
        lv_image_set_rotation(popup_img_preview, 0);

        if (is_type_fallback && photographer && photographer[0]) {
            // Zdjecie poglądowe danego typu (fallback po ICAO type code, nie
            // konkretnego egzemplarza) - photo_service juz sklada pelny
            // podpis "[Model] <kod>", tu tylko go wyswietlamy.
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

// ================= BUDOWA STRUKTURY LVGL =================
bool radar_ui_init(void) {
    ui_slots = (AircraftSlotUI *)heap_caps_calloc(MAX_AIRCRAFT_CAPACITY, sizeof(AircraftSlotUI), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ui_slots) {
        ESP_LOGE(TAG, "Nie udalo sie zaalokowac pamieci slotow UI w PSRAM!");
        return false;
    }
    g_refresh_serialize_mutex = xSemaphoreCreateMutex();
    if (!g_refresh_serialize_mutex) {
        ESP_LOGE(TAG, "Nie udalo sie utworzyc g_refresh_serialize_mutex!");
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
    lv_obj_set_style_pad_column(top_bar, 6, 0);
    lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(top_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    btn_airport_toggle = lv_obj_create(top_bar);
    lv_obj_set_size(btn_airport_toggle, 0, 26);
    lv_obj_set_flex_grow(btn_airport_toggle, 1);
    lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_airport_toggle, 1, 0);
    lv_obj_set_style_radius(btn_airport_toggle, 6, 0);
    lv_obj_set_style_pad_all(btn_airport_toggle, 0, 0);
    lv_obj_add_flag(btn_airport_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_airport_toggle, airport_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_airport_toggle = lv_label_create(btn_airport_toggle);
    lv_label_set_text(lbl_airport_toggle, show_airports ? "APTS: ON" : "APTS: OFF");
    lv_obj_set_style_text_color(lbl_airport_toggle, lv_color_hex(0x00ff88), 0);
    lv_obj_center(lbl_airport_toggle);
    if (!show_airports) {
        lv_obj_set_style_bg_color(btn_airport_toggle, lv_color_hex(0x550000), 0);
        lv_obj_set_style_border_color(btn_airport_toggle, lv_color_hex(0xff5555), 0);
    }

    btn_filter_ground = lv_obj_create(top_bar);
    lv_obj_set_size(btn_filter_ground, 0, 26);
    lv_obj_set_flex_grow(btn_filter_ground, 1);
    lv_obj_set_style_bg_color(btn_filter_ground, lv_color_hex(0x002b11), 0);
    lv_obj_set_style_border_color(btn_filter_ground, lv_color_hex(0x005522), 0);
    lv_obj_set_style_border_width(btn_filter_ground, 1, 0);
    lv_obj_set_style_radius(btn_filter_ground, 6, 0);
    lv_obj_set_style_pad_all(btn_filter_ground, 0, 0);
    lv_obj_add_flag(btn_filter_ground, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_filter_ground, filter_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_filter_ground = lv_label_create(btn_filter_ground);
    lv_obj_center(lbl_filter_ground);
    apply_air_filter_style();

    btn_gnd_toggle = lv_obj_create(top_bar);
    lv_obj_set_size(btn_gnd_toggle, 0, 26);
    lv_obj_set_flex_grow(btn_gnd_toggle, 1);
    lv_obj_set_style_border_width(btn_gnd_toggle, 1, 0);
    lv_obj_set_style_radius(btn_gnd_toggle, 6, 0);
    lv_obj_set_style_pad_all(btn_gnd_toggle, 0, 0);
    lv_obj_add_flag(btn_gnd_toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(btn_gnd_toggle, gnd_toggle_click_event_cb, LV_EVENT_CLICKED, NULL);

    lbl_gnd_toggle = lv_label_create(btn_gnd_toggle);
    lv_obj_center(lbl_gnd_toggle);
    apply_gnd_filter_style();

    lv_obj_t *btn_range = lv_obj_create(top_bar);
    lv_obj_set_size(btn_range, 0, 26);
    lv_obj_set_flex_grow(btn_range, 1);
    lv_obj_clear_flag(btn_range, LV_OBJ_FLAG_SCROLLABLE);
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
    lv_obj_add_flag(list_cont, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(list_cont, popup_close_event_cb, LV_EVENT_CLICKED, NULL);

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
    lv_obj_set_style_bg_color(popup_top_box, lv_color_hex(0x0a0e14), 0);
    lv_obj_set_style_border_color(popup_top_box, lv_color_hex(0x38bdf8), 0);
    lv_obj_set_style_border_width(popup_top_box, 1, 0);
    lv_obj_set_style_radius(popup_top_box, 10, 0);
    lv_obj_set_style_pad_all(popup_top_box, 0, 0);
    lv_obj_clear_flag(popup_top_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_clip_corner(popup_top_box, true, 0);

    popup_lbl_count = lv_label_create(popup_top_box);
    lv_label_set_text(popup_lbl_count, "0");
    lv_obj_set_style_text_color(popup_lbl_count, lv_color_hex(0xffffff), 0);
    lv_obj_align(popup_lbl_count, LV_ALIGN_TOP_LEFT, 8, 4);

    popup_img_preview = lv_image_create(popup_top_box);
    lv_image_set_src(popup_img_preview, &aircraft_yellow_25);
    lv_obj_set_size(popup_img_preview, 300, 165);
    lv_image_set_inner_align(popup_img_preview, LV_IMAGE_ALIGN_CENTER);
    lv_obj_align(popup_img_preview, LV_ALIGN_CENTER, 0, 0);

    // Etykieta autora zdjecia (styl FR24) - polprzezroczysta plakietka w lewym
    // dolnym rogu kafelka ze zdjeciem; w trakcie ladowania/braku zdjecia
    // wyswietla tez status.
    popup_lbl_credit = lv_label_create(popup_top_box);
    lv_label_set_text(popup_lbl_credit, "RADAR STATION FEED");
    lv_obj_set_style_text_color(popup_lbl_credit, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_bg_color(popup_lbl_credit, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(popup_lbl_credit, LV_OPA_70, 0);
    lv_obj_set_style_pad_hor(popup_lbl_credit, 6, 0);
    lv_obj_set_style_pad_ver(popup_lbl_credit, 2, 0);
    lv_obj_set_style_radius(popup_lbl_credit, 4, 0);
    lv_obj_align(popup_lbl_credit, LV_ALIGN_BOTTOM_LEFT, 8, -8);

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

    bsp_display_unlock();
}
