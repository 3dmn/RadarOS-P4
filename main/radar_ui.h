#pragma once

#include <stdbool.h>

#include "lvgl.h"
#include "aircraft_types.h"
#include "mqtt_service.h"

// Projects any lat/lon to a radar-relative screen pixel (RADAR_CENTER_X/Y
// origin) using the same Web Mercator world-pixel math and current zoom
// level map_tile_service.c renders the OSM tile background with - see the
// doc comment in radar_ui.c. Safe to call from any task; reads only
// g_radar_lat/g_radar_lon and map_tile_service_get_current_zoom().
void radar_geo_to_screen_px(double lat, double lon, int *out_x, int *out_y);

// Allocates the PSRAM buffers needed by UI views (aircraft slots). Call
// before radar_ui_build(). Returns false if allocation failed.
bool radar_ui_init(void);

// Builds the entire LVGL radar structure. Requires a successful prior
// bsp_display_start().
void radar_ui_build(void);

// Recomputes positions/labels from the current live_fleet (adsb_service)
// content and refreshes the LVGL views. Safe to call from any task - it
// synchronizes internally with adsb_service_lock()/bsp_display_lock().
void radar_ui_refresh(void);

// Called by photo_service once fetching/decoding an aircraft photo for a
// given hex finishes. Safe from any task (synchronizes with
// bsp_display_lock()). Takes ownership of img_dsc (and img_dsc->data,
// PSRAM) - radar_ui frees it later; if hex is no longer the currently
// selected aircraft, dsc is freed and discarded immediately. img_dsc==NULL
// means no photo found in the database (photographer is then ignored).
// is_type_fallback==true means the photo is only a representative image for
// the aircraft type (fallback by ICAO type code), not the specific airframe -
// photographer then carries the type code instead of a name.
void radar_ui_set_aircraft_photo(const lv_image_dsc_t *img_dsc, const char *photographer, const char *hex, bool is_type_fallback);

// ================= LIVE CONTROL API =================
// Shared by the on-screen HUD touch buttons and mqtt_service - both paths
// converge here so Home Assistant and the touchscreen always agree, and any
// change (from either source) triggers a display refresh plus an MQTT state
// publish. Safe to call from any task (internally synchronized with
// bsp_display_lock(), same as the touch event handlers).

// Jumps the radar scale to the closest supported step to km (see
// range_steps[] in aircraft_types.h) and persists it to NVS (wifi_manager's
// "default range"), so the radar reopens at the same range after a reboot.
void radar_ui_set_range_km(float km);
float radar_ui_get_range_km(void);

// AIR traffic filter (ALL/CIVIL/MIL) - persisted to NVS immediately.
void radar_ui_set_air_filter(air_filter_mode_t mode);
air_filter_mode_t radar_ui_get_air_filter(void);

// Ground traffic (GND) visibility - true shows aircraft on the ground, false
// hides them (matches the on-screen "GND: ON/OFF" label). Persisted to NVS
// immediately.
void radar_ui_set_show_ground(bool show);
bool radar_ui_get_show_ground(void);

// Nearby-airports overlay (APTS) - persisted to NVS immediately.
void radar_ui_set_airports_enabled(bool on);
bool radar_ui_get_airports_enabled(void);

// Background tile map layer (MAP) - persisted to NVS immediately. Thin
// wrapper over map_tile_service that also updates the on-screen MAP button
// style and notifies mqtt_service.
void radar_ui_set_map_enabled(bool on);
bool radar_ui_get_map_enabled(void);

// ================= WI-FI STATUS NOTIFICATION =================
// Overlay card reporting the current Wi-Fi connection state, styled to
// match the radar cockpit (dark translucent background, rounded corners,
// neon border). Safe to call from any task, including from wifi_manager's
// Wi-Fi event handler (not the LVGL task) and even before radar_ui_build()
// has run - the state is cached and applied once the card widget exists.

// Persistent card shown while the device serves its own SoftAP (setup
// mode). ap_password may be NULL/empty for an open network.
void radar_ui_wifi_notify_ap_mode(const char *ap_ssid, const char *ap_password);

// Persistent card shown while attempting to join a saved Wi-Fi network.
void radar_ui_wifi_notify_connecting(const char *ssid);

// Card shown after obtaining an IP address; auto-hides after ~3.5s.
void radar_ui_wifi_notify_connected(const char *ip_str);

// ================= MQTT STATUS NOTIFICATION =================
// Second overlay card (independent of the Wi-Fi one above, positioned below
// it) reporting the MQTT/Home Assistant broker connection state. The
// caller (mqtt_service) is only expected to invoke these while MQTT is
// enabled in the configuration. Safe to call from any task.

// Persistent card shown while attempting to (re)connect to the broker.
void radar_ui_mqtt_notify_connecting(void);

// Card shown once the broker connection is established; auto-hides after ~3.5s.
void radar_ui_mqtt_notify_connected(void);

// Card shown on a broker disconnect/error while Wi-Fi itself is still up;
// auto-hides after ~5s (or sooner, if superseded by a connected/connecting
// state).
void radar_ui_mqtt_notify_error(void);

// ================= HUD STATUS BADGE =================
// Compact, persistent badge in the bottom-left corner of radar_area showing
// small colored LEDs for Wi-Fi (always) and MQTT (only while enabled).
// Replaces the old "<> 250 km" range pill - range is already shown by the
// RNG button (top-right) and the range rings themselves. Safe to call from
// any task, including before radar_ui_build() has run (the state is cached
// and applied once the badge widget exists).

typedef enum {
    WIFI_STATUS_CONNECTED = 0,
    WIFI_STATUS_CONNECTING, // also used for AP/setup mode
    WIFI_STATUS_ERROR,      // disconnected / connection failure
} wifi_status_t;

void radar_ui_update_wifi_status(wifi_status_t status);

// enabled=false hides the MQTT LED entirely (and the badge shrinks to fit
// just the Wi-Fi indicator); status is only meaningful when enabled=true.
void radar_ui_update_mqtt_status(bool enabled, mqtt_conn_status_t status);

// Firmware update indicator, appended to the same HUD status badge - built
// the same way as the Wi-Fi/MQTT chips (a pulsing amber dot + "FW" label).
// Hidden by default; appears only once ota_update_service.c has confirmed a
// newer release is available. Tapping it shows a brief toast with the
// version number and release notes (when known). available=false hides the
// chip again (and the badge shrinks back down); latest_version/release_notes
// may be NULL/ignored when available=false.
void radar_ui_update_fw_status(bool available, const char *latest_version, const char *release_notes);
