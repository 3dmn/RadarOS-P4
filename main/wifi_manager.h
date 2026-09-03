#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "i18n.h"
#include "airports.h"

#define STATION_NAME_LEN  32
#define WIFI_SSID_MAX_LEN 33
#define WIFI_PASS_MAX_LEN 65
#define MQTT_HOST_LEN      64
#define MQTT_USER_LEN      32
#define MQTT_PASS_LEN      64
#define MQTT_DEVICE_ID_LEN 32
// Scratch buffer size for a single form field parsed out of the web panel's
// settings POST body (save_post_handler in wifi_manager.c) - sized for the
// largest field.
#define WEB_FORM_PARAM_LEN 192

extern float g_radar_lat;
extern float g_radar_lon;
extern char g_station_name[STATION_NAME_LEN];
extern char g_wifi_ssid[WIFI_SSID_MAX_LEN];
extern char g_wifi_pass[WIFI_PASS_MAX_LEN];

// MQTT broker connection settings from NVS - see mqtt_service.c. g_mqtt_pass
// is intentionally excluded from JSON config export/import (security).
extern char g_mqtt_host[MQTT_HOST_LEN];
extern uint16_t g_mqtt_port;
extern char g_mqtt_user[MQTT_USER_LEN];
extern char g_mqtt_pass[MQTT_PASS_LEN];
extern char g_mqtt_device_id[MQTT_DEVICE_ID_LEN];

// Initializes NVS, loads the saved configuration, tries to connect to Wi-Fi
// in STA mode (with a timeout). On failure, starts the "RadarADSB-Setup"
// SoftAP along with the web configuration panel (port 80).
void wifi_manager_init(void);

// Blocks the calling task until a Wi-Fi connection is established
// (WIFI_CONNECTED_BIT).
void wifi_manager_wait_connected(void);

// Non-blocking check of the same Wi-Fi connection state as
// wifi_manager_wait_connected() (WIFI_CONNECTED_BIT). Used to suppress
// MQTT error notifications while the underlying Wi-Fi link itself is down.
bool wifi_mgr_is_connected(void);

// Screen brightness from NVS, range 10-100%.
uint8_t wifi_mgr_get_brightness(void);

// Sets the LCD backlight brightness immediately (bsp_display_brightness_set)
// and persists it to NVS. Value is clamped to 10-100.
void wifi_mgr_set_brightness(uint8_t pct);

// Radar range (RNG) in km from NVS - also updated immediately (persisted)
// every time the RNG HUD button, web panel, or Home Assistant changes it,
// so the radar reopens in the same range it was left in after a reboot or
// power cycle.
int wifi_mgr_get_default_range(void);
void wifi_mgr_set_default_range(uint16_t range_km);

// AIR traffic filter mode from NVS (0 = ALL, 1 = CIVIL, 2 = MIL) - persisted
// immediately on every change (HUD/web/Home Assistant), same as the range above.
uint8_t wifi_mgr_get_default_air_mode(void);
void wifi_mgr_set_default_air_mode(uint8_t mode);

// Airport overlay (APTS) display state from NVS (1 = ON, 0 = OFF) -
// persisted immediately on every change (HUD/web/Home Assistant).
uint8_t wifi_mgr_get_default_apts_mode(void);
void wifi_mgr_set_default_apts_mode(bool on);

// Mask of airport types shown on the radar, from NVS (APT_TYPE_* bits from
// airports.h). All types enabled by default. Unlike the other "default_*"
// getters above, this one is read live every radar_ui_refresh() (no
// separate runtime copy), so the setter below takes effect immediately.
uint8_t wifi_mgr_get_apt_filter_mask(void);
void wifi_mgr_set_apt_filter_mask(uint8_t mask);

// Whether ground traffic (on_ground aircraft) is hidden, from NVS -
// persisted immediately on every change (HUD/web/Home Assistant).
bool wifi_mgr_get_hide_ground(void);
void wifi_mgr_set_hide_ground(bool hide);

// Whether the background map layer (MAP) is shown, from NVS - persisted
// immediately on every change (HUD/web/Home Assistant).
bool wifi_mgr_get_map_enabled(void);
void wifi_mgr_set_map_enabled(bool on);

// Whether to show the pulsing alert banner when an emergency squawk code
// (7700/7600/7500) is detected, from NVS. Read live every
// radar_ui_refresh() (no separate runtime copy), so the setter below takes
// effect immediately without a reboot.
bool wifi_mgr_get_squawk_alert_enabled(void);
void wifi_mgr_set_squawk_alert_enabled(bool on);
bool wifi_mgr_get_mil_priority_enabled(void);
void wifi_mgr_set_mil_priority_enabled(bool on);

// Action performed when an aircraft is tapped/clicked, from NVS - see
// aircraft_click_action_t in aircraft_types.h. The setter persists
// immediately (HUD/web/Home Assistant).
uint8_t wifi_mgr_get_click_action(void);
void wifi_mgr_set_click_action(uint8_t action);

// Maximum number of simultaneously parsed/displayed aircraft from NVS
// (10-100; the hard hardware buffer limit MAX_AIRCRAFT_CAPACITY, unrelated
// to this user-facing ceiling, is still 200).
uint16_t wifi_mgr_get_max_aircraft(void);
void wifi_mgr_set_max_aircraft(uint16_t max_val);

// Active UI language (LCD + web panel) from NVS.
app_lang_t wifi_mgr_get_lang(void);

// Sets the language (syncs i18n_set_lang()) and persists it to NVS
// immediately, so it survives a reboot regardless of the caller (web panel,
// Home Assistant/MQTT). Applying the new language to every static LCD/web
// string still requires a reboot - callers that change it live (e.g.
// mqtt_service's language command) are responsible for restarting.
void wifi_mgr_set_lang(app_lang_t lang);

// Whether the MQTT client and Home Assistant discovery should start, from NVS.
bool wifi_mgr_get_mqtt_enabled(void);

// Whether to publish Home Assistant MQTT Discovery config topics on
// connect, from NVS.
bool wifi_mgr_get_mqtt_ha_discovery(void);
