#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "i18n.h"
#include "airports.h"

#define STATION_NAME_LEN  32
#define WIFI_SSID_MAX_LEN 33
#define WIFI_PASS_MAX_LEN 65

extern float g_radar_lat;
extern float g_radar_lon;
extern char g_station_name[STATION_NAME_LEN];
extern char g_wifi_ssid[WIFI_SSID_MAX_LEN];
extern char g_wifi_pass[WIFI_PASS_MAX_LEN];

// Initializes NVS, loads the saved configuration, tries to connect to Wi-Fi
// in STA mode (with a timeout). On failure, starts the "RadarADSB-Setup"
// SoftAP along with the web configuration panel (port 80).
void wifi_manager_init(void);

// Blocks the calling task until a Wi-Fi connection is established
// (WIFI_CONNECTED_BIT).
void wifi_manager_wait_connected(void);

// Screen brightness from NVS, range 10-100%.
uint8_t wifi_mgr_get_brightness(void);

// Default startup range (RNG) in km from NVS.
int wifi_mgr_get_default_range(void);

// Default AIR filter mode from NVS (0 = ALL, 1 = CIVIL, 2 = MIL).
uint8_t wifi_mgr_get_default_air_mode(void);

// Default airport display state from NVS (1 = ON, 0 = OFF).
uint8_t wifi_mgr_get_default_apts_mode(void);

// Mask of airport types shown on the radar, from NVS (APT_TYPE_* bits from
// airports.h). All types enabled by default.
uint8_t wifi_mgr_get_apt_filter_mask(void);

// Whether to hide ground traffic (on_ground aircraft) by default, from NVS.
bool wifi_mgr_get_hide_ground(void);

// Whether to show the background map layer (MAP) by default, from NVS.
bool wifi_mgr_get_map_enabled(void);

// Whether to show the pulsing alert banner when an emergency squawk code
// (7700/7600/7500) is detected, from NVS.
bool wifi_mgr_get_squawk_alert_enabled(void);

// Flight trail length (number of track-history points drawn behind an
// aircraft) from NVS. 0 = trail disabled. Allowed values: 0/15/30/60/120.
uint8_t wifi_mgr_get_trail_len(void);
void wifi_mgr_set_trail_len(uint8_t len);

// Maximum number of simultaneously parsed/displayed aircraft from NVS
// (10-200; the hard hardware limit MAX_AIRCRAFT_CAPACITY is 200).
uint16_t wifi_mgr_get_max_aircraft(void);
void wifi_mgr_set_max_aircraft(uint16_t max_val);

// Active UI language (LCD + web panel) from NVS.
app_lang_t wifi_mgr_get_lang(void);

// Sets the language in memory (and syncs i18n_set_lang()) - does not save to
// NVS by itself; that's done by save_settings_to_nvs() in the web panel.
void wifi_mgr_set_lang(app_lang_t lang);
