#pragma once

typedef enum {
    LANG_EN = 0, // Domyslny
    LANG_PL = 1,
    LANG_COUNT
} app_lang_t;

typedef enum {
    STR_FETCHING_PHOTO = 0,
    STR_NO_PHOTO_DB,
    STR_NO_PHOTO,
    STR_CONNECTING_WIFI,
    STR_UPDATING_ADSB,
    STR_NO_TRAFFIC,
    STR_TRAFFIC_FMT,
    STR_WEB_LANGUAGE,
    STR_WEB_STATION_NAME,
    STR_WEB_LATITUDE,
    STR_WEB_LONGITUDE,
    STR_WEB_SELECT_ON_MAP,
    STR_WEB_BRIGHTNESS,
    STR_WEB_DEFAULT_RANGE,
    STR_WEB_AIR_FILTER,
    STR_WEB_APTS,
    STR_WEB_GND,
    STR_WEB_AIR_ALL,
    STR_WEB_AIR_CIVIL,
    STR_WEB_AIR_MIL,
    STR_WEB_APTS_ON,
    STR_WEB_APTS_OFF,
    STR_WEB_GND_HIDE,
    STR_WEB_GND_SHOW,
    STR_WEB_TRAIL_LEN,
    STR_WEB_TRAIL_0,
    STR_WEB_TRAIL_15,
    STR_WEB_TRAIL_30,
    STR_WEB_TRAIL_60,
    STR_WEB_TRAIL_120,
    STR_WEB_MAX_AIRCRAFT,
    STR_WEB_MAX_AIRCRAFT_HELP,
    STR_WEB_CARD_LANG,
    STR_WEB_CARD_LOCATION,
    STR_WEB_CARD_DISPLAY,
    STR_WEB_SCAN_FOUND_PREFIX,
    STR_WEB_SCAN_FOUND_SUFFIX,
    STR_WEB_SCAN_NONE,
    STR_WEB_SAVE_REBOOT,
    STR_WEB_PAGE_TITLE,
    STR_WEB_WIFI_SECTION,
    STR_WEB_SSID,
    STR_WEB_PASSWORD,
    STR_WEB_PASS_PLACEHOLDER,
    STR_WEB_SCAN_NETWORKS,
    STR_WEB_SCANNING,
    STR_WEB_SAVING_MSG,
    STR_WEB_REBOOTING_MSG,
    STR_WEB_SAVE_ERROR,
    STR_COUNT
} i18n_str_id_t;

// Ustawia aktywny jezyk dla T(). Wolane po odczycie z NVS
// (wifi_mgr_get_lang()) - bezpieczne do wywolania wielokrotnie.
void i18n_set_lang(app_lang_t lang);
app_lang_t i18n_get_lang(void);

// Zwraca przetlumaczony tekst dla biezacego jezyka (nigdy NULL).
const char *T(i18n_str_id_t id);
