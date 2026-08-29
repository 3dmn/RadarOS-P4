#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "i18n.h"

#define STATION_NAME_LEN  32
#define WIFI_SSID_MAX_LEN 33
#define WIFI_PASS_MAX_LEN 65

extern float g_radar_lat;
extern float g_radar_lon;
extern char g_station_name[STATION_NAME_LEN];
extern char g_wifi_ssid[WIFI_SSID_MAX_LEN];
extern char g_wifi_pass[WIFI_PASS_MAX_LEN];

// Inicjalizuje NVS, wczytuje zapisana konfiguracje, probuje polaczyc sie
// z Wi-Fi w trybie STA (timeout). Jesli sie nie uda - uruchamia SoftAP
// "RadarADSB-Setup" wraz z panelem WWW konfiguracji (port 80).
void wifi_manager_init(void);

// Blokuje wywolujacy task do czasu uzyskania polaczenia Wi-Fi (WIFI_CONNECTED_BIT).
void wifi_manager_wait_connected(void);

// Jasnosc ekranu z NVS, zakres 10-100%.
uint8_t wifi_mgr_get_brightness(void);

// Domyslny startowy zasieg (RNG) w km z NVS.
int wifi_mgr_get_default_range(void);

// Domyslny tryb filtru AIR z NVS (0 = ALL, 1 = CIVIL, 2 = MIL).
uint8_t wifi_mgr_get_default_air_mode(void);

// Domyslny stan wyswietlania lotnisk z NVS (1 = ON, 0 = OFF).
uint8_t wifi_mgr_get_default_apts_mode(void);

// Czy domyslnie ukrywac ruch naziemny (samoloty on_ground) z NVS.
bool wifi_mgr_get_hide_ground(void);

// Czy domyslnie wyswietlac warstwe mapy w tle radaru (MAP) z NVS.
bool wifi_mgr_get_map_enabled(void);

// Czy pokazywac pulsujacy baner alarmowy przy wykryciu kodu awaryjnego
// squawk (7700/7600/7500) z NVS.
bool wifi_mgr_get_squawk_alert_enabled(void);

// Dlugosc sladu lotu (liczba punktow historii trasy rysowanych za
// samolotem) z NVS. 0 = slad wylaczony. Dozwolone wartosci: 0/15/30/60/120.
uint8_t wifi_mgr_get_trail_len(void);
void wifi_mgr_set_trail_len(uint8_t len);

// Maksymalna liczba jednoczesnie parsowanych/wyswietlanych samolotow z NVS
// (10-200, twardy limit sprzetowy MAX_AIRCRAFT_CAPACITY to 200).
uint16_t wifi_mgr_get_max_aircraft(void);
void wifi_mgr_set_max_aircraft(uint16_t max_val);

// Aktywny jezyk interfejsu (LCD + panel WWW) z NVS.
app_lang_t wifi_mgr_get_lang(void);

// Ustawia jezyk w pamieci (i synchronizuje i18n_set_lang()) - nie zapisuje
// samo w sobie do NVS, to robi save_settings_to_nvs() w panelu WWW.
void wifi_mgr_set_lang(app_lang_t lang);
