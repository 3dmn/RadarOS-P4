#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "aircraft_types.h"

// Flota samolotow aktualnie widocznych w zasiegu - wlasnosc adsb_service,
// odczyt z zewnatrz (UI) dozwolony wylacznie pod adsb_service_lock()/unlock().
extern AircraftData *live_fleet;
extern int total_aircraft_in_zone;

// Alokuje bufory PSRAM (live_fleet/temp_fleet/track_db) i tworzy g_data_mutex.
// Zwraca false, jesli alokacja sie nie powiodla (aplikacja powinna przerwac start).
bool adsb_service_init(void);

// Uruchamia task FreeRTOS cyklicznie odpytujacy adsb.fi / airplanes.live.
void adsb_service_start(void);

// Muteks chroniacy live_fleet/total_aircraft_in_zone. Timeout w ms; przy
// przekroczeniu loguje ostrzezenie (ESP_LOGW) i zwraca false.
bool adsb_service_lock(uint32_t timeout_ms);
void adsb_service_unlock(void);

uint32_t get_time_ms(void);
void calculate_coords(float lat, float lon, float *out_dist_km, float *out_bearing_deg);
AircraftTrackHistory* find_aircraft_track(const char *hex);
