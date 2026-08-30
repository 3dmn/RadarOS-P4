#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "aircraft_types.h"

// Fleet of aircraft currently visible in range - owned by adsb_service,
// external (UI) reads only allowed under adsb_service_lock()/unlock().
extern AircraftData *live_fleet;
extern int total_aircraft_in_zone;

// Allocates PSRAM buffers (live_fleet/temp_fleet/track_db) and creates
// g_data_mutex. Returns false if allocation failed (the app should abort
// startup).
bool adsb_service_init(void);

// Starts the FreeRTOS task that cyclically polls adsb.fi / airplanes.live.
void adsb_service_start(void);

// Mutex protecting live_fleet/total_aircraft_in_zone. Timeout in ms; logs a
// warning (ESP_LOGW) and returns false if it expires.
bool adsb_service_lock(uint32_t timeout_ms);
void adsb_service_unlock(void);

uint32_t get_time_ms(void);
void calculate_coords(float lat, float lon, float *out_dist_km, float *out_bearing_deg);
AircraftTrackHistory* find_aircraft_track(const char *hex);
