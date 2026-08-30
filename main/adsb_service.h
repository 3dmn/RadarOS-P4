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

// Suspends/resumes the polling loop's API requests (the loop keeps running,
// it just skips fetching and waits on a notification/short poll instead) -
// used by map_tile_service.c to keep the ADS-B task off the network while a
// tile grid download is in progress, so the two never hold concurrent TLS
// sessions and starve the Wi-Fi SDIO driver's internal DMA buffers. Safe to
// call from any task.
void adsb_service_pause(void);
void adsb_service_resume(void);

// Wakes the polling task immediately (instead of waiting for its next 500 ms
// pause-poll or the regular API_FETCH_SEC interval) and marks the next
// successful fetch as the one that should call radar_ui_hide_loading() -
// used by map_tile_service.c right after adsb_service_resume(), at the end
// of its sequential map-tiles-then-ADS-B fetch. Safe to call from any task.
void adsb_service_request_immediate_fetch(void);

uint32_t get_time_ms(void);
void calculate_coords(float lat, float lon, float *out_dist_km, float *out_bearing_deg);
AircraftTrackHistory* find_aircraft_track(const char *hex);
