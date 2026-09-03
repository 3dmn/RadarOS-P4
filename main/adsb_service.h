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

// Fetched flight trace for the "Flight Trace (API)" aircraft click action -
// written by the async trace fetch task, read by radar_ui under
// adsb_service_lock()/unlock() (same mutex as live_fleet). g_trace_hex is
// empty (or does not match the currently selected aircraft) while no trace
// is loaded / a fetch is still in flight - callers must compare it against
// the selected hex before drawing g_trace_points.
//
// Once the initial history fetch completes, adsb_worker_task appends the
// selected aircraft's current position to g_trace_points on every polling
// cycle (live breadcrumbs), so the drawn trace keeps following turns instead
// of jumping straight from the last fetched history point to the live
// position. g_trace_points is a FIFO ring past MAX_TRACE_POINTS - oldest
// points are dropped so the trace can keep growing indefinitely.
#define MAX_TRACE_POINTS 400

typedef struct {
    float lat;
    float lon;
    int   alt_ft;
} TracePoint;

extern TracePoint *g_trace_points;
extern int g_trace_point_count;
extern char g_trace_hex[8];

// Starts an asynchronous HTTP GET of the flight trace for the given ICAO hex
// (globe.adsb.fi, falling back to globe.airplanes.live) - debounced by
// 300ms and cancels/replaces any request already in flight (a rapid
// re-click never opens more than one HTTPS session). Safe to call from any
// task.
void adsb_service_request_trace(const char *hex);

// Clears the trace buffer and cancels/discards a pending fetch - call when
// the selection is cleared or changed away from the Flight Trace action.
void adsb_service_clear_trace(void);

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
