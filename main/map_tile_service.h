#pragma once

#include <stdbool.h>

#include "lvgl.h"

// Allocates a persistent PSRAM buffer for the map canvas and starts the
// FreeRTOS task that fetches/decodes tiles. No LVGL dependency - call once
// at application startup, before radar_ui_build(). Returns false if the
// buffer allocation or queue/task creation failed.
bool map_tile_service_init(void);

// Creates an lv_canvas on the buffer from map_tile_service_init() as the
// first (deepest) child of the given radar_area container - must be called
// before any other objects are added to radar_area, so the map layer stays
// beneath the grid/rings/aircraft. Requires bsp_display_start() to have run.
void map_tile_service_set_canvas_parent(lv_obj_t *radar_area);

// Requests (asynchronously, in the background) a tile reload for the given
// radar center and range. Debounced: the actual fetch only starts 1.5s
// after the last call with no further call in between, so a burst of rapid
// RNG/zoom changes triggers only one tile grid download instead of one per
// call - see MAP_RELOAD_DEBOUNCE_US in map_tile_service.c.
void map_tile_service_request_reload(float center_lat, float center_lon, float range_km);

// Shows/hides the map layer (the tile-fetching task keeps running).
void map_tile_service_set_enabled(bool on);
bool map_tile_service_is_enabled(void);

// OSM zoom level of the tile grid currently drawn on the canvas - lets
// other modules (e.g. radar_geo_to_screen_px() in radar_ui.c) project
// lat/lon to screen pixels with the exact same Web Mercator zoom the map
// background itself is rendered at. -1 if no reload has completed yet.
int map_tile_service_get_current_zoom(void);

// True while a tile grid reload is actively fetching (from the start of a
// reload until the last tile attempt finishes) - lets adsb_service.c skip
// its own polling while the map download owns the shared HTTPS mutex/SDIO
// bandwidth. See radar_ui_show_map_loading() for the matching status bubble.
bool map_tile_is_downloading(void);

// Suspends/resumes tile fetching (the worker task keeps running, it just
// skips fetching and sleeps) - used by ota_update_service.c to keep this
// task off the network entirely while esp_https_ota() is streaming/
// flashing firmware. A tile download already in flight is aborted within a
// few chunks - the owning task itself checks the pause flag and bails out,
// no cross-task client access. Safe to call from any task.
void map_tile_service_pause(void);
void map_tile_service_resume(void);
