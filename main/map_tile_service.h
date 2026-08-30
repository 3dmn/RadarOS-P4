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
// radar center and range. A new call before the previous one finishes
// overwrites the queued request - only the most recently requested view
// matters.
void map_tile_service_request_reload(float center_lat, float center_lon, float range_km);

// Shows/hides the map layer (the tile-fetching task keeps running).
void map_tile_service_set_enabled(bool on);
bool map_tile_service_is_enabled(void);
