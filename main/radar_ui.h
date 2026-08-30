#pragma once

#include <stdbool.h>

#include "lvgl.h"

// Allocates the PSRAM buffers needed by UI views (aircraft slots). Call
// before radar_ui_build(). Returns false if allocation failed.
bool radar_ui_init(void);

// Builds the entire LVGL radar structure. Requires a successful prior
// bsp_display_start().
void radar_ui_build(void);

// Recomputes positions/labels from the current live_fleet (adsb_service)
// content and refreshes the LVGL views. Safe to call from any task - it
// synchronizes internally with adsb_service_lock()/bsp_display_lock().
void radar_ui_refresh(void);

// Called by photo_service once fetching/decoding an aircraft photo for a
// given hex finishes. Safe from any task (synchronizes with
// bsp_display_lock()). Takes ownership of img_dsc (and img_dsc->data,
// PSRAM) - radar_ui frees it later; if hex is no longer the currently
// selected aircraft, dsc is freed and discarded immediately. img_dsc==NULL
// means no photo found in the database (photographer is then ignored).
// is_type_fallback==true means the photo is only a representative image for
// the aircraft type (fallback by ICAO type code), not the specific airframe -
// photographer then carries the type code instead of a name.
void radar_ui_set_aircraft_photo(const lv_image_dsc_t *img_dsc, const char *photographer, const char *hex, bool is_type_fallback);
