#pragma once

// Starts the FreeRTOS task that fetches and decodes aircraft photos from
// Planespotters.net. Call once at application startup.
void photo_service_start(void);

// Requests (asynchronously, in the background) a photo fetch for the given
// ICAO hex code. registration (the "r" field) and type_code (the "t" field,
// e.g. "B738", "AS50") may be NULL/empty - these are progressively weaker
// search criteria used in the fallback chain (hex -> reg -> type_code) when
// earlier ones find no photo. A new call before the previous one finishes
// overwrites the queued request - only the most recently clicked aircraft
// matters. The result is delivered to radar_ui_set_aircraft_photo().
void photo_service_request(const char *hex, const char *registration, const char *type_code);
