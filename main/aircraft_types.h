#pragma once

#include <stdint.h>
#include <stdbool.h>

// Checks whether an ICAO type code (ADS-B "t" field, e.g. "EC35", "H125",
// "R44") corresponds to a helicopter - full code table in aircraft_types.c.
bool is_helicopter(const char *type);

// ================= RADAR & NETWORK CONFIGURATION =================
// Hard, static size of the buffers (live_fleet/temp_fleet/ui_slots) in PSRAM.
// The actually parsed/displayed aircraft count is further capped at runtime
// by wifi_mgr_get_max_aircraft() (NVS/web panel, 10-200).
#define MAX_AIRCRAFT_CAPACITY    200
#define MAX_SEEN_DB              512
// Upper bound on flight trail length - the actually displayed point count is
// further capped at runtime by wifi_mgr_get_trail_len() (NVS/web panel, max
// 120). MAX_TRACK_HISTORY_PTS is the per-aircraft history buffer capacity in
// track_db (PSRAM), MAX_TRACK_POINTS is the per-UI-slot trail line render
// buffer - both must be >= the largest allowed trail_len value.
#define MAX_TRACK_POINTS         120
#define MAX_TRACK_HISTORY_PTS    120
#define MAX_HUD_SEGS             32
#define PLANE_TIMEOUT_MS         45000
#define API_FETCH_SEC            4

#define SCREEN_WIDTH             1024
#define SCREEN_HEIGHT            600
#define MAP_SIZE                 580
#define RADAR_CENTER_X           290
#define RADAR_CENTER_Y           290
#define RADAR_MAX_RADIUS         260

#define DEG_TO_RAD               0.017453292519943295f
#define RAD_TO_DEG               57.29577951308232f

#define HTTP_BUFFER_SIZE         (256 * 1024)
// ===============================================================

typedef enum {
    AC_SMALL = 0,
    AC_NORMAL,
    AC_LARGE,
    AC_HELI
} AircraftType;

typedef enum {
    AIR_FILTER_ALL = 0,    // All aircraft
    AIR_FILTER_CIVIL = 1,  // Civil / airliner only (is_military == false)
    AIR_FILTER_MIL = 2     // Military only (is_military == true)
} air_filter_mode_t;

typedef struct {
    char hex[8];
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    int pt_count;
    float lats[MAX_TRACK_HISTORY_PTS];
    float lons[MAX_TRACK_HISTORY_PTS];
    int   alts[MAX_TRACK_HISTORY_PTS];
    uint32_t times[MAX_TRACK_HISTORY_PTS];
} AircraftTrackHistory;

typedef struct {
    char hex[8];
    char callsign[12];
    char registration[16];
    char model[8];
    char category[4];
    char squawk[8];
    char vsi_str[10];
    int vsi_fpm;
    float lat;
    float lon;
    float distance_km;
    float bearing_deg;
    int altitude_ft;
    int speed_kt;
    int heading_deg;
    AircraftType type;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    bool active;
    bool is_military;
    bool on_ground;
} AircraftData;

// Airport database (global_airports) moved to main/airports.h / airports.c.

// Radar range steps in km, widest first (index 0 is the default/initial
// range) - each one maps 1:1 to a fixed OSM zoom level in
// map_tile_service.c's RANGE_TO_ZOOM table. Keep in sync with
// RADAR_RANGE_STEPS_KM in wifi_manager.c and RANGE_OPTIONS in mqtt_service.c.
static const float range_steps[] = {400.0f, 200.0f, 100.0f, 50.0f, 25.0f};
#define NUM_RANGE_STEPS (sizeof(range_steps) / sizeof(range_steps[0]))
