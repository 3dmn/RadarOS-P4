#pragma once

#include <stdint.h>
#include <stdbool.h>

// Sprawdza, czy kod typu ICAO (pole "t" z ADS-B, np. "EC35", "H125", "R44")
// odpowiada smiglowcowi - pelna baza kodow w aircraft_types.c.
bool is_helicopter(const char *type);

// ================= KONFIGURACJA RADARU I SIECI =================
// Twardy, statyczny rozmiar buforow (live_fleet/temp_fleet/ui_slots) w PSRAM.
// Realnie parsowana/wyswietlana liczba samolotow jest dodatkowo ograniczana
// w locie przez wifi_mgr_get_max_aircraft() (NVS/panel WWW, 10-200).
#define MAX_AIRCRAFT_CAPACITY    200
#define MAX_SEEN_DB              512
// Gorny limit dlugosci sladu lotu - realnie wyswietlana liczba punktow jest
// dodatkowo ograniczana w locie przez wifi_mgr_get_trail_len() (NVS/panel
// WWW, max 120). MAX_TRACK_HISTORY_PTS to pojemnosc bufora historii per
// samolot w track_db (PSRAM), MAX_TRACK_POINTS to bufor renderowania linii
// sladu per slot UI - musza byc >= najwiekszej dopuszczalnej wartosci trail_len.
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

#define HTTP_BUFFER_SIZE         (128 * 1024)
// ===============================================================

typedef enum {
    AC_SMALL = 0,
    AC_NORMAL,
    AC_LARGE,
    AC_HELI
} AircraftType;

typedef enum {
    AIR_FILTER_ALL = 0,    // Wszystkie statki
    AIR_FILTER_CIVIL = 1,  // Tylko cywilne / pasazerskie (is_military == false)
    AIR_FILTER_MIL = 2     // Tylko wojskowe (is_military == true)
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

// Baza lotnisk (global_airports) przeniesiona do main/airports.h / airports.c.

static const float range_steps[] = {250.0f, 200.0f, 150.0f, 100.0f, 50.0f, 30.0f, 20.0f, 10.0f};
#define NUM_RANGE_STEPS (sizeof(range_steps) / sizeof(range_steps[0]))
