#pragma once

#include <stdbool.h>

#include "lvgl.h"

// Alokuje bufory PSRAM potrzebne widokom UI (sloty samolotow). Wywolac przed
// radar_ui_build(). Zwraca false, jesli alokacja sie nie powiodla.
bool radar_ui_init(void);

// Buduje cala strukture LVGL radaru. Wymaga wczesniej udanego bsp_display_start().
void radar_ui_build(void);

// Przelicza pozycje/etykiety na podstawie biezacej zawartosci live_fleet
// (adsb_service) i odswieza widoki LVGL. Bezpieczne do wywolania z dowolnego
// tasku - samo synchronizuje sie z adsb_service_lock()/bsp_display_lock().
void radar_ui_refresh(void);

// Wywolywane przez photo_service po zakonczeniu pobierania/dekodowania zdjecia
// samolotu o danym hex. Bezpieczne z dowolnego tasku (synchronizuje sie z
// bsp_display_lock()). Przejmuje wlasnosc img_dsc (oraz img_dsc->data, PSRAM) -
// radar_ui zwalnia je pozniej; jesli hex nie jest juz aktualnie wybrany, dsc
// jest natychmiast zwalniany i odrzucany. img_dsc==NULL oznacza brak zdjecia
// w bazie (photographer wtedy ignorowany). is_type_fallback==true oznacza, ze
// zdjecie jest tylko poglądowe dla danego typu (fallback po ICAO type code),
// nie konkretnego egzemplarza - photographer niesie wtedy kod typu, nie nazwisko.
void radar_ui_set_aircraft_photo(const lv_image_dsc_t *img_dsc, const char *photographer, const char *hex, bool is_type_fallback);
