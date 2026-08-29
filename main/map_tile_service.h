#pragma once

#include <stdbool.h>

#include "lvgl.h"

// Alokuje trwaly bufor PSRAM na canvas mapy oraz uruchamia task FreeRTOS
// pobierajacy/dekodujacy kafelki. Bez zaleznosci od LVGL - wywolac raz przy
// starcie aplikacji, przed radar_ui_build(). Zwraca false, jesli alokacja
// bufora lub utworzenie kolejki/tasku sie nie powiodlo.
bool map_tile_service_init(void);

// Tworzy lv_canvas na buforze z map_tile_service_init() jako pierwsze
// (najglebiej polozone) dziecko podanego kontenera radar_area - musi byc
// wywolane zanim jakiekolwiek inne obiekty zostana dodane do radar_area,
// zeby warstwa mapy pozostala pod siatka/pierscieniami/samolotami. Wymaga
// wczesniejszego bsp_display_start().
void map_tile_service_set_canvas_parent(lv_obj_t *radar_area);

// Zleca (asynchronicznie, w tle) przeladowanie kafelkow dla podanego srodka
// i zasiegu radaru. Kolejne wywolanie przed zakonczeniem poprzedniego
// nadpisuje zadanie w kolejce - liczy sie tylko ostatnio zadany widok.
void map_tile_service_request_reload(float center_lat, float center_lon, float range_km);

// Pokazuje/ukrywa warstwe mapy (task pobierajacy kafelki dziala dalej).
void map_tile_service_set_enabled(bool on);
bool map_tile_service_is_enabled(void);
