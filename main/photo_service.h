#pragma once

// Uruchamia task FreeRTOS obslugujacy pobieranie i dekodowanie zdjec samolotow
// z Planespotters.net. Wywolac raz przy starcie aplikacji.
void photo_service_start(void);

// Zleca (asynchronicznie, w tle) pobranie zdjecia dla podanego kodu hex ICAO.
// registration (pole "r") i type_code (pole "t", np. "B738", "AS50") moga byc
// NULL/puste - to kolejne, coraz slabsze kryteria wyszukiwania stosowane w
// lancuchu fallback (hex -> reg -> type_code), gdy poprzednie nie znajda
// zdjecia. Kolejne wywolanie przed zakonczeniem poprzedniego nadpisuje
// zadanie w kolejce - liczy sie tylko ostatnio klikniety samolot. Wynik
// trafia do radar_ui_set_aircraft_photo().
void photo_service_request(const char *hex, const char *registration, const char *type_code);
