# ESP32-P4 ADS-B Flight Radar HUD ✈️📡

## 🇬🇧 English Documentation

### Overview
High-performance standalone ADS-B flight radar station built on ESP32-P4 with a 7" Touch LCD display and ESP-Hosted Wi-Fi 6. Real-time air traffic tracking, flight path visualization, aircraft database integration, and intuitive web configuration.

### Key Features
- **High-Performance Radar Display (LVGL 9):**
  - Smooth 60 FPS vector HUD rendering with concentric range rings and bearing markers.
  - Heading-oriented aircraft icons with dedicated rotary-wing icons for helicopters (AS350, R44, Bell, H60, Mi, etc.).
  - Altitude-based color telemetry (climb/descent rates, flight level, ground speed, squawk).
- **Interactive Aircraft Popup & 3-Tier Photo Engine:**
  - Tap any aircraft to display full flight parameters and high-res photography.
  - Tier 1 & 2: Real airframe photos queried by ICAO Hex / Registration (Planespotters & Airport-Data API) with photographer credit.
  - Tier 3: Wikipedia REST API fallback by ICAO aircraft type (e.g., C208, B738, A320, AS50) with `[Model]` tag.
  - Dismiss popup by tapping anywhere on the radar screen.
- **On-Screen Touch HUD Controls:**
  - `APTS` (Airports overlay ON / OFF).
  - `AIR` (Traffic filter: ALL / CIVIL / MILITARY).
  - `GND` (Ground traffic filter: Show / Hide aircraft on runway/apron).
  - `RNG` (Dynamic range cycling: 10, 20, 30, 50, 100, 150, 200, 250 km).
- **Flight Trail & History:**
  - Real-time flight paths stored in 32 MB PSRAM (configurable: 0, 15, 30, 60, 120 points).
- **High Aircraft Capacity:**
  - Hardware support for up to 200 simultaneous aircraft in active memory.
- **Modern Responsive Web Configuration Panel (HTTP Server):**
  - Card-based dark UI with multi-language support (English / Polish).
  - Wi-Fi network scanner (`/scan`) with SSID selector.
  - Interactive map-based coordinate picker for exact radar station positioning.
  - Brightness control slider, startup defaults, NVS persistence, and remote reboot.

### Hardware Requirements
- **MCU:** Espressif ESP32-P4 (RISC-V dual-core @ 400 MHz, 32 MB PSRAM).
- **Display:** 7-inch DSI Touch LCD (1024x600).
- **Connectivity:** ESP32-C6 / ESP-Hosted SDIO Wi-Fi 6 co-processor.

### Build & Flash
```bash
idf.py set-target esp32p4
idf.py build
idf.py -p PORT flash monitor
```

---

## 🇵🇱 Dokumentacja Polska

### Opis projektu
Wydajna, samodzielna stacja radarowa ADS-B oparta na układzie ESP32-P4 z 7-calowym ekranem dotykowym LCD i łącznością Wi-Fi 6 (ESP-Hosted). Umożliwia śledzenie ruchu lotniczego w czasie rzeczywistym, wizualizację tras, pobieranie zdjęć maszyn oraz wygodną konfigurację przez panel WWW.

### Główne Funkcje
- **Zaawansowany Interfejs Radaru (LVGL 9):**
  - Płynne renderowanie wektorowe HUD w 60 FPS z okręgami zasięgu i kompasem.
  - Ikony zorientowane wg kursu (HDG) z dedykowanymi symbolami śmigłowców (AS50, R44, Bell, Black Hawk H60 itp.).
  - Kolorystyka i telemetria zależna od pułapu (prędkość pionowa V/S, pułap ALT, prędkość SPD, kod SQK).
- **Interaktywny Popup i 3-stopniowy Silnik Zdjęć:**
  - Kliknięcie w dowolny statek powietrzny otwiera szczegółowy kafelek z parametrami i zdjęciem.
  - Stopień 1 i 2: Rzeczywiste zdjęcia konkretnego egzemplarza po kodzie HEX i rejestracji (Planespotters / Airport-Data) z podpisem autora `(c)`.
  - Stopień 3: Pobieranie zdjęć poglądowych z REST API Wikipedii po kodzie ICAO typu (np. C208, B738, AS50) z oznaczeniem `[Model]`.
  - Dotknięcie pustego tła natychmiast zamyka okienko.
- **Dotykowy Pasek Kontrolny (HUD):**
  - `APTS` (Przełączanie widoczności pobliskich lotnisk).
  - `AIR` (Filtr statków: WSZYSTKIE / CYWILNE / WOJSKOWE).
  - `GND` (Ruch naziemny: Ukryj / Pokaż maszyny na płycie lotniska).
  - `RNG` (Szybka zmiana skali: 10, 20, 30, 50, 100, 150, 200, 250 km).
- **Ślad Lotu (Flight Trail):**
  - Płynne rysowanie historii trasy w pamięci PSRAM (konfigurowalne: 0, 15, 30, 60, 120 punktów).
- **Pojemność do 200 Maszyn:**
  - Sprzętowa obsługa śledzenia do 200 samolotów jednocześnie.
- **Panel Konfiguracyjny WWW:**
  - Nowoczesny, responsywny interfejs w ciemnym motywie z obsługą języków EN i PL.
  - Skaner sieci Wi-Fi (`/scan`) z listą wyboru SSID.
  - Interaktywna mapa do precyzyjnego wyboru współrzędnych stacji bazowej.
  - Płynny suwak jasności ekranu, zapis konfiguracji w pamięci NVS oraz bezpieczny restart.

### Wymagania sprzętowe
- **MCU:** Espressif ESP32-P4 (dwurdzeniowy RISC-V @ 400 MHz, 32 MB PSRAM).
- **Wyświetlacz:** 7-calowy dotykowy LCD DSI (1024x600).
- **Łączność:** Koprocesor ESP32-C6 / ESP-Hosted SDIO Wi-Fi 6.

### Kompilacja i wgranie
```bash
idf.py set-target esp32p4
idf.py build
idf.py -p PORT flash monitor
```
