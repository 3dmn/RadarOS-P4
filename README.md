# ESP32-P4 ADS-B Flight Radar HUD ✈️📡

Standalone real-time ADS-B air traffic radar station built on the Espressif ESP32-P4, with a 7" touch LCD, offline tile map, aircraft photography, military/rescue traffic highlighting, and an emergency squawk alert system.

## Table of Contents

- [🇬🇧 English Documentation](#-english-documentation)
  - [Overview](#overview)
  - [Key Features](#key-features)
  - [Web Configuration Panel](#web-configuration-panel)
  - [Technical Specifications](#technical-specifications)
  - [Build & Flash](#build--flash)
- [🇵🇱 Dokumentacja Polska](#-dokumentacja-polska)
  - [Opis projektu](#opis-projektu)
  - [Główne Funkcje](#główne-funkcje)
  - [Panel Konfiguracyjny WWW](#panel-konfiguracyjny-www)
  - [Parametry Techniczne](#parametry-techniczne)
  - [Kompilacja i Wgranie](#kompilacja-i-wgranie)

---

## 🇬🇧 English Documentation

### Overview
High-performance standalone ADS-B flight radar station built on ESP32-P4 with a 7" Touch LCD display and ESP-Hosted Wi-Fi 6. Real-time air traffic tracking, flight path visualization, aircraft database integration, and intuitive web configuration.

### Key Features

- **High-Performance Radar Display (LVGL 9):**
  - Smooth 60 FPS vector HUD rendering with concentric range rings and bearing markers.
  - Heading-oriented aircraft icons with dedicated rotary-wing icons for helicopters (AS350, R44, Bell, H60, Mi, etc.).
  - Altitude-based color telemetry (climb/descent rates, flight level, ground speed, squawk).
- **Multi-Layer Tile Map:**
  - Offline/cached map tile rendering on the ESP32-P4 display, layered directly beneath the vector radar scope.
- **Military & Special Mission Highlighting (`[MIL]`):**
  - Automatic detection and red highlighting of Polish Air Force (PLF), NATO (AWACS), RAF (RRR), tanker (LAGR, NCHO, MMF), SIGINT/recon (JAKE, FORTE, HOMER, REDEYE) and other allied callsign traffic.
- **Rescue Helicopters (LPR / SAR):**
  - Dedicated blue highlighting for Polish Medical Air Rescue (LPR / RAT) crews.
- **Emergency Squawk Alert System:**
  - Detects transponder codes 7700 (General Emergency), 7600 (Radio Failure) and 7500 (Hijack) with a flashing red warning banner at the top of the screen.
- **Vertical Trend Indicators:**
  - Climb (▲) / descent (▼) arrows on the target list when |VSI| > 500 ft/min, level flight shown as (—).
- **Flight Trails & History:**
  - Real-time flight paths stored in 32 MB PSRAM (configurable: 0, 15, 30, 60, 120 points), colored by altitude band (FR24-style palette).
- **Interactive Aircraft Popup & 3-Tier Photo Engine:**
  - Tap any aircraft to display full flight parameters and high-res photography.
  - Tier 1 & 2: Real airframe photos queried by ICAO Hex / Registration (Planespotters & Airport-Data API) with photographer credit.
  - Tier 3: Wikipedia REST API fallback by ICAO aircraft type (e.g., C208, B738, A320, AS50) with `[Model]` tag.
  - Dismiss popup by tapping anywhere on the radar screen.
- **On-Screen Touch HUD Controls:**
  - `APTS` (Airports overlay ON / OFF).
  - `AIR` (Traffic filter: ALL / CIVIL / MILITARY).
  - `GND` (Ground traffic filter: Show / Hide aircraft on runway/apron).
  - `MAP` (Background tile map layer ON / OFF).
  - `RNG` (Dynamic range cycling: 10, 20, 30, 50, 100, 150, 200, 250 km).
- **High Aircraft Capacity:**
  - Hardware support for up to 200 simultaneous aircraft in active memory.
- **Modern Responsive Web Configuration Panel:**
  - Full bilingual support (i18n: English / Polish) with inline help tooltips (ⓘ).
  - Wi-Fi network scanner (`/scan`) with SSID selector, and interactive map-based coordinate picker.
  - Brightness slider, startup defaults, NVS persistence, and remote reboot.

### Web Configuration Panel

All settings below live on the `Radar & Display` card of the web panel and persist to NVS across reboots.

| Setting | Options | Description |
|---|---|---|
| Language | English / Polski | Switches both the LCD HUD and the web panel UI language. |
| Screen Brightness | 10 – 100% | Live-updates the LCD backlight without saving. |
| Default Range | 10 / 20 / 30 / 50 / 100 / 150 / 200 / 250 km | Startup radar scale. |
| Max Aircraft on Radar | 10 – 200 | Hard cap on simultaneously tracked/rendered targets. |
| Default Traffic Filter (AIR) | All / Civil Only / Military Only | Startup state of the `AIR` HUD filter; emergency/LPR traffic always stays visible. |
| Ground Traffic (GND) | Airborne only / Show GND traffic | Startup state of the `GND` HUD filter. |
| Background Map (MAP) | ON / OFF | Startup state of the tile-map layer. |
| Emergency Squawk Banner | ON / OFF | Enables/disables the flashing 7700/7600/7500 alert banner. |
| Flight Trail Length | Off / 15 / 30 / 60 / 120 pts | Length of the rendered track-history trail. |
| Default Airports (APTS) | ON / OFF | Startup state of the nearby-airports overlay. |

### Technical Specifications

| Component | Specification |
|---|---|
| MCU | Espressif ESP32-P4 (dual-core RISC-V @ 400 MHz) |
| Memory | 32 MB PSRAM |
| Display | 7" DSI Touch LCD, 1024×600 |
| Connectivity | ESP32-C6 / ESP-Hosted SDIO Wi-Fi 6 co-processor |
| Aircraft Capacity | Up to 200 simultaneous targets |
| Flight Trail History | Up to 120 points per aircraft |
| Data Sources | adsb.fi / airplanes.live (ADS-B), Planespotters / Airport-Data / Wikipedia (photos) |
| Firmware Framework | ESP-IDF ≥ 5.3, LVGL 9.5 |

### Build & Flash

Requires **ESP-IDF v5.3+** with the `esp32p4` target support installed.

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
- **Wielowarstwowa Mapa Podkładowa (Tile Map):**
  - Renderowanie kafelków mapy offline/cache na wyświetlaczu ESP32-P4, w warstwie pod wektorową tarczą radaru.
- **Wyróżnianie Lotnictwa Wojskowego i Misji Specjalnych (`[MIL]`):**
  - Automatyczna detekcja i oznaczanie na czerwono maszyn Sił Powietrznych RP (PLF), NATO (AWACS), RAF (RRR), tankowców (LAGR, NCHO, MMF), rozpoznania radioelektronicznego (JAKE, FORTE, HOMER, REDEYE) i innych sił sojuszniczych.
- **Śmigłowce Ratunkowe (LPR / SAR):**
  - Dedykowane błękitne wyróżnienie dla załóg Lotniczego Pogotowia Ratunkowego (LPR / RAT).
- **System Alarmowy Squawk (Emergency Alerts):**
  - Wykrywanie transponderów 7700 (General Emergency), 7600 (Radio Failure), 7500 (Hijack) z pulsującym czerwonym banerem ostrzegawczym na górze ekranu.
- **Wskaźniki Pionowe (Vertical Trend):**
  - Strzałki wznoszenia (▲) i zniżania (▼) na liście celów przy prędkości pionowej |VSI| > 500 ft/min oraz poziom (—).
- **Ślad Lotu (Flight Trails):**
  - Płynne rysowanie historii trasy w pamięci PSRAM (konfigurowalne: 0, 15, 30, 60, 120 punktów), kolorowanej wg warstw wysokościowych (paleta FR24).
- **Interaktywny Popup i 3-stopniowy Silnik Zdjęć:**
  - Kliknięcie w dowolny statek powietrzny otwiera szczegółowy kafelek z parametrami i zdjęciem.
  - Stopień 1 i 2: Rzeczywiste zdjęcia konkretnego egzemplarza po kodzie HEX i rejestracji (Planespotters / Airport-Data) z podpisem autora `(c)`.
  - Stopień 3: Pobieranie zdjęć poglądowych z REST API Wikipedii po kodzie ICAO typu (np. C208, B738, AS50) z oznaczeniem `[Model]`.
  - Dotknięcie pustego tła natychmiast zamyka okienko.
- **Dotykowy Pasek Kontrolny (HUD):**
  - `APTS` (Przełączanie widoczności pobliskich lotnisk).
  - `AIR` (Filtr statków: WSZYSTKIE / CYWILNE / WOJSKOWE).
  - `GND` (Ruch naziemny: Ukryj / Pokaż maszyny na płycie lotniska).
  - `MAP` (Warstwa mapy podkładowej WŁ / WYŁ).
  - `RNG` (Szybka zmiana skali: 10, 20, 30, 50, 100, 150, 200, 250 km).
- **Pojemność do 200 Maszyn:**
  - Sprzętowa obsługa śledzenia do 200 samolotów jednocześnie.
- **Zaawansowany Panel Konfiguracyjny WWW:**
  - Pełna obsługa dwujęzyczna (i18n: Polski / English) z dymkami pomocy (tooltips ⓘ).
  - Skaner sieci Wi-Fi (`/scan`) z listą wyboru SSID oraz interaktywna mapa do wyboru współrzędnych stacji.
  - Suwak jasności ekranu, zapis konfiguracji w pamięci NVS oraz bezpieczny zdalny restart.

### Panel Konfiguracyjny WWW

Poniższe ustawienia znajdują się na karcie „Radar & Display” panelu WWW i są trwale zapisywane w NVS.

| Ustawienie | Opcje | Opis |
|---|---|---|
| Język | English / Polski | Przełącza język zarówno na ekranie LCD, jak i w panelu WWW. |
| Jasność ekranu | 10 – 100% | Zmienia podświetlenie LCD na żywo, bez zapisu. |
| Domyślny zasięg | 10 / 20 / 30 / 50 / 100 / 150 / 200 / 250 km | Skala radaru po uruchomieniu. |
| Maksymalna liczba samolotów | 10 – 200 | Twardy limit jednocześnie śledzonych/renderowanych celów. |
| Domyślny filtr ruchu (AIR) | Wszystkie / Tylko cywilne / Tylko wojskowe | Startowy stan filtra `AIR`; ruch ratunkowy/LPR zawsze pozostaje widoczny. |
| Ruch naziemny (GND) | Tylko w locie / Pokaż na ziemi | Startowy stan filtra `GND`. |
| Mapa w tle (MAP) | WŁ / WYŁ | Startowy stan warstwy mapy kafelkowej. |
| Alarm Squawk Ratunkowy | WŁ / WYŁ | Włącza/wyłącza pulsujący baner alarmowy 7700/7600/7500. |
| Długość śladu lotu | Wyłączony / 15 / 30 / 60 / 120 pkt | Długość rysowanej historii trasy. |
| Domyślny stan lotnisk (APTS) | WŁ / WYŁ | Startowy stan warstwy pobliskich lotnisk. |

### Parametry Techniczne

| Element | Specyfikacja |
|---|---|
| MCU | Espressif ESP32-P4 (dwurdzeniowy RISC-V @ 400 MHz) |
| Pamięć | 32 MB PSRAM |
| Wyświetlacz | 7" dotykowy LCD DSI, 1024×600 |
| Łączność | Koprocesor ESP32-C6 / ESP-Hosted SDIO Wi-Fi 6 |
| Pojemność floty | Do 200 jednoczesnych celów |
| Historia śladu lotu | Do 120 punktów na samolot |
| Źródła danych | adsb.fi / airplanes.live (ADS-B), Planespotters / Airport-Data / Wikipedia (zdjęcia) |
| Framework firmware | ESP-IDF ≥ 5.3, LVGL 9.5 |

### Kompilacja i Wgranie

Wymaga **ESP-IDF v5.3+** ze wsparciem dla targetu `esp32p4`.

```bash
idf.py set-target esp32p4
idf.py build
idf.py -p PORT flash monitor
```
