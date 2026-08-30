📖 [English version (README.md)](README.md)

# RadarOS P4 📡✈️

**Samodzielna stacja radaru lotniczego ADS-B na ESP32-P4 — HUD i panel WWW.**

RadarOS P4 zamienia moduł ESP32-P4 z 7-calowym ekranem dotykowym w samodzielną konsolę radaru lotniczego: wektorowy HUD w 60 FPS, mapę kafelkową offline, globalną skategoryzowaną bazę lotnisk, zdjęcia statków powietrznych, oznaczanie ruchu wojskowego/ratunkowego, alarmy squawk awaryjnych oraz nowoczesny panel konfiguracyjny WWW z bezprzewodową aktualizacją firmware — bez aplikacji towarzyszącej, bez konta w chmurze, bez komputera po pierwszym wgraniu.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform: ESP32-P4](https://img.shields.io/badge/Platform-ESP32--P4-blue.svg)](#sprzęt)
[![Framework: ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF%20v5.3%2B-red.svg)](https://github.com/espressif/esp-idf)
[![UI: LVGL 9](https://img.shields.io/badge/UI-LVGL%209-9cf.svg)](https://lvgl.io/)

---

## Spis treści

- [Kluczowe funkcje](#kluczowe-funkcje)
- [Sprzęt](#sprzęt)
- [Szybki start](#szybki-start)
- [Panel konfiguracyjny WWW](#panel-konfiguracyjny-www)
- [Źródła danych](#źródła-danych)
- [Nota prawna i licencja](#nota-prawna-i-licencja)

---

## Kluczowe funkcje

- **Kokpitowy HUD na 7-calowym ekranie dotykowym** — wektorowe renderowanie LVGL 9 w 60 FPS, okręgi zasięgu, kompas namiarowy, ikony statków/śmigłowców zorientowane wg kursu i płynne animacje.
- **Dwutorowy silnik danych** — jednoczesna obsługa dekoderów ADS-B w czasie rzeczywistym oraz chmurowych źródeł Wi-Fi (adsb.fi / airplanes.live), dzięki czemu stacja działa zarówno jako klient sieciowy, jak i przy lokalnym sprzęcie dekodującym.
- **Globalna skategoryzowana baza lotnisk** — setki lotnisk na całym świecie podzielonych na **komunikacyjne (Commercial Hubs)**, **bazy wojskowe (Military Air Bases)** oraz **aerokluby i lądowiska (General Aviation)**, każda kategoria niezależnie przełączalna i renderowana wyłącznie w aktywnym zasięgu radaru — dla pełnej wydajności 60 FPS.
- **Alarmy squawk awaryjnych** — natychmiastowy, pulsujący baner na całą szerokość HUD-u przy kodach transpondera **7700** (Emergency), **7600** (Awaria radia) i **7500** (Porwanie).
- **Nowoczesny panel WWW z zakładkami** — responsywny ciemny interfejs w stylu kokpitu lotniczego (Radar i wyświetlacz · Lokalizacja · Sieć Wi-Fi · System) z automatycznym wykrywaniem trybu AP, który przełącza domyślną zakładkę na Wi-Fi, gdy urządzenie serwuje własny punkt dostępowy konfiguracyjny.
- **Interaktywny kwadratowy selektor lokalizacji** — mapa Leaflet o proporcjach dokładnie 1:1 do wskazania współrzędnych GPS stacji jednym dotknięciem.
- **Kopia zapasowa i przywracanie konfiguracji JSON** — eksport jednym kliknięciem wszystkich ustawień z NVS do pliku `radar_config.json` oraz import przywracający lub klonujący konfigurację stacji.
- **Bezprzewodowa aktualizacja firmware (Web OTA)** — układ dwóch 8 MB partycji OTA i wgrywanie firmware wprost z przeglądarki (wybierz plik `.bin`, obserwuj pasek postępu, automatyczny restart).
- **Bogata telemetria celów** — sanityzowane callsigny, automatyczne oznaczanie `[MIL]` ruchu wojskowego NATO/sojuszniczego, strzałki trendu wysokości (▲/▼), prędkość względem ziemi, kurs oraz kolorowane ślady lotu o konfigurowalnej długości historii.
- **Interaktywne okienka statków i 3-stopniowy silnik zdjęć** — dotknięcie dowolnego celu pokazuje pełne parametry lotu wraz z prawdziwym zdjęciem egzemplarza (Planespotters / Airport-Data) lub zdjęciem poglądowym z Wikipedii dla danego typu.

## Sprzęt

| Element | Specyfikacja |
|---|---|
| MCU | Espressif **ESP32-P4** — dwurdzeniowy RISC-V @ 400 MHz, z pamięcią PSRAM |
| Wyświetlacz | **7-calowy ekran dotykowy IPS**, DSI, 1024×600 |
| Łączność | Koprocesor ESP32-C6 / ESP-Hosted SDIO Wi-Fi 6 |
| Pojemność floty | Do 200 jednoczesnych celów |
| Historia śladu lotu | Do 120 punktów na samolot |
| Framework firmware | ESP-IDF ≥ 5.3, LVGL 9.5 |

## Szybki start

### 1. Klonowanie repozytorium i konfiguracja ESP-IDF

```bash
git clone <adres-repozytorium>
cd radar-adsb-i-ikony
idf.py set-target esp32p4
```

Wymagane jest **ESP-IDF v5.3+** ze wsparciem dla targetu `esp32p4`.

### 2. Pierwsze wgranie przez kabel USB

RadarOS P4 korzysta z **tablicy partycji z dwoma 8 MB slotami OTA**. Pierwsze wgranie musi zostać wykonane przez kabel USB, aby bootloader i oba sloty OTA zostały poprawnie zapisane:

```bash
idf.py -p COMx flash monitor
```

### 3. Konfiguracja stacji

Przy pierwszym uruchomieniu (lub gdy brak zapisanej sieci Wi-Fi) urządzenie uruchamia własny punkt dostępowy konfiguracyjny:

- Połącz się z siecią Wi-Fi **`RadarADSB-Setup`**.
- Otwórz w przeglądarce **`http://192.168.4.1`** — panel otwiera się od razu na zakładce **Sieć Wi-Fi**.
- Zeskanuj i wybierz swoją domową sieć, ustaw współrzędne GPS stacji (dotknięciem na kwadratowej mapie) i zapisz. Urządzenie zrestartuje się i dołączy do lokalnej sieci.

Po połączeniu ten sam panel jest dostępny pod adresem IP stacji w sieci lokalnej do codziennej konfiguracji.

### 4. Kolejne aktualizacje — bezprzewodowo

Do każdej kolejnej aktualizacji firmware kabel nie jest już potrzebny: otwórz zakładkę **System** w panelu WWW, wybierz nowy plik `.bin` w sekcji **Aktualizacja oprogramowania (OTA)** i kliknij **Aktualizuj**. Urządzenie wgrywa firmware na nieaktywną partycję OTA i restartuje się automatycznie.

## Panel konfiguracyjny WWW

| Zakładka | Zawartość |
|---|---|
| **Radar i wyświetlacz** | Jasność ekranu, domyślny zasięg, limit celów, filtr ruchu (ALL/CIVIL/MIL), ruch naziemny, mapa w tle, baner alarmu squawk, checkboxy kategorii lotnisk, długość śladu lotu. |
| **Lokalizacja** | Szerokość/długość geograficzna stacji z interaktywnym kwadratowym selektorem mapy. |
| **Sieć Wi-Fi** | SSID/hasło, skaner sieci, aktualny status połączenia i adres IP. |
| **System** | Język (English/Polski), nazwa stacji, wersja firmware, kopia zapasowa/przywracanie konfiguracji, aktualizacja Web OTA. |

Wszystkie ustawienia są trwale zapisywane w NVS i przetrwają restart; stała stopka na dole formularza pokazuje aktualną wersję firmware na każdej zakładce.

## Źródła danych

- **Ruch ADS-B:** [adsb.fi](https://adsb.fi) / [airplanes.live](https://airplanes.live)
- **Zdjęcia statków powietrznych:** Planespotters.net, Airport-Data.com, Wikipedia REST API (zdjęcia poglądowe wg typu)
- **Kafelki mapy:** OpenStreetMap

## Nota prawna i licencja

RadarOS P4 jest **projektem hobbystycznym, open source i edukacyjnym**. Powstał z myślą o entuzjastach chcących wizualizować publicznie nadawany ruch ADS-B i **nie jest certyfikowany ani przeznaczony do użytku w kontroli ruchu lotniczego, planowaniu lotów, separacji ani jakichkolwiek innych decyzjach operacyjnych związanych z bezpieczeństwem lotów**. W rzeczywistych operacjach lotniczych zawsze należy korzystać z oficjalnych, certyfikowanych źródeł danych.

Projekt jest udostępniany na licencji [MIT](LICENSE).
