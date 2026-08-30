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
- [Integracja Home Assistant / MQTT](#integracja-home-assistant--mqtt)
- [Stan połączenia na ekranie](#stan-połączenia-na-ekranie)
- [Źródła danych](#źródła-danych)
- [Nota prawna i licencja](#nota-prawna-i-licencja)

---

## Kluczowe funkcje

- **Kokpitowy HUD na 7-calowym ekranie dotykowym** — wektorowe renderowanie LVGL 9 w 60 FPS, okręgi zasięgu, kompas namiarowy, ikony statków/śmigłowców zorientowane wg kursu i płynne animacje.
- **Dwutorowy silnik danych** — jednoczesna obsługa dekoderów ADS-B w czasie rzeczywistym oraz chmurowych źródeł Wi-Fi (adsb.fi / airplanes.live), dzięki czemu stacja działa zarówno jako klient sieciowy, jak i przy lokalnym sprzęcie dekodującym.
- **Globalna skategoryzowana baza lotnisk** — setki lotnisk na całym świecie podzielonych na **komunikacyjne (Commercial Hubs)**, **bazy wojskowe (Military Air Bases)** oraz **aerokluby i lądowiska (General Aviation)**, każda kategoria niezależnie przełączalna i renderowana wyłącznie w aktywnym zasięgu radaru — dla pełnej wydajności 60 FPS.
- **Alarmy squawk awaryjnych** — natychmiastowy, pulsujący baner na całą szerokość HUD-u przy kodach transpondera **7700** (Emergency), **7600** (Awaria radia) i **7500** (Porwanie).
- **Integracja Home Assistant / MQTT** — pełna dwukierunkowa kontrola stacji z Home Assistant przez MQTT Discovery: 11 automatycznie wykrywanych encji sterujących (jasność, zasięg, filtr ruchu, ruch naziemny, mapa, baner squawk, warstwa lotnisk + przełączniki per kategoria, długość śladu lotu, język, zdalny restart) oraz 8 encji telemetrii/bezpieczeństwa na żywo (liczba samolotów, alarm squawk i jego szczegóły, flaga aktywności wojskowej, najbliższy samolot, RSSI Wi-Fi, uptime, wolna pamięć). Zobacz [poniżej](#integracja-home-assistant--mqtt).
- **Nowoczesny panel WWW z zakładkami** — responsywny ciemny interfejs w stylu kokpitu lotniczego w pięciu zakładkach (Radar i wyświetlacz · Lokalizacja · Sieć Wi-Fi · System · MQTT) z animowanymi przełącznikami w stylu iOS, automatycznym wykrywaniem trybu AP (domyślna zakładka Wi-Fi, gdy urządzenie serwuje własny punkt dostępowy) oraz odpytywaną na żywo diodą statusu MQTT.
- **Interaktywny kwadratowy selektor lokalizacji** — mapa Leaflet o proporcjach dokładnie 1:1 do wskazania współrzędnych GPS stacji jednym dotknięciem.
- **Kopia zapasowa i przywracanie konfiguracji JSON** — eksport jednym kliknięciem wszystkich ustawień z NVS (bez danych logowania Wi-Fi/MQTT) do pliku `radar_config.json` oraz import przywracający lub klonujący konfigurację stacji.
- **Bezprzewodowa aktualizacja firmware (Web OTA)** — układ dwóch 8 MB partycji OTA i wgrywanie firmware wprost z przeglądarki (wybierz plik `.bin`, obserwuj pasek postępu, automatyczny restart), z walidacją magic byte obrazu i zapisem w kawałkach bezpiecznym dla watchdoga.
- **Stan połączenia na ekranie** — animowane dymki powiadomień dla zmian stanu Wi-Fi (AP/łączenie/połączono) i MQTT (łączenie/połączono/błąd) oraz stała dioda statusu HUD z efektem pulsowania/oddychania. Zobacz [poniżej](#stan-połączenia-na-ekranie).
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
| **Radar i wyświetlacz** | Jasność ekranu, domyślny zasięg, limit celów, filtr ruchu (ALL/CIVIL/MIL), ruch naziemny, mapa w tle, baner alarmu squawk, warstwa lotnisk + przełączniki per kategoria (Komercyjne/Wojskowe/Aerokluby), długość śladu lotu. |
| **Lokalizacja** | Szerokość/długość geograficzna stacji z interaktywnym kwadratowym (1:1) selektorem mapy. |
| **Sieć Wi-Fi** | SSID/hasło, skaner sieci, aktualny status połączenia i adres IP. |
| **System** | Język (English/Polski), nazwa stacji, wersja firmware, kopia zapasowa/przywracanie konfiguracji JSON, aktualizacja firmware przez Web OTA. |
| **MQTT** | Włączenie MQTT, host/port/login/hasło brokera, prefiks topików (node ID Home Assistant), przełącznik Home Assistant Auto-Discovery oraz odpytywana na żywo dioda statusu połączenia. |

Wszystkie ustawienia binarne używają animowanych przełączników w stylu iOS. Każde ustawienie jest trwale zapisywane w NVS i przetrwa restart; stała stopka na dole formularza pokazuje aktualną wersję firmware na każdej zakładce.

## Integracja Home Assistant / MQTT

Włącz MQTT w zakładce **MQTT**, wskaż swojego brokera, a RadarOS P4 opublikuje trwałe (retained) topiki konfiguracyjne [Home Assistant MQTT Discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery) zaraz po połączeniu — stacja i wszystkie jej encje pojawią się automatycznie pod jedną kartą urządzenia, bez potrzeby pisania YAML.

**Dostępność i ponowne łączenie:** Last Will and Testament (`homeassistant/sensor/<node_id>/status/state`, `online`/`offline`) utrzymuje poprawny status dostępności każdej encji nawet przy nagłym rozłączeniu (zanik Wi-Fi, utrata zasilania); klient łączy się z brokerem ponownie automatycznie.

**Encje sterujące** (zmiana natychmiast z poziomu Home Assistant, odzwierciedlana na żywo na ekranie dotykowym i odwrotnie):

| Encja | Typ | Opcje |
|---|---|---|
| Jasność ekranu | `number` | 10–100% |
| Zasięg radaru | `select` | 10 / 20 / 30 / 50 / 100 / 150 / 200 / 250 km |
| Filtr ruchu | `select` | All / Civil Only / Military & Rescue |
| Ruch naziemny | `switch` | Pokaż / Ukryj |
| Mapa w tle | `switch` | Wł. / Wył. |
| Baner alarmu Squawk | `switch` | Wł. / Wył. |
| Warstwa lotnisk | `switch` | Wł. / Wył. |
| Lotniska komercyjne / wojskowe / aerokluby | `switch` ×3 | Wł. / Wył. per kategoria |
| Długość śladu lotu | `select` | Short / Medium / Long / Maximum |
| Język interfejsu | `select` | English / Polski |
| Restart urządzenia | `button` | Restartuje ESP32-P4 |

**Sensory** (publikowane przy każdej zmianie oraz co ~10 s):

| Encja | Opis |
|---|---|
| Liczba samolotów | Cele aktualnie w zasięgu, z atrybutami szczegółowymi |
| Alarm Squawk | Sensor binarny, `ON` gdy aktywny kod 7700/7600/7500 |
| Szczegóły alarmu | Callsign, kod squawk i typ alarmu |
| Aktywność wojskowa | Sensor binarny dla pobliskiego ruchu wojskowego |
| Najbliższy samolot | Callsign z atrybutami typu/dystansu/wysokości |
| Sygnał Wi-Fi | RSSI w dBm |
| Czas działania | Sekundy od uruchomienia |
| Wolna pamięć | Wolna pamięć heap w kB |

Zmiana nazwy stacji w zakładce **System** aktualizuje nazwę urządzenia w Home Assistant po kolejnym restarcie; prefiks topików MQTT / node ID konfiguruje się niezależnie w zakładce **MQTT**.

## Stan połączenia na ekranie

Dwa półprzezroczyste, zaokrąglone dymki powiadomień (ciemne tło, neonowo-zielona/turkusowa ramka, spójne ze stylem kokpitu) informują na ekranie dotykowym o stanie połączeń podczas konfiguracji i ponownych prób łączenia, bez zaśmiecania widoku radaru:

- **Dymek Wi-Fi** — pokazuje SSID/hasło/adres URL SoftAP w trybie konfiguracji (trwale), komunikat „Łączenie z Wi-Fi…” podczas dołączania do zapisanej sieci oraz zielony komunikat „Połączono! IP: …”, który znika automatycznie po ~3,5 s.
- **Dymek MQTT** — analogicznie dla połączenia z brokerem (turkusowe „Łączenie…”, zielone „Połączono!” znikające po ~3,5 s, bursztynowo-czerwone „Błąd połączenia / ponawianie…” znikające po ~5 s); widoczny tylko, gdy MQTT jest włączone, i wyciszany podczas zaniku Wi-Fi, aby uniknąć powielania alarmu.

Stała **dioda statusu HUD** w lewym dolnym rogu radaru (zastępująca dawny wskaźnik zasięgu — zasięg jest już widoczny na przycisku `RNG` i na okręgach radaru) pokazuje małe pulsujące/oddychające diody LED: zielona i wolna przy połączeniu, żółta i szybka podczas łączenia, czerwona i migająca przy błędzie — jedna dla Wi-Fi i druga dla MQTT, widoczna tylko, gdy MQTT jest włączone.

## Źródła danych

- **Ruch ADS-B:** [adsb.fi](https://adsb.fi) / [airplanes.live](https://airplanes.live)
- **Zdjęcia statków powietrznych:** Planespotters.net, Airport-Data.com, Wikipedia REST API (zdjęcia poglądowe wg typu)
- **Kafelki mapy:** OpenStreetMap

## Nota prawna i licencja

RadarOS P4 jest **projektem hobbystycznym, open source i edukacyjnym**. Powstał z myślą o entuzjastach chcących wizualizować publicznie nadawany ruch ADS-B i **nie jest certyfikowany ani przeznaczony do użytku w kontroli ruchu lotniczego, planowaniu lotów, separacji ani jakichkolwiek innych decyzjach operacyjnych związanych z bezpieczeństwem lotów**. W rzeczywistych operacjach lotniczych zawsze należy korzystać z oficjalnych, certyfikowanych źródeł danych.

Projekt jest udostępniany na licencji [MIT](LICENSE).
