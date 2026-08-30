📖 [English version (README.md)](README.md)

# RadarOS-P4 📡✈️

**Otwartoźródłowy radar lotniczy ADS-B i wyświetlacz ruchu lotniczego na żywo.**

RadarOS-P4 zamienia moduł **ESP32-P4** z **7-calowym ekranem dotykowym MIPI-DSI (1024×600)** w samodzielną, działającą w czasie rzeczywistym konsolę radaru lotniczego: wektorowy HUD w 60 FPS, renderowaną offline mapę kafelkową OpenStreetMap w tle, globalną skategoryzowaną bazę lotnisk, zdjęcia statków powietrznych, oznaczanie ruchu wojskowego/ratunkowego, alarmy squawk awaryjnych, pełną integrację z Home Assistant/MQTT oraz nowoczesny panel konfiguracyjny WWW — bez aplikacji towarzyszącej, bez konta w chmurze i bez komputera po pierwszym wgraniu firmware.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform: ESP32-P4](https://img.shields.io/badge/Platform-ESP32--P4-blue.svg)](#sprzęt)
[![Framework: ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF%20v5.3%2B-red.svg)](https://github.com/espressif/esp-idf)
[![UI: LVGL 9](https://img.shields.io/badge/UI-LVGL%209.5-9cf.svg)](https://lvgl.io/)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-MQTT%20Discovery-41BDF5.svg)](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery)
[![Version](https://img.shields.io/badge/firmware-v1.0.0-brightgreen.svg)](main/version.h)

---

## Spis treści

- [Kluczowe funkcje](#kluczowe-funkcje)
- [Wymagania sprzętowe](#sprzęt)
- [Kompilacja i wgrywanie](#kompilacja-i-wgrywanie)
- [Aktualizacje oprogramowania](#aktualizacje-oprogramowania)
- [Panel konfiguracyjny WWW](#panel-konfiguracyjny-www)
- [Integracja Home Assistant / MQTT](#integracja-home-assistant--mqtt)
- [Stan połączenia na ekranie](#stan-połączenia-na-ekranie)
- [Źródła danych](#źródła-danych)
- [Nota prawna i licencja](#nota-prawna-i-licencja)

---

## Kluczowe funkcje

- **Śledzenie ruchu ADS-B na żywo** — dane w czasie rzeczywistym z [adsb.fi](https://adsb.fi) i [airplanes.live](https://airplanes.live), z **dynamicznym promieniem zapytania**: odległość zapytania (w milach morskich) jest wyliczana z aktualnie wybranego zasięgu HUD (np. 50 km → 27 NM, 100 km → 54 NM, 250 km → 135 NM) zamiast stałego, maksymalnego promienia — dzięki temu odpowiedzi API pozostają małe i szybkie nawet nad gęstymi aglomeracjami.
- **Interaktywny HUD na 7-calowym ekranie dotykowym (LVGL 9.5)** — wektorowe renderowanie w 60 FPS z okręgami zasięgu, kompasem namiarowym, ikonami statków/śmigłowców zorientowanymi wg kursu, stałą kapsułą statusu w lewym dolnym rogu (Wi-Fi, MQTT oraz pulsująca bursztynowa dioda **● FW** sygnalizująca dostępną aktualizację) i przyciemnioną mapą OpenStreetMap renderowaną do bufora w pamięci PSRAM.
- **Home Assistant i MQTT Discovery** — pełna autokonfiguracja po połączeniu: **23 encje** (sterujące, sensory oraz dedykowana encja `update` z opisem zmian) pojawiają się pod jedną kartą urządzenia, bez pisania YAML. Zobacz [poniżej](#integracja-home-assistant--mqtt).
- **Panel zarządzania WWW** — responsywny, pięciozakładkowy interfejs w stylu kokpitu, eksport/import konfiguracji do JSON, zapamiętywanie otwartej zakładki po odświeżeniu strony (hash w adresie URL + `localStorage`) oraz wbudowany mechanizm sprawdzania wydań na GitHubie z bezpośrednim linkiem do pobrania.
- **Dwujęzyczny interfejs (i18n)** — każdy tekst na ekranie i w panelu WWW dostępny jest w języku **angielskim** i **polskim**, przełączanym na żywo z ekranu dotykowego, panelu WWW lub Home Assistant.
- **Zoptymalizowana architektura pamięci** — 32 MB zewnętrznej pamięci PSRAM przechowuje bufor odpowiedzi ADS-B, bufor mapy kafelkowej oraz (dzięki `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC`/`CONFIG_MBEDTLS_DYNAMIC_BUFFER`) bufory sesji mbedTLS, pozostawiając wewnętrzną pamięć SRAM z dostępem DMA wyłącznie dla sterownika Wi-Fi ESP-Hosted. Dodatkowy globalny mutex gwarantuje, że w danej chwili otwarta jest tylko jedna sesja HTTPS/TLS — dla odpytywania ADS-B, kafelków mapy i sprawdzania wersji — co eliminuje awarie pamięci `sdio_rx_get_buffer` powodowane wcześniej przez równoległe sesje TLS.
- **Globalna skategoryzowana baza lotnisk** — setki lotnisk na całym świecie podzielonych na **komunikacyjne (Commercial Hubs)**, **bazy wojskowe (Military Air Bases)** oraz **aerokluby i lądowiska (General Aviation)**, każda kategoria niezależnie przełączalna i renderowana wyłącznie w aktywnym zasięgu radaru.
- **Alarmy squawk awaryjnych** — natychmiastowy, pulsujący baner na całą szerokość HUD-u przy kodach transpondera **7700** (Emergency), **7600** (Awaria radia) i **7500** (Porwanie).
- **Bogata telemetria celów** — sanityzowane callsigny, automatyczne oznaczanie `[MIL]` ruchu wojskowego NATO/sojuszniczego, strzałki trendu wysokości (▲/▼), prędkość względem ziemi, kurs oraz kolorowane ślady lotu o konfigurowalnej długości historii.
- **Interaktywne okienka statków i 3-stopniowy silnik zdjęć** — dotknięcie dowolnego celu pokazuje pełne parametry lotu wraz z prawdziwym zdjęciem egzemplarza (Planespotters / Airport-Data) lub zdjęciem poglądowym z Wikipedii dla danego typu.

## Sprzęt

| Element | Specyfikacja |
|---|---|
| Płytka | **Waveshare ESP32-P4-WIFI6-Touch-LCD-7B** (lub zgodna płytka ESP32-P4 z tym samym okablowaniem ekranu i Wi-Fi) |
| MCU | Espressif **ESP32-P4** — dwurdzeniowy RISC-V @ 400 MHz |
| Pamięć | **32 MB zewnętrznej pamięci PSRAM** (wymagana — przechowuje bufory sieciowe, mapę kafelkową i sesje mbedTLS) |
| Wyświetlacz | **7-calowy ekran dotykowy IPS**, MIPI DSI, **1024×600**, sterownik panelu EK79007 |
| Dotyk | Kontroler pojemnościowy **GT911** |
| Łączność | Koprocesor **ESP32-C6 przez ESP-Hosted**, magistrala SDIO, **4-bit, 40 MHz** |
| Pojemność floty | Do 200 jednoczesnych celów |
| Historia śladu lotu | Do 120 punktów na samolot |
| Framework firmware | ESP-IDF ≥ 5.3, LVGL 9.5 |

## Kompilacja i wgrywanie

Wymagane jest **ESP-IDF v5.3.5+** ze wsparciem dla targetu `esp32p4`, poprawnie wczytane do środowiska (`. $IDF_PATH/export.sh` / `export.bat`).

```bash
git clone <adres-repozytorium>
cd radar-adsb-i-ikony

idf.py set-target esp32p4
idf.py build
idf.py -p COMx flash monitor
```

RadarOS-P4 korzysta z **tablicy partycji z dwoma 8 MB slotami OTA** (`partitions.csv`). Pierwsze wgranie musi zostać wykonane przez kabel USB, aby bootloader i oba sloty OTA zostały poprawnie zapisane; każda kolejna aktualizacja może odbyć się bezprzewodowo — zobacz [Aktualizacje oprogramowania](#aktualizacje-oprogramowania).

### Pierwsza konfiguracja

Przy pierwszym uruchomieniu (lub gdy brak zapisanej sieci Wi-Fi) urządzenie uruchamia własny punkt dostępowy konfiguracyjny:

1. Połącz się z siecią Wi-Fi **`RadarADSB-Setup`**.
2. Otwórz w przeglądarce **`http://192.168.4.1`** — panel otwiera się od razu na zakładce **Sieć Wi-Fi**.
3. Zeskanuj i wybierz swoją domową sieć, ustaw współrzędne GPS stacji (dotknięciem na kwadratowej mapie) i zapisz. Urządzenie zrestartuje się i dołączy do lokalnej sieci.

Po połączeniu ten sam panel jest dostępny pod adresem IP stacji w sieci lokalnej do codziennej konfiguracji.

## Aktualizacje oprogramowania

RadarOS-P4 korzysta z lekkiego mechanizmu **wyłącznie powiadomień** — urządzenie nigdy samodzielnie nie pobiera ani nie instaluje nowego obrazu firmware:

- Zadanie w tle pobiera niewielki manifest `version.json` (`{"version", "url", "notes"}`) mniej więcej co 4 godziny, a dodatkowo raz ~20 s po połączeniu z Wi-Fi. Adres manifestu domyślnie wskazuje na plik [`version.json`](version.json) w gałęzi `main` tego repozytorium i może zostać nadpisany indywidualnie dla urządzenia w panelu WWW.
- Ręczne sprawdzenie można w każdej chwili wywołać przyciskiem **Check for Updates Now** w zakładce **System**, bez przeładowania strony.
- Gdy opublikowana zostanie nowsza wersja, stacja sygnalizuje to jednocześnie w trzech miejscach:
  - **Na 7-calowym HUD** — w kapsule statusu pojawia się pulsująca bursztynowa dioda **● FW**; dotknięcie jej pokazuje w dymku numer nowej wersji i notatkę o wydaniu.
  - **W Home Assistant** — encja `update.firmware` raportuje `installed_version`/`latest_version`/`release_url`/`release_summary`.
  - **W panelu WWW** — zakładka **System** wyświetla przycisk **📦 Download Firmware v*X.Y.Z* (.bin)**, prowadzący bezpośrednio do pliku wydania na GitHubie.
- Aby zainstalować aktualizację: pobierz plik `.bin` z powyższego linku na komputer lub telefon, a następnie wgraj go w sekcji **Firmware Update (OTA)** na zakładce **System** i kliknij **Flash Firmware**. Przeglądarkowy flasher zapisuje plik bezpośrednio na nieaktywnej partycji OTA (z walidacją magic byte i zapisem w kawałkach bezpiecznym dla watchdoga) i po sukcesie automatycznie restartuje urządzenie — bez kabla, bez narzędzia szeregowego.

## Panel konfiguracyjny WWW

| Zakładka | Zawartość |
|---|---|
| **Radar i wyświetlacz** | Jasność ekranu, domyślny zasięg, limit celów, filtr ruchu (ALL/CIVIL/MIL), ruch naziemny, mapa w tle, baner alarmu squawk, warstwa lotnisk + przełączniki per kategoria (Komercyjne/Wojskowe/Aerokluby), długość śladu lotu. |
| **Lokalizacja** | Szerokość/długość geograficzna stacji z interaktywnym kwadratowym (1:1) selektorem mapy. |
| **Sieć Wi-Fi** | SSID/hasło, skaner sieci, aktualny status połączenia i adres IP. |
| **System** | Język (English/Polski), nazwa stacji, wersja firmware, kopia zapasowa/przywracanie konfiguracji JSON, ręczne wgrywanie pliku w sekcji **Firmware Update (OTA)** oraz sekcja **Firmware Update Check** (adres URL sprawdzania wersji, przycisk Check for Updates Now, link do pobrania z GitHuba). |
| **MQTT** | Włączenie MQTT, host/port/login/hasło brokera, prefiks topików (node ID Home Assistant), przełącznik Home Assistant Auto-Discovery oraz odpytywana na żywo dioda statusu połączenia. |

Wszystkie ustawienia binarne używają animowanych przełączników w stylu iOS, a każde ustawienie jest trwale zapisywane w NVS i przetrwa restart. Aktualnie otwarta zakładka jest zapamiętywana po odświeżeniu strony lub po „Save & Reboot” dzięki hashowi w adresie URL i `localStorage`, więc panel nigdy nie wraca do pierwszej zakładki.

## Integracja Home Assistant / MQTT

Włącz MQTT w zakładce **MQTT**, wskaż swojego brokera, a RadarOS-P4 opublikuje trwałe (retained) topiki konfiguracyjne [Home Assistant MQTT Discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery) zaraz po połączeniu — stacja i wszystkie **23 encje** pojawią się automatycznie pod jedną kartą urządzenia, bez potrzeby pisania YAML.

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

**Encja aktualizacji firmware:**

| Encja | Typ | Raportuje |
|---|---|---|
| Firmware | `update` | Raportuje `installed_version`, `latest_version`, `release_url` oraz `release_summary` (opis zmian z `version.json`) — to wyłącznie powiadomienie, RadarOS-P4 nigdy nie instaluje wydania samodzielnie. Aby zaktualizować, pobierz plik `.bin` spod adresu `release_url` i wgraj go przez formularz **Flash Firmware** w panelu WWW (zakładka System) lub przez UART/USB. |

**Sensory** (publikowane przy każdej zmianie oraz co ~10 s):

| Encja | Opis |
|---|---|
| Liczba samolotów | Cele aktualnie w zasięgu, z atrybutami szczegółowymi |
| Alarm Squawk | Sensor binarny, `ON` gdy aktywny kod 7700/7600/7500 |
| Szczegóły alarmu | Callsign, kod squawk i typ alarmu |
| Aktywność wojskowa | Sensor binarny dla pobliskiego ruchu wojskowego |
| Dostępna aktualizacja | Sensor binarny, `ON` gdy `version.json` wskazuje nowsze wydanie |
| Najbliższy samolot | Callsign z atrybutami typu/dystansu/wysokości |
| Sygnał Wi-Fi | RSSI w dBm |
| Czas działania | Sekundy od uruchomienia |
| Wolna pamięć | Wolna pamięć heap w kB |

Zmiana nazwy stacji w zakładce **System** aktualizuje nazwę urządzenia w Home Assistant po kolejnym restarcie; prefiks topików MQTT / node ID konfiguruje się niezależnie w zakładce **MQTT**.

## Stan połączenia na ekranie

Dwa półprzezroczyste, zaokrąglone dymki powiadomień (ciemne tło, neonowo-zielona/turkusowa ramka, spójne ze stylem kokpitu) informują na ekranie dotykowym o stanie połączeń podczas konfiguracji i ponownych prób łączenia, bez zaśmiecania widoku radaru:

- **Dymek Wi-Fi** — pokazuje SSID/hasło/adres URL SoftAP w trybie konfiguracji (trwale), komunikat „Łączenie z Wi-Fi…” podczas dołączania do zapisanej sieci oraz zielony komunikat „Połączono! IP: …”, który znika automatycznie po ~3,5 s.
- **Dymek MQTT** — analogicznie dla połączenia z brokerem (turkusowe „Łączenie…”, zielone „Połączono!” znikające po ~3,5 s, bursztynowo-czerwone „Błąd połączenia / ponawianie…” znikające po ~5 s); widoczny tylko, gdy MQTT jest włączone, i wyciszany podczas zaniku Wi-Fi, aby uniknąć powielania alarmu.

Stała **dioda statusu HUD** w lewym dolnym rogu radaru pokazuje małe pulsujące/oddychające diody LED — po jednej na każdy podsystem: zielona i wolna przy połączeniu, żółta i szybka podczas łączenia, czerwona i migająca przy błędzie.

| Wskaźnik | Znaczenie |
|---|---|
| ● WIFI | Stan połączenia Wi-Fi (widoczny zawsze) |
| ● MQTT | Stan połączenia z brokerem (widoczny tylko, gdy MQTT jest włączone) |
| ● FW | Domyślnie ukryty; pojawia się i pulsuje na bursztynowo, gdy `version.json` wskazuje nowsze wydanie firmware. Dotknięcie pokazuje dymek z numerem wersji i notatką o wydaniu. |

## Źródła danych

- **Ruch ADS-B:** [adsb.fi](https://adsb.fi) / [airplanes.live](https://airplanes.live)
- **Zdjęcia statków powietrznych:** Planespotters.net, Airport-Data.com, Wikipedia REST API (zdjęcia poglądowe wg typu)
- **Kafelki mapy:** OpenStreetMap

## Nota prawna i licencja

RadarOS-P4 jest **projektem hobbystycznym, open source i edukacyjnym**. Powstał z myślą o entuzjastach chcących wizualizować publicznie nadawany ruch ADS-B i **nie jest certyfikowany ani przeznaczony do użytku w kontroli ruchu lotniczego, planowaniu lotów, separacji ani jakichkolwiek innych decyzjach operacyjnych związanych z bezpieczeństwem lotów**. W rzeczywistych operacjach lotniczych zawsze należy korzystać z oficjalnych, certyfikowanych źródeł danych.

Projekt jest udostępniany na licencji [MIT](LICENSE).
