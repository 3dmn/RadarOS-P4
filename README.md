# Radar ADSB - ESP32-P4 Touch LCD 7B

Projekt radaru lotniczego ADSB oparty na mikrokontrolerze ESP32-P4 i wyświetlaczu dotykowym 7" (1024x600 EK79007 + GT911).

## Funkcjonalności
* Wizualizacja ruchu lotniczego w czasie rzeczywistym
* Dedykowane ikony samolotów i śmigłowców (LVGL)
* Komunikacja Wi-Fi 6

## Kompilacja i wgranie
```bash
idf.py set-target esp32p4
idf.py build
idf.py -p PORT flash monitor