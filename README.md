📖 [Wersja polska (README.pl.md)](README.pl.md)

# RadarOS P4 📡✈️

**Standalone ESP32-P4 ADS-B Flight Radar HUD & Web Station.**

RadarOS P4 turns an ESP32-P4 with a 7" touchscreen into a self-contained air-traffic radar console: a 60 FPS vector HUD, offline tile map, categorized global airport database, aircraft photography, military/rescue traffic tagging, emergency squawk alerts, and a modern web configuration panel with wireless firmware updates — no companion app, no cloud account, no PC required after the first flash.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform: ESP32-P4](https://img.shields.io/badge/Platform-ESP32--P4-blue.svg)](#hardware)
[![Framework: ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF%20v5.3%2B-red.svg)](https://github.com/espressif/esp-idf)
[![UI: LVGL 9](https://img.shields.io/badge/UI-LVGL%209-9cf.svg)](https://lvgl.io/)

---

## Table of Contents

- [Key Features](#key-features)
- [Hardware](#hardware)
- [Getting Started](#getting-started)
- [Web Configuration Panel](#web-configuration-panel)
- [Data Sources](#data-sources)
- [Disclaimer & License](#disclaimer--license)

---

## Key Features

- **7-inch Touchscreen Cockpit HUD** — 60 FPS LVGL 9 vector rendering with concentric range rings, bearing compass, heading-oriented aircraft/helicopter icons, and smooth animations.
- **Dual Mode Data Engine** — real-time live ADS-B decoders and Wi-Fi cloud feeds (adsb.fi / airplanes.live), so the station works both as a networked receiver client and alongside local decoding hardware.
- **Global Categorized Airport Database** — hundreds of airports worldwide split into **Commercial Hubs**, **Military Air Bases**, and **General Aviation & Aeroclubs**, each independently toggleable and rendered only within the active radar range for full 60 FPS performance.
- **Emergency Squawk Alerts** — instant, pulsing full-width HUD banner on transponder codes **7700** (Emergency), **7600** (Radio Failure), and **7500** (Hijack).
- **Modern Tabbed Web Interface** — responsive dark/neon cockpit UI (Radar & Display · Location · Wi-Fi & Network · System) with Wi-Fi AP auto-switch, defaulting to the Wi-Fi tab when the device is serving its setup access point.
- **Interactive Square Map Location Picker** — Leaflet-powered, true 1:1 aspect-ratio map for pinpointing the station's GPS coordinates with a tap.
- **JSON Configuration Backup & Restore** — one-click export of every NVS setting to `radar_config.json`, and one-click import to restore or clone a station's configuration.
- **Seamless Web OTA Updates** — dual 8 MB OTA partition layout with a browser-based firmware flasher (drag a `.bin`, watch the progress bar, auto-reboot).
- **Rich Target Telemetry** — sanitized callsigns, automatic `[MIL]` tagging for NATO/allied military traffic, altitude trend arrows (▲/▼), ground speed, heading, and colored flight trails with configurable history length.
- **Interactive Aircraft Popups & 3-Tier Photo Engine** — tap any target for full flight parameters plus a real airframe photo (Planespotters / Airport-Data) or a Wikipedia type-fallback image.

## Hardware

| Component | Specification |
|---|---|
| MCU | Espressif **ESP32-P4** — dual-core RISC-V @ 400 MHz, with PSRAM |
| Display | **7" IPS touchscreen**, DSI, 1024×600 |
| Connectivity | ESP32-C6 / ESP-Hosted SDIO Wi-Fi 6 co-processor |
| Aircraft Capacity | Up to 200 simultaneous targets |
| Flight Trail History | Up to 120 points per aircraft |
| Firmware Framework | ESP-IDF ≥ 5.3, LVGL 9.5 |

## Getting Started

### 1. Clone & configure ESP-IDF

```bash
git clone <this-repo-url>
cd radar-adsb-i-ikony
idf.py set-target esp32p4
```

Requires **ESP-IDF v5.3+** with `esp32p4` target support installed.

### 2. First flash over USB

RadarOS P4 ships with a **dual 8 MB OTA partition layout**. The very first flash must be done over a USB cable so the bootloader and both OTA slots are written correctly:

```bash
idf.py -p COMx flash monitor
```

### 3. Configure the station

On first boot (or whenever no Wi-Fi network is saved), the device starts its own setup access point:

- Connect to the Wi-Fi network **`RadarADSB-Setup`**.
- Open **`http://192.168.4.1`** in a browser — the panel opens directly on the **Wi-Fi & Network** tab.
- Scan for and select your home network, set the station's GPS coordinates (tap the square map), and save. The device reboots and joins your local network.

Once connected, the same panel is reachable at the station's local IP address for day-to-day configuration.

### 4. Future updates — wireless

For every subsequent firmware update, no cable is required: open the **System** tab in the web panel, choose the new `.bin` under **Firmware Update (OTA)**, and click **Flash Firmware**. The device flashes the inactive OTA partition and reboots automatically.

## Web Configuration Panel

| Tab | Contents |
|---|---|
| **Radar & Display** | Brightness, default range, max aircraft, traffic filter (ALL/CIVIL/MIL), ground traffic, background map, emergency squawk banner, airport category checkboxes, flight trail length. |
| **Location** | Station latitude/longitude with an interactive square map picker. |
| **Wi-Fi & Network** | SSID/password, network scanner, live connection status and IP address. |
| **System** | Language (English/Polski), station name, firmware version, configuration backup/restore, Web OTA updater. |

All settings persist to NVS and survive reboots; a fixed footer shows the running firmware version on every tab.

## Data Sources

- **ADS-B traffic:** [adsb.fi](https://adsb.fi) / [airplanes.live](https://airplanes.live)
- **Aircraft photography:** Planespotters.net, Airport-Data.com, Wikipedia REST API (type fallback)
- **Map tiles:** OpenStreetMap

## Disclaimer & License

RadarOS P4 is an **open-source, hobbyist, and educational project**. It is built for enthusiasts to visualize publicly broadcast ADS-B traffic and is **not certified for, nor intended to be used in, air traffic control, flight planning, separation, or any other safety-of-flight or operational aviation decision**. Always rely on official, certified sources for real-world aviation operations.

Licensed under the [MIT License](LICENSE).
