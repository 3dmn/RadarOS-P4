📖 [Wersja polska (README.pl.md)](README.pl.md)

# RadarOS-P4 📡✈️

**Open-Source Live ADS-B Flight Radar & Air Traffic Display.**

RadarOS-P4 turns an **ESP32-P4** with a **7" MIPI-DSI touchscreen (1024×600)** into a self-contained, real-time air-traffic radar console: a 60 FPS vector HUD, offline-rendered OpenStreetMap tile background, categorized global airport database, aircraft photography, military/rescue traffic tagging, emergency squawk alerts, full Home Assistant/MQTT integration, and a modern web configuration panel — no companion app, no cloud account, and no PC required after the first flash.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform: ESP32-P4](https://img.shields.io/badge/Platform-ESP32--P4-blue.svg)](#hardware-requirements)
[![Framework: ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF%20v5.3%2B-red.svg)](https://github.com/espressif/esp-idf)
[![UI: LVGL 9](https://img.shields.io/badge/UI-LVGL%209.5-9cf.svg)](https://lvgl.io/)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-MQTT%20Discovery-41BDF5.svg)](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery)
[![Version](https://img.shields.io/badge/firmware-v1.0.0-brightgreen.svg)](main/version.h)

---

## Table of Contents

- [Key Features](#key-features)
- [Hardware Requirements](#hardware-requirements)
- [Building & Flashing](#building--flashing)
- [Firmware Updates](#firmware-updates)
- [Web Configuration Panel](#web-configuration-panel)
- [Home Assistant / MQTT Integration](#home-assistant--mqtt-integration)
- [On-Screen Connectivity Status](#on-screen-connectivity-status)
- [Data Sources](#data-sources)
- [Disclaimer & License](#disclaimer--license)

---

## Key Features

- **Live ADS-B Tracking** — real-time feed from [adsb.fi](https://adsb.fi) and [airplanes.live](https://airplanes.live), with a **dynamic query radius**: the request distance (in nautical miles) is derived from the HUD's currently selected range (e.g. 50 km → 27 NM, 100 km → 54 NM, 250 km → 135 NM) instead of a fixed worst-case radius, keeping API responses small and fast even over dense metro airspace.
- **Interactive 7" Touch HUD (LVGL 9.5)** — 60 FPS vector rendering with concentric range rings, bearing compass, heading-oriented aircraft/helicopter icons, a persistent bottom-left status capsule (Wi-Fi, MQTT, and a pulsing amber **● FW** firmware-update indicator), and a darkened OpenStreetMap tile background rendered into a PSRAM canvas.
- **Home Assistant & MQTT Discovery** — full auto-discovery on connect: **23 entities** (controls, sensors, and a dedicated `update` entity with changelog) appear under one device card with zero YAML. See [below](#home-assistant--mqtt-integration).
- **Web Management Panel** — five-tab responsive dark/neon cockpit UI, JSON configuration export/import, a tab selection that survives a page refresh (URL hash + `localStorage`), and a built-in GitHub release checker with a one-tap download link.
- **Dual-Language UI (i18n)** — every on-screen and web-panel string is available in **English** and **Polski**, switchable live from the touchscreen, the web panel, or Home Assistant.
- **Optimized Memory Architecture** — the 32 MB external PSRAM hosts the ADS-B response buffer, the map tile canvas, and (via `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC`/`CONFIG_MBEDTLS_DYNAMIC_BUFFER`) the mbedTLS session buffers, leaving the internal DMA-capable SRAM free for the ESP-Hosted Wi-Fi/SDIO driver. A global mutex further guarantees only one HTTPS/TLS session is ever open at a time across ADS-B polling, map tiles, and version checks, eliminating the `sdio_rx_get_buffer` memory crashes that concurrent TLS sessions used to cause.
- **Global Categorized Airport Database** — hundreds of airports worldwide split into **Commercial Hubs**, **Military Air Bases**, and **General Aviation & Aeroclubs**, each independently toggleable and rendered only within the active radar range.
- **Emergency Squawk Alerts** — instant, pulsing full-width HUD banner on transponder codes **7700** (Emergency), **7600** (Radio Failure), and **7500** (Hijack).
- **Rich Target Telemetry** — sanitized callsigns, automatic `[MIL]` tagging for NATO/allied military traffic, altitude trend arrows (▲/▼), ground speed, heading, and colored flight trails with configurable history length.
- **Interactive Aircraft Popups & 3-Tier Photo Engine** — tap any target for full flight parameters plus a real airframe photo (Planespotters / Airport-Data) or a Wikipedia type-fallback image.

## Hardware Requirements

| Component | Specification |
|---|---|
| Board | **Waveshare ESP32-P4-WIFI6-Touch-LCD-7B** (or a compatible ESP32-P4 board with the same display/Wi-Fi wiring) |
| MCU | Espressif **ESP32-P4** — dual-core RISC-V @ 400 MHz |
| Memory | **32 MB external PSRAM** (required — hosts network buffers, the map canvas, and mbedTLS sessions) |
| Display | **7" IPS touchscreen**, MIPI DSI, **1024×600**, EK79007 panel driver |
| Touch | **GT911** capacitive touch controller |
| Connectivity | **ESP32-C6 co-processor via ESP-Hosted**, SDIO bus, **4-bit, 40 MHz** |
| Aircraft Capacity | Up to 200 simultaneous targets |
| Flight Trail History | Up to 120 points per aircraft |
| Firmware Framework | ESP-IDF ≥ 5.3, LVGL 9.5 |

## Building & Flashing

Requires **ESP-IDF v5.3.5+** with `esp32p4` target support installed and sourced (`. $IDF_PATH/export.sh` / `export.bat`).

```bash
git clone <this-repo-url>
cd radar-adsb-i-ikony

idf.py set-target esp32p4
idf.py build
idf.py -p COMx flash monitor
```

RadarOS-P4 ships with a **dual 8 MB OTA partition layout** (`partitions.csv`). The very first flash must be done over USB so the bootloader and both OTA slots are written correctly; every subsequent update can be done wirelessly — see [Firmware Updates](#firmware-updates).

### First-time setup

On first boot (or whenever no Wi-Fi network is saved), the device starts its own setup access point:

1. Connect to the Wi-Fi network **`RadarADSB-Setup`**.
2. Open **`http://192.168.4.1`** in a browser — the panel opens directly on the **Wi-Fi & Network** tab.
3. Scan for and select your home network, set the station's GPS coordinates (tap the square map), and save. The device reboots and joins your local network.

Once connected, the same panel is reachable at the station's local IP address for day-to-day configuration.

## Firmware Updates

RadarOS-P4 uses a lightweight **notification-only** update mechanism — the device never downloads or flashes a new firmware image on its own:

- A background task fetches a small `version.json` manifest (`{"version", "url", "notes"}`) roughly every 4 hours, plus once ~20 s after Wi-Fi connects. The manifest URL defaults to [`version.json`](version.json) on this repository's `main` branch and can be overridden per-device in the web panel.
- A manual check can be triggered at any time from the **System** tab's **Check for Updates Now** button, without reloading the page.
- When a newer version is published, the station surfaces it in three places simultaneously:
  - **On the 7" HUD** — a pulsing amber **● FW** indicator appears in the status capsule; tapping it shows the new version number and release notes in a toast.
  - **In Home Assistant** — the `update.firmware` entity reports `installed_version`/`latest_version`/`release_url`/`release_summary`.
  - **In the web panel** — the **System** tab shows a **📦 Download Firmware v*X.Y.Z* (.bin)** button linking straight to the GitHub release asset.
- To install: download the `.bin` from the link above to a computer or phone, then upload it under **Firmware Update (OTA)** on the **System** tab and click **Flash Firmware**. The browser-based flasher streams the file directly into the inactive OTA partition (magic-byte validation, watchdog-safe chunked writes) and reboots automatically on success — no cable, no serial tool.

## Web Configuration Panel

| Tab | Contents |
|---|---|
| **Radar & Display** | Brightness, default range, max aircraft, traffic filter (ALL/CIVIL/MIL), ground traffic, background map, emergency squawk banner, airport layer + per-category toggles (Commercial/Military/Aeroclubs), flight trail length. |
| **Location** | Station latitude/longitude with an interactive square (1:1) map picker. |
| **Wi-Fi & Network** | SSID/password, network scanner, live connection status and IP address. |
| **System** | Language (English/Polski), station name, firmware version, JSON configuration backup/restore, manual **Firmware Update (OTA)** file upload, and the **Firmware Update Check** section (version check URL, Check for Updates Now, GitHub download link). |
| **MQTT** | Enable MQTT, broker host/port/username/password, topic prefix (Home Assistant node ID), Home Assistant Auto-Discovery toggle, and a live-polled connection status LED. |

All binary settings use animated iOS-style toggle switches, and every setting persists to NVS and survives reboots. The currently open tab is remembered across a page refresh or a "Save & Reboot" via the URL hash and `localStorage`, so you never get bounced back to the first tab.

## Home Assistant / MQTT Integration

Enable MQTT on the **MQTT** tab, point it at your broker, and RadarOS-P4 publishes retained [Home Assistant MQTT Discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery) config topics on connect — the station and all **23 entities** appear automatically under one device card, no YAML required.

**Availability & reconnection:** a Last Will and Testament (`homeassistant/sensor/<node_id>/status/state`, `online`/`offline`) keeps every entity's availability accurate even on an unclean disconnect (Wi-Fi drop, power loss); the client reconnects to the broker automatically.

**Controls** (change instantly from Home Assistant, mirrored live on the touchscreen and vice versa):

| Entity | Type | Options |
|---|---|---|
| Screen Brightness | `number` | 10–100 % |
| Radar Range | `select` | 10 / 20 / 30 / 50 / 100 / 150 / 200 / 250 km |
| Traffic Filter | `select` | All / Civil Only / Military & Rescue |
| Ground Traffic | `switch` | Show / Hide |
| Background Map | `switch` | On / Off |
| Emergency Squawk Banner | `switch` | On / Off |
| Airports Layer | `switch` | On / Off |
| Commercial / Military / Aeroclub Airports | `switch` ×3 | On / Off per category |
| Flight Trail Length | `select` | Short / Medium / Long / Maximum |
| Interface Language | `select` | English / Polski |
| Restart Device | `button` | Reboots the ESP32-P4 |

**Firmware update entity:**

| Entity | Type | Reports |
|---|---|---|
| Firmware | `update` | Reports `installed_version`, `latest_version`, `release_url`, and `release_summary` (the changelog from `version.json`) — a notification only, RadarOS-P4 never installs a release by itself. To update, download the `.bin` from `release_url` and flash it either through the web panel's **Flash Firmware** form (System tab) or over UART/USB. |

**Sensors** (published on every change and every ~10 s):

| Entity | Description |
|---|---|
| Aircraft Count | Targets currently in range, with attribute breakdown |
| Emergency Squawk | Binary sensor, `ON` while 7700/7600/7500 is active |
| Emergency Details | Callsign, squawk code, and alert type |
| Military Aircraft Active | Binary sensor for nearby military traffic |
| Update Available | Binary sensor, `ON` when a newer `version.json` release is found |
| Closest Aircraft | Callsign with type/distance/altitude attributes |
| Wi-Fi Signal | RSSI in dBm |
| Uptime | Seconds since boot |
| Free Heap | Free heap memory in kB |

Renaming the station in the **System** tab updates the Home Assistant device name on the next reboot; the MQTT topic prefix / node ID is configured independently on the **MQTT** tab.

## On-Screen Connectivity Status

Two translucent, rounded overlay cards (dark background, neon green/cyan border, matching the cockpit theme) keep the touchscreen informative during setup and reconnects without cluttering the radar view:

- **Wi-Fi card** — shows the SoftAP SSID/password/URL while in setup mode (persistent), a "Connecting to Wi-Fi…" message while joining a saved network, and a green "Connected! IP: …" toast that auto-hides after ~3.5 s.
- **MQTT card** — mirrors the same pattern for the broker connection (cyan "Connecting…", green "Connected!" auto-hiding after ~3.5 s, amber/red "Connection error / reconnecting…" auto-hiding after ~5 s); only shown while MQTT is enabled, and suppressed during a Wi-Fi outage to avoid a redundant alert.

A persistent **HUD status badge** in the bottom-left corner of the radar displays small breathing/pulsing LEDs, one per subsystem: green and slow when connected, yellow and fast while connecting, red and blinking on error.

| Indicator | Meaning |
|---|---|
| ● WIFI | Wi-Fi connection health (always shown) |
| ● MQTT | Broker connection health (shown only while MQTT is enabled) |
| ● FW | Hidden by default; appears and pulses amber when `version.json` reports a newer firmware release. Tap it for a toast with the version number and release notes. |

## Data Sources

- **ADS-B traffic:** [adsb.fi](https://adsb.fi) / [airplanes.live](https://airplanes.live)
- **Aircraft photography:** Planespotters.net, Airport-Data.com, Wikipedia REST API (type fallback)
- **Map tiles:** OpenStreetMap

## Disclaimer & License

RadarOS-P4 is an **open-source, hobbyist, and educational project**. It is built for enthusiasts to visualize publicly broadcast ADS-B traffic and is **not certified for, nor intended to be used in, air traffic control, flight planning, separation, or any other safety-of-flight or operational aviation decision**. Always rely on official, certified sources for real-world aviation operations.

Licensed under the [MIT License](LICENSE).
