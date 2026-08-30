#pragma once

#include <stdbool.h>

// Starts the background task that periodically fetches the version.json
// manifest at the configured URL (wifi_mgr_get/set via g_ota_version_url) -
// {"version":"1.0.1","url":"https://.../RadarOS-P4.bin","notes":"..."} -
// compares it to the firmware currently running, and, when Auto-Update is
// enabled in NVS, downloads and flashes it automatically. The first check
// runs ~20s after Wi-Fi connects, then every 4 hours. Idle (no HTTP calls)
// if no URL is configured. Safe to call once at startup, after
// wifi_manager_init().
void ota_update_service_start(void);

// Triggers an out-of-cycle version check in the background. Safe to call
// from any task.
void ota_update_check_now(void);

// Downloads and flashes the latest known version, then reboots on success.
// No-op if no newer version is currently known (call after a successful
// check that found one). Runs in the background - returns immediately.
// Safe to call from any task (web panel, MQTT command).
void ota_update_install_now(void);

// True once a check has found a manifest version newer than the running
// firmware.
bool ota_update_is_available(void);

// Currently running firmware version, e.g. "1.1.0" (no leading 'v').
const char *ota_update_get_installed_version(void);

// Latest version found in the manifest, e.g. "1.2.0" (no leading 'v');
// empty string if no successful check has completed yet.
const char *ota_update_get_latest_version(void);

// Firmware .bin download URL from the manifest's "url" field; empty string
// if unknown.
const char *ota_update_get_release_url(void);

// Free-form release notes from the manifest's "notes" field; empty string
// if unknown.
const char *ota_update_get_release_notes(void);
