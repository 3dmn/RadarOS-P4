#pragma once

#include <stdbool.h>

// Built-in fallback for the version.json manifest URL, used whenever the
// System tab's "Version Check URL" field is left empty in NVS - the update
// checker is active out of the box, no configuration required.
#define DEFAULT_OTA_MANIFEST_URL "https://raw.githubusercontent.com/vmisiek/RadarOS-P4/main/version.json"

// Starts the background task that periodically fetches the version.json
// manifest at ota_update_get_manifest_url() -
// {"version":"1.0.1","url":"https://.../RadarOS-P4.bin","notes":"..."} -
// compares it to the firmware currently running, and, when Auto-Update is
// enabled in NVS, downloads and flashes it automatically. The first check
// runs ~20s after Wi-Fi connects, then every 4 hours. Safe to call once at
// startup, after wifi_manager_init().
void ota_update_service_start(void);

// Performs an out-of-cycle version check immediately. Blocking - waits for
// the manifest HTTP request to complete (a few seconds) before returning,
// so the caller's own task is held for that duration; does not trigger
// Auto-Update even if a newer version is found (only the periodic
// background task installs automatically). Safe to call from any task.
void ota_update_check_now(void);

// Downloads and flashes the latest known version, then reboots on success.
// No-op if no newer version is currently known (call after a successful
// check that found one). Runs in the background - returns immediately.
// Safe to call from any task that does not need to know the outcome (e.g.
// an MQTT command handler).
void ota_update_install_now(void);

// Same download-and-flash as ota_update_install_now(), but blocking and
// without a self-triggered restart - returns true/false once the attempt
// finishes so the caller (the web panel's /api/ota_start_download handler)
// can report the outcome and schedule the restart itself. No-op (returns
// false) if no newer version is currently known.
bool ota_update_install_now_sync(void);

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

// The manifest URL actually in effect: g_ota_version_url (wifi_manager.h)
// if the user configured one, otherwise DEFAULT_OTA_MANIFEST_URL. Always
// non-empty - the update checker is active by default.
const char *ota_update_get_manifest_url(void);
