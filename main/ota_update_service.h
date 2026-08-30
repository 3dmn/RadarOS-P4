#pragma once

#include <stdbool.h>

// Built-in fallback for the version.json manifest URL, used whenever the
// System tab's "Version Check URL" field is left empty in NVS - the update
// checker is active out of the box, no configuration required.
#define DEFAULT_OTA_MANIFEST_URL "https://raw.githubusercontent.com/vmisiek/RadarOS-P4/main/version.json"

// Starts the background task that periodically fetches the version.json
// manifest at ota_update_get_manifest_url() -
// {"version":"1.0.1","url":"https://.../RadarOS-P4.bin","notes":"..."} -
// and compares it to the firmware currently running. Notification only -
// this device never downloads or flashes the .bin itself; the "url" field
// is only reported (web panel link, MQTT/Home Assistant) so the user can
// download it manually. The first check runs ~20s after Wi-Fi connects,
// then every 4 hours. Safe to call once at startup, after
// wifi_manager_init().
void ota_update_service_start(void);

// Performs an out-of-cycle version check immediately. Blocking - waits for
// the manifest HTTP request to complete (a few seconds) before returning,
// so the caller's own task is held for that duration. Safe to call from
// any task.
void ota_update_check_now(void);

// Disabled stub, kept only so the Home Assistant "Install" command on the
// MQTT update entity (mqtt_service.c) has a defined target - logs a
// warning and does nothing. Direct in-device installation was removed;
// firmware is downloaded manually via ota_update_get_release_url() and
// flashed through the web panel's existing file-upload OTA form.
void ota_update_install_now(void);

// True once a check has found a manifest version newer than the running
// firmware.
bool ota_update_is_available(void);

// Currently running firmware version, e.g. "1.0.0" (no leading 'v').
const char *ota_update_get_installed_version(void);

// Latest version found in the manifest, e.g. "1.2.0" (no leading 'v');
// empty string if no successful check has completed yet.
const char *ota_update_get_latest_version(void);

// Firmware .bin download URL from the manifest's "url" field, for the user
// to open manually (web panel download link, MQTT release_url); empty
// string if unknown.
const char *ota_update_get_release_url(void);

// Free-form release notes from the manifest's "notes" field; empty string
// if unknown.
const char *ota_update_get_release_notes(void);

// The manifest URL actually in effect: g_ota_version_url (wifi_manager.h)
// if the user configured one, otherwise DEFAULT_OTA_MANIFEST_URL. Always
// non-empty - the update checker is active by default.
const char *ota_update_get_manifest_url(void);
