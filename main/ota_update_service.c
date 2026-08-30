#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

#include "ota_update_service.h"
#include "wifi_manager.h"
#include "radar_ui.h"
#include "version.h"

static const char *TAG = "OTA_UPDATE";

#define OTA_CHECK_FIRST_DELAY_MS  (20 * 1000)
#define OTA_CHECK_INTERVAL_MS     (4UL * 60UL * 60UL * 1000UL)
#define OTA_MANIFEST_BUF_SIZE     4096
#define OTA_HTTP_USER_AGENT       "RadarOS-P4-OTA/1.0"

static char s_installed_version[16] = "";
static char s_latest_version[16] = "";
static char s_asset_url[256] = "";
static char s_release_notes[128] = "";
static volatile bool s_update_available = false;

// ================= VERSION COMPARISON =================
// Strips a leading 'v'/'V' (both FW_VERSION and version.json commonly use
// it) so comparisons and displayed strings are consistent either way.
static const char *strip_v(const char *s) {
    if (s && (*s == 'v' || *s == 'V') && s[1] != '\0') return s + 1;
    return s ? s : "";
}

static bool parse_semver(const char *s, int *maj, int *min, int *patch) {
    s = strip_v(s);
    int a = 0, b = 0, c = 0;
    if (sscanf(s, "%d.%d.%d", &a, &b, &c) < 1) return false;
    *maj = a; *min = b; *patch = c;
    return true;
}

// True when 'latest' is a strictly newer semantic version than 'installed'.
// Falls back to "not newer" (false) if either string does not parse as a
// version, so a malformed manifest can never trigger an update.
static bool is_newer_version(const char *latest, const char *installed) {
    int lmaj, lmin, lpatch, imaj, imin, ipatch;
    if (!parse_semver(latest, &lmaj, &lmin, &lpatch) || !parse_semver(installed, &imaj, &imin, &ipatch)) {
        return false;
    }
    if (lmaj != imaj) return lmaj > imaj;
    if (lmin != imin) return lmin > imin;
    return lpatch > ipatch;
}

// ================= VERSION MANIFEST CHECK =================
static bool http_get_to_buffer(const char *url, uint8_t *buf, int buf_size, int *out_len) {
    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
        .buffer_size = 4096,
        .max_redirection_count = 5,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return false;
    esp_http_client_set_header(client, "User-Agent", OTA_HTTP_USER_AGENT);

    // g_https_mutex serializes this against adsb_service.c/map_tile_service.c
    // TLS fetches - see the comment on g_https_mutex in wifi_manager.h.
    bool ok = false;
    xSemaphoreTake(g_https_mutex, portMAX_DELAY);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);

        int total = 0, r;
        while ((r = esp_http_client_read(client, (char *)(buf + total), buf_size - total - 1)) > 0) {
            total += r;
            if (total >= buf_size - 1) break;
        }
        buf[total] = '\0';
        *out_len = total;
        ok = (status == 200 && total > 0);
        if (!ok) {
            ESP_LOGW(TAG, "Version manifest request failed: HTTP %d (%s)", status, url);
        }
        esp_http_client_close(client);
    } else {
        ESP_LOGW(TAG, "Version manifest request failed: %s (%s)", esp_err_to_name(err), url);
    }
    xSemaphoreGive(g_https_mutex);
    esp_http_client_cleanup(client);
    return ok;
}

// Fetches the version.json manifest at ota_update_get_manifest_url()
// ({"version":...,"url":...,"notes":...}) and updates s_latest_version/
// s_asset_url/s_release_notes/s_update_available. s_asset_url is only used
// to report a GitHub release download link (web panel, MQTT/Home Assistant)
// - it is never fetched directly by this device.
static void perform_version_check(void) {
    const char *manifest_url = ota_update_get_manifest_url();

    uint8_t *buf = heap_caps_malloc(OTA_MANIFEST_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate version manifest response buffer in PSRAM");
        return;
    }

    int len = 0;
    bool ok = http_get_to_buffer(manifest_url, buf, OTA_MANIFEST_BUF_SIZE, &len);
    if (!ok) {
        heap_caps_free(buf);
        return;
    }

    cJSON *root = cJSON_Parse((const char *)buf);
    heap_caps_free(buf);
    if (!root) {
        ESP_LOGW(TAG, "Failed to parse version manifest JSON");
        return;
    }

    cJSON *version = cJSON_GetObjectItem(root, "version");
    if (!cJSON_IsString(version)) {
        ESP_LOGW(TAG, "Version manifest is missing the 'version' field");
        cJSON_Delete(root);
        return;
    }
    snprintf(s_latest_version, sizeof(s_latest_version), "%s", strip_v(version->valuestring));

    cJSON *url = cJSON_GetObjectItem(root, "url");
    s_asset_url[0] = '\0';
    if (cJSON_IsString(url)) {
        snprintf(s_asset_url, sizeof(s_asset_url), "%s", url->valuestring);
    }

    cJSON *notes = cJSON_GetObjectItem(root, "notes");
    s_release_notes[0] = '\0';
    if (cJSON_IsString(notes)) {
        snprintf(s_release_notes, sizeof(s_release_notes), "%s", notes->valuestring);
    }

    cJSON_Delete(root);

    bool newer = is_newer_version(s_latest_version, s_installed_version);
    s_update_available = newer;
    ESP_LOGI(TAG, "Version check: installed=%s latest=%s update_available=%s",
             s_installed_version, s_latest_version, newer ? "yes" : "no");
    if (newer && !s_asset_url[0]) {
        ESP_LOGW(TAG, "Newer version found but the manifest has no 'url' field - no download link to report");
    }

    radar_ui_update_fw_status(newer, s_latest_version, s_release_notes);
}

// ================= SERVICE LIFECYCLE =================
static void ota_check_task(void *arg) {
    (void)arg;
    wifi_manager_wait_connected();
    vTaskDelay(pdMS_TO_TICKS(OTA_CHECK_FIRST_DELAY_MS));

    while (1) {
        perform_version_check();
        vTaskDelay(pdMS_TO_TICKS(OTA_CHECK_INTERVAL_MS));
    }
}

void ota_update_service_start(void) {
    // FW_VERSION (version.h) is the clean, manually-maintained semver used
    // for comparisons - esp_app_get_description()->version is the auto
    // git-describe build identifier (e.g. a commit hash) shown elsewhere in
    // the System tab, and would never semver-parse against a manifest.
    snprintf(s_installed_version, sizeof(s_installed_version), "%s", strip_v(FW_VERSION));
    xTaskCreatePinnedToCore(ota_check_task, "ota_update", 8192, NULL, 2, NULL, 1);
}

void ota_update_check_now(void) {
    perform_version_check();
}

// Direct in-device installation (esp_https_ota) has been removed entirely -
// firmware is downloaded manually from the GitHub release link (web panel/
// MQTT) and flashed through the existing "Firmware Update (OTA)" file
// upload instead. This stub only exists so the Home Assistant "Install"
// button on the MQTT update entity (still wired in mqtt_service.c) has a
// defined target to call; it does not download or flash anything.
void ota_update_install_now(void) {
    ESP_LOGW(TAG, "Direct install requested but is disabled - download the release .bin and flash it via the web panel instead");
}

bool ota_update_is_available(void) {
    return s_update_available;
}

const char *ota_update_get_installed_version(void) {
    return s_installed_version;
}

const char *ota_update_get_latest_version(void) {
    return s_latest_version;
}

const char *ota_update_get_release_url(void) {
    return s_asset_url;
}

const char *ota_update_get_release_notes(void) {
    return s_release_notes;
}

const char *ota_update_get_manifest_url(void) {
    return OTA_VERSION_CHECK_URL;
}
