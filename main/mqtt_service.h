#pragma once

#include <stdbool.h>
#include <stddef.h>

// Normalizes an arbitrary string (station name, user input) into a valid
// MQTT topic segment / Home Assistant node_id: lowercase ASCII
// letters/digits/underscore only, other characters replaced with '_'.
// Falls back to "radaros_p4" if the result would be empty.
void mqtt_sanitize_topic_id(const char *in, char *out, size_t out_size);

typedef enum {
    MQTT_STATUS_DISABLED = 0,
    MQTT_STATUS_CONNECTING,
    MQTT_STATUS_CONNECTED,
    MQTT_STATUS_ERROR,
} mqtt_conn_status_t;

// Starts the FreeRTOS task that owns the MQTT client. No-op (task exits
// immediately) if MQTT is disabled in NVS (wifi_mgr_get_mqtt_enabled()).
// Waits for a Wi-Fi connection internally - safe to call once at startup,
// after wifi_manager_init(), adsb_service_init() and radar_ui_init().
void mqtt_service_start(void);

// Current broker connection status, for the web panel status readout.
mqtt_conn_status_t mqtt_service_get_status(void);

// Publishes the full current state (all Home Assistant entities) to their
// state/attributes topics. Called after any control changes locally (touch
// screen or web panel) so Home Assistant reflects it without waiting for the
// next periodic telemetry publish. Safe no-op when not connected.
void mqtt_service_publish_state(void);
