#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "mqtt_service.h"
#include "wifi_manager.h"
#include "version.h"
#include "radar_ui.h"
#include "adsb_service.h"
#include "aircraft_types.h"
#include "ota_update_service.h"

static const char *TAG = "MQTT_SVC";

// homeassistant/<component>/<node_id>/<object_id>/<leaf>
#define MQTT_TOPIC_MAX_LEN 128
#define MQTT_NODE_ID_LEN    40

static esp_mqtt_client_handle_t s_client = NULL;
static volatile mqtt_conn_status_t s_status = MQTT_STATUS_DISABLED;
static char s_node_id[MQTT_NODE_ID_LEN] = "radaros_p4";
static char s_availability_topic[MQTT_TOPIC_MAX_LEN];

// ================= TOPIC ID SANITIZATION =================
void mqtt_sanitize_topic_id(const char *in, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    size_t o = 0;
    if (in) {
        for (size_t i = 0; in[i] != '\0' && o < out_size - 1; i++) {
            unsigned char c = (unsigned char)in[i];
            if (isalnum(c)) {
                out[o++] = (char)tolower(c);
            } else if (o > 0 && out[o - 1] != '_') {
                out[o++] = '_';
            }
        }
    }
    while (o > 0 && out[o - 1] == '_') o--;
    out[o] = '\0';
    if (out[0] == '\0') {
        snprintf(out, out_size, "radaros_p4");
    }
}

// ================= TOPIC BUILDERS =================
static void topic_build(char *out, size_t n, const char *component, const char *object_id, const char *leaf) {
    snprintf(out, n, "homeassistant/%s/%s/%s/%s", component, s_node_id, object_id, leaf);
}

// ================= HOME ASSISTANT DISCOVERY =================
// Generic descriptor covering every entity this device exposes - a table
// driven approach keeps the ~19 discovery payloads consistent (device info,
// availability, unique_id) without repeating that boilerplate per entity.
typedef struct {
    const char *component;   // "number","select","switch","button","sensor","binary_sensor"
    const char *object_id;
    const char *name;
    const char *icon;             // may be NULL
    const char *device_class;     // may be NULL
    const char *unit;             // may be NULL
    const char *state_class;      // may be NULL ("measurement")
    bool has_command;             // adds command_topic (controls only)
    bool has_state;                // adds state_topic (everything except button)
    int num_min, num_max, num_step; // number entities only
    const char **options;          // select entities only
    int num_options;
    bool json_attributes;          // sensor entities with an extra attributes topic
} discovery_spec_t;

static void add_device_json(cJSON *root) {
    cJSON *dev = cJSON_CreateObject();
    cJSON *ids = cJSON_CreateArray();
    cJSON_AddItemToArray(ids, cJSON_CreateString(s_node_id));
    cJSON_AddItemToObject(dev, "identifiers", ids);
    cJSON_AddStringToObject(dev, "name", g_station_name[0] ? g_station_name : FW_NAME);
    cJSON_AddStringToObject(dev, "manufacturer", "RadarOS");
    cJSON_AddStringToObject(dev, "model", FW_NAME);
    cJSON_AddStringToObject(dev, "sw_version", FW_VERSION);
    cJSON_AddItemToObject(root, "device", dev);
}

static void publish_json(const char *topic, cJSON *root, bool retain) {
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return;
    esp_mqtt_client_publish(s_client, topic, payload, 0, 1, retain ? 1 : 0);
    free(payload);
}

static void publish_discovery_entity(const discovery_spec_t *spec) {
    char cfg_topic[MQTT_TOPIC_MAX_LEN], state_topic[MQTT_TOPIC_MAX_LEN], cmd_topic[MQTT_TOPIC_MAX_LEN];
    char attr_topic[MQTT_TOPIC_MAX_LEN], unique_id[80];
    topic_build(cfg_topic, sizeof(cfg_topic), spec->component, spec->object_id, "config");
    topic_build(state_topic, sizeof(state_topic), spec->component, spec->object_id, "state");
    topic_build(cmd_topic, sizeof(cmd_topic), spec->component, spec->object_id, "set");
    topic_build(attr_topic, sizeof(attr_topic), spec->component, spec->object_id, "attributes");
    snprintf(unique_id, sizeof(unique_id), "%s_%s", s_node_id, spec->object_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", spec->name);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "availability_topic", s_availability_topic);
    if (spec->has_state) cJSON_AddStringToObject(root, "state_topic", state_topic);
    if (spec->has_command) cJSON_AddStringToObject(root, "command_topic", cmd_topic);
    if (spec->icon) cJSON_AddStringToObject(root, "icon", spec->icon);
    if (spec->device_class) cJSON_AddStringToObject(root, "device_class", spec->device_class);
    if (spec->unit) cJSON_AddStringToObject(root, "unit_of_measurement", spec->unit);
    if (spec->state_class) cJSON_AddStringToObject(root, "state_class", spec->state_class);
    if (spec->json_attributes) cJSON_AddStringToObject(root, "json_attributes_topic", attr_topic);

    if (strcmp(spec->component, "number") == 0) {
        cJSON_AddNumberToObject(root, "min", spec->num_min);
        cJSON_AddNumberToObject(root, "max", spec->num_max);
        cJSON_AddNumberToObject(root, "step", spec->num_step);
    } else if (strcmp(spec->component, "select") == 0) {
        cJSON *opts = cJSON_CreateArray();
        for (int i = 0; i < spec->num_options; i++) {
            cJSON_AddItemToArray(opts, cJSON_CreateString(spec->options[i]));
        }
        cJSON_AddItemToObject(root, "options", opts);
    } else if (strcmp(spec->component, "switch") == 0 || strcmp(spec->component, "binary_sensor") == 0) {
        cJSON_AddStringToObject(root, "payload_on", "ON");
        cJSON_AddStringToObject(root, "payload_off", "OFF");
    } else if (strcmp(spec->component, "button") == 0) {
        cJSON_AddStringToObject(root, "payload_press", "PRESS");
    }

    add_device_json(root);
    publish_json(cfg_topic, root, true);
}

// Home Assistant's MQTT "update" platform uses its own JSON state schema
// (installed_version/latest_version/release_url in one state payload)
// instead of the generic discovery_spec_t on/off or plain-string entities,
// so it gets its own discovery/state publisher rather than a table row.
static void publish_update_discovery(void) {
    char cfg_topic[MQTT_TOPIC_MAX_LEN], state_topic[MQTT_TOPIC_MAX_LEN], cmd_topic[MQTT_TOPIC_MAX_LEN];
    char unique_id[80];
    topic_build(cfg_topic, sizeof(cfg_topic), "update", "firmware", "config");
    topic_build(state_topic, sizeof(state_topic), "update", "firmware", "state");
    topic_build(cmd_topic, sizeof(cmd_topic), "update", "firmware", "set");
    snprintf(unique_id, sizeof(unique_id), "%s_firmware", s_node_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", "Firmware");
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "availability_topic", s_availability_topic);
    cJSON_AddStringToObject(root, "state_topic", state_topic);
    cJSON_AddStringToObject(root, "command_topic", cmd_topic);
    cJSON_AddStringToObject(root, "payload_install", "INSTALL");
    cJSON_AddStringToObject(root, "device_class", "firmware");
    cJSON_AddStringToObject(root, "icon", "mdi:cloud-download-outline");
    add_device_json(root);
    publish_json(cfg_topic, root, true);
}

static void publish_update_state(void) {
    char topic[MQTT_TOPIC_MAX_LEN];
    topic_build(topic, sizeof(topic), "update", "firmware", "state");

    const char *installed = ota_update_get_installed_version();
    const char *latest = ota_update_get_latest_version();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "title", FW_NAME);
    cJSON_AddStringToObject(root, "installed_version", installed);
    // Falls back to the installed version when no check has completed yet,
    // so the entity reads "up to date" instead of a blank/invalid version.
    cJSON_AddStringToObject(root, "latest_version", latest[0] ? latest : installed);
    const char *release_url = ota_update_get_release_url();
    if (release_url[0]) cJSON_AddStringToObject(root, "release_url", release_url);
    const char *notes = ota_update_get_release_notes();
    if (notes[0]) cJSON_AddStringToObject(root, "release_summary", notes);
    publish_json(topic, root, true);
}

static const char *RANGE_OPTIONS[] = {"10 km", "20 km", "30 km", "50 km", "100 km", "150 km", "200 km", "250 km"};
static const char *AIR_FILTER_OPTIONS[] = {"All", "Civil Only", "Military & Rescue"};
static const char *TRAIL_LEN_OPTIONS[] = {"Short", "Medium", "Long", "Maximum"};
static const char *LANGUAGE_OPTIONS[] = {"English", "Polski"};

static void publish_all_discovery(void) {
    discovery_spec_t s;

    memset(&s, 0, sizeof(s));
    s.component = "number"; s.object_id = "brightness"; s.name = "Screen Brightness";
    s.icon = "mdi:brightness-6"; s.unit = "%"; s.has_command = true; s.has_state = true;
    s.num_min = 10; s.num_max = 100; s.num_step = 5;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "select"; s.object_id = "range"; s.name = "Radar Range";
    s.icon = "mdi:radar"; s.has_command = true; s.has_state = true;
    s.options = RANGE_OPTIONS; s.num_options = sizeof(RANGE_OPTIONS) / sizeof(RANGE_OPTIONS[0]);
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "select"; s.object_id = "air_filter"; s.name = "Traffic Filter";
    s.icon = "mdi:airplane-search"; s.has_command = true; s.has_state = true;
    s.options = AIR_FILTER_OPTIONS; s.num_options = sizeof(AIR_FILTER_OPTIONS) / sizeof(AIR_FILTER_OPTIONS[0]);
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "gnd"; s.name = "Ground Traffic";
    s.icon = "mdi:airport"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "map"; s.name = "Background Map";
    s.icon = "mdi:map"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "squawk_alert"; s.name = "Emergency Squawk Banner";
    s.icon = "mdi:alert-octagon"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "apts"; s.name = "Airports Layer";
    s.icon = "mdi:airport"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "apt_commercial"; s.name = "Commercial Airports";
    s.icon = "mdi:airplane"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "apt_military"; s.name = "Military Airports";
    s.icon = "mdi:jet-fighter"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "apt_aeroclubs"; s.name = "Aeroclub Airports";
    s.icon = "mdi:paragliding"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "select"; s.object_id = "trail_length"; s.name = "Flight Trail Length";
    s.icon = "mdi:chart-line-variant"; s.has_command = true; s.has_state = true;
    s.options = TRAIL_LEN_OPTIONS; s.num_options = sizeof(TRAIL_LEN_OPTIONS) / sizeof(TRAIL_LEN_OPTIONS[0]);
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "select"; s.object_id = "language"; s.name = "Interface Language";
    s.icon = "mdi:translate"; s.has_command = true; s.has_state = true;
    s.options = LANGUAGE_OPTIONS; s.num_options = sizeof(LANGUAGE_OPTIONS) / sizeof(LANGUAGE_OPTIONS[0]);
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "button"; s.object_id = "restart"; s.name = "Restart Device";
    s.icon = "mdi:restart"; s.device_class = "restart"; s.has_command = true; s.has_state = false;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "sensor"; s.object_id = "aircraft_count"; s.name = "Aircraft Count";
    s.icon = "mdi:radar"; s.unit = "aircraft"; s.state_class = "measurement";
    s.has_state = true; s.json_attributes = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "binary_sensor"; s.object_id = "emergency_squawk"; s.name = "Emergency Squawk";
    s.device_class = "safety"; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "sensor"; s.object_id = "emergency_details"; s.name = "Emergency Details";
    s.icon = "mdi:alert-octagon-outline"; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "binary_sensor"; s.object_id = "military_active"; s.name = "Military Aircraft Active";
    s.device_class = "occupancy"; s.icon = "mdi:jet-fighter"; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "sensor"; s.object_id = "closest_aircraft"; s.name = "Closest Aircraft";
    s.icon = "mdi:airplane-marker"; s.has_state = true; s.json_attributes = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "sensor"; s.object_id = "wifi_rssi"; s.name = "Wi-Fi Signal";
    s.device_class = "signal_strength"; s.unit = "dBm"; s.state_class = "measurement"; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "sensor"; s.object_id = "uptime"; s.name = "Uptime";
    s.icon = "mdi:clock-outline"; s.unit = "s"; s.state_class = "total_increasing"; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "sensor"; s.object_id = "free_heap"; s.name = "Free Heap";
    s.icon = "mdi:memory"; s.unit = "kB"; s.state_class = "measurement"; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "switch"; s.object_id = "auto_update"; s.name = "Auto-Update";
    s.icon = "mdi:cloud-sync-outline"; s.has_command = true; s.has_state = true;
    publish_discovery_entity(&s);

    memset(&s, 0, sizeof(s));
    s.component = "binary_sensor"; s.object_id = "update_available"; s.name = "Update Available";
    s.device_class = "update"; s.icon = "mdi:cloud-download-outline"; s.has_state = true;
    publish_discovery_entity(&s);

    publish_update_discovery();

    ESP_LOGI(TAG, "Published Home Assistant MQTT Discovery config for all entities");
}

// ================= LABEL <-> VALUE MAPPING =================
static const char *air_filter_to_label(air_filter_mode_t mode) {
    switch (mode) {
        case AIR_FILTER_CIVIL: return "Civil Only";
        case AIR_FILTER_MIL:   return "Military & Rescue";
        default:               return "All";
    }
}

static air_filter_mode_t air_filter_from_label(const char *label) {
    if (strcmp(label, "Civil Only") == 0) return AIR_FILTER_CIVIL;
    if (strcmp(label, "Military & Rescue") == 0) return AIR_FILTER_MIL;
    return AIR_FILTER_ALL;
}

static const char *trail_len_to_label(uint8_t len) {
    if (len <= 15) return "Short";
    if (len <= 30) return "Medium";
    if (len <= 60) return "Long";
    return "Maximum";
}

static uint8_t trail_len_from_label(const char *label) {
    if (strcmp(label, "Short") == 0) return 15;
    if (strcmp(label, "Long") == 0) return 60;
    if (strcmp(label, "Maximum") == 0) return 120;
    return 30;
}

// ================= STATE PUBLISHING =================
// Emergency squawk priority: 7500 (hijack) > 7700 (general emergency) > 7600
// (radio failure) - a small local copy of radar_ui.c's squawk_emergency_rank()
// logic, kept independent so mqtt_service does not depend on radar_ui internals.
static int squawk_rank(const char *squawk, const char **out_label) {
    if (!squawk) return 0;
    if (strcmp(squawk, "7500") == 0) { if (out_label) *out_label = "HIJACK"; return 3; }
    if (strcmp(squawk, "7700") == 0) { if (out_label) *out_label = "EMERGENCY"; return 2; }
    if (strcmp(squawk, "7600") == 0) { if (out_label) *out_label = "RADIO FAILURE"; return 1; }
    return 0;
}

typedef struct {
    int aircraft_count;
    bool emergency_active;
    char emergency_callsign[12];
    char emergency_squawk[8];
    const char *emergency_type;
    bool military_active;
    bool has_closest;
    char closest_callsign[12];
    char closest_model[8];
    float closest_distance_km;
    int closest_altitude_ft;
} telemetry_t;

static void compute_telemetry(telemetry_t *out) {
    memset(out, 0, sizeof(*out));
    out->emergency_type = "";
    if (!live_fleet || !adsb_service_lock(200)) return;

    uint32_t now = get_time_ms();
    int emg_rank = 0;
    float closest_dist = 0.0f;

    for (int i = 0; i < MAX_AIRCRAFT_CAPACITY; i++) {
        if (!live_fleet[i].active || (now - live_fleet[i].last_seen_ms) >= PLANE_TIMEOUT_MS) continue;
        out->aircraft_count++;

        if (live_fleet[i].is_military) out->military_active = true;

        const char *lbl = NULL;
        int rank = squawk_rank(live_fleet[i].squawk, &lbl);
        if (rank > emg_rank) {
            emg_rank = rank;
            out->emergency_active = true;
            out->emergency_type = lbl;
            snprintf(out->emergency_callsign, sizeof(out->emergency_callsign), "%s", live_fleet[i].callsign);
            snprintf(out->emergency_squawk, sizeof(out->emergency_squawk), "%s", live_fleet[i].squawk);
        }

        if (!out->has_closest || live_fleet[i].distance_km < closest_dist) {
            out->has_closest = true;
            closest_dist = live_fleet[i].distance_km;
            out->closest_distance_km = live_fleet[i].distance_km;
            out->closest_altitude_ft = live_fleet[i].altitude_ft;
            snprintf(out->closest_callsign, sizeof(out->closest_callsign), "%s", live_fleet[i].callsign);
            snprintf(out->closest_model, sizeof(out->closest_model), "%s", live_fleet[i].model);
        }
    }

    adsb_service_unlock();
}

static void publish_state_str(const char *component, const char *object_id, const char *value) {
    char topic[MQTT_TOPIC_MAX_LEN];
    topic_build(topic, sizeof(topic), component, object_id, "state");
    esp_mqtt_client_publish(s_client, topic, value, 0, 1, 1);
}

static void publish_attr_json(const char *component, const char *object_id, cJSON *root) {
    char topic[MQTT_TOPIC_MAX_LEN];
    topic_build(topic, sizeof(topic), component, object_id, "attributes");
    publish_json(topic, root, true);
}

void mqtt_service_publish_state(void) {
    if (!s_client || s_status != MQTT_STATUS_CONNECTED) return;

    char buf[32];

    snprintf(buf, sizeof(buf), "%u", wifi_mgr_get_brightness());
    publish_state_str("number", "brightness", buf);

    snprintf(buf, sizeof(buf), "%.0f km", radar_ui_get_range_km());
    publish_state_str("select", "range", buf);

    publish_state_str("select", "air_filter", air_filter_to_label(radar_ui_get_air_filter()));
    publish_state_str("switch", "gnd", radar_ui_get_show_ground() ? "ON" : "OFF");
    publish_state_str("switch", "map", radar_ui_get_map_enabled() ? "ON" : "OFF");
    publish_state_str("switch", "squawk_alert", wifi_mgr_get_squawk_alert_enabled() ? "ON" : "OFF");
    publish_state_str("switch", "apts", radar_ui_get_airports_enabled() ? "ON" : "OFF");

    uint8_t apt_mask = wifi_mgr_get_apt_filter_mask();
    publish_state_str("switch", "apt_commercial", (apt_mask & APT_TYPE_CIVIL) ? "ON" : "OFF");
    publish_state_str("switch", "apt_military", (apt_mask & APT_TYPE_MIL) ? "ON" : "OFF");
    publish_state_str("switch", "apt_aeroclubs", (apt_mask & APT_TYPE_GA) ? "ON" : "OFF");

    publish_state_str("select", "trail_length", trail_len_to_label(wifi_mgr_get_trail_len()));
    publish_state_str("select", "language", wifi_mgr_get_lang() == LANG_PL ? "Polski" : "English");

    telemetry_t t;
    compute_telemetry(&t);

    snprintf(buf, sizeof(buf), "%d", t.aircraft_count);
    publish_state_str("sensor", "aircraft_count", buf);
    {
        cJSON *attrs = cJSON_CreateObject();
        cJSON_AddNumberToObject(attrs, "total_aircraft_in_zone", t.aircraft_count);
        cJSON_AddBoolToObject(attrs, "military_active", t.military_active);
        publish_attr_json("sensor", "aircraft_count", attrs);
    }

    publish_state_str("binary_sensor", "emergency_squawk", t.emergency_active ? "ON" : "OFF");
    publish_state_str("binary_sensor", "military_active", t.military_active ? "ON" : "OFF");

    if (t.emergency_active) {
        snprintf(buf, sizeof(buf), "%s %s %s", t.emergency_callsign, t.emergency_squawk, t.emergency_type);
        publish_state_str("sensor", "emergency_details", buf);
    } else {
        publish_state_str("sensor", "emergency_details", "-");
    }

    if (t.has_closest) {
        publish_state_str("sensor", "closest_aircraft", t.closest_callsign);
        cJSON *attrs = cJSON_CreateObject();
        cJSON_AddStringToObject(attrs, "type", t.closest_model);
        cJSON_AddNumberToObject(attrs, "distance_km", t.closest_distance_km);
        cJSON_AddNumberToObject(attrs, "altitude_ft", t.closest_altitude_ft);
        publish_attr_json("sensor", "closest_aircraft", attrs);
    } else {
        publish_state_str("sensor", "closest_aircraft", "-");
    }

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%d", ap_info.rssi);
        publish_state_str("sensor", "wifi_rssi", buf);
    }

    snprintf(buf, sizeof(buf), "%lld", (long long)(esp_timer_get_time() / 1000000LL));
    publish_state_str("sensor", "uptime", buf);

    snprintf(buf, sizeof(buf), "%u", (unsigned)(esp_get_free_heap_size() / 1024));
    publish_state_str("sensor", "free_heap", buf);

    publish_state_str("switch", "auto_update", wifi_mgr_get_auto_update_enabled() ? "ON" : "OFF");
    publish_state_str("binary_sensor", "update_available", ota_update_is_available() ? "ON" : "OFF");
    publish_update_state();
}

// ================= COMMAND HANDLING (.../set topics) =================
// arg: delay in ms (passed as a pointer value, not an address) - long
// enough for the MQTT ack/state publish and the NVS write it follows to
// fully complete before the reboot tears down the network stack.
static void restart_task(void *arg) {
    uint32_t delay_ms = (uint32_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    esp_restart();
}

// Accepts every reasonable spelling Home Assistant (or a manual MQTT
// publish) might send for each language, so a mismatched string casing or
// a raw language code never silently falls back to the wrong language.
static app_lang_t language_from_payload(const char *payload) {
    if (strcasecmp(payload, "Polski") == 0 || strcasecmp(payload, "Polish") == 0 ||
        strcasecmp(payload, "pl") == 0) {
        return LANG_PL;
    }
    return LANG_EN;
}

static void set_apt_type_bit(uint8_t bit, bool on) {
    uint8_t mask = wifi_mgr_get_apt_filter_mask();
    mask = on ? (mask | bit) : (mask & ~bit);
    wifi_mgr_set_apt_filter_mask(mask);
    radar_ui_refresh();
}

static bool topic_is(const char *topic, const char *component, const char *object_id) {
    char expected[MQTT_TOPIC_MAX_LEN];
    topic_build(expected, sizeof(expected), component, object_id, "set");
    return strcmp(topic, expected) == 0;
}

// Applies a command received from Home Assistant, updates NVS/live UI state
// through the same setters the touchscreen and web panel use, then
// publishes the fresh state back so Home Assistant reflects it immediately.
static void handle_command(const char *topic, const char *payload) {
    if (topic_is(topic, "number", "brightness")) {
        wifi_mgr_set_brightness((uint8_t)atoi(payload));
    } else if (topic_is(topic, "select", "range")) {
        radar_ui_set_range_km((float)atof(payload));
    } else if (topic_is(topic, "select", "air_filter")) {
        radar_ui_set_air_filter(air_filter_from_label(payload));
    } else if (topic_is(topic, "switch", "gnd")) {
        radar_ui_set_show_ground(strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "switch", "map")) {
        radar_ui_set_map_enabled(strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "switch", "squawk_alert")) {
        wifi_mgr_set_squawk_alert_enabled(strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "switch", "apts")) {
        radar_ui_set_airports_enabled(strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "switch", "apt_commercial")) {
        set_apt_type_bit(APT_TYPE_CIVIL, strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "switch", "apt_military")) {
        set_apt_type_bit(APT_TYPE_MIL, strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "switch", "apt_aeroclubs")) {
        set_apt_type_bit(APT_TYPE_GA, strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "select", "trail_length")) {
        wifi_mgr_set_trail_len(trail_len_from_label(payload));
    } else if (topic_is(topic, "select", "language")) {
        // Matches the existing web panel behavior: a language change is
        // applied to every static LCD/web string only after a reboot.
        app_lang_t new_lang = language_from_payload(payload);
        wifi_mgr_set_lang(new_lang); // persists to NVS immediately, see wifi_manager.c
        ESP_LOGI(TAG, "Language changed to %s via MQTT/Home Assistant, restarting to apply...",
                 new_lang == LANG_PL ? "Polish" : "English");
        // Publish the fresh state (language included, retained) before the
        // reboot, so Home Assistant reflects the change immediately even
        // though the device becomes briefly unavailable.
        mqtt_service_publish_state();
        // Give the MQTT publish and the NVS write time to fully complete
        // before esp_restart() tears down the network stack.
        xTaskCreate(restart_task, "mqtt_restart", 2048, (void *)(uintptr_t)1200, 5, NULL);
        return;
    } else if (topic_is(topic, "button", "restart")) {
        ESP_LOGI(TAG, "Restart command received via MQTT/Home Assistant");
        xTaskCreate(restart_task, "mqtt_restart", 2048, (void *)(uintptr_t)1000, 5, NULL);
        return;
    } else if (topic_is(topic, "switch", "auto_update")) {
        wifi_mgr_set_auto_update_en(strcmp(payload, "ON") == 0);
    } else if (topic_is(topic, "update", "firmware")) {
        ESP_LOGI(TAG, "Firmware install command received via MQTT/Home Assistant");
        ota_update_install_now();
        return;
    } else {
        ESP_LOGW(TAG, "Received command on unrecognized topic: %s", topic);
        return;
    }

    mqtt_service_publish_state();
}

// ================= MQTT EVENT HANDLING =================
static void subscribe_all_commands(void) {
    static const struct { const char *component; const char *object_id; } controls[] = {
        {"number", "brightness"}, {"select", "range"}, {"select", "air_filter"},
        {"switch", "gnd"}, {"switch", "map"}, {"switch", "squawk_alert"}, {"switch", "apts"},
        {"switch", "apt_commercial"}, {"switch", "apt_military"}, {"switch", "apt_aeroclubs"},
        {"select", "trail_length"}, {"select", "language"}, {"button", "restart"},
        {"switch", "auto_update"}, {"update", "firmware"},
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); i++) {
        char topic[MQTT_TOPIC_MAX_LEN];
        topic_build(topic, sizeof(topic), controls[i].component, controls[i].object_id, "set");
        esp_mqtt_client_subscribe(s_client, topic, 1);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            s_status = MQTT_STATUS_CONNECTING;
            ESP_LOGI(TAG, "MQTT client connecting to broker %s:%d", g_mqtt_host, (int)g_mqtt_port);
            radar_ui_mqtt_notify_connecting();
            radar_ui_update_mqtt_status(true, s_status);
            break;

        case MQTT_EVENT_CONNECTED:
            s_status = MQTT_STATUS_CONNECTED;
            ESP_LOGI(TAG, "MQTT client connected to broker %s:%d", g_mqtt_host, (int)g_mqtt_port);
            radar_ui_mqtt_notify_connected();
            radar_ui_update_mqtt_status(true, s_status);
            esp_mqtt_client_publish(s_client, s_availability_topic, "online", 0, 1, 1);
            subscribe_all_commands();
            if (wifi_mgr_get_mqtt_ha_discovery()) {
                publish_all_discovery();
            }
            mqtt_service_publish_state();
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_status = MQTT_STATUS_ERROR;
            ESP_LOGW(TAG, "MQTT client disconnected from broker, will reconnect automatically");
            // The persistent status badge LED always reflects reality, but
            // the transient alert card is suppressed while Wi-Fi itself is
            // down - it already shows its own notification card
            // (radar_ui_wifi_notify_connecting()), so stacking a redundant/
            // confusing MQTT error card on top of it would be noise.
            radar_ui_update_mqtt_status(true, s_status);
            if (wifi_mgr_is_connected()) {
                radar_ui_mqtt_notify_error();
            }
            break;

        case MQTT_EVENT_DATA: {
            char topic_buf[MQTT_TOPIC_MAX_LEN];
            char payload_buf[64];
            int tlen = event->topic_len < (int)sizeof(topic_buf) - 1 ? event->topic_len : (int)sizeof(topic_buf) - 1;
            int plen = event->data_len < (int)sizeof(payload_buf) - 1 ? event->data_len : (int)sizeof(payload_buf) - 1;
            memcpy(topic_buf, event->topic, tlen);
            topic_buf[tlen] = '\0';
            memcpy(payload_buf, event->data, plen);
            payload_buf[plen] = '\0';
            ESP_LOGI(TAG, "MQTT command received: %s = %s", topic_buf, payload_buf);
            handle_command(topic_buf, payload_buf);
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT client error (type %d)", (int)event->error_handle->error_type);
            s_status = MQTT_STATUS_ERROR;
            radar_ui_update_mqtt_status(true, s_status);
            if (wifi_mgr_is_connected()) {
                radar_ui_mqtt_notify_error();
            }
            break;

        default:
            break;
    }
}

// ================= SERVICE LIFECYCLE =================
static void mqtt_worker_task(void *arg) {
    (void)arg;
    wifi_manager_wait_connected();

    if (!wifi_mgr_get_mqtt_enabled()) {
        ESP_LOGI(TAG, "MQTT disabled in configuration, service not started");
        vTaskDelete(NULL);
        return;
    }
    if (g_mqtt_host[0] == '\0') {
        ESP_LOGW(TAG, "MQTT enabled but no broker host configured, service not started");
        vTaskDelete(NULL);
        return;
    }

    mqtt_sanitize_topic_id(g_mqtt_device_id[0] ? g_mqtt_device_id : g_station_name, s_node_id, sizeof(s_node_id));
    snprintf(s_availability_topic, sizeof(s_availability_topic), "homeassistant/sensor/%s/status/state", s_node_id);

    esp_mqtt_client_config_t cfg = {
        .broker.address.hostname = g_mqtt_host,
        .broker.address.port = g_mqtt_port,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .credentials.client_id = s_node_id,
        .session.keepalive = 30,
        .session.last_will.topic = s_availability_topic,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 0,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };
    if (g_mqtt_user[0]) {
        cfg.credentials.username = g_mqtt_user;
    }
    if (g_mqtt_pass[0]) {
        cfg.credentials.authentication.password = g_mqtt_pass;
    }

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        vTaskDelete(NULL);
        return;
    }
    s_status = MQTT_STATUS_CONNECTING;
    radar_ui_mqtt_notify_connecting();
    radar_ui_update_mqtt_status(true, s_status);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
    // esp-mqtt reconnects automatically on Wi-Fi/broker loss (default
    // behavior, disable_auto_reconnect is false unless overridden above) -
    // this task only needs to drive the periodic telemetry publish below.
    ESP_LOGI(TAG, "MQTT client starting, node_id='%s', broker=%s:%d", s_node_id, g_mqtt_host, (int)g_mqtt_port);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        mqtt_service_publish_state();
    }
}

void mqtt_service_start(void) {
    xTaskCreatePinnedToCore(mqtt_worker_task, "mqtt_worker", 6144, NULL, 3, NULL, 1);
}

mqtt_conn_status_t mqtt_service_get_status(void) {
    return s_status;
}
