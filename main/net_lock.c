#include "net_lock.h"

#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "NET_LOCK";

static SemaphoreHandle_t s_net_http_mutex = NULL;

void net_lock_init(void) {
    s_net_http_mutex = xSemaphoreCreateMutex();
    if (!s_net_http_mutex) {
        ESP_LOGE(TAG, "Failed to create the global HTTP/HTTPS network lock!");
    }
}

bool net_http_lock(TickType_t timeout_ticks) {
    if (!s_net_http_mutex) {
        ESP_LOGE(TAG, "net_http_lock() called before net_lock_init()");
        return false;
    }
    return xSemaphoreTake(s_net_http_mutex, timeout_ticks) == pdTRUE;
}

bool net_http_try_lock(void) {
    return net_http_lock(0);
}

void net_http_unlock(void) {
    if (s_net_http_mutex) {
        xSemaphoreGive(s_net_http_mutex);
    }
}
