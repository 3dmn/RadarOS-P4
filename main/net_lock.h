#pragma once

#include <stdbool.h>

#include "freertos/FreeRTOS.h"

// Global HTTP/HTTPS network lock - guarantees at most one esp_http_client
// TLS session is open anywhere in the app at any time. The ESP32-P4's Wi-Fi
// runs over the ESP-Hosted SDIO transport, whose internal DMA-capable RX
// buffer pool is small and shared by every TLS connection; concurrent
// sessions (e.g. a map tile fetch racing an OTA manifest check or a photo
// download) can exhaust it and crash the SDIO driver
// ("assert failed: sdio_rx_get_buffer"). Every module that opens an
// esp_http_client session (map_tile_service.c, ota_update_service.c,
// adsb_service.c, photo_service.c) must hold this lock for the full
// open/read/close/cleanup sequence.

// Creates the underlying mutex - call once at startup, before any
// networking task (adsb_service_start(), map_tile_service_init(),
// photo_service_start(), ota_update_service_start(), ...) is created.
void net_lock_init(void);

// Blocks until the lock is acquired or timeout_ticks elapses (portMAX_DELAY
// to wait forever). Returns false on timeout or if net_lock_init() has not
// run yet.
bool net_http_lock(TickType_t timeout_ticks);

// Non-blocking variant of net_http_lock() - returns immediately.
bool net_http_try_lock(void);

// Releases a lock acquired via net_http_lock()/net_http_try_lock().
void net_http_unlock(void);
