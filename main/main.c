#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"
#include "lvgl.h"

#include "wifi_manager.h"
#include "adsb_service.h"
#include "radar_ui.h"
#include "photo_service.h"
#include "map_tile_service.h"
#include "mqtt_service.h"
#include "ota_update_service.h"
#include "net_lock.h"

void app_main(void) {
    // Must run before any networking task (started below) can open an
    // esp_http_client session - see net_lock.h.
    net_lock_init();

    wifi_manager_init();

    if (!adsb_service_init()) return;
    if (!map_tile_service_init()) return;
    if (!radar_ui_init()) return;

    // Core 1 (UI & Graphics) is reserved for LVGL - explicit config instead
    // of the BSP's bsp_display_start() default (task_affinity=-1, floats on
    // either core) so it never lands on core 0 and contends with the
    // network/SDIO tasks pinned there. Otherwise identical to
    // bsp_display_start()'s own defaults (esp32_p4_wifi6_touch_lcd_7b.c).
    bsp_display_cfg_t disp_cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
            .buff_dma = false,
#else
            .buff_dma = true,
#endif
            .buff_spiram = false,
            .sw_rotate = true,
        },
    };
    disp_cfg.lvgl_port_cfg.task_affinity = 1;

    lv_display_t *disp = bsp_display_start_with_config(&disp_cfg);
    if (disp == NULL) return;
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_180);
    bsp_display_brightness_set(wifi_mgr_get_brightness());

    radar_ui_build();

    adsb_service_start();
    photo_service_start();
    mqtt_service_start();
    ota_update_service_start();
}
