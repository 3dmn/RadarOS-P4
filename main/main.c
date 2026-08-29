#include "bsp/esp32_p4_wifi6_touch_lcd_7b.h"
#include "lvgl.h"

#include "wifi_manager.h"
#include "adsb_service.h"
#include "radar_ui.h"
#include "photo_service.h"
#include "map_tile_service.h"

void app_main(void) {
    wifi_manager_init();

    if (!adsb_service_init()) return;
    if (!map_tile_service_init()) return;
    if (!radar_ui_init()) return;

    lv_display_t *disp = bsp_display_start();
    if (disp == NULL) return;
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_180);
    bsp_display_brightness_set(wifi_mgr_get_brightness());

    radar_ui_build();

    adsb_service_start();
    photo_service_start();
}
