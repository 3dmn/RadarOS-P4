idf_component_register(
    SRCS "airports.c" "map_tile_service.c" "radar_ui.c" "adsb_service.c" "wifi_manager.c" "photo_service.c" "aircraft_types.c" "i18n.c" "main.c"
         "aircraft_yellow_15.c" "aircraft_yellow_20.c" "aircraft_yellow_25.c" "helicopter_yellow_20.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash esp_netif esp_http_client esp_http_server esp-tls mbedtls json esp_app_format app_update waveshare__esp32_p4_wifi6_touch_lcd_7b lvgl__lvgl
)