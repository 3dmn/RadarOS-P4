idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event nvs_flash esp_netif esp_http_client esp_http_server esp-tls mbedtls json waveshare__esp32_p4_wifi6_touch_lcd_7b lvgl__lvgl
)