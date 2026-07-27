set(COMMON_SRCS
    "main.c"
    "bench_udp.c"
    "wifi_mgr.c"
    "webserver.c"
    "ota_mgr.c"
    "timekeep.c"
    "oled.c"
    "buzzer.c"
    "led.c"
)

idf_component_register(
    SRCS            ${COMMON_SRCS}
    INCLUDE_DIRS    "."
                    "${CMAKE_SOURCE_DIR}/boards/${BOARD}"
    EMBED_TXTFILES  "index.html"
    PRIV_REQUIRES   esp_driver_gpio
                    esp_driver_ledc
                    esp_wifi
                    esp_event
                    esp_netif
                    esp_timer
                    nvs_flash
                    esp_http_server
                    mdns
                    esp_app_format
                    app_update
                    esp_driver_i2c
                    driver
)
