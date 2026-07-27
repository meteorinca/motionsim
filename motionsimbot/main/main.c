#include "config.h"
#include "led.h"
#include "buzzer.h"
#include "wifi_mgr.h"
#include "timekeep.h"
#include "oled.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "pid_controller.h"
#include "motion_udp.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define TAG "MAIN"

static void estop_button_task(void *arg) {
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << ESTOP_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_cfg);

    bool last_state = false;

    while (1) {
        bool btn_pressed = (gpio_get_level(ESTOP_BTN_GPIO) == 0);
        if (btn_pressed != last_state) {
            if (btn_pressed) {
                ESP_LOGW(TAG, "Hardware Emergency Stop Button Pressed!");
                motor_set_estop(true);
            }
            last_state = btn_pressed;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_init();
    buzzer_init();
    oled_init();

    // Initialize Motor Drivers & Sensors
    motor_driver_init();
    hall_sensor_init();
    pid_controller_init();

    // Start WiFi & Timekeeping
    EventGroupHandle_t wifi_events = wifi_init();
    timekeep_init();
    timekeep_start_scheduler();

    led_start_heartbeat(wifi_events, WIFI_CONNECTED_BIT);
    xTaskCreate(estop_button_task, "estop_btn", 2048, NULL, 10, NULL);

    // Initialize 60 Hz Motion UDP Receiver
    motion_udp_init();

    ESP_LOGI(TAG, "MotionSimBot Firmware Fully System Ready — v%s — %s.local:%d",
             FW_VERSION, MDNS_HOSTNAME, WEB_SERVER_PORT);
}
