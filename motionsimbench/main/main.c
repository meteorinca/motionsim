#include "config.h"
#include "led.h"
#include "buzzer.h"
#include "wifi_mgr.h"
#include "timekeep.h"
#include "oled.h"
#include "bench_udp.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/Task.h"
#include "freertos/event_groups.h"

#define TAG "MAIN"

static void button_task(void *arg) {
    bool last_btn1 = false;
    bool last_btn2 = false;

    while (1) {
        bool btn1 = (gpio_get_level(BTN_1_GPIO) == 0);
        bool btn2 = (gpio_get_level(BTN_2_GPIO) == 0);

        if (btn1 && !last_btn1) {
            ESP_LOGI(TAG, "BTN1 Pressed — Cycling OLED Mode");
            oled_cycle_mode();
            buzzer_play_tone(1000, 30);
        }

        if (btn2 && !last_btn2) {
            ESP_LOGI(TAG, "BTN2 Pressed — Resetting Bench Stats");
            bench_udp_reset_stats();
            buzzer_play_tone(1500, 40);
        }

        // BOOT Button wifi reset on 7s hold
        if (gpio_get_level(BTN_BOOT_GPIO) == 0) {
            int hold_time = 0;
            while (gpio_get_level(BTN_BOOT_GPIO) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                hold_time += 100;
                if (hold_time >= 7000) {
                    ESP_LOGW(TAG, "BOOT Button 7s hold detected — resetting WiFi creds");
                    oled_set_text("WIFI RESET\nRESTARTING...", 5000);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    wifi_forget_all();
                }
            }
        }

        last_btn1 = btn1;
        last_btn2 = btn2;
        vTaskDelay(pdMS_TO_TICKS(50));
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

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_BOOT_GPIO) | (1ULL << BTN_1_GPIO) | (1ULL << BTN_2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_conf);

    EventGroupHandle_t wifi_events = wifi_init();
    timekeep_init();
    timekeep_start_scheduler();

    led_start_heartbeat(wifi_events, WIFI_CONNECTED_BIT);
    xTaskCreate(button_task, "btn_task", 3072, NULL, 5, NULL);

    // Initialize high-performance UDP benchmarking server
    bench_udp_init();

    ESP_LOGI(TAG, "MotionSim Bench Firmware Ready — v%s — %s.local:%d",
             FW_VERSION, MDNS_HOSTNAME, WEB_SERVER_PORT);
}
