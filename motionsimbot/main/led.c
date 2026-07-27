#include "led.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <math.h>

static bool g_led_state = false;
static EventGroupHandle_t s_wifi_events;
static EventBits_t s_connected_bit;

void led_init(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_set(false);
}

void led_set(bool on) {
    gpio_set_level(LED_GPIO, LED_ACTIVE_LOW ? (on ? 0 : 1) : (on ? 1 : 0));
}

void led_action_set(bool state) {
    g_led_state = state;
    led_set(g_led_state);
}

void led_action_toggle(void) {
    led_action_set(!g_led_state);
}

void led_grn_set(bool on) {}
void led_red_set(bool on) {}

static void led_heartbeat_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
        bool connected = (xEventGroupGetBits(s_wifi_events) & s_connected_bit) != 0;
        if (connected) {
            float t = esp_timer_get_time() / 1000000.0f;
            float breathe = (sinf(t * 3.14159f * 2.666f) + 1.0f) / 2.0f;
            led_set(breathe > 0.5f);
        } else {
            led_set((esp_timer_get_time() / 200000) % 2 == 0);
        }
    }
}

void led_start_heartbeat(EventGroupHandle_t wifi_events, EventBits_t connected_bit) {
    s_wifi_events = wifi_events; s_connected_bit = connected_bit;
    xTaskCreate(led_heartbeat_task, "led_hb", 2048, NULL, 3, NULL);
}
