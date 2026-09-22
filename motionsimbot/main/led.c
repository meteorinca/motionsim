#include "led.h"
#include "config.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <math.h>

#if defined(BOARD_HAS_NEOPIXEL) && BOARD_HAS_NEOPIXEL
#include "led_strip.h"
#include "esp_log.h"
static const char *TAG = "NEOPIXEL";
static led_strip_handle_t s_led_strip = NULL;
#else
#include "driver/gpio.h"
#endif

extern bool motor_get_estop(void);

static bool g_led_state = false;
static EventGroupHandle_t s_wifi_events = NULL;
static EventBits_t s_connected_bit = 0;

void led_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
#if defined(BOARD_HAS_NEOPIXEL) && BOARD_HAS_NEOPIXEL
    if (s_led_strip) {
        led_strip_set_pixel(s_led_strip, 0, red, green, blue);
        led_strip_refresh(s_led_strip);
    }
#else
    bool on = (red > 0 || green > 0 || blue > 0);
    led_set(on);
#endif
}

void led_init(void) {
#if defined(BOARD_HAS_NEOPIXEL) && BOARD_HAS_NEOPIXEL
    ESP_LOGI(TAG, "Initializing onboard WS2812 NeoPixel on GPIO %d", LED_GPIO);
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        }
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz resolution
        .flags = {
            .with_dma = false,
        }
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (err == ESP_OK) {
        led_strip_clear(s_led_strip);
        // Initial soft cyan glow on boot
        led_set_rgb(0, 20, 20);
    } else {
        ESP_LOGE(TAG, "Failed to initialize NeoPixel: %s", esp_err_to_name(err));
    }
#else
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_set(false);
#endif
}

void led_set(bool on) {
    g_led_state = on;
#if defined(BOARD_HAS_NEOPIXEL) && BOARD_HAS_NEOPIXEL
    if (on) {
        led_set_rgb(0, 30, 30);
    } else {
        led_set_rgb(0, 0, 0);
    }
#else
    gpio_set_level(LED_GPIO, LED_ACTIVE_LOW ? (on ? 0 : 1) : (on ? 1 : 0));
#endif
}

void led_action_set(bool state) {
    g_led_state = state;
    led_set(g_led_state);
}

void led_action_toggle(void) {
    led_action_set(!g_led_state);
}

void led_grn_set(bool on) {
#if defined(BOARD_HAS_NEOPIXEL) && BOARD_HAS_NEOPIXEL
    led_set_rgb(0, on ? 35 : 0, 0);
#else
    led_set(on);
#endif
}

void led_red_set(bool on) {
#if defined(BOARD_HAS_NEOPIXEL) && BOARD_HAS_NEOPIXEL
    led_set_rgb(on ? 45 : 0, 0, 0);
#else
    led_set(on);
#endif
}

static void led_heartbeat_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        // Priority 1: Emergency Stop active -> Blinking RED
        if (motor_get_estop()) {
            bool blink = ((esp_timer_get_time() / 250000) % 2) == 0;
            led_set_rgb(blink ? 60 : 0, 0, 0);
            continue;
        }

        // Priority 2: Normal operation / WiFi state
        bool connected = false;
        if (s_wifi_events) {
            connected = (xEventGroupGetBits(s_wifi_events) & s_connected_bit) != 0;
        }

        if (connected) {
            // Connected: Breathing emerald green
            float t = esp_timer_get_time() / 1000000.0f;
            float breathe = (sinf(t * 3.14159f * 1.5f) + 1.0f) / 2.0f;
            uint8_t val = (uint8_t)(breathe * 35.0f);
            led_set_rgb(0, val, val / 4);
        } else {
            // Connecting / Standby: Soft Amber pulsing
            float t = esp_timer_get_time() / 1000000.0f;
            float breathe = (sinf(t * 3.14159f * 2.5f) + 1.0f) / 2.0f;
            uint8_t r = (uint8_t)(breathe * 35.0f);
            uint8_t g = (uint8_t)(breathe * 15.0f);
            led_set_rgb(r, g, 0);
        }
    }
}

void led_start_heartbeat(EventGroupHandle_t wifi_events, EventBits_t connected_bit) {
    s_wifi_events = wifi_events;
    s_connected_bit = connected_bit;
    xTaskCreate(led_heartbeat_task, "led_hb", 2048, NULL, 3, NULL);
}
