// oled.c — OLED display stub.
// The SSD1306 display is currently disabled. Only the public API stubs are
// compiled; all rendering, I2C, and framebuffer code is excluded to keep
// the build warning-free. Re-enable by restoring the full implementation
// and calling oled_init() with i2c_new_master_bus() + xTaskCreate().

#include "oled.h"
#include "esp_log.h"

#define TAG "OLED"

static oled_mode_t s_mode = OLED_MODE_DIAGNOSTICS;

void oled_init(void) {
    ESP_LOGI(TAG, "Potentiometer angle sensing active; OLED display disabled.");
}

void oled_set_mode(oled_mode_t mode) {
    s_mode = mode;
}

void oled_cycle_mode(void) {
    s_mode = (s_mode + 1) % OLED_MODE_COUNT;
}

oled_mode_t oled_get_mode(void) {
    return s_mode;
}
