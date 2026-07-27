#include "oled.h"
#include "font5x7.h"
#include "config.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "pid_controller.h"
#include "motion_udp.h"
#include "wifi_mgr.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "OLED"
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

static uint8_t s_framebuf[OLED_WIDTH * OLED_HEIGHT / 8];
static oled_mode_t s_mode = OLED_MODE_DIAGNOSTICS;
static i2c_master_dev_handle_t s_oled_dev = NULL;

static void i2c_send_cmd(uint8_t cmd) {
    if (!s_oled_dev) return;
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_transmit(s_oled_dev, buf, 2, 100);
}

static void i2c_send_data(const uint8_t *data, size_t len) {
    if (!s_oled_dev) return;
    uint8_t buf[129];
    buf[0] = 0x40;
    for (size_t i = 0; i < len; i += 128) {
        size_t chunk = (len - i > 128) ? 128 : (len - i);
        memcpy(&buf[1], &data[i], chunk);
        i2c_master_transmit(s_oled_dev, buf, chunk + 1, 100);
    }
}

static void fb_clear(void) {
    memset(s_framebuf, 0, sizeof(s_framebuf));
}

static void fb_draw_pixel(int x, int y, bool color) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    if (color) {
        s_framebuf[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
    } else {
        s_framebuf[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

static void fb_draw_char(int x, int y, char c, bool color) {
    if (c < 32 || c > 127) c = '?';
    int idx = (c - 32) * 5;
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[idx + i];
        for (int j = 0; j < 7; j++) {
            if (line & (1 << j)) {
                fb_draw_pixel(x + i, y + j, color);
            }
        }
    }
}

static void fb_draw_string(int x, int y, const char *str, bool color) {
    while (*str) {
        fb_draw_char(x, y, *str, color);
        x += 6;
        str++;
    }
}

static void fb_draw_line_h(int x, int y, int w, bool color) {
    for (int i = 0; i < w; i++) fb_draw_pixel(x + i, y, color);
}

static void fb_flush(void) {
    for (uint8_t page = 0; page < 8; page++) {
        i2c_send_cmd(0xB0 + page);
        i2c_send_cmd(0x00);
        i2c_send_cmd(0x10);
        i2c_send_data(&s_framebuf[page * OLED_WIDTH], OLED_WIDTH);
    }
}

static void ssd1306_init_sequence(void) {
    uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(cmds); i++) {
        i2c_send_cmd(cmds[i]);
    }
}

static void draw_diagnostics(void) {
    fb_draw_string(0, 0, "=== MOTIONSIMBOT ===", true);
    fb_draw_line_h(0, 9, 128, true);

    char l1[32], l2[32], l3[32], l4[32];
    motion_stats_t stats = motion_udp_get_stats();

    snprintf(l1, sizeof(l1), "M1: T%.1f A%.1f", pid_get_target_angle(1), hall_read_angle(1));
    snprintf(l2, sizeof(l2), "M2: T%.1f A%.1f", pid_get_target_angle(2), hall_read_angle(2));
    snprintf(l3, sizeof(l3), "DUTY:%d  ERR:%.1f", motor_get_duty(1), pid_get_error(1));
    snprintf(l4, sizeof(l4), "RATE:%.1fHz %s", stats.pkt_rate_hz, motor_get_estop() ? "ESTOP" : "OK");

    fb_draw_string(0, 14, l1, true);
    fb_draw_string(0, 26, l2, true);
    fb_draw_string(0, 38, l3, true);
    fb_draw_string(0, 52, l4, true);
}

static void oled_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250)); // 4 Hz render
        fb_clear();
        draw_diagnostics();
        fb_flush();
    }
}

void oled_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_oled_dev));

    ssd1306_init_sequence();
    fb_clear();
    fb_draw_string(10, 24, "MOTIONSIMBOT", true);
    fb_draw_string(24, 38, "BOOTING", true);
    fb_flush();

    xTaskCreate(oled_task, "oled_render", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "OLED Display initialized");
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
