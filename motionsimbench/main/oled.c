#include "oled.h"
#include "font5x7.h"
#include "config.h"
#include "bench_udp.h"
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
static oled_mode_t s_mode = OLED_MODE_BENCH_STATS;
static eye_emotion_t s_emotion = EYE_EMOTION_NORMAL;
static char s_custom_text[64] = {0};
static int64_t s_text_until_us = 0;

static i2c_master_dev_handle_t s_oled_dev = NULL;

// Sparkline history buffer (128 samples across screen width)
static uint8_t s_rtt_history[OLED_WIDTH] = {0};
static int s_history_idx = 0;

void oled_update_rtt_sample(float rtt_ms) {
    uint8_t val = (uint8_t)fminf(rtt_ms * 4.0f, 63.0f); // Scale 0-15ms to screen height
    s_rtt_history[s_history_idx] = val;
    s_history_idx = (s_history_idx + 1) % OLED_WIDTH;
}

static void i2c_send_cmd(uint8_t cmd) {
    if (!s_oled_dev) return;
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_transmit(s_oled_dev, buf, 2, 100);
}

static void i2c_send_data(const uint8_t *data, size_t len) {
    if (!s_oled_dev) return;
    uint8_t buf[129];
    buf[0] = 0x40; // Data mode
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
        0xAE,       // Display OFF
        0xD5, 0x80, // Set Display Clock Divide Ratio
        0xA8, 0x3F, // Set Multiplex Ratio (64)
        0xD3, 0x00, // Set Display Offset
        0x40,       // Set Start Line (0)
        0x8D, 0x14, // Enable Charge Pump
        0x20, 0x00, // Memory Addressing Mode (Horizontal)
        0xA1,       // Segment Remap (Column 127 mapped to SEG0)
        0xC8,       // COM Output Scan Direction (Reversed)
        0xDA, 0x12, // Set COM Pins Hardware Config
        0x81, 0xCF, // Set Contrast Control
        0xD9, 0xF1, // Set Pre-charge Period
        0xDB, 0x40, // Set VCOMH Deselect Level
        0xA4,       // Entire Display ON (Resume to RAM)
        0xA6,       // Normal Display (Not Inverted)
        0xAF        // Display ON
    };
    for (size_t i = 0; i < sizeof(cmds); i++) {
        i2c_send_cmd(cmds[i]);
    }
}

static void draw_render_mode_stats(bench_stats_t stats) {
    fb_draw_string(0, 0, "=== MOTION BENCH ===", true);
    fb_draw_line_h(0, 9, 128, true);

    char line1[32], line2[32], line3[32], line4[32];
    snprintf(line1, sizeof(line1), "RATE: %.1f Hz", stats.pkt_rate_hz);
    snprintf(line2, sizeof(line2), "LOSS: %ld  DUP: %ld", (long)stats.lost_count, (long)stats.dup_count);
    snprintf(line3, sizeof(line3), "MAX STALL: %ld us", (long)stats.max_rx_interval_us);
    snprintf(line4, sizeof(line4), "IP:%s", wifi_mgr_get_ip());

    fb_draw_string(0, 14, line1, true);
    fb_draw_string(0, 26, line2, true);
    fb_draw_string(0, 38, line3, true);
    fb_draw_string(0, 52, line4, true);
}

static void draw_render_mode_graph(bench_stats_t stats) {
    fb_draw_string(0, 0, "LATENCY SPARKLINE", true);
    fb_draw_line_h(0, 9, 128, true);

    // Draw baseline
    fb_draw_line_h(0, 63, 128, true);

    // Draw sparkline history
    for (int i = 0; i < OLED_WIDTH; i++) {
        int sample_idx = (s_history_idx + i) % OLED_WIDTH;
        uint8_t h = s_rtt_history[sample_idx];
        if (h > 50) h = 50;
        fb_draw_pixel(i, 63 - h, true);
    }
}

static void draw_render_mode_safety_stop(bench_stats_t stats) {
    fb_draw_string(10, 10, "!!! ALERT !!!", true);
    fb_draw_string(0, 26, "SAFETY STOP TRIPPED", true);
    fb_draw_string(12, 42, "NO UDP PACKETS", true);
    fb_draw_string(18, 54, "FOR > 100 ms", true);
}

static void draw_render_mode_wifi_info(void) {
    fb_draw_string(0, 0, "=== NETWORK INFO ===", true);
    fb_draw_line_h(0, 9, 128, true);

    char buf[32];
    snprintf(buf, sizeof(buf), "HOST: %s", MDNS_HOSTNAME);
    fb_draw_string(0, 16, buf, true);

    snprintf(buf, sizeof(buf), "IP: %s", wifi_mgr_get_ip());
    fb_draw_string(0, 28, buf, true);

    snprintf(buf, sizeof(buf), "RSSI: %d dBm", wifi_mgr_get_rssi());
    fb_draw_string(0, 40, buf, true);

    snprintf(buf, sizeof(buf), "MODE: %s", wifi_is_ap_mode() ? "SoftAP" : "Station");
    fb_draw_string(0, 52, buf, true);
}

static void oled_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250)); // Refresh at 4 Hz (does not block 60 Hz UDP!)

        bench_stats_t stats = bench_udp_get_stats();
        fb_clear();

        int64_t now_us = esp_timer_get_time();
        if (now_us < s_text_until_us && s_custom_text[0] != '\0') {
            fb_draw_string(0, 20, s_custom_text, true);
        } else if (stats.safety_stopped) {
            draw_render_mode_safety_stop(stats);
        } else {
            switch (s_mode) {
                case OLED_MODE_BENCH_STATS:
                    draw_render_mode_stats(stats);
                    break;
                case OLED_MODE_BENCH_GRAPH:
                    draw_render_mode_graph(stats);
                    break;
                case OLED_MODE_WIFI_INFO:
                    draw_render_mode_wifi_info();
                    break;
                default:
                    draw_render_mode_stats(stats);
                    break;
            }
        }

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
    fb_draw_string(10, 24, "MOTION BENCH", true);
    fb_draw_string(24, 38, "INITIALIZING", true);
    fb_flush();

    xTaskCreate(oled_task, "oled_render", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "OLED master initialized on SDA:%d SCL:%d", OLED_SDA_PIN, OLED_SCL_PIN);
}

void oled_set_text(const char* msg, int duration_ms) {
    strncpy(s_custom_text, msg, sizeof(s_custom_text) - 1);
    s_custom_text[sizeof(s_custom_text) - 1] = '\0';
    s_text_until_us = esp_timer_get_time() + (duration_ms * 1000LL);
}

void oled_set_mode(oled_mode_t mode) {
    s_mode = mode;
    ESP_LOGI(TAG, "OLED mode changed to %d", mode);
}

oled_mode_t oled_get_mode(void) {
    return s_mode;
}

void oled_cycle_mode(void) {
    s_mode = (s_mode + 1) % 4;
    ESP_LOGI(TAG, "OLED mode cycled to %d", s_mode);
}

void oled_set_emotion(eye_emotion_t emotion) {
    s_emotion = emotion;
}

eye_emotion_t oled_get_emotion(void) {
    return s_emotion;
}
