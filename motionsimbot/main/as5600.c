#include "as5600.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

#define TAG "AS5600"
#define NVS_NAMESPACE "as5600_cal"
#define MAX_CHANNELS  2

static i2c_master_dev_handle_t s_as5600_dev = NULL;
static i2c_master_dev_handle_t s_mux_dev = NULL;
static bool s_has_mux = false;
static bool s_detected[MAX_CHANNELS] = {false, false};
static float s_zero_offsets[MAX_CHANNELS] = {0.0f, 0.0f};
static int s_current_channel = -1;

// Last known valid raw count to prevent 0-value glitches during noise/vibration
static uint16_t s_last_valid_raw[MAX_CHANNELS] = {2048, 2048};
static int s_consecutive_errors[MAX_CHANNELS] = {0, 0};

// Telemetry cache to prevent I2C bus flooding
static as5600_telemetry_t s_cached_telem[MAX_CHANNELS];
static uint32_t s_last_telem_ms[MAX_CHANNELS] = {0, 0};

static SemaphoreHandle_t s_i2c_mutex = NULL;

static void load_calibration(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            char key[16];
            snprintf(key, sizeof(key), "zero_%d", i);
            uint32_t val_mdeg = 0;
            if (nvs_get_u32(h, key, &val_mdeg) == ESP_OK) {
                s_zero_offsets[i] = (float)val_mdeg / 1000.0f;
                ESP_LOGI(TAG, "Loaded AS5600 ch%d zero offset: %.2f deg", i, s_zero_offsets[i]);
            }
        }
        nvs_close(h);
    }
}

static void save_calibration(int ch) {
    if (ch < 0 || ch >= MAX_CHANNELS) return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        char key[16];
        snprintf(key, sizeof(key), "zero_%d", ch);
        uint32_t val_mdeg = (uint32_t)(s_zero_offsets[ch] * 1000.0f);
        nvs_set_u32(h, key, val_mdeg);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved AS5600 ch%d zero offset: %.2f deg", ch, s_zero_offsets[ch]);
    }
}

esp_err_t as5600_select_channel(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return ESP_ERR_INVALID_ARG;
    if (channel == s_current_channel && !s_has_mux) return ESP_OK;

    if (s_has_mux && s_mux_dev) {
        uint8_t mux_mask = (1 << channel);
        esp_err_t ret = i2c_master_transmit(s_mux_dev, &mux_mask, 1, 20);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    s_current_channel = channel;
    return ESP_OK;
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t len) {
    if (!s_as5600_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_as5600_dev, &reg, 1, data, len, 25);
}

esp_err_t as5600_init(void) {
    if (!s_i2c_mutex) {
        s_i2c_mutex = xSemaphoreCreateMutex();
    }

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (!bus) {
        ESP_LOGE(TAG, "I2C bus not available");
        return ESP_FAIL;
    }

    // Check for TCA9548A multiplexer at 0x70
    if (i2c_bus_probe(TCA9548A_I2C_ADDR) == ESP_OK) {
        i2c_device_config_t mux_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = TCA9548A_I2C_ADDR,
            .scl_speed_hz = 400000, // 400 kHz Fast Mode
        };
        if (i2c_master_bus_add_device(bus, &mux_cfg, &s_mux_dev) == ESP_OK) {
            s_has_mux = true;
            ESP_LOGI(TAG, "TCA9548A I2C Multiplexer detected at 0x70");
        }
    } else {
        s_has_mux = false;
        ESP_LOGI(TAG, "Direct I2C connection (no TCA9548A multiplexer detected)");
    }

    // Register AS5600 device with 400 kHz Fast Mode
    i2c_device_config_t as5600_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = 400000, // 400 kHz Fast Mode (5x faster than 100 kHz)
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &as5600_cfg, &s_as5600_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register AS5600 I2C device: %s", esp_err_to_name(err));
        return err;
    }

    load_calibration();

    // Probe channel 0
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        as5600_select_channel(0);
        uint8_t status = 0;
        if (read_registers(AS5600_REG_STATUS, &status, 1) == ESP_OK) {
            s_detected[0] = true;
            ESP_LOGI(TAG, "AS5600 Encoder detected on channel 0 (STATUS: 0x%02X, MD: %d, ML: %d, MH: %d)",
                     status,
                     (status & AS5600_STATUS_MD) ? 1 : 0,
                     (status & AS5600_STATUS_ML) ? 1 : 0,
                     (status & AS5600_STATUS_MH) ? 1 : 0);
        } else {
            // Even if STATUS read had a blip, try reading RAW angle
            uint8_t buf[2] = {0};
            if (read_registers(AS5600_REG_RAW_ANGLE_H, buf, 2) == ESP_OK) {
                s_detected[0] = true;
                ESP_LOGI(TAG, "AS5600 Encoder responsive on channel 0 (Raw Angle detected)");
            } else {
                s_detected[0] = false;
                ESP_LOGW(TAG, "AS5600 Encoder NOT responding on channel 0 at 0x36");
            }
        }
        xSemaphoreGive(s_i2c_mutex);
    }

    return s_detected[0] ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool as5600_is_detected(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return false;
    return s_detected[channel];
}

uint16_t as5600_get_raw_counts(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return 0;
    if (!s_detected[channel]) return 0;

    if (!s_i2c_mutex || xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(15)) != pdTRUE) {
        return s_last_valid_raw[channel];
    }

    if (as5600_select_channel(channel) != ESP_OK) {
        xSemaphoreGive(s_i2c_mutex);
        return s_last_valid_raw[channel];
    }

    uint8_t buf[2] = {0};
    esp_err_t err = read_registers(AS5600_REG_RAW_ANGLE_H, buf, 2);
    xSemaphoreGive(s_i2c_mutex);

    if (err == ESP_OK) {
        uint16_t raw = (((uint16_t)(buf[0] & 0x0F)) << 8) | buf[1];
        s_last_valid_raw[channel] = raw;
        s_consecutive_errors[channel] = 0;
        return raw;
    } else {
        s_consecutive_errors[channel]++;
        // Return last valid raw counts so single I2C glitch never fakes a 180 deg jump!
        return s_last_valid_raw[channel];
    }
}

float as5600_get_calibrated_angle(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return 0.0f;
    uint16_t raw = as5600_get_raw_counts(channel);
    float raw_deg = (float)raw * AS5600_DEG_PER_COUNT;

    float cal_deg = raw_deg - s_zero_offsets[channel];
    while (cal_deg < 0.0f)   cal_deg += 360.0f;
    while (cal_deg >= 360.0f) cal_deg -= 360.0f;

    return cal_deg;
}

esp_err_t as5600_read_telemetry(int channel, as5600_telemetry_t *out) {
    if (!out || channel < 0 || channel >= MAX_CHANNELS) return ESP_ERR_INVALID_ARG;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // Return cached telemetry if read within last 250 ms to avoid I2C bus flooding
    if (s_last_telem_ms[channel] > 0 && (now_ms - s_last_telem_ms[channel]) < 250) {
        memcpy(out, &s_cached_telem[channel], sizeof(as5600_telemetry_t));
        // Update raw and calibrated angle with latest values
        uint16_t raw = s_last_valid_raw[channel];
        out->raw_counts = raw;
        out->raw_deg = (float)raw * AS5600_DEG_PER_COUNT;
        float cal = out->raw_deg - s_zero_offsets[channel];
        while (cal < 0.0f)   cal += 360.0f;
        while (cal >= 360.0f) cal -= 360.0f;
        out->cal_deg = cal;
        return ESP_OK;
    }

    if (!s_i2c_mutex || xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        memcpy(out, &s_cached_telem[channel], sizeof(as5600_telemetry_t));
        return ESP_OK;
    }

    if (as5600_select_channel(channel) != ESP_OK) {
        xSemaphoreGive(s_i2c_mutex);
        memcpy(out, &s_cached_telem[channel], sizeof(as5600_telemetry_t));
        return ESP_OK;
    }

    uint8_t status = 0;
    esp_err_t err_st = read_registers(AS5600_REG_STATUS, &status, 1);

    uint8_t agc = 0;
    read_registers(AS5600_REG_AGC, &agc, 1);

    uint8_t mag_buf[2] = {0};
    read_registers(AS5600_REG_MAGNITUDE_H, mag_buf, 2);

    xSemaphoreGive(s_i2c_mutex);

    out->detected = s_detected[channel];
    if (err_st == ESP_OK) {
        // Magnet is considered OK if MD bit is 1, OR if raw angle is actively returning non-zero counts
        out->magnet_detected = ((status & AS5600_STATUS_MD) != 0) || (s_last_valid_raw[channel] > 10);
        out->magnet_too_weak = (status & AS5600_STATUS_ML) != 0;
        out->magnet_too_strong = (status & AS5600_STATUS_MH) != 0;
    } else {
        // If status register read missed, fallback to whether raw angle is healthy
        out->magnet_detected = (s_consecutive_errors[channel] < 10);
        out->magnet_too_weak = false;
        out->magnet_too_strong = false;
    }

    out->agc = agc;
    out->magnitude = (((uint16_t)(mag_buf[0] & 0x0F)) << 8) | mag_buf[1];

    uint16_t raw = as5600_get_raw_counts(channel);
    out->raw_counts = raw;
    out->raw_deg = (float)raw * AS5600_DEG_PER_COUNT;

    float cal = out->raw_deg - s_zero_offsets[channel];
    while (cal < 0.0f)   cal += 360.0f;
    while (cal >= 360.0f) cal -= 360.0f;
    out->cal_deg = cal;
    out->zero_offset = s_zero_offsets[channel];

    // Save to cache
    memcpy(&s_cached_telem[channel], out, sizeof(as5600_telemetry_t));
    s_last_telem_ms[channel] = now_ms;

    return ESP_OK;
}

void as5600_calibrate_zero(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return;
    uint16_t raw = as5600_get_raw_counts(channel);
    s_zero_offsets[channel] = (float)raw * AS5600_DEG_PER_COUNT;
    save_calibration(channel);
}

float as5600_get_zero_offset(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return 0.0f;
    return s_zero_offsets[channel];
}

void as5600_set_zero_offset(int channel, float offset_deg) {
    if (channel < 0 || channel >= MAX_CHANNELS) return;
    while (offset_deg < 0.0f)   offset_deg += 360.0f;
    while (offset_deg >= 360.0f) offset_deg -= 360.0f;
    s_zero_offsets[channel] = offset_deg;
    save_calibration(channel);
}

bool as5600_has_multiplexer(void) {
    return s_has_mux;
}
