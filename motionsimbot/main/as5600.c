#include "as5600.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
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
        esp_err_t ret = i2c_master_transmit(s_mux_dev, &mux_mask, 1, 50);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to select mux channel %d: %s", channel, esp_err_to_name(ret));
            return ret;
        }
    }
    s_current_channel = channel;
    return ESP_OK;
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t len) {
    if (!s_as5600_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_as5600_dev, &reg, 1, data, len, 100);
}

esp_err_t as5600_init(void) {
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
            .scl_speed_hz = 400000,
        };
        if (i2c_master_bus_add_device(bus, &mux_cfg, &s_mux_dev) == ESP_OK) {
            s_has_mux = true;
            ESP_LOGI(TAG, "TCA9548A I2C Multiplexer detected at 0x70");
        }
    } else {
        s_has_mux = false;
        ESP_LOGI(TAG, "Direct I2C connection (no TCA9548A multiplexer detected)");
    }

    // Register AS5600 device
    i2c_device_config_t as5600_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &as5600_cfg, &s_as5600_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register AS5600 I2C device: %s", esp_err_to_name(err));
        return err;
    }

    load_calibration();

    // Probe channel 0
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
        s_detected[0] = false;
        ESP_LOGW(TAG, "AS5600 Encoder NOT found on channel 0 at 0x36");
    }

    return s_detected[0] ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool as5600_is_detected(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return false;
    return s_detected[channel];
}

uint16_t as5600_get_raw_counts(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return 0;
    if (as5600_select_channel(channel) != ESP_OK) return 0;

    uint8_t buf[2] = {0};
    if (read_registers(AS5600_REG_RAW_ANGLE_H, buf, 2) != ESP_OK) {
        return 0;
    }
    return (((uint16_t)(buf[0] & 0x0F)) << 8) | buf[1];
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
    memset(out, 0, sizeof(*out));

    if (as5600_select_channel(channel) != ESP_OK) {
        out->detected = false;
        return ESP_FAIL;
    }

    uint8_t status = 0;
    if (read_registers(AS5600_REG_STATUS, &status, 1) != ESP_OK) {
        out->detected = false;
        s_detected[channel] = false;
        return ESP_FAIL;
    }

    out->detected = true;
    s_detected[channel] = true;
    out->magnet_detected = (status & AS5600_STATUS_MD) != 0;
    out->magnet_too_weak = (status & AS5600_STATUS_ML) != 0;
    out->magnet_too_strong = (status & AS5600_STATUS_MH) != 0;

    uint8_t agc = 0;
    if (read_registers(AS5600_REG_AGC, &agc, 1) == ESP_OK) {
        out->agc = agc;
    }

    uint8_t mag_buf[2] = {0};
    if (read_registers(AS5600_REG_MAGNITUDE_H, mag_buf, 2) == ESP_OK) {
        out->magnitude = (((uint16_t)(mag_buf[0] & 0x0F)) << 8) | mag_buf[1];
    }

    uint16_t raw = as5600_get_raw_counts(channel);
    out->raw_counts = raw;
    out->raw_deg = (float)raw * AS5600_DEG_PER_COUNT;

    float cal = out->raw_deg - s_zero_offsets[channel];
    while (cal < 0.0f)   cal += 360.0f;
    while (cal >= 360.0f) cal -= 360.0f;
    out->cal_deg = cal;
    out->zero_offset = s_zero_offsets[channel];

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
