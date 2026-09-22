#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// AS5600 7-bit I2C Slave Address
#define AS5600_I2C_ADDR         0x36
#define TCA9548A_I2C_ADDR       0x70

// AS5600 Register Addresses
#define AS5600_REG_ZMCO         0x00
#define AS5600_REG_ZPOS_H       0x01
#define AS5600_REG_ZPOS_L       0x02
#define AS5600_REG_CONF_H       0x07
#define AS5600_REG_CONF_L       0x08
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_RAW_ANGLE_L  0x0D
#define AS5600_REG_ANGLE_H      0x0E
#define AS5600_REG_ANGLE_L      0x0F
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE_H  0x1B
#define AS5600_REG_MAGNITUDE_L  0x1C

// Status Register bit masks
#define AS5600_STATUS_MD        (1 << 5) // Magnet Detected
#define AS5600_STATUS_ML        (1 << 4) // Magnet Too Weak
#define AS5600_STATUS_MH        (1 << 3) // Magnet Too Strong

#define AS5600_RESOLUTION_BITS  12
#define AS5600_MAX_COUNTS       4096
#define AS5600_DEG_PER_COUNT    (360.0f / 4096.0f) // 0.087890625 deg

typedef struct {
    bool detected;
    bool magnet_detected;
    bool magnet_too_weak;
    bool magnet_too_strong;
    uint8_t agc;
    uint16_t magnitude;
    uint16_t raw_counts;
    float raw_deg;
    float cal_deg;
    float zero_offset;
} as5600_telemetry_t;

esp_err_t as5600_init(void);
bool as5600_is_detected(int channel);
esp_err_t as5600_read_telemetry(int channel, as5600_telemetry_t *out);
float as5600_get_calibrated_angle(int channel);
uint16_t as5600_get_raw_counts(int channel);
void as5600_calibrate_zero(int channel);
float as5600_get_zero_offset(int channel);
void as5600_set_zero_offset(int channel, float offset_deg);

// Multiplexer helper (for TCA9548A when second AS5600 is added)
bool as5600_has_multiplexer(void);
esp_err_t as5600_select_channel(int channel);

#ifdef __cplusplus
}
#endif
