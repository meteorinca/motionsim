#include "hall_sensor.h"
#include "as5600.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>

#define TAG "ANGLE_SENSOR"

static float s_ema_angles[JOINT_COUNT] = {0.0f};

void hall_sensor_init(void) {
    // Initialize AS5600 Magnetic Encoder on I2C bus (analog Hall sensors disabled)
    as5600_init();

    for (int i = 0; i < JOINT_COUNT; i++) {
        s_ema_angles[i] = 0.0f;
    }

    ESP_LOGI(TAG, "Angle Sensing initialized: AS5600 Digital I2C mode (Analog Hall sensors disabled)");
}

uint16_t hall_read_raw(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0;
    int ch = joint_id - 1;
    if (as5600_is_detected(ch)) {
        return as5600_get_raw_counts(ch);
    }
    return 0;
}

float hall_read_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    int idx = joint_id - 1;

    // Read high-precision 12-bit digital magnetic angle from AS5600
    if (as5600_is_detected(idx)) {
        float as_angle = as5600_get_calibrated_angle(idx);
        // Small EMA filter (alpha = 0.5) to reject magnetic jitter
        s_ema_angles[idx] = (0.5f * as_angle) + (0.5f * s_ema_angles[idx]);
        return s_ema_angles[idx];
    }

    return 0.0f;
}

void hall_calibrate_zero(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    int ch = joint_id - 1;
    if (as5600_is_detected(ch)) {
        as5600_calibrate_zero(ch);
    }
}

float hall_get_zero_offset(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    int ch = joint_id - 1;
    if (as5600_is_detected(ch)) {
        return as5600_get_zero_offset(ch);
    }
    return 0.0f;
}
