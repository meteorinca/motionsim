#include "hall_sensor.h"
#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <math.h>

#define TAG "HALL_SENSOR"
#define NVS_NAMESPACE "hall_cal"

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static float s_zero_offsets[JOINT_COUNT] = {0.0f};
static float s_ema_angles[JOINT_COUNT] = {0.0f};

static const adc_channel_t s_channels[JOINT_COUNT] = {
    HALL_JOINT1_ADC_CH,
    HALL_JOINT2_ADC_CH,
    HALL_JOINT2_ADC_CH // Shared or fallback for joint 3
};

static void load_calibration(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < JOINT_COUNT; i++) {
            char key[16];
            snprintf(key, sizeof(key), "zero_%d", i);
            uint32_t val_raw = 0;
            if (nvs_get_u32(h, key, &val_raw) == ESP_OK) {
                s_zero_offsets[i] = (float)val_raw / 1000.0f;
                ESP_LOGI(TAG, "Loaded calibration joint %d zero offset: %.2f deg", i + 1, s_zero_offsets[i]);
            }
        }
        nvs_close(h);
    }
}

static void save_calibration(int idx) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        char key[16];
        snprintf(key, sizeof(key), "zero_%d", idx);
        uint32_t val_raw = (uint32_t)(s_zero_offsets[idx] * 1000.0f);
        nvs_set_u32(h, key, val_raw);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved calibration joint %d zero offset: %.2f deg", idx + 1, s_zero_offsets[idx]);
    }
}

void hall_sensor_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = HALL_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12, // 0 - 3.3V range
    };

    for (int i = 0; i < JOINT_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_channels[i], &chan_cfg));
    }

    load_calibration();
    ESP_LOGI(TAG, "12-Bit Hall Angle Sensors initialized (Resolution: %.5f deg / count)", HALL_RESOLUTION_DEG);
}

uint16_t hall_read_raw(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT || !s_adc_handle) return 0;
    int raw = 0;
    adc_oneshot_read(s_adc_handle, s_channels[joint_id - 1], &raw);
    return (uint16_t)raw;
}

float hall_read_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    int idx = joint_id - 1;

    uint16_t raw = hall_read_raw(joint_id);
    float raw_angle = (float)raw * HALL_RESOLUTION_DEG;

    float cal_angle = raw_angle - s_zero_offsets[idx];
    while (cal_angle < 0.0f) cal_angle += 360.0f;
    while (cal_angle >= 360.0f) cal_angle -= 360.0f;

    // EMA Filter (alpha = 0.3)
    s_ema_angles[idx] = (0.3f * cal_angle) + (0.7f * s_ema_angles[idx]);

    return s_ema_angles[idx];
}

void hall_calibrate_zero(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    int idx = joint_id - 1;

    uint16_t raw = hall_read_raw(joint_id);
    s_zero_offsets[idx] = (float)raw * HALL_RESOLUTION_DEG;
    save_calibration(idx);
}

float hall_get_zero_offset(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_zero_offsets[joint_id - 1];
}
