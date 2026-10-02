// hall_sensor.c
// Single-turn 10k potentiometer angle sensing via ESP32 ADC.
// Three pots are wired to analog GPIO pins defined in board_config.h:
//   POT_JOINT1_ADC_CH, POT_JOINT2_ADC_CH, POT_JOINT3_ADC_CH
// Each pot output is read as a 12-bit value (0–4095) and converted to
// degrees (0–360°) with an optional per-joint zero calibration offset
// stored in NVS.

#include "hall_sensor.h"
#include "config.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include <string.h>

#define TAG             "POT_SENSOR"
#define NVS_NAMESPACE   "pot_cal"

// EMA filter coefficient (lower = smoother; 0 = frozen, 1 = no filter)
#define EMA_ALPHA       0.3f

// Spike rejection: maximum allowed angle change per tick (~10ms).
// Anything larger is treated as sensor noise and the previous value is held.
#define MAX_SLEW_DEG            15.0f
// After this many consecutive rejections, accept anyway (genuine fast motion).
#define MAX_CONSECUTIVE_REJECTS 3

static adc_oneshot_unit_handle_t s_adc_handle = NULL;

// ADC channel for each joint (index 0 = joint 1)
static const adc_channel_t s_adc_channels[JOINT_COUNT] = {
    POT_JOINT1_ADC_CH,
    POT_JOINT2_ADC_CH,
    POT_JOINT3_ADC_CH,
};

static const gpio_num_t s_gpio_pins[JOINT_COUNT] = {
    POT_JOINT1_GPIO,
    POT_JOINT2_GPIO,
    POT_JOINT3_GPIO,
};

static float   s_zero_offsets[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float   s_ema_angles[JOINT_COUNT]   = {0.0f, 0.0f, 0.0f};
static bool    s_ema_initialized[JOINT_COUNT] = {false, false, false};
static bool    s_connected[JOINT_COUNT]       = {false, false, false};
static uint16_t s_last_raw[JOINT_COUNT]       = {0, 0, 0};
static float   s_last_good_angle[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static bool    s_raw_initialized[JOINT_COUNT] = {false, false, false};
static int     s_reject_count[JOINT_COUNT]    = {0, 0, 0};

// ---------------------------------------------------------------------------
// NVS persistence
// ---------------------------------------------------------------------------
static void load_nvs_offsets(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < JOINT_COUNT; i++) {
            char key[16];
            snprintf(key, sizeof(key), "zero%d", i);
            uint32_t raw = 0;
            if (nvs_get_u32(h, key, &raw) == ESP_OK) {
                memcpy(&s_zero_offsets[i], &raw, sizeof(float));
                ESP_LOGI(TAG, "Loaded joint%d zero offset: %.2f deg", i + 1, s_zero_offsets[i]);
            }
        }
        nvs_close(h);
    }
}

static void save_nvs_offset(int idx) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        char key[16];
        snprintf(key, sizeof(key), "zero%d", idx);
        uint32_t raw;
        memcpy(&raw, &s_zero_offsets[idx], sizeof(float));
        nvs_set_u32(h, key, raw);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved joint%d zero offset: %.2f deg", idx + 1, s_zero_offsets[idx]);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void hall_sensor_init(void) {
    // Configure ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = POT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    // Configure each channel: 12-bit, 11dB attenuation (0–3.3V full scale)
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (int i = 0; i < JOINT_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_adc_channels[i], &ch_cfg));
        s_ema_angles[i]     = 0.0f;
        s_ema_initialized[i] = false;
    }

    load_nvs_offsets();

    for (int i = 0; i < JOINT_COUNT; i++) {
        hall_check_connection(i + 1);
        ESP_LOGI(TAG, "Joint %d pot plugged in: %s (pin GPIO%d)", i + 1, s_connected[i] ? "YES" : "NO", (int)s_gpio_pins[i]);
    }

    ESP_LOGI(TAG, "Potentiometer angle sensing initialised: 3× 10kΩ single-turn pots on ADC channels %d/%d/%d",
             (int)s_adc_channels[0], (int)s_adc_channels[1], (int)s_adc_channels[2]);
}

// ---------------------------------------------------------------------------
// Median-of-5 ADC pre-filter: takes 5 rapid samples and returns the median.
// Completely rejects single-sample outlier spikes (wiper bounce, noise).
// ---------------------------------------------------------------------------
static int median_adc_read(adc_channel_t channel) {
    int samples[5];
    int valid = 0;
    for (int s = 0; s < 5; s++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_handle, channel, &raw) == ESP_OK) {
            samples[valid++] = raw;
        }
    }
    if (valid == 0) return -1; // All reads failed
    // Simple insertion sort (n<=5, negligible cost)
    for (int i = 1; i < valid; i++) {
        int key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }
    return samples[valid / 2]; // Median
}

uint16_t hall_read_raw(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT || !s_adc_handle) return 0;
    int idx = joint_id - 1;
    int raw = median_adc_read(s_adc_channels[idx]);
    if (raw < 0) {
        // All ADC reads failed — return last known good value
        return s_raw_initialized[idx] ? s_last_raw[idx] : 0;
    }
    if (raw > 4095) raw = 4095;
    s_last_raw[idx] = (uint16_t)raw;
    s_raw_initialized[idx] = true;
    return (uint16_t)raw;
}

float hall_read_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    int idx = joint_id - 1;
    if (!s_adc_handle) return s_last_good_angle[idx];

    int raw_int = median_adc_read(s_adc_channels[idx]);
    if (raw_int < 0) {
        // All ADC reads failed — return last known good angle
        return s_last_good_angle[idx];
    }
    if (raw_int > 4095) raw_int = 4095;

    // Convert raw ADC → degrees with zero offset
    float angle = (float)raw_int * POT_DEG_PER_COUNT - s_zero_offsets[idx];

    // Wrap to [0, 360)
    while (angle < 0.0f)    angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;

    // Spike rejection: if the new sample jumps more than MAX_SLEW_DEG from
    // the last accepted value, reject it and hold previous. After
    // MAX_CONSECUTIVE_REJECTS in a row, accept anyway (genuine fast motion).
    if (s_ema_initialized[idx]) {
        float slew = angle - s_ema_angles[idx];
        while (slew >  180.0f) slew -= 360.0f;
        while (slew < -180.0f) slew += 360.0f;
        if (fabsf(slew) > MAX_SLEW_DEG && s_reject_count[idx] < MAX_CONSECUTIVE_REJECTS) {
            s_reject_count[idx]++;
            ESP_LOGD(TAG, "J%d spike rejected: %.1f deg jump (reject #%d)",
                     idx + 1, slew, s_reject_count[idx]);
            return s_ema_angles[idx]; // Hold previous filtered value
        }
        s_reject_count[idx] = 0; // Reset on accepted sample
    }

    // Circular EMA to smooth ADC noise without 0/360° wrap glitch
    if (!s_ema_initialized[idx]) {
        s_ema_angles[idx]     = angle;
        s_ema_initialized[idx] = true;
        s_last_good_angle[idx] = angle;
        return angle;
    }
    float diff = angle - s_ema_angles[idx];
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    s_ema_angles[idx] += EMA_ALPHA * diff;
    while (s_ema_angles[idx] <    0.0f)  s_ema_angles[idx] += 360.0f;
    while (s_ema_angles[idx] >= 360.0f)  s_ema_angles[idx] -= 360.0f;

    s_last_good_angle[idx] = s_ema_angles[idx];
    return s_ema_angles[idx];
}

void hall_calibrate_zero(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT || !s_adc_handle) return;
    int idx = joint_id - 1;

    int raw_int = median_adc_read(s_adc_channels[idx]);
    if (raw_int < 0) return;
    if (raw_int > 4095) raw_int = 4095;

    s_zero_offsets[idx] = (float)raw_int * POT_DEG_PER_COUNT;
    s_ema_initialized[idx] = false; // Re-seed EMA after zero change
    s_reject_count[idx] = 0;       // Reset spike rejection state
    save_nvs_offset(idx);
    ESP_LOGI(TAG, "Joint %d zeroed at raw=%d (%.2f deg)", joint_id, raw_int, s_zero_offsets[idx]);
}

float hall_get_zero_offset(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_zero_offsets[joint_id - 1];
}

bool hall_check_connection(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT || !s_adc_handle) return false;
    int idx = joint_id - 1;
    gpio_num_t pin = s_gpio_pins[idx];

    // Reset pull states to ensure clean starting measurement
    gpio_pullup_dis(pin);
    gpio_pulldown_dis(pin);

    // 1. Pull-up test: enable internal pull-up (~45k to 3.3V)
    gpio_pullup_en(pin);
    esp_rom_delay_us(250);
    int raw_pu = 0;
    adc_oneshot_read(s_adc_handle, s_adc_channels[idx], &raw_pu);
    gpio_pullup_dis(pin);

    // 2. Pull-down test: enable internal pull-down (~45k to GND)
    gpio_pulldown_en(pin);
    esp_rom_delay_us(250);
    int raw_pd = 0;
    adc_oneshot_read(s_adc_handle, s_adc_channels[idx], &raw_pd);
    gpio_pulldown_dis(pin);

    // If an analog sensor (10k pot) is plugged in, its output impedance is <= 2.5kΩ,
    // so the 45kΩ pull resistors cause only a small change (diff < 1200 counts).
    // If the sensor is UNPLUGGED (floating pin), pullup charges it to ~4095 and
    // pulldown discharges it to ~0, producing a huge difference (> 1500 counts).
    int diff = raw_pu - raw_pd;
    if (diff < 0) diff = -diff;

    bool connected = (diff < 1200);
    s_connected[idx] = connected;
    return connected;
}

bool hall_is_connected(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return false;
    return s_connected[joint_id - 1];
}
