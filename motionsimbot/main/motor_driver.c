#include "motor_driver.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "pid_controller.h"
#include "hall_sensor.h"
#include <stdlib.h>

#define TAG "MOTOR_DRIVER"
#define PWM_FREQ_HZ 20000 // 20 kHz for silent operation
#define NVS_NAMESPACE "motor_cfg"

static bool s_estop = false;
static bool s_armed = false;
static int16_t s_current_duties[MOTOR_COUNT] = {0};
static bool s_inverted[MOTOR_COUNT] = {false, false, false};
static TimerHandle_t s_jog_timers[MOTOR_COUNT] = {NULL, NULL, NULL};
static bool s_jogging[MOTOR_COUNT] = {false, false, false};

// LEDC Channels for 3 motors (2 channels per motor = 6 channels)
static const ledc_channel_t s_rpwm_channels[MOTOR_COUNT] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_2, LEDC_CHANNEL_4
};
static const ledc_channel_t s_lpwm_channels[MOTOR_COUNT] = {
    LEDC_CHANNEL_1, LEDC_CHANNEL_3, LEDC_CHANNEL_5
};

static const gpio_num_t s_rpwm_gpios[MOTOR_COUNT] = {
    MOTOR1_RPWM_GPIO, MOTOR2_RPWM_GPIO, MOTOR3_RPWM_GPIO
};
static const gpio_num_t s_lpwm_gpios[MOTOR_COUNT] = {
    MOTOR1_LPWM_GPIO, MOTOR2_LPWM_GPIO, MOTOR3_LPWM_GPIO
};

static void load_nvs_config(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            char key[16];
            snprintf(key, sizeof(key), "inv_%d", i);
            uint8_t inv = 0;
            if (nvs_get_u8(h, key, &inv) == ESP_OK) {
                s_inverted[i] = (inv != 0);
                ESP_LOGI(TAG, "Motor %d direction inverted: %s", i + 1, s_inverted[i] ? "YES" : "NO");
            }
        }
        nvs_close(h);
    }
}

static void save_nvs_inversion(int idx) {
    if (idx < 0 || idx >= MOTOR_COUNT) return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        char key[16];
        snprintf(key, sizeof(key), "inv_%d", idx);
        nvs_set_u8(h, key, s_inverted[idx] ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved motor %d inversion: %d", idx + 1, s_inverted[idx] ? 1 : 0);
    }
}

static void jog_timer_cb(TimerHandle_t xTimer) {
    int idx = (int)(intptr_t)pvTimerGetTimerID(xTimer);
    if (idx >= 0 && idx < MOTOR_COUNT) {
        s_jogging[idx] = false;
        motor_set_duty(idx + 1, 0);

        // When open-loop jog finishes, hold the new physical position
        float curr = hall_read_angle(idx + 1);
        if (curr > 0.01f) {
            pid_set_target_angle(idx + 1, curr);
        }
        ESP_LOGI(TAG, "Jog finished for Motor %d, holding at %.2f deg", idx + 1, curr);
    }
}

void motor_driver_init(void) {
    // Enable GPIO (Hardware E-Stop / Shared IBT-2 R_EN + L_EN)
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << MOTOR_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&en_conf);
    gpio_set_level(MOTOR_ENABLE_GPIO, 0); // Boot in SAFE DISARMED state (motors cannot move)
    s_armed = false;
    s_estop = false;

    // Load persisted direction settings
    load_nvs_config();

    // Timer Config (20 kHz, 10-bit resolution = 0..1023)
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    // Channel Config for 3 motors × 2 direction PWM pins
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t rpwm_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = s_rpwm_channels[i],
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = s_rpwm_gpios[i],
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&rpwm_cfg);

        ledc_channel_config_t lpwm_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = s_lpwm_channels[i],
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = s_lpwm_gpios[i],
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&lpwm_cfg);

        char timer_name[16];
        snprintf(timer_name, sizeof(timer_name), "jog_tmr_%d", i);
        s_jog_timers[i] = xTimerCreate(timer_name, pdMS_TO_TICKS(500), pdFALSE, (void *)(intptr_t)i, jog_timer_cb);
    }

    motor_stop_all();
    ESP_LOGI(TAG, "IBT-2 Motor Driver initialized (20 kHz PWM, Max Duty Clamp: %d)", MOTOR_CLAMP_DUTY);
}

void motor_set_duty(int motor_id, int16_t duty) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return;
    int idx = motor_id - 1;

    // Safety Interlock: disarmed or E-stop forces duty to 0, UNLESS actively jogging
    if ((!s_armed || s_estop) && !s_jogging[idx]) {
        duty = 0;
    }

    // Apply polarity inversion if configured
    if (s_inverted[idx]) {
        duty = -duty;
    }

    // Clamp duty cycle to protection limit
    if (duty > MOTOR_CLAMP_DUTY) duty = MOTOR_CLAMP_DUTY;
    if (duty < -MOTOR_CLAMP_DUTY) duty = -MOTOR_CLAMP_DUTY;

    s_current_duties[idx] = duty;

    uint32_t rpwm = 0;
    uint32_t lpwm = 0;

    if (duty > 0) {
        rpwm = duty;
        lpwm = 0;
    } else if (duty < 0) {
        rpwm = 0;
        lpwm = -duty;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_rpwm_channels[idx], rpwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_rpwm_channels[idx]);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, s_lpwm_channels[idx], lpwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, s_lpwm_channels[idx]);
}

void motor_stop_all(void) {
    for (int i = 1; i <= MOTOR_COUNT; i++) {
        motor_set_duty(i, 0);
    }
}

void motor_arm(bool arm) {
    if (arm) {
        if (s_estop) {
            ESP_LOGW(TAG, "Cannot ARM motors: Emergency Stop is active! Clear E-Stop first.");
            return;
        }
        s_armed = true;
        gpio_set_level(MOTOR_ENABLE_GPIO, 1);
        ESP_LOGI(TAG, "MOTORS ARMED: IBT-2 driver enable active (GPIO %d = 1)", MOTOR_ENABLE_GPIO);
    } else {
        s_armed = false;
        gpio_set_level(MOTOR_ENABLE_GPIO, 0);
        motor_stop_all();
        ESP_LOGI(TAG, "MOTORS DISARMED: Safe State (GPIO %d = 0)", MOTOR_ENABLE_GPIO);
    }
}

bool motor_is_armed(void) {
    return s_armed && !s_estop;
}

void motor_set_estop(bool estop) {
    s_estop = estop;
    if (estop) {
        s_armed = false;
        gpio_set_level(MOTOR_ENABLE_GPIO, 0);
        motor_stop_all();
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (s_jog_timers[i] && xTimerIsTimerActive(s_jog_timers[i])) {
                xTimerStop(s_jog_timers[i], 0);
            }
            s_jogging[i] = false;
        }
        ESP_LOGW(TAG, "EMERGENCY STOP ACTIVATED! All motors disarmed and disabled.");
    } else {
        ESP_LOGI(TAG, "Emergency Stop Cleared. Ready to Arm or Jog.");
    }
}

bool motor_get_estop(void) {
    return s_estop;
}

int16_t motor_get_duty(int motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return 0;
    return s_current_duties[motor_id - 1];
}

void motor_set_inverted(int motor_id, bool inverted) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return;
    int idx = motor_id - 1;
    s_inverted[idx] = inverted;
    save_nvs_inversion(idx);
}

bool motor_is_inverted(int motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return false;
    return s_inverted[motor_id - 1];
}

// Open-loop jog works all the time (easy small steps to test / recover)
void motor_jog(int motor_id, int16_t duty, uint32_t duration_ms) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return;
    int idx = motor_id - 1;

    if (duty == 0) {
        motor_jog_stop(motor_id);
        return;
    }

    if (duration_ms == 0) duration_ms = 350; // Quick gentle pulse by default
    if (duration_ms > 3000) duration_ms = 3000;

    // Clear any prior E-Stop so user can jog freely in open loop
    if (s_estop) {
        s_estop = false;
    }

    // Enable hardware bridge and set jogging flag
    s_jogging[idx] = true;
    s_armed = true;
    gpio_set_level(MOTOR_ENABLE_GPIO, 1);

    // Apply open-loop duty
    motor_set_duty(motor_id, duty);

    // Start auto-stop timer
    if (s_jog_timers[idx]) {
        xTimerChangePeriod(s_jog_timers[idx], pdMS_TO_TICKS(duration_ms), 0);
        xTimerStart(s_jog_timers[idx], 0);
    }
    ESP_LOGI(TAG, "Jogging Motor %d: duty=%d for %lu ms (Open Loop)", motor_id, duty, (unsigned long)duration_ms);
}

void motor_jog_stop(int motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return;
    int idx = motor_id - 1;
    if (s_jog_timers[idx]) xTimerStop(s_jog_timers[idx], 0);
    s_jogging[idx] = false;
    motor_set_duty(motor_id, 0);

    float curr = hall_read_angle(motor_id);
    if (curr > 0.01f) {
        pid_set_target_angle(motor_id, curr);
    }
}

bool motor_is_jogging(int motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return false;
    return s_jogging[motor_id - 1];
}
