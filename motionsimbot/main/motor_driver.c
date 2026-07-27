#include "motor_driver.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdlib.h>

#define TAG "MOTOR_DRIVER"
#define PWM_FREQ_HZ 20000 // 20 kHz for silent operation

static bool s_estop = false;
static int16_t s_current_duties[MOTOR_COUNT] = {0};

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

void motor_driver_init(void) {
    // Enable GPIO
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << MOTOR_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&en_conf);
    gpio_set_level(MOTOR_ENABLE_GPIO, 1); // Enable IBT-2 drivers by default

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
    }

    motor_stop_all();
    ESP_LOGI(TAG, "IBT-2 Motor Driver initialized (20 kHz PWM, Max Duty Clamp: %d)", MOTOR_CLAMP_DUTY);
}

void motor_set_duty(int motor_id, int16_t duty) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return;
    int idx = motor_id - 1;

    if (s_estop) {
        duty = 0;
    }

    // Clamp duty cycle to 80% max limit
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

void motor_set_estop(bool estop) {
    s_estop = estop;
    gpio_set_level(MOTOR_ENABLE_GPIO, estop ? 0 : 1);
    if (estop) {
        motor_stop_all();
        ESP_LOGW(TAG, "EMERGENCY STOP ACTIVATED! All motors disabled.");
    } else {
        ESP_LOGI(TAG, "Emergency Stop Cleared. Motor drivers enabled.");
    }
}

bool motor_get_estop(void) {
    return s_estop;
}

int16_t motor_get_duty(int motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) return 0;
    return s_current_duties[motor_id - 1];
}
