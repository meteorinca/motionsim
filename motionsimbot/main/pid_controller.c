#include "pid_controller.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "PID"

static pid_params_t s_params = {
    .kp = 8.5f,
    .ki = 0.2f,
    .kd = 1.2f,
    .integral_max = 300.0f
};

static float s_targets[JOINT_COUNT] = {180.0f, 180.0f, 180.0f};
static float s_errors[JOINT_COUNT] = {0.0f};
static float s_integrals[JOINT_COUNT] = {0.0f};
static float s_last_errors[JOINT_COUNT] = {0.0f};

static void pid_task(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 0.001f; // 1 ms loop period (1 kHz)

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1)); // Precise 1 kHz timing

        if (motor_get_estop()) {
            motor_stop_all();
            continue;
        }

        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;
            float actual_angle = hall_read_angle(motor_id);
            float target_angle = s_targets[i];

            float error = target_angle - actual_angle;
            // Shortest angle wrap around (-180 to +180)
            while (error > 180.0f)  error -= 360.0f;
            while (error < -180.0f) error += 360.0f;

            s_errors[i] = error;

            // Integral accumulator with anti-windup clamp
            s_integrals[i] += error * dt;
            if (s_integrals[i] > s_params.integral_max)  s_integrals[i] = s_params.integral_max;
            if (s_integrals[i] < -s_params.integral_max) s_integrals[i] = -s_params.integral_max;

            // Derivative calculation
            float derivative = (error - s_last_errors[i]) / dt;
            s_last_errors[i] = error;

            // PID Output Calculation
            float output = (s_params.kp * error) + (s_params.ki * s_integrals[i]) + (s_params.kd * derivative);

            int16_t duty = (int16_t)output;
            motor_set_duty(motor_id, duty);
        }
    }
}

void pid_controller_init(void) {
    xTaskCreate(pid_task, "pid_loop", 4096, NULL, 22, NULL);
    ESP_LOGI(TAG, "1 kHz Closed-Loop PID Controller initialized (Kp:%.1f Ki:%.1f Kd:%.1f)",
             s_params.kp, s_params.ki, s_params.kd);
}

void pid_set_target_angle(int joint_id, float target_deg) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    while (target_deg < 0.0f)   target_deg += 360.0f;
    while (target_deg >= 360.0f) target_deg -= 360.0f;
    s_targets[joint_id - 1] = target_deg;
}

float pid_get_target_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_targets[joint_id - 1];
}

float pid_get_error(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_errors[joint_id - 1];
}

void pid_set_params(float kp, float ki, float kd) {
    s_params.kp = kp;
    s_params.ki = ki;
    s_params.kd = kd;
    ESP_LOGI(TAG, "PID Parameters updated: Kp=%.2f Ki=%.2f Kd=%.2f", kp, ki, kd);
}

pid_params_t pid_get_params(void) {
    return s_params;
}
