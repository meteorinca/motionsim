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

static pid_control_mode_t s_control_mode = PID_MODE_STEP_TEST; // Default to step test mode until UDP commands arrive
static float s_targets[JOINT_COUNT] = {180.0f, 180.0f, 180.0f};
static float s_errors[JOINT_COUNT] = {0.0f};
static float s_integrals[JOINT_COUNT] = {0.0f};
static float s_last_errors[JOINT_COUNT] = {0.0f};
static int16_t s_manual_duties[JOINT_COUNT] = {0};

void pid_reset_integrals(void) {
    for (int i = 0; i < JOINT_COUNT; i++) {
        s_integrals[i] = 0.0f;
        s_last_errors[i] = 0.0f;
    }
}

static void pid_task(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t delay_ticks = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
    const float dt = (float)delay_ticks / (float)configTICK_RATE_HZ;

    while (1) {
        vTaskDelayUntil(&last_wake, delay_ticks);

        // Always read sensors so telemetry is active in all states
        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;
            float actual_angle = hall_read_angle(motor_id);
            float target_angle = s_targets[i];

            float error = target_angle - actual_angle;
            // Shortest angle wrap around (-180 to +180)
            while (error > 180.0f)  error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
            s_errors[i] = error;
        }

        // Safety Interlock: if motors are not armed or E-Stop is active, keep motors stopped
        if (!motor_is_armed() || motor_get_estop()) {
            motor_stop_all();
            pid_reset_integrals();
            continue;
        }

        if (s_control_mode == PID_MODE_MANUAL_DUTY) {
            for (int i = 0; i < JOINT_COUNT; i++) {
                motor_set_duty(i + 1, s_manual_duties[i]);
            }
            continue;
        }

        // Closed-loop PID control (Step Test or UDP follow)
        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;
            float error = s_errors[i];

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

void pid_set_mode(pid_control_mode_t mode) {
    s_control_mode = mode;
    pid_reset_integrals();
}

pid_control_mode_t pid_get_mode(void) {
    return s_control_mode;
}

void pid_set_manual_duty(int joint_id, int16_t duty) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    s_manual_duties[joint_id - 1] = duty;
    s_control_mode = PID_MODE_MANUAL_DUTY;
}
