#include "pid_controller.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "as5600.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define TAG "PID"
#define NVS_PID_NAMESPACE "pid_cfg"

static pid_params_t s_params = {
    .kp = 8.5f,
    .ki = 0.2f,
    .kd = 1.2f,
    .integral_max = 300.0f
};

static pid_control_mode_t s_control_mode = PID_MODE_FOLLOW;
static float s_targets[JOINT_COUNT] = {180.0f, 180.0f, 180.0f};
static float s_actuals[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_last_actuals[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_errors[JOINT_COUNT] = {0.0f};
static float s_integrals[JOINT_COUNT] = {0.0f};
static float s_last_errors[JOINT_COUNT] = {0.0f};
static int16_t s_manual_duties[JOINT_COUNT] = {0};
static bool s_target_initialized[JOINT_COUNT] = {false, false, false};

// Automatic E-Stop Supervision
static bool s_auto_estop_enabled = true;
static estop_reason_code_t s_estop_reason_code = ESTOP_REASON_NONE;
static char s_estop_reason_str[96] = "System Normal";
static int s_runaway_counters[JOINT_COUNT] = {0};
static int s_stall_counters[JOINT_COUNT] = {0};
static float s_stall_ref_angles[JOINT_COUNT] = {0.0f};
static int s_magnet_err_counters[JOINT_COUNT] = {0};
static int s_grace_period_ticks = 0; // Settling period after reset/clear to prevent immediate re-trip

// Autotune State
static autotune_state_t s_autotune_state = AUTOTUNE_IDLE;
static int s_autotune_joint = 1;
static uint32_t s_autotune_start_ms = 0;
static float s_autotune_start_angle = 0.0f;
static float s_autotune_peak_angle = 0.0f;
static uint32_t s_autotune_rise_ms = 0;
static char s_autotune_status[96] = "Idle";

static float shortest_angle_diff(float target, float actual) {
    float diff = target - actual;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

static void load_pid_nvs(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_PID_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        uint32_t kp_m = 0, ki_m = 0, kd_m = 0;
        if (nvs_get_u32(h, "kp", &kp_m) == ESP_OK) s_params.kp = (float)kp_m / 1000.0f;
        if (nvs_get_u32(h, "ki", &ki_m) == ESP_OK) s_params.ki = (float)ki_m / 1000.0f;
        if (nvs_get_u32(h, "kd", &kd_m) == ESP_OK) s_params.kd = (float)kd_m / 1000.0f;
        uint8_t auto_es = 1;
        if (nvs_get_u8(h, "auto_estop", &auto_es) == ESP_OK) s_auto_estop_enabled = (auto_es != 0);
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded PID from NVS: Kp=%.2f Ki=%.2f Kd=%.2f (Auto-Estop: %d)",
                 s_params.kp, s_params.ki, s_params.kd, s_auto_estop_enabled);
    }
}

esp_err_t pid_save_params_to_nvs(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_PID_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u32(h, "kp", (uint32_t)(s_params.kp * 1000.0f));
    nvs_set_u32(h, "ki", (uint32_t)(s_params.ki * 1000.0f));
    nvs_set_u32(h, "kd", (uint32_t)(s_params.kd * 1000.0f));
    nvs_set_u8(h, "auto_estop", s_auto_estop_enabled ? 1 : 0);
    err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved PID params to NVS: Kp=%.2f Ki=%.2f Kd=%.2f",
             s_params.kp, s_params.ki, s_params.kd);
    return err;
}

void pid_trigger_estop(estop_reason_code_t reason, const char *detail) {
    s_estop_reason_code = reason;
    if (detail && strlen(detail) > 0) {
        strncpy(s_estop_reason_str, detail, sizeof(s_estop_reason_str) - 1);
        s_estop_reason_str[sizeof(s_estop_reason_str) - 1] = '\0';
    } else {
        snprintf(s_estop_reason_str, sizeof(s_estop_reason_str), "E-Stop Code %d", (int)reason);
    }
    motor_set_estop(true);
    pid_reset_integrals();
    ESP_LOGW(TAG, "TRIP E-STOP [%d]: %s", (int)reason, s_estop_reason_str);
}

void pid_clear_estop(void) {
    s_estop_reason_code = ESTOP_REASON_NONE;
    snprintf(s_estop_reason_str, sizeof(s_estop_reason_str), "System Normal");
    motor_set_estop(false);

    // Re-align all targets to current actual angles so error starts at exactly 0.00 deg
    for (int i = 0; i < JOINT_COUNT; i++) {
        s_runaway_counters[i] = 0;
        s_stall_counters[i] = 0;
        s_magnet_err_counters[i] = 0;
        s_last_actuals[i] = s_actuals[i];
        s_stall_ref_angles[i] = s_actuals[i];
        if (s_actuals[i] > 0.01f) {
            s_targets[i] = s_actuals[i];
        }
        s_errors[i] = 0.0f;
        s_last_errors[i] = 0.0f;
        s_integrals[i] = 0.0f;
    }
    // Set 1-second (100 ticks @ 100 Hz) settling grace period
    s_grace_period_ticks = 100;
    ESP_LOGI(TAG, "Emergency Stop Cleared by User (grace period 1000ms active).");
}

estop_reason_code_t pid_get_estop_reason_code(void) {
    return s_estop_reason_code;
}

const char* pid_get_estop_reason_str(void) {
    return s_estop_reason_str;
}

void pid_enable_auto_estop(bool enable) {
    s_auto_estop_enabled = enable;
    ESP_LOGI(TAG, "Auto E-Stop supervision: %s", enable ? "ENABLED" : "DISABLED");
}

bool pid_is_auto_estop_enabled(void) {
    return s_auto_estop_enabled;
}

void pid_reset_integrals(void) {
    for (int i = 0; i < JOINT_COUNT; i++) {
        s_integrals[i] = 0.0f;
        s_last_errors[i] = 0.0f;
        s_runaway_counters[i] = 0;
        s_stall_counters[i] = 0;
        s_magnet_err_counters[i] = 0;
    }
}

// Perform automated step-response autotune
static void process_autotune(uint32_t now_ms) {
    int j_idx = s_autotune_joint - 1;
    float current_angle = s_actuals[j_idx];

    switch (s_autotune_state) {
        case AUTOTUNE_BASELINE:
            if (now_ms - s_autotune_start_ms > 400) {
                s_autotune_start_angle = current_angle;
                s_autotune_peak_angle = current_angle;
                s_autotune_state = AUTOTUNE_STEP_FWD;
                s_autotune_start_ms = now_ms;
                // Command a small safe +8 degree step
                s_targets[j_idx] = s_autotune_start_angle + 8.0f;
                if (s_targets[j_idx] >= 360.0f) s_targets[j_idx] -= 360.0f;
                snprintf(s_autotune_status, sizeof(s_autotune_status), "Step +8 deg in progress...");
            }
            break;

        case AUTOTUNE_STEP_FWD: {
            float delta = current_angle - s_autotune_start_angle;
            while (delta > 180.0f)  delta -= 360.0f;
            while (delta < -180.0f) delta += 360.0f;

            if (delta > s_autotune_peak_angle - s_autotune_start_angle) {
                s_autotune_peak_angle = current_angle;
            }

            // Check if 90% of target step reached
            if (delta >= 7.2f && s_autotune_rise_ms == 0) {
                s_autotune_rise_ms = now_ms - s_autotune_start_ms;
            }

            if (now_ms - s_autotune_start_ms > 1500) {
                // Return step
                s_autotune_state = AUTOTUNE_STEP_REV;
                s_targets[j_idx] = s_autotune_start_angle;
                s_autotune_start_ms = now_ms;
                snprintf(s_autotune_status, sizeof(s_autotune_status), "Returning to baseline...");
            }
            break;
        }

        case AUTOTUNE_STEP_REV:
            if (now_ms - s_autotune_start_ms > 1500) {
                s_autotune_state = AUTOTUNE_COMPUTING;
            }
            break;

        case AUTOTUNE_COMPUTING: {
            float step_mag = 8.0f;
            float max_val = s_autotune_peak_angle - s_autotune_start_angle;
            float overshoot = 0.0f;
            if (max_val > step_mag) {
                overshoot = (max_val - step_mag) / step_mag;
            }

            // Heuristic PD calculation based on rise time and overshoot
            float calculated_kp = s_params.kp;
            float calculated_kd = s_params.kd;

            if (s_autotune_rise_ms > 0) {
                if (s_autotune_rise_ms > 600) {
                    calculated_kp = fminf(20.0f, s_params.kp * 1.35f);
                } else if (s_autotune_rise_ms < 200) {
                    calculated_kp = fmaxf(4.0f, s_params.kp * 0.85f);
                }

                if (overshoot > 0.10f) {
                    calculated_kd = fminf(6.0f, s_params.kd * (1.0f + overshoot * 2.0f));
                } else if (overshoot < 0.02f) {
                    calculated_kd = fmaxf(0.5f, s_params.kd * 0.9f);
                }
            } else {
                calculated_kp = fminf(20.0f, s_params.kp * 1.25f);
            }

            s_params.kp = roundf(calculated_kp * 10.0f) / 10.0f;
            s_params.kd = roundf(calculated_kd * 10.0f) / 10.0f;

            snprintf(s_autotune_status, sizeof(s_autotune_status),
                     "Tuned! Kp:%.1f Kd:%.1f (Rise:%lums OS:%.0f%%)",
                     s_params.kp, s_params.kd, (unsigned long)s_autotune_rise_ms, overshoot * 100.0f);
            s_autotune_state = AUTOTUNE_SUCCESS;
            s_control_mode = PID_MODE_FOLLOW;
            ESP_LOGI(TAG, "Autotune complete: %s", s_autotune_status);
            break;
        }

        default:
            break;
    }
}

static void pid_task(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    // 100 Hz closed-loop PID rate (10 ms per tick)
    const TickType_t delay_ticks = pdMS_TO_TICKS(10);
    const float dt = 0.010f;

    while (1) {
        vTaskDelayUntil(&last_wake, delay_ticks);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // Decrement settling grace period counter
        if (s_grace_period_ticks > 0) {
            s_grace_period_ticks--;
        }

        // 1. Read all sensors & update telemetry
        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;
            float current_angle = hall_read_angle(motor_id);
            s_actuals[i] = current_angle;

            // Auto-seed target on boot / first detection so motor never snaps
            if (!s_target_initialized[i] && current_angle > 0.01f) {
                s_targets[i] = current_angle;
                s_last_actuals[i] = current_angle;
                s_stall_ref_angles[i] = current_angle;
                s_target_initialized[i] = true;
                ESP_LOGI(TAG, "Joint %d initialized target to current physical angle: %.2f deg", motor_id, current_angle);
            }

            float error = shortest_angle_diff(s_targets[i], current_angle);
            s_errors[i] = error;
        }

        // 2. Multi-Hazard Automatic E-Stop Supervision
        // Skipped during settling grace period or when motors are jogging in open loop
        if (s_auto_estop_enabled && motor_is_armed() && !motor_get_estop() && s_grace_period_ticks == 0) {
            for (int i = 0; i < JOINT_COUNT; i++) {
                int motor_id = i + 1;
                // If this motor is actively jogging in open loop, suspend auto-estop checks on it
                if (motor_is_jogging(motor_id)) continue;
                if (!as5600_is_detected(i)) continue;

                float current_angle = s_actuals[i];
                float last_angle = s_last_actuals[i];
                float error = s_errors[i];
                int16_t duty = motor_get_duty(motor_id);

                // Hazard A: Velocity Jump (require physically impossible jump > 45.0 deg in 10ms = 4500 deg/s)
                float d_angle = fabsf(shortest_angle_diff(current_angle, last_angle));
                if (d_angle > 45.0f) {
                    char err_msg[96];
                    snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: Sensor Jump (%.1f deg in 10ms on Joint %d)", d_angle, motor_id);
                    pid_trigger_estop(ESTOP_REASON_VELOCITY_JUMP, err_msg);
                    break;
                }

                // Hazard B: Magnet Loss (require 60 consecutive failed reads = 600ms to avoid false triggers)
                uint16_t raw = hall_read_raw(motor_id);
                if (raw == 0) {
                    s_magnet_err_counters[i]++;
                    if (s_magnet_err_counters[i] > 60) {
                        char err_msg[96];
                        snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: AS5600 Sensor Disconnect on Joint %d", motor_id);
                        pid_trigger_estop(ESTOP_REASON_MAGNET_FAULT, err_msg);
                        break;
                    }
                } else {
                    s_magnet_err_counters[i] = 0;
                }

                // Hazard C: Runaway / Reverse Feedback
                // Only if applying strong duty (> 250) towards target, but error steadily grows for > 350 ms
                if (s_control_mode != PID_MODE_MANUAL_DUTY && fabsf(error) > 6.0f && abs(duty) > 250) {
                    float d_err = fabsf(error) - fabsf(s_last_errors[i]);
                    if (d_err > 0.06f) {
                        s_runaway_counters[i]++;
                        if (s_runaway_counters[i] > 35) { // 350 ms continuous divergence
                            char err_msg[96];
                            snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: Runaway on Joint %d (Check Motor Direction Invert)", motor_id);
                            pid_trigger_estop(ESTOP_REASON_RUNAWAY, err_msg);
                            break;
                        }
                    } else {
                        if (s_runaway_counters[i] > 0) s_runaway_counters[i]--;
                    }
                } else {
                    s_runaway_counters[i] = 0;
                }

                // Hazard D: Actuator Stall / Mechanical Jam
                // High duty (> 300) for > 1.5s with less than 0.3 deg motion
                if (s_control_mode != PID_MODE_MANUAL_DUTY && fabsf(error) > 6.0f && abs(duty) > 300) {
                    float motion = fabsf(shortest_angle_diff(current_angle, s_stall_ref_angles[i]));
                    if (motion < 0.3f) {
                        s_stall_counters[i]++;
                        if (s_stall_counters[i] > 150) { // 1500 ms stalled
                            char err_msg[96];
                            snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: Stall / Jam on Joint %d (>30%% duty, no motion)", motor_id);
                            pid_trigger_estop(ESTOP_REASON_STALL, err_msg);
                            break;
                        }
                    } else {
                        s_stall_counters[i] = 0;
                        s_stall_ref_angles[i] = current_angle;
                    }
                } else {
                    s_stall_counters[i] = 0;
                    s_stall_ref_angles[i] = current_angle;
                }
            }
        }

        // Store last actuals for velocity calculations
        for (int i = 0; i < JOINT_COUNT; i++) {
            s_last_actuals[i] = s_actuals[i];
        }

        // 3. Safety Interlock: if motors are not armed or E-Stop is active, keep motors stopped (unless jogging)
        bool any_jogging = false;
        for (int i = 1; i <= JOINT_COUNT; i++) {
            if (motor_is_jogging(i)) any_jogging = true;
        }

        if ((!motor_is_armed() || motor_get_estop()) && !any_jogging) {
            motor_stop_all();
            pid_reset_integrals();
            continue;
        }

        // 4. Handle Autotuning sequence if active
        if (s_autotune_state != AUTOTUNE_IDLE && s_autotune_state != AUTOTUNE_SUCCESS && s_autotune_state != AUTOTUNE_FAILED) {
            process_autotune(now_ms);
        }

        // 5. Manual Duty Mode
        if (s_control_mode == PID_MODE_MANUAL_DUTY) {
            for (int i = 0; i < JOINT_COUNT; i++) {
                if (!motor_is_jogging(i + 1)) {
                    motor_set_duty(i + 1, s_manual_duties[i]);
                }
            }
            continue;
        }

        // 6. Closed-Loop PID / PD Control Loop
        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;

            // If this joint is actively running an open-loop jog, skip PID calculation
            if (motor_is_jogging(motor_id)) {
                continue;
            }

            // Only drive motors that have an active angle sensor detected
            if (!as5600_is_detected(i)) {
                motor_set_duty(motor_id, 0);
                continue;
            }

            float error = s_errors[i];

            // Integral accumulator with anti-windup clamp
            s_integrals[i] += error * dt;
            if (s_integrals[i] > s_params.integral_max)  s_integrals[i] = s_params.integral_max;
            if (s_integrals[i] < -s_params.integral_max) s_integrals[i] = -s_params.integral_max;

            // Derivative calculation (error rate of change)
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
    load_pid_nvs();
    xTaskCreate(pid_task, "pid_loop", 4096, NULL, 22, NULL);
    ESP_LOGI(TAG, "100 Hz Closed-Loop PID Controller initialized (Kp:%.1f Ki:%.1f Kd:%.1f)",
             s_params.kp, s_params.ki, s_params.kd);
}

void pid_set_target_angle(int joint_id, float target_deg) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    while (target_deg < 0.0f)   target_deg += 360.0f;
    while (target_deg >= 360.0f) target_deg -= 360.0f;
    s_targets[joint_id - 1] = target_deg;
    s_target_initialized[joint_id - 1] = true;
}

float pid_get_target_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_targets[joint_id - 1];
}

float pid_get_actual_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_actuals[joint_id - 1];
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

esp_err_t pid_start_autotune(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return ESP_ERR_INVALID_ARG;
    if (!as5600_is_detected(joint_id - 1)) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Error: No AS5600 sensor detected on Joint %d", joint_id);
        return ESP_ERR_NOT_FOUND;
    }
    if (motor_get_estop()) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Error: E-Stop active");
        return ESP_ERR_INVALID_STATE;
    }

    s_autotune_joint = joint_id;
    s_autotune_rise_ms = 0;
    s_autotune_start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_autotune_state = AUTOTUNE_BASELINE;
    s_control_mode = PID_MODE_AUTOTUNE;
    motor_arm(true);
    snprintf(s_autotune_status, sizeof(s_autotune_status), "Calibrating baseline...");
    ESP_LOGI(TAG, "Starting PD Autotune on Joint %d", joint_id);
    return ESP_OK;
}

void pid_abort_autotune(void) {
    s_autotune_state = AUTOTUNE_IDLE;
    s_control_mode = PID_MODE_FOLLOW;
    snprintf(s_autotune_status, sizeof(s_autotune_status), "Autotune Aborted");
}

bool pid_is_autotuning(void) {
    return (s_autotune_state != AUTOTUNE_IDLE && s_autotune_state != AUTOTUNE_SUCCESS && s_autotune_state != AUTOTUNE_FAILED);
}

const char* pid_get_autotune_status(void) {
    return s_autotune_status;
}
