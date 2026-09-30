#include "pid_controller.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "config.h"
#include "serial_ctrl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TAG "PID"
#define NVS_PID_NAMESPACE "pid_cfg"

static pid_params_t s_params = {
    .kp = 14.0f,               // Fast responsive stiffness
    .ki = 0.20f,               // Steady-state offset elimination
    .kd = 1.8f,                // Velocity damping against oscillations (active D-term)
    .kff = 2.2f,               // Velocity feedforward
    .integral_max = 200.0f,
    .max_speed_deg_s = 80.0f,  // Cruise speed: 80 deg/s (fast & agile)
    .max_accel_deg_s2 = 180.0f,// Accel & Decel ramp: 180 deg/s^2 (stops in 0.44s)
    .min_duty = 90.0f,         // Guaranteed static friction breakaway
    .deadband_deg = 0.25f,     // In-position deadband: 0.25 deg
    .max_duty_limit = 700.0f,  // Max duty safety clamp (default 700 out of 819)
    .soft_min_deg = 0.0f,      // Full travel range 0 to 360
    .soft_max_deg = 360.0f,
    .beta = 0.6f,
    .k_aw = 0.5f
};

static pid_control_mode_t s_control_mode = PID_MODE_FOLLOW;
static float s_targets[JOINT_COUNT] = {180.0f, 180.0f, 180.0f};
static float s_unwrapped_profiled[JOINT_COUNT];
static float s_unwrapped_actuals[JOINT_COUNT];
static float s_unwrapped_targets[JOINT_COUNT];
static float s_mapped_targets[JOINT_COUNT];
static float s_move_starts[JOINT_COUNT];
static bool s_angle_initialized[JOINT_COUNT];
static bool s_target_mapped[JOINT_COUNT];
static float s_breakaway_elapsed[JOINT_COUNT];
static int s_friction_direction[JOINT_COUNT];
static float s_profiled_vels[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_actuals[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_last_actuals[JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_errors[JOINT_COUNT] = {0.0f};
static float s_integrals[JOINT_COUNT] = {0.0f};
static float s_d_filtered[JOINT_COUNT] = {0.0f};
static int16_t s_last_cmd_duty[JOINT_COUNT] = {0, 0, 0};
static int16_t s_manual_duties[JOINT_COUNT] = {0};
static bool s_target_initialized[JOINT_COUNT] = {false, false, false};
typedef struct { float p, d, ff, friction, integral; } control_terms_t;
static control_terms_t s_terms[JOINT_COUNT];
static pid_trace_sample_t s_trace[PID_TRACE_CAPACITY];
static uint32_t s_trace_ids[PID_TRACE_CAPACITY], s_trace_total;
static size_t s_trace_next, s_trace_count;
static bool s_trace_frozen;
static portMUX_TYPE s_trace_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_live_lock = portMUX_INITIALIZER_UNLOCKED;
static pid_live_snapshot_t s_live;

void pid_get_live_snapshot(pid_live_snapshot_t *out) {
    if (!out) return;
    portENTER_CRITICAL(&s_live_lock);
    *out = s_live;
    portEXIT_CRITICAL(&s_live_lock);
}

static void publish_live(int64_t tick_start, uint32_t elapsed_us) {
    static uint32_t sequence, late, max_work;
    uint32_t work = (uint32_t)(esp_timer_get_time() - tick_start);
    if (work > max_work) max_work = work;
    if (elapsed_us > 12000) late++;
    pid_live_snapshot_t sample = {.us = tick_start, .sequence = ++sequence,
        .dt_us = elapsed_us, .work_us = work, .late_ticks = late, .max_work_us = max_work,
        .armed = motor_is_armed(), .estop = motor_get_estop()};
    for (int i = 0; i < JOINT_COUNT; i++) {
        sample.joint[i] = (pid_live_joint_t){s_actuals[i], s_targets[i], s_errors[i],
            motor_get_logical_duty(i + 1), hall_read_raw(i + 1), 0};
    }
    portENTER_CRITICAL(&s_live_lock); s_live = sample; portEXIT_CRITICAL(&s_live_lock);
}

static void record_trace(uint32_t ms, float dt) {
    if (s_trace_frozen) return;
    pid_trace_sample_t sample = {
        .ms = ms, .dt = dt, .sensor_age_ms = 0,
        .raw = hall_read_raw(1), .mode = s_control_mode,
        .target = s_unwrapped_targets[0], .profiled = s_unwrapped_profiled[0],
        .actual = s_unwrapped_actuals[0], .velocity = s_d_filtered[0],
        .p = s_terms[0].p, .integral = s_terms[0].integral, .d = s_terms[0].d,
        .ff = s_terms[0].ff, .friction = s_terms[0].friction,
        .logical_duty = motor_get_logical_duty(1), .physical_duty = motor_get_duty(1)
    };
    portENTER_CRITICAL(&s_trace_lock);
    s_trace[s_trace_next] = sample;
    s_trace_ids[s_trace_next] = s_trace_total++;
    s_trace_next = (s_trace_next + 1) % PID_TRACE_CAPACITY;
    if (s_trace_count < PID_TRACE_CAPACITY) s_trace_count++;
    portEXIT_CRITICAL(&s_trace_lock);
}

size_t pid_copy_trace(pid_trace_sample_t *out, size_t capacity) {
    if (!out) return 0;
    portENTER_CRITICAL(&s_trace_lock);
    size_t count = capacity < s_trace_count ? capacity : s_trace_count;
    size_t first = (s_trace_next + PID_TRACE_CAPACITY - count) % PID_TRACE_CAPACITY;
    uint32_t first_id = s_trace_total - count;
    portEXIT_CRITICAL(&s_trace_lock);
    // Copy one row per short critical section so a download cannot mask ticks.
    for (size_t i = 0; i < count; i++) {
        size_t index = (first + i) % PID_TRACE_CAPACITY;
        portENTER_CRITICAL(&s_trace_lock);
        bool valid = s_trace_ids[index] == first_id + i;
        if (valid) out[i] = s_trace[index];
        portEXIT_CRITICAL(&s_trace_lock);
        if (!valid) return 0; // History was overwritten during a slow download.
    }
    return count;
}

// Automatic E-Stop Supervision
static bool s_auto_estop_enabled = true;
static estop_reason_code_t s_estop_reason_code = ESTOP_REASON_NONE;
static char s_estop_reason_str[96] = "System Normal";
static int s_runaway_counters[JOINT_COUNT] = {0};
static int s_stall_counters[JOINT_COUNT] = {0};
static float s_stall_ref_angles[JOINT_COUNT] = {0.0f};
static int s_grace_period_ticks = 0; // Settling period after reset/clear

// Autotune State
static autotune_state_t s_autotune_state = AUTOTUNE_IDLE;
static int s_autotune_joint = 1;
static uint32_t s_autotune_start_ms = 0;
static float s_autotune_start_angle = 0.0f;
static pid_params_t s_autotune_original;
static const float s_tune_offsets[] = {5, 0, -5, 0, 10, 0, -10, 0, 15, 0, -15, 0};
#define TUNE_LEGS (sizeof(s_tune_offsets) / sizeof(s_tune_offsets[0]))
static unsigned s_tune_leg;
static bool s_tune_validation;
static uint32_t s_tune_settled_ms;
static float s_tune_leg_start;
static float s_tune_goal;
static float s_tune_iae, s_tune_baseline_iae, s_tune_residual, s_tune_overshoot;
static unsigned s_tune_samples, s_tune_baseline_samples, s_tune_residual_samples;
static uint32_t s_params_revision;
static char s_autotune_status[96] = "Idle";

// Dual-Pulse Polarity Auto-detection State
typedef enum {
    AUTODETECT_IDLE = 0,
    AUTODETECT_SAMPLE_A0,
    AUTODETECT_PULSE_FWD,
    AUTODETECT_SETTLE_A1,
    AUTODETECT_PULSE_REV,
    AUTODETECT_SETTLE_A2,
    AUTODETECT_EVAL
} autodetect_state_t;

static autodetect_state_t s_autodetect_state = AUTODETECT_IDLE;
static int s_autodetect_joint = 1;
static uint32_t s_autodetect_start_ms = 0;
static float s_ad_angle_a0 = 0.0f;
static float s_ad_angle_a1 = 0.0f;
static float s_ad_angle_a2 = 0.0f;

static float shortest_angle_diff(float target, float actual) {
    float diff = target - actual;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

static float wrap_angle(float angle) {
    float wrapped = fmodf(angle, 360.0f);
    return wrapped < 0.0f ? wrapped + 360.0f : wrapped;
}

static float clampf(float value, float low, float high) {
    return fminf(high, fmaxf(low, value));
}

static void validate_new_params(void) {
    s_params.beta = isfinite(s_params.beta) ? clampf(s_params.beta, 0.0f, 1.0f) : 0.6f;
    s_params.k_aw = isfinite(s_params.k_aw) ? clampf(s_params.k_aw, 0.0f, 100.0f) : 0.5f;
    float *friction[] = { &s_params.breakaway_fwd, &s_params.breakaway_rev,
                         &s_params.coulomb_fwd, &s_params.coulomb_rev };
    for (int i = 0; i < 4; i++) {
        *friction[i] = isfinite(*friction[i]) ? clampf(*friction[i], 0.0f, MOTOR_CLAMP_DUTY) : 0.0f;
    }
}

static void load_pid_nvs(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_PID_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        uint32_t kp_m = 0, ki_m = 0, kd_m = 0, kff_m = 0, spd_m = 0, acc_m = 0;
        uint32_t stic_m = 0, db_m = 0, maxd_m = 0, smin_m = 0, smax_m = 0;

        if (nvs_get_u32(h, "kp", &kp_m) == ESP_OK) s_params.kp = (float)kp_m / 1000.0f;
        if (nvs_get_u32(h, "ki", &ki_m) == ESP_OK) s_params.ki = (float)ki_m / 1000.0f;
        if (nvs_get_u32(h, "kd", &kd_m) == ESP_OK) s_params.kd = (float)kd_m / 1000.0f;
        if (nvs_get_u32(h, "kff", &kff_m) == ESP_OK) s_params.kff = (float)kff_m / 1000.0f;
        if (nvs_get_u32(h, "max_speed", &spd_m) == ESP_OK) s_params.max_speed_deg_s = (float)spd_m / 1000.0f;
        if (nvs_get_u32(h, "max_accel", &acc_m) == ESP_OK) s_params.max_accel_deg_s2 = (float)acc_m / 1000.0f;
        if (nvs_get_u32(h, "min_duty", &stic_m) == ESP_OK) s_params.min_duty = (float)stic_m;
        if (nvs_get_u32(h, "deadband", &db_m) == ESP_OK) s_params.deadband_deg = (float)db_m / 1000.0f;
        if (nvs_get_u32(h, "max_duty", &maxd_m) == ESP_OK && maxd_m >= 200) s_params.max_duty_limit = (float)maxd_m;
        if (nvs_get_u32(h, "soft_min", &smin_m) == ESP_OK) s_params.soft_min_deg = (float)smin_m / 1000.0f;
        if (nvs_get_u32(h, "soft_max", &smax_m) == ESP_OK && smax_m > 0) s_params.soft_max_deg = (float)smax_m / 1000.0f;
        uint32_t value;
        if (nvs_get_u32(h, "beta", &value) == ESP_OK) s_params.beta = value / 1000.0f;
        if (nvs_get_u32(h, "k_aw", &value) == ESP_OK) s_params.k_aw = value / 1000.0f;
        if (nvs_get_u32(h, "breakaway_fwd", &value) == ESP_OK) s_params.breakaway_fwd = value / 1000.0f;
        if (nvs_get_u32(h, "breakaway_rev", &value) == ESP_OK) s_params.breakaway_rev = value / 1000.0f;
        if (nvs_get_u32(h, "coulomb_fwd", &value) == ESP_OK) s_params.coulomb_fwd = value / 1000.0f;
        if (nvs_get_u32(h, "coulomb_rev", &value) == ESP_OK) s_params.coulomb_rev = value / 1000.0f;
        validate_new_params();

        // Safety clamps on loaded values
        if (s_params.max_duty_limit < 200.0f) s_params.max_duty_limit = 700.0f;
        if (s_params.soft_max_deg <= s_params.soft_min_deg || s_params.soft_max_deg <= 0.0f || s_params.soft_max_deg > 360.0f ||
            (s_params.soft_min_deg >= 29.0f && s_params.soft_max_deg <= 331.0f)) {
            s_params.soft_min_deg = 0.0f;
            s_params.soft_max_deg = 360.0f;
        }
        if (s_params.min_duty < 60.0f) s_params.min_duty = 90.0f;
        if (s_params.kp < 2.0f) s_params.kp = 14.0f;
        if (s_params.kd < 0.2f) s_params.kd = 1.8f;
        if (s_params.max_speed_deg_s < 30.0f) s_params.max_speed_deg_s = 80.0f;
        if (s_params.max_accel_deg_s2 < 50.0f) s_params.max_accel_deg_s2 = 180.0f;

        uint8_t auto_es = 1;
        if (nvs_get_u8(h, "auto_estop", &auto_es) == ESP_OK) s_auto_estop_enabled = (auto_es != 0);
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded PID: Kp=%.1f Ki=%.2f Kd=%.2f Kff=%.1f Spd=%.0f Acc=%.0f MinDuty=%.0f MaxDuty=%.0f",
                 s_params.kp, s_params.ki, s_params.kd, s_params.kff,
                 s_params.max_speed_deg_s, s_params.max_accel_deg_s2, s_params.min_duty, s_params.max_duty_limit);
    }
}

esp_err_t pid_save_params_to_nvs(void) {
    if (pid_is_autotuning()) return ESP_ERR_INVALID_STATE;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_PID_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u32(h, "kp", (uint32_t)(s_params.kp * 1000.0f));
    nvs_set_u32(h, "ki", (uint32_t)(s_params.ki * 1000.0f));
    nvs_set_u32(h, "kd", (uint32_t)(s_params.kd * 1000.0f));
    nvs_set_u32(h, "kff", (uint32_t)(s_params.kff * 1000.0f));
    nvs_set_u32(h, "max_speed", (uint32_t)(s_params.max_speed_deg_s * 1000.0f));
    nvs_set_u32(h, "max_accel", (uint32_t)(s_params.max_accel_deg_s2 * 1000.0f));
    nvs_set_u32(h, "min_duty", (uint32_t)s_params.min_duty);
    nvs_set_u32(h, "deadband", (uint32_t)(s_params.deadband_deg * 1000.0f));
    nvs_set_u32(h, "max_duty", (uint32_t)s_params.max_duty_limit);
    nvs_set_u32(h, "soft_min", (uint32_t)(s_params.soft_min_deg * 1000.0f));
    nvs_set_u32(h, "soft_max", (uint32_t)(s_params.soft_max_deg * 1000.0f));
    nvs_set_u32(h, "beta", (uint32_t)(s_params.beta * 1000.0f));
    nvs_set_u32(h, "k_aw", (uint32_t)(s_params.k_aw * 1000.0f));
    nvs_set_u32(h, "breakaway_fwd", (uint32_t)(s_params.breakaway_fwd * 1000.0f));
    nvs_set_u32(h, "breakaway_rev", (uint32_t)(s_params.breakaway_rev * 1000.0f));
    nvs_set_u32(h, "coulomb_fwd", (uint32_t)(s_params.coulomb_fwd * 1000.0f));
    nvs_set_u32(h, "coulomb_rev", (uint32_t)(s_params.coulomb_rev * 1000.0f));
    nvs_set_u8(h, "auto_estop", s_auto_estop_enabled ? 1 : 0);
    err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved PID params to NVS");
    return err;
}

void pid_trigger_estop(estop_reason_code_t reason, const char *detail) {
    record_trace((uint32_t)(esp_timer_get_time() / 1000ULL), 0.0f);
    s_trace_frozen = true;
    if (pid_is_autotuning()) pid_abort_autotune();
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

    // Re-align all targets and trajectories to current physical angles
    for (int i = 0; i < JOINT_COUNT; i++) {
        s_runaway_counters[i] = 0;
        s_stall_counters[i] = 0;
        s_last_actuals[i] = s_actuals[i];
        s_stall_ref_angles[i] = s_actuals[i];
        s_last_cmd_duty[i] = 0;
        if (s_angle_initialized[i]) {
            s_targets[i] = s_actuals[i];
            s_unwrapped_profiled[i] = s_unwrapped_actuals[i];
            s_unwrapped_targets[i] = s_unwrapped_actuals[i];
            s_target_mapped[i] = false;
            s_profiled_vels[i] = 0.0f;
        }
        s_errors[i] = 0.0f;
        s_d_filtered[i] = 0.0f;
        s_integrals[i] = 0.0f;
        s_breakaway_elapsed[i] = 0.0f;
        s_friction_direction[i] = 0;
    }
    s_grace_period_ticks = 100;
    ESP_LOGI(TAG, "Emergency Stop Cleared by User (grace period active).");
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
        s_breakaway_elapsed[i] = 0.0f;
        s_friction_direction[i] = 0;
        s_unwrapped_profiled[i] = s_unwrapped_actuals[i];
        s_target_mapped[i] = false;
        s_d_filtered[i] = 0.0f;
        s_profiled_vels[i] = 0.0f;
        s_last_cmd_duty[i] = 0;
        s_runaway_counters[i] = 0;
        s_stall_counters[i] = 0;
    }
}

// Multi-point tuning uses measured error, overshoot and residual error.
// Candidate gains are temporary until a second pass verifies their performance.
static void tune_hold_current(void) {
    pid_reset_integrals();
    for (int i = 0; i < JOINT_COUNT; i++) s_targets[i] = s_actuals[i];
}

static void tune_fail(const char *reason) {
    s_trace_frozen = true;
    s_params = s_autotune_original;
    s_params_revision++;
    s_autotune_state = AUTOTUNE_FAILED;
    s_control_mode = PID_MODE_FOLLOW;
    tune_hold_current();
    motor_stop_all();
    snprintf(s_autotune_status, sizeof(s_autotune_status), "Failed: %s; original gains restored", reason);
}

static void tune_begin_leg(uint32_t now_ms) {
    int i = s_autotune_joint - 1;
    s_tune_leg_start = s_unwrapped_actuals[i];
    s_tune_goal = s_autotune_start_angle + s_tune_offsets[s_tune_leg];
    s_targets[i] = wrap_angle(s_tune_goal);
    s_autotune_start_ms = now_ms;
    s_tune_settled_ms = 0;
    s_autotune_state = AUTOTUNE_STEP_FWD;
    snprintf(s_autotune_status, sizeof(s_autotune_status), "%s %u/%u: target %.1f deg",
             s_tune_validation ? "Validating" : "Measuring", s_tune_leg + 1,
             (unsigned)TUNE_LEGS, s_targets[i]);
}

static void process_autotune(uint32_t now_ms) {
    int i = s_autotune_joint - 1;
    if (!motor_is_armed() || motor_get_estop()) {
        tune_fail("motor stopped or E-Stop active");
        return;
    }
    if (s_autotune_state == AUTOTUNE_BASELINE) {
        if (fabsf(s_d_filtered[i]) < 3.0f) {
            if (s_tune_settled_ms == 0) s_tune_settled_ms = now_ms;
            if (now_ms - s_tune_settled_ms >= 1000) tune_begin_leg(now_ms);
        } else {
            s_tune_settled_ms = 0;
        }
        if (s_autotune_state == AUTOTUNE_BASELINE && now_ms - s_autotune_start_ms > 6000)
            tune_fail("baseline did not settle");
        return;
    }
    if (s_autotune_state != AUTOTUNE_STEP_FWD) return;

    float error = s_tune_goal - s_unwrapped_actuals[i];
    float distance = fabsf(error);
    uint32_t elapsed = now_ms - s_autotune_start_ms;
    s_tune_iae += distance;
    s_tune_samples++;
    float direction = s_tune_goal >= s_tune_leg_start ? 1.0f : -1.0f;
    s_tune_overshoot = fmaxf(s_tune_overshoot, -direction * error);
    if (elapsed >= 1500) {
        s_tune_residual += distance;
        s_tune_residual_samples++;
    }
    if (distance <= fmaxf(s_params.deadband_deg, 0.5f) && fabsf(s_d_filtered[i]) < 3.0f) {
        if (s_tune_settled_ms == 0) s_tune_settled_ms = now_ms;
    } else {
        s_tune_settled_ms = 0;
    }
    // Runaway during commissioning fails even if automatic E-stop was disabled.
    if (fabsf(s_unwrapped_actuals[i] - s_autotune_start_angle) > 30.0f) {
        tune_fail("excess travel");
        motor_arm(false);
        return;
    }
    bool settled = s_tune_settled_ms != 0 && now_ms - s_tune_settled_ms >= 500;
    if (elapsed < 2000 || (!settled && elapsed < 6000)) return;
    if (!settled) {
        tune_fail("step did not settle within 6 s");
        return;
    }
    if (++s_tune_leg < TUNE_LEGS) {
        tune_begin_leg(now_ms);
        return;
    }
    if (!s_tune_validation) {
        s_tune_baseline_iae = s_tune_iae;
        s_tune_baseline_samples = s_tune_samples;
        float residual = s_tune_residual / fmaxf(1.0f, (float)s_tune_residual_samples);
        // Bounded heuristic refinement, not a motor model identification.
        if (s_tune_overshoot > 1.0f) {
            s_params.kp *= 0.90f;
            s_params.kd *= 1.15f;
            s_params.ki *= 0.80f;
        } else {
            s_params.kp *= 1.05f;
            if (residual > s_params.deadband_deg)
                s_params.ki = fmaxf(0.05f, s_params.ki * 1.10f);
        }
        s_params.kp = clampf(s_params.kp, 1.0f, 30.0f);
        s_params.kd = clampf(s_params.kd, 0.0f, 10.0f);
        s_params.ki = clampf(s_params.ki, 0.0f, 5.0f);
        s_params_revision++;
        s_tune_validation = true;
        s_tune_leg = 0;
        s_tune_iae = s_tune_residual = s_tune_overshoot = 0.0f;
        s_tune_samples = s_tune_residual_samples = 0;
        pid_reset_integrals();
        tune_begin_leg(now_ms);
        return;
    }
    float baseline = s_tune_baseline_iae / fmaxf(1.0f, (float)s_tune_baseline_samples);
    float candidate = s_tune_iae / fmaxf(1.0f, (float)s_tune_samples);
    if (candidate > baseline || s_tune_overshoot > 2.0f) {
        tune_fail("candidate did not improve response");
        return;
    }
    s_autotune_state = AUTOTUNE_SUCCESS;
    s_control_mode = PID_MODE_FOLLOW;
    s_params_revision++;
    tune_hold_current();
    snprintf(s_autotune_status, sizeof(s_autotune_status),
             "Applied Kp %.2f Ki %.2f Kd %.2f. Use Save to Flash to keep.",
             s_params.kp, s_params.ki, s_params.kd);
}
// -------------------------------------------------------------
// Rock-Solid 2-Phase Dual-Pulse Direction Finder
// -------------------------------------------------------------
static void process_autodetect(uint32_t now_ms) {
    int j_idx = s_autodetect_joint - 1;
    int motor_id = s_autodetect_joint;

    switch (s_autodetect_state) {
        case AUTODETECT_SAMPLE_A0:
            motor_stop_all();
            if (now_ms - s_autodetect_start_ms >= 150) {
                s_ad_angle_a0 = s_actuals[j_idx];
                s_autodetect_state = AUTODETECT_PULSE_FWD;
                s_autodetect_start_ms = now_ms;
                // Direct hardware pulse on forward pin (RPWM) without software inversion
                motor_set_raw_duty(motor_id, 340);
                snprintf(s_autotune_status, sizeof(s_autotune_status), "Phase 1: Testing Hardware Forward Pulse (+340)...");
            }
            break;

        case AUTODETECT_PULSE_FWD:
            if (now_ms - s_autodetect_start_ms >= 320) {
                motor_set_raw_duty(motor_id, 0);
                s_autodetect_state = AUTODETECT_SETTLE_A1;
                s_autodetect_start_ms = now_ms;
                snprintf(s_autotune_status, sizeof(s_autotune_status), "Phase 1: Settling forward movement...");
            }
            break;

        case AUTODETECT_SETTLE_A1:
            motor_set_raw_duty(motor_id, 0);
            if (now_ms - s_autodetect_start_ms >= 250) {
                s_ad_angle_a1 = s_actuals[j_idx];
                s_autodetect_state = AUTODETECT_PULSE_REV;
                s_autodetect_start_ms = now_ms;
                // Direct hardware pulse on reverse pin (LPWM) to test return motion
                motor_set_raw_duty(motor_id, -340);
                snprintf(s_autotune_status, sizeof(s_autotune_status), "Phase 2: Testing Hardware Reverse Pulse (-340)...");
            }
            break;

        case AUTODETECT_PULSE_REV:
            if (now_ms - s_autodetect_start_ms >= 320) {
                motor_set_raw_duty(motor_id, 0);
                s_autodetect_state = AUTODETECT_SETTLE_A2;
                s_autodetect_start_ms = now_ms;
                snprintf(s_autotune_status, sizeof(s_autotune_status), "Phase 2: Settling reverse return...");
            }
            break;

        case AUTODETECT_SETTLE_A2:
            motor_set_raw_duty(motor_id, 0);
            if (now_ms - s_autodetect_start_ms >= 250) {
                s_ad_angle_a2 = s_actuals[j_idx];
                s_autodetect_state = AUTODETECT_EVAL;
            }
            break;

        case AUTODETECT_EVAL: {
            motor_set_raw_duty(motor_id, 0);

            float delta_fwd = shortest_angle_diff(s_ad_angle_a1, s_ad_angle_a0);
            float delta_rev = shortest_angle_diff(s_ad_angle_a2, s_ad_angle_a1);

            ESP_LOGI(TAG, "Autodetect Result Joint %d: dFwd=%.2f deg, dRev=%.2f deg", motor_id, delta_fwd, delta_rev);

            bool fwd_moved = fabsf(delta_fwd) >= 0.40f;
            bool rev_moved = fabsf(delta_rev) >= 0.40f;

            if (!fwd_moved && !rev_moved) {
                snprintf(s_autotune_status, sizeof(s_autotune_status),
                         "Warning: Motor didn't move (dFwd=%.2f°, dRev=%.2f°). Check 24V power/wiring!", delta_fwd, delta_rev);
                ESP_LOGW(TAG, "Polarity check failed on Joint %d: no motion detected", motor_id);
            }
            else if (delta_fwd > 0.35f && delta_rev < -0.35f) {
                // Both forward and reverse tests definitively agree on NORMAL
                motor_set_inverted(motor_id, false);
                snprintf(s_autotune_status, sizeof(s_autotune_status),
                         "VERIFIED NORMAL POLARITY! +PWM increases angle (+%.1f°, %.1f°)", delta_fwd, delta_rev);
                ESP_LOGI(TAG, "Joint %d confirmed NORMAL polarity (dual-phase match)", motor_id);
            }
            else if (delta_fwd < -0.35f && delta_rev > 0.35f) {
                // Both forward and reverse tests definitively agree on INVERTED
                motor_set_inverted(motor_id, true);
                snprintf(s_autotune_status, sizeof(s_autotune_status),
                         "VERIFIED INVERTED POLARITY! Corrected to Inverted (%.1f°, +%.1f°)", delta_fwd, delta_rev);
                ESP_LOGI(TAG, "Joint %d confirmed INVERTED polarity (dual-phase match)", motor_id);
            }
            else if (delta_fwd > 0.60f) {
                motor_set_inverted(motor_id, false);
                snprintf(s_autotune_status, sizeof(s_autotune_status),
                         "VERIFIED NORMAL POLARITY (dFwd=+%.1f°)", delta_fwd);
                ESP_LOGI(TAG, "Joint %d set NORMAL polarity based on forward stroke", motor_id);
            }
            else if (delta_fwd < -0.60f) {
                motor_set_inverted(motor_id, true);
                snprintf(s_autotune_status, sizeof(s_autotune_status),
                         "VERIFIED INVERTED POLARITY (dFwd=%.1f°)", delta_fwd);
                ESP_LOGI(TAG, "Joint %d set INVERTED polarity based on forward stroke", motor_id);
            }
            else {
                snprintf(s_autotune_status, sizeof(s_autotune_status),
                         "Inconclusive backlash motion (dFwd=%.2f°, dRev=%.2f°). Please test again.", delta_fwd, delta_rev);
                ESP_LOGW(TAG, "Polarity check ambiguous on Joint %d", motor_id);
            }

            // Sync trajectory to current physical position so motor rests still
            float curr = s_actuals[j_idx];
            s_targets[j_idx] = curr;
            s_unwrapped_profiled[j_idx] = s_unwrapped_actuals[j_idx];
            s_unwrapped_targets[j_idx] = s_unwrapped_actuals[j_idx];
            s_target_mapped[j_idx] = false;
            s_integrals[j_idx] = 0.0f;
            s_d_filtered[j_idx] = 0.0f;
            s_breakaway_elapsed[j_idx] = 0.0f;
            s_friction_direction[j_idx] = 0;
            s_profiled_vels[j_idx] = 0.0f;
            s_last_cmd_duty[j_idx] = 0;
            s_autodetect_state = AUTODETECT_IDLE;
            s_control_mode = PID_MODE_FOLLOW;
            break;
        }

        default:
            break;
    }
}

static void pid_task(void *pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t delay_ticks = pdMS_TO_TICKS(10);
    int64_t previous_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake, delay_ticks);
        int64_t now_us = esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000ULL);
        uint32_t elapsed_us = (uint32_t)(now_us - previous_us);
        float dt = clampf((float)elapsed_us / 1000000.0f, 0.001f, 0.100f);
        previous_us = now_us;
        serial_ctrl_poll();

        if (s_grace_period_ticks > 0) {
            s_grace_period_ticks--;
        }

        // 1. Read all sensors & execute stable Trapezoidal Motion Profiler
        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;
            // Potentiometers are always present; hall_read_angle returns 0.0f on ADC error.
            float current_angle = hall_read_angle(motor_id);
            if (!isfinite(current_angle)) continue;
            s_actuals[i] = current_angle;

            // Seed measurement independently of commands, including an exact zero reading.
            if (!s_angle_initialized[i]) {
                if (!s_target_initialized[i]) s_targets[i] = current_angle;
                s_unwrapped_actuals[i] = current_angle;
                s_unwrapped_profiled[i] = current_angle;
                s_profiled_vels[i] = 0.0f;
                s_last_actuals[i] = current_angle;
                s_stall_ref_angles[i] = current_angle;
                s_target_initialized[i] = true;
                s_angle_initialized[i] = true;
                ESP_LOGI(TAG, "Joint %d initialized target to current physical angle: %.2f deg", motor_id, current_angle);
            } else {
                s_unwrapped_actuals[i] += shortest_angle_diff(current_angle, wrap_angle(s_unwrapped_actuals[i]));
            }

            // Choose a destination once per changed command. Never reselect its turn while moving.
            if (!s_target_mapped[i] || s_mapped_targets[i] != s_targets[i]) {
                float new_distance = shortest_angle_diff(s_targets[i], current_angle);
                // Do not carry stored torque or a departing profile into a reversal.
                if (s_integrals[i] * new_distance < 0.0f) s_integrals[i] = 0.0f;
                if (s_profiled_vels[i] * new_distance < 0.0f) {
                    s_unwrapped_profiled[i] = s_unwrapped_actuals[i];
                    s_profiled_vels[i] = 0.0f;
                }
                s_move_starts[i] = s_unwrapped_actuals[i];
                s_unwrapped_targets[i] = s_unwrapped_actuals[i] + shortest_angle_diff(s_targets[i], current_angle);
                s_mapped_targets[i] = s_targets[i];
                s_target_mapped[i] = true;
            }

            // Calculate actual velocity and filtered derivative using last tick's position
            float actual_vel = shortest_angle_diff(current_angle, s_last_actuals[i]) / dt;
            const float alpha_d = 0.25f;
            s_d_filtered[i] = (alpha_d * actual_vel) + ((1.0f - alpha_d) * s_d_filtered[i]);

            // --- Analytical Continuous-Deceleration Trapezoidal Motion Profiler ---
            float pos_diff = s_unwrapped_targets[i] - s_unwrapped_profiled[i];
            float current_v = s_profiled_vels[i];
            float max_v = s_params.max_speed_deg_s;
            float max_a = s_params.max_accel_deg_s2;
            if (max_v < 1.0f) max_v = 1.0f;
            if (max_a < 1.0f) max_a = 1.0f;

            // Signed stopping distance: d_brake = (v * |v|) / (2 * a)
            float d_brake = (current_v * fabsf(current_v)) / (2.0f * max_a);
            // Projected stopping error if deceleration begins right now
            float stop_err = pos_diff - d_brake;

            if (fabsf(pos_diff) < 0.10f && fabsf(current_v) < 1.5f) {
                // In close range and low speed: lock to target
                s_unwrapped_profiled[i] = s_unwrapped_targets[i];
                s_profiled_vels[i] = 0.0f;
            } else {
                // Accelerate forward (+max_v) if stop position is behind target, else decelerate (-max_v)
                float target_v = (stop_err > 0.0f) ? max_v : -max_v;

                // Slew velocity by max_a * dt
                float dv = target_v - current_v;
                float max_dv = max_a * dt;
                if (dv > max_dv)  dv = max_dv;
                if (dv < -max_dv) dv = -max_dv;
                current_v += dv;

                if (current_v > max_v)  current_v = max_v;
                if (current_v < -max_v) current_v = -max_v;
                s_profiled_vels[i] = current_v;

                s_unwrapped_profiled[i] += current_v * dt;
            }

            // Closed-Loop Tracking Lead/Lag Clamp:
            // Ensure the profiled setpoint never pulls away from physical motor angle by more than MAX_LEAD_DEG.
            // Bound tracking lead without changing the selected destination or wrapping the error.
            const float MAX_LEAD_DEG = 25.0f;
            float track_err = s_unwrapped_profiled[i] - s_unwrapped_actuals[i];
            if (track_err > MAX_LEAD_DEG) {
                s_unwrapped_profiled[i] = s_unwrapped_actuals[i] + MAX_LEAD_DEG;
                s_profiled_vels[i] = fminf(s_profiled_vels[i], 0.0f);
            } else if (track_err < -MAX_LEAD_DEG) {
                s_unwrapped_profiled[i] = s_unwrapped_actuals[i] - MAX_LEAD_DEG;
                s_profiled_vels[i] = fmaxf(s_profiled_vels[i], 0.0f);
            }

            // Tracking error relative to the smooth profiled trajectory
            s_errors[i] = s_unwrapped_profiled[i] - s_unwrapped_actuals[i];
        }

        // 2. Multi-Hazard Automatic E-Stop Supervision
        if (s_auto_estop_enabled && motor_is_armed() && !motor_get_estop() &&
            s_grace_period_ticks == 0 && s_control_mode != PID_MODE_AUTODETECT) {

            for (int i = 0; i < JOINT_COUNT; i++) {
                int motor_id = i + 1;
                if (motor_is_jogging(motor_id)) continue;

                float current_angle = s_actuals[i];
                float last_angle = s_last_actuals[i];
                float error = s_errors[i];
                int16_t logical_duty = motor_get_logical_duty(motor_id);

                // Hazard A: Velocity Jump (> 50.0 deg in 10ms)
                // Pot ADC values are always synchronous; a jump this large indicates a wiring fault.
                float d_angle = fabsf(shortest_angle_diff(current_angle, last_angle));
                if (d_angle > 50.0f) {
                    char err_msg[96];
                    snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: Sensor Jump (%.1f deg in 10ms on Joint %d)", d_angle, motor_id);
                    pid_trigger_estop(ESTOP_REASON_VELOCITY_JUMP, err_msg);
                    break;
                }

                // Hazard C: True Divergence Watchdog
                // Detects wrong polarity: commanded duty is driving motor FURTHER away from target
                // (e.g. positive error where motor should move positive, but duty is pushed negative, or vice versa)
                // Sustained for 500 ms (50 ticks).
                if (s_control_mode != PID_MODE_MANUAL_DUTY && fabsf(error) > 15.0f && abs(logical_duty) > 250) {
                    float measured_vel = shortest_angle_diff(current_angle, last_angle) / dt;
                    bool driving_away = false;

                    // True divergence: high duty driving in direction that increases error
                    if (error > 15.0f && logical_duty > 250 && measured_vel < -5.0f) {
                        driving_away = true;
                    } else if (error < -15.0f && logical_duty < -250 && measured_vel > 5.0f) {
                        driving_away = true;
                    }

                    if (driving_away) {
                        s_runaway_counters[i]++;
                        if (s_runaway_counters[i] > 50) { // 500 ms continuous divergence
                            char err_msg[96];
                            snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: Polarity Divergence on Joint %d (Run Auto-Detect)", motor_id);
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
                if (s_control_mode != PID_MODE_MANUAL_DUTY && s_target_mapped[i]) {
                    float low = fminf(s_move_starts[i], s_unwrapped_targets[i]) - 30.0f;
                    float high = fmaxf(s_move_starts[i], s_unwrapped_targets[i]) + 30.0f;
                    if (s_unwrapped_actuals[i] < low || s_unwrapped_actuals[i] > high) {
                        pid_trigger_estop(ESTOP_REASON_RUNAWAY, "Move exceeded travel corridor by 30 deg");
                        break;
                    }
                }
                if (s_control_mode != PID_MODE_MANUAL_DUTY && fabsf(error) > 10.0f && abs(logical_duty) > 400) {
                    float motion = fabsf(shortest_angle_diff(current_angle, s_stall_ref_angles[i]));
                    if (motion < 0.35f) {
                        s_stall_counters[i]++;
                        if (s_stall_counters[i] > 200) { // 2000 ms stalled
                            char err_msg[96];
                            snprintf(err_msg, sizeof(err_msg), "Auto E-Stop: Actuator Stall on Joint %d (>40%% duty, no motion)", motor_id);
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

        // 3. Safety Interlock
        bool any_jogging = false;
        for (int i = 1; i <= JOINT_COUNT; i++) {
            if (motor_is_jogging(i)) any_jogging = true;
        }

        if ((!motor_is_armed() || motor_get_estop()) && !any_jogging) {
            if (pid_is_autotuning()) pid_abort_autotune();
            motor_stop_all();
            pid_reset_integrals();
            for (int i = 0; i < JOINT_COUNT; i++) {
                s_last_actuals[i] = s_actuals[i];
            }
            publish_live(now_us, elapsed_us);
            continue;
        }

        // 4. Polarity Auto-detection Sequence
        if (s_control_mode == PID_MODE_AUTODETECT) {
            process_autodetect(now_ms);
            for (int i = 0; i < JOINT_COUNT; i++) {
                s_last_actuals[i] = s_actuals[i];
            }
            publish_live(now_us, elapsed_us);
            continue;
        }

        // 5. Autotuning Sequence
        if (s_autotune_state != AUTOTUNE_IDLE && s_autotune_state != AUTOTUNE_SUCCESS && s_autotune_state != AUTOTUNE_FAILED) {
            process_autotune(now_ms);
        }

        // 6. Manual Duty Mode
        if (s_control_mode == PID_MODE_MANUAL_DUTY) {
            for (int i = 0; i < JOINT_COUNT; i++) {
                if (!motor_is_jogging(i + 1)) {
                    motor_set_duty(i + 1, s_manual_duties[i]);
                }
            }
            for (int i = 0; i < JOINT_COUNT; i++) {
                s_last_actuals[i] = s_actuals[i];
            }
            publish_live(now_us, elapsed_us);
            continue;
        }

        // 7. Closed-Loop PID + Velocity Feedforward + Smooth Stiction
        for (int i = 0; i < JOINT_COUNT; i++) {
            int motor_id = i + 1;

            if (motor_is_jogging(motor_id)) continue;
            // Pot is always present; if angle is uninitialized, coast the motor.
            if (!s_angle_initialized[i]) {
                motor_set_duty(motor_id, 0);
                continue;
            }

            float error = s_errors[i];
            s_terms[i] = (control_terms_t){0};
            float dist_to_target = fabsf(s_unwrapped_targets[i] - s_unwrapped_actuals[i]);
            float traj_dist = fabsf(s_unwrapped_targets[i] - s_unwrapped_profiled[i]);

            // 7.1 In-position deadband check: ONLY when motion profile has arrived at final target and motor is within deadband
            if (traj_dist < 0.15f && dist_to_target <= s_params.deadband_deg &&
                fabsf(s_d_filtered[i]) < 3.0f && fabsf(s_profiled_vels[i]) < 0.5f) {
                motor_set_duty(motor_id, 0);
                s_integrals[i] *= 0.95f;
                s_last_cmd_duty[i] = 0;
                s_breakaway_elapsed[i] = 0.0f;
                s_friction_direction[i] = 0;
                continue;
            }

            // 7.2 Velocity Feedforward (drives motor to track trajectory velocity effortlessly)
            float ff_term = s_params.kff * s_profiled_vels[i];

            // Weight tracking error only. Referencing the final destination here
            // adds Kp*(1-beta)*remaining_distance and bypasses the motion ramp.
            float p_term = s_params.kp * s_params.beta * error;
            // Integral state is stored directly in duty units.
            float i_term = s_integrals[i];

            // 7.5 Filtered Derivative Damping (active counter-torque against velocity)
            float d_term = - (s_params.kd * s_d_filtered[i]);

            // 7.6 Smooth Stiction Breakaway
            float stiction = 0.0f;
            if (s_params.breakaway_fwd > 0.0f) {
                float demand = fabsf(error) > s_params.deadband_deg ? error : s_profiled_vels[i];
                int direction = demand > 0.0f ? 1 : (demand < 0.0f ? -1 : 0);
                if (direction != s_friction_direction[i]) {
                    s_friction_direction[i] = direction;
                    s_breakaway_elapsed[i] = 0.0f;
                }
                float coulomb = direction > 0 ? s_params.coulomb_fwd : s_params.coulomb_rev;
                float breakaway = direction > 0 ? s_params.breakaway_fwd : s_params.breakaway_rev;
                // One pulse of at most 100 ms per start or reversal. A stalled
                // motor cannot keep receiving or retriggering a starting pulse.
                if (fabsf(s_d_filtered[i]) >= 3.0f) s_breakaway_elapsed[i] = 0.1f;
                bool pulse = direction != 0 && s_breakaway_elapsed[i] < 0.1f;
                stiction = direction * (pulse ? fmaxf(coulomb, breakaway) : coulomb);
                s_breakaway_elapsed[i] += dt;
            } else if (fabsf(s_profiled_vels[i]) > 0.4f) {
                float sgn = (s_profiled_vels[i] > 0.0f) ? 1.0f : -1.0f;
                stiction = sgn * s_params.min_duty;
            } else if (dist_to_target > s_params.deadband_deg) {
                // Min Duty must remain available outside the deadband. Scaling
                // it toward zero can strand the shaft just short of the target.
                stiction = error > 0.0f ? s_params.min_duty : (error < 0.0f ? -s_params.min_duty : 0.0f);
            }

            // Sum all control terms
            float raw_output = p_term + i_term + d_term + ff_term + stiction;
            s_terms[i] = (control_terms_t){p_term, d_term, ff_term, stiction, i_term};
            float target_duty = raw_output;

            // Clamp to user safety ceiling
            float clamp_limit = s_params.max_duty_limit;
            if (clamp_limit > MOTOR_CLAMP_DUTY) clamp_limit = MOTOR_CLAMP_DUTY;
            if (clamp_limit < 150.0f) clamp_limit = 150.0f;

            if (target_duty > clamp_limit)  target_duty = clamp_limit;
            if (target_duty < -clamp_limit) target_duty = -clamp_limit;

            // Asymmetric Slew-Rate Limiter:
            // 120 PWM/10ms (12000 PWM/s) for forward acceleration, but 350 PWM/10ms for active braking / direction reversals.
            // This ensures instant counter-torque deceleration on overshoots, eliminating trailing forward power.
            float last_duty_f = (float)s_last_cmd_duty[i];
            bool is_braking_or_reversing = (target_duty * last_duty_f < 0.0f) || (fabsf(target_duty) < fabsf(last_duty_f));
            float max_duty_step = is_braking_or_reversing ? 350.0f : 120.0f;

            float d_duty = target_duty - last_duty_f;
            if (d_duty > max_duty_step)  d_duty = max_duty_step;
            if (d_duty < -max_duty_step) d_duty = -max_duty_step;
            int16_t duty = (int16_t)(last_duty_f + d_duty);
            // Include both output saturation and slew limiting in back-calculation.
            if (s_params.ki > 0.0f) {
                s_integrals[i] += (s_params.ki * error + s_params.k_aw * ((float)duty - raw_output)) * dt;
                s_integrals[i] = clampf(s_integrals[i], -s_params.integral_max, s_params.integral_max);
            } else {
                s_integrals[i] = 0.0f;
            }
            s_last_cmd_duty[i] = duty;

            motor_set_duty(motor_id, duty);
        }

        // Store last actuals for velocity calculations in the next tick
        record_trace(now_ms, dt);
        for (int i = 0; i < JOINT_COUNT; i++) {
            s_last_actuals[i] = s_actuals[i];
        }
        publish_live(now_us, elapsed_us);
    }
}

void pid_controller_init(void) {
    load_pid_nvs();
    xTaskCreate(pid_task, "pid_loop", 4096, NULL, 22, NULL);
    ESP_LOGI(TAG, "Trapezoidal Profiled PID Controller initialized (Kp:%.1f Ki:%.2f Kd:%.2f Kff:%.1f MaxDuty:%.0f)",
             s_params.kp, s_params.ki, s_params.kd, s_params.kff, s_params.max_duty_limit);
}

void pid_set_target_angle(int joint_id, float target_deg) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    if (pid_is_autotuning()) return;
    if (!isfinite(target_deg)) return;

    // Apply soft limits to prevent spinning across physical boundaries
    if (s_params.soft_max_deg > s_params.soft_min_deg && (s_params.soft_max_deg - s_params.soft_min_deg < 359.0f)) {
        if (target_deg < s_params.soft_min_deg) target_deg = s_params.soft_min_deg;
        if (target_deg > s_params.soft_max_deg) target_deg = s_params.soft_max_deg;
    }

    if (target_deg < 0.0f || target_deg > 360.0f) target_deg = wrap_angle(target_deg);
    target_deg = clampf(target_deg, 0.5f, 359.5f);
    s_targets[joint_id - 1] = target_deg;
    if (!motor_get_estop()) s_trace_frozen = false;
    s_target_initialized[joint_id - 1] = true;

    // Auto-arm motors if not in E-Stop so commanded motion executes immediately
    if (!motor_get_estop() && !motor_is_armed()) {
        motor_arm(true);
        ESP_LOGI(TAG, "Auto-armed motors on target angle command");
    }
}

float pid_get_target_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_targets[joint_id - 1];
}

float pid_get_profiled_target(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return wrap_angle(s_unwrapped_profiled[joint_id - 1]);
}

float pid_get_profiled_velocity(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_profiled_vels[joint_id - 1];
}

float pid_get_actual_angle(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_actuals[joint_id - 1];
}

float pid_get_error(int joint_id) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return 0.0f;
    return s_errors[joint_id - 1];
}

void pid_set_params(const pid_params_t *params) {
    if (!params) return;
    if (pid_is_autotuning()) return;
    s_params = *params;
    validate_new_params();
    s_params_revision++;
    ESP_LOGI(TAG, "PID Parameters updated: Kp=%.2f Ki=%.2f Kd=%.2f Kff=%.2f Spd=%.1f Acc=%.1f MaxDuty=%.0f",
             s_params.kp, s_params.ki, s_params.kd, s_params.kff,
             s_params.max_speed_deg_s, s_params.max_accel_deg_s2, s_params.max_duty_limit);
}

pid_params_t pid_get_params(void) {
    return s_params;
}

uint32_t pid_get_params_revision(void) { return s_params_revision; }

void pid_set_mode(pid_control_mode_t mode) {
    // Streaming commands must not erase derivative history or reselect turns.
    if (mode == s_control_mode || pid_is_autotuning()) return;
    s_control_mode = mode;
    pid_reset_integrals();
}

pid_control_mode_t pid_get_mode(void) {
    return s_control_mode;
}

void pid_set_manual_duty(int joint_id, int16_t duty) {
    if (joint_id < 1 || joint_id > JOINT_COUNT) return;
    if (pid_is_autotuning()) return;
    s_manual_duties[joint_id - 1] = duty;
    s_control_mode = PID_MODE_MANUAL_DUTY;
}

esp_err_t pid_autodetect_polarity(int joint_id) {
    if (serial_ctrl_owns_motion()) return ESP_ERR_INVALID_STATE;
    if (pid_is_autotuning()) return ESP_ERR_INVALID_STATE;
    if (joint_id < 1 || joint_id > JOINT_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_angle_initialized[joint_id - 1]) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Error: Joint %d sensor not yet initialized", joint_id);
        return ESP_ERR_NOT_FOUND;
    }
    if (motor_get_estop()) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Error: E-Stop active, clear first");
        return ESP_ERR_INVALID_STATE;
    }

    s_autodetect_joint = joint_id;
    s_trace_frozen = false;
    s_autodetect_start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    motor_arm(true);
    if (!motor_is_armed()) return ESP_ERR_INVALID_STATE;
    s_autodetect_state = AUTODETECT_SAMPLE_A0;
    s_control_mode = PID_MODE_AUTODETECT;

    snprintf(s_autotune_status, sizeof(s_autotune_status), "Starting 2-Phase Direction Verification on Joint %d...", joint_id);
    ESP_LOGI(TAG, "Starting Dual-Pulse Polarity Auto-detection on Joint %d", joint_id);
    return ESP_OK;
}

esp_err_t pid_start_autotune(int joint_id) {
    if (serial_ctrl_owns_motion()) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Disable USB motion control before tuning");
        return ESP_ERR_INVALID_STATE;
    }
    if (pid_is_autotuning()) return ESP_ERR_INVALID_STATE;
    if (joint_id < 1 || joint_id > JOINT_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_angle_initialized[joint_id - 1]) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Error: Joint %d sensor not yet initialized", joint_id);
        return ESP_ERR_NOT_FOUND;
    }
    if (motor_get_estop()) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Error: E-Stop active");
        return ESP_ERR_INVALID_STATE;
    }

    int i = joint_id - 1;
    float angle = s_actuals[i];
    if (!s_angle_initialized[i] || angle - 15.0f < fmaxf(0.5f, s_params.soft_min_deg) ||
        angle + 15.0f > fminf(359.5f, s_params.soft_max_deg)) {
        snprintf(s_autotune_status, sizeof(s_autotune_status), "Move at least 15 deg inside travel limits before tuning");
        return ESP_ERR_INVALID_STATE;
    }
    for (int j = 1; j <= JOINT_COUNT; j++) {
        if (motor_is_jogging(j)) return ESP_ERR_INVALID_STATE;
    }
    s_autotune_joint = joint_id;
    s_trace_frozen = false;
    s_autotune_original = s_params;
    s_autotune_start_angle = s_unwrapped_actuals[i];
    s_tune_leg = 0;
    s_tune_validation = false;
    s_tune_settled_ms = 0;
    s_tune_iae = s_tune_residual = s_tune_overshoot = 0.0f;
    s_tune_samples = s_tune_residual_samples = 0;
    tune_hold_current();
    s_autotune_start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    motor_arm(true);
    if (!motor_is_armed()) return ESP_ERR_INVALID_STATE;
    s_control_mode = PID_MODE_AUTOTUNE;
    // Publish active state only after the driver is armed.
    s_autotune_state = AUTOTUNE_BASELINE;
    snprintf(s_autotune_status, sizeof(s_autotune_status), "Settling baseline; 12 measurement + 12 validation moves");
    ESP_LOGI(TAG, "Starting multi-point PID refinement on Joint %d", joint_id);
    return ESP_OK;
}

void pid_abort_autotune(void) {
    if (s_control_mode == PID_MODE_AUTOTUNE) {
        s_params = s_autotune_original;
        s_params_revision++;
    }
    s_autotune_state = AUTOTUNE_IDLE;
    s_autodetect_state = AUTODETECT_IDLE;
    s_control_mode = PID_MODE_FOLLOW;
    tune_hold_current();
    motor_stop_all();
    snprintf(s_autotune_status, sizeof(s_autotune_status), "Autotune Aborted");
}

bool pid_is_autotuning(void) {
    return (s_autotune_state != AUTOTUNE_IDLE && s_autotune_state != AUTOTUNE_SUCCESS && s_autotune_state != AUTOTUNE_FAILED) ||
           (s_autodetect_state != AUTODETECT_IDLE);
}

const char* pid_get_autotune_status(void) {
    return s_autotune_status;
}
