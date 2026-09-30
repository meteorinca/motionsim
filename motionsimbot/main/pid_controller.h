#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define JOINT_COUNT 3
#define PID_TRACE_CAPACITY 512
typedef struct {
    float actual, target, error;
    int16_t duty;
    uint16_t raw;
    uint32_t age_ms;
} pid_live_joint_t;
typedef struct {
    int64_t us;
    uint32_t sequence, dt_us, work_us, late_ticks, max_work_us;
    bool armed, estop;
    pid_live_joint_t joint[JOINT_COUNT];
} pid_live_snapshot_t;
void pid_get_live_snapshot(pid_live_snapshot_t *out);

typedef struct {
    uint32_t ms, sensor_age_ms;
    float dt, target, profiled, actual, velocity;
    float p, integral, d, ff, friction;
    int16_t logical_duty, physical_duty;
    uint16_t raw;
    uint8_t mode;
} pid_trace_sample_t;

size_t pid_copy_trace(pid_trace_sample_t *out, size_t capacity);

typedef struct {
    float kp;               // Position Proportional stiffness
    float ki;               // Integral gain
    float kd;               // Velocity Damping gain
    float kff;              // Velocity Feedforward gain
    float integral_max;     // Anti-windup clamp
    float max_speed_deg_s;  // Trajectory generator maximum cruise speed (deg/s)
    float max_accel_deg_s2; // Trajectory generator acceleration & deceleration (deg/s^2)
    float min_duty;         // Stiction compensation feedforward duty
    float deadband_deg;     // In-position deadband (deg)
    float max_duty_limit;   // Maximum duty safety clamp (prevents runaway to full speed)
    float soft_min_deg;     // Soft limit minimum angle (deg)
    float soft_max_deg;     // Soft limit maximum angle (deg)
    float beta;             // Proportional tracking-error scale, 0 to 1
    float k_aw;             // Integral back-calculation gain (1/s)
    float breakaway_fwd;    // Forward starting pulse duty; zero selects legacy friction
    float breakaway_rev;    // Reverse starting pulse duty
    float coulomb_fwd;      // Forward moving friction duty
    float coulomb_rev;      // Reverse moving friction duty
} pid_params_t;

typedef enum {
    PID_MODE_FOLLOW = 0,     // Normal mode (follows UDP/target)
    PID_MODE_STEP_TEST = 1,  // Closed-loop step angle test
    PID_MODE_MANUAL_DUTY = 2,// Direct manual duty step test
    PID_MODE_AUTOTUNE = 3,   // Automated tuning routine in progress
    PID_MODE_AUTODETECT = 4  // Polarity auto-detection pulse
} pid_control_mode_t;

typedef enum {
    ESTOP_REASON_NONE = 0,
    ESTOP_REASON_HARDWARE,
    ESTOP_REASON_SOFTWARE,
    ESTOP_REASON_RUNAWAY,
    ESTOP_REASON_STALL,
    ESTOP_REASON_VELOCITY_JUMP,
    ESTOP_REASON_LIMIT_EXCEEDED
} estop_reason_code_t;

typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_BASELINE,
    AUTOTUNE_STEP_FWD,
    AUTOTUNE_STEP_REV,
    AUTOTUNE_COMPUTING,
    AUTOTUNE_SUCCESS,
    AUTOTUNE_FAILED
} autotune_state_t;

void pid_controller_init(void);

// Target and Angle APIs
void pid_set_target_angle(int joint_id, float target_deg);
float pid_get_target_angle(int joint_id);
float pid_get_profiled_target(int joint_id);
float pid_get_profiled_velocity(int joint_id);
float pid_get_actual_angle(int joint_id);
float pid_get_error(int joint_id);

// Parameter tuning and storage
void pid_set_params(const pid_params_t *params);
pid_params_t pid_get_params(void);
uint32_t pid_get_params_revision(void);
esp_err_t pid_save_params_to_nvs(void);

// Control mode
void pid_set_mode(pid_control_mode_t mode);
pid_control_mode_t pid_get_mode(void);
void pid_set_manual_duty(int joint_id, int16_t duty);
void pid_reset_integrals(void);

// Multi-hazard E-Stop supervisor
void pid_trigger_estop(estop_reason_code_t reason, const char *detail);
void pid_clear_estop(void);
estop_reason_code_t pid_get_estop_reason_code(void);
const char* pid_get_estop_reason_str(void);
void pid_enable_auto_estop(bool enable);
bool pid_is_auto_estop_enabled(void);

// Commissioning & Calibration
esp_err_t pid_autodetect_polarity(int joint_id);
esp_err_t pid_start_autotune(int joint_id);
void pid_abort_autotune(void);
bool pid_is_autotuning(void);
const char* pid_get_autotune_status(void);

#endif // PID_CONTROLLER_H
