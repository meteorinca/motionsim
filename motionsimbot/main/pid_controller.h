#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define JOINT_COUNT 3

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;
} pid_params_t;

typedef enum {
    PID_MODE_FOLLOW = 0,     // Normal mode (follows UDP/target)
    PID_MODE_STEP_TEST = 1,  // Closed-loop step angle test
    PID_MODE_MANUAL_DUTY = 2,// Direct manual duty step test
    PID_MODE_AUTOTUNE = 3    // Automated tuning routine in progress
} pid_control_mode_t;

typedef enum {
    ESTOP_REASON_NONE = 0,
    ESTOP_REASON_HARDWARE,
    ESTOP_REASON_SOFTWARE,
    ESTOP_REASON_RUNAWAY,
    ESTOP_REASON_STALL,
    ESTOP_REASON_VELOCITY_JUMP,
    ESTOP_REASON_MAGNET_FAULT,
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
float pid_get_actual_angle(int joint_id);
float pid_get_error(int joint_id);

// Parameter tuning and storage
void pid_set_params(float kp, float ki, float kd);
pid_params_t pid_get_params(void);
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

// PD Autocalibration
esp_err_t pid_start_autotune(int joint_id);
void pid_abort_autotune(void);
bool pid_is_autotuning(void);
const char* pid_get_autotune_status(void);

#endif // PID_CONTROLLER_H
