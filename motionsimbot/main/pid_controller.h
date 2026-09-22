#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;
} pid_params_t;

typedef enum {
    PID_MODE_FOLLOW = 0,     // Normal mode (follows UDP/target)
    PID_MODE_STEP_TEST = 1,  // Closed-loop step angle test
    PID_MODE_MANUAL_DUTY = 2 // Direct manual duty step test
} pid_control_mode_t;

void pid_controller_init(void);
void pid_set_target_angle(int joint_id, float target_deg);
float pid_get_target_angle(int joint_id);
float pid_get_error(int joint_id);
void pid_set_params(float kp, float ki, float kd);
pid_params_t pid_get_params(void);

void pid_set_mode(pid_control_mode_t mode);
pid_control_mode_t pid_get_mode(void);
void pid_set_manual_duty(int joint_id, int16_t duty);
void pid_reset_integrals(void);

#endif // PID_CONTROLLER_H
