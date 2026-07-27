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

void pid_controller_init(void);
void pid_set_target_angle(int joint_id, float target_deg);
float pid_get_target_angle(int joint_id);
float pid_get_error(int joint_id);
void pid_set_params(float kp, float ki, float kd);
pid_params_t pid_get_params(void);

#endif // PID_CONTROLLER_H
