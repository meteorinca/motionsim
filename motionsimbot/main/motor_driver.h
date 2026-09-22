#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_COUNT 3
#define MOTOR_MAX_DUTY 1023
#define MOTOR_CLAMP_DUTY 819  // 80% duty max limit for driver protection

void motor_driver_init(void);
void motor_set_duty(int motor_id, int16_t duty);
void motor_stop_all(void);
void motor_set_estop(bool estop);
bool motor_get_estop(void);
int16_t motor_get_duty(int motor_id);
void motor_arm(bool arm);
bool motor_is_armed(void);

// Direction inversion per motor (persisted in NVS)
void motor_set_inverted(int motor_id, bool inverted);
bool motor_is_inverted(int motor_id);

// Safe timed jog execution (stops automatically after duration_ms)
void motor_jog(int motor_id, int16_t duty, uint32_t duration_ms);
void motor_jog_stop(int motor_id);
bool motor_is_jogging(int motor_id);

#endif // MOTOR_DRIVER_H
