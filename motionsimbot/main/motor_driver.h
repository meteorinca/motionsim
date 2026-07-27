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

#endif // MOTOR_DRIVER_H
