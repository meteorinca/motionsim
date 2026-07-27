#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define JOINT_COUNT 3
#define HALL_RESOLUTION_DEG (360.0f / 4095.0f) // 0.08789° approx 0.088°

void hall_sensor_init(void);
uint16_t hall_read_raw(int joint_id);
float hall_read_angle(int joint_id);
void hall_calibrate_zero(int joint_id);
float hall_get_zero_offset(int joint_id);

#endif // HALL_SENSOR_H
