#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// Single-turn 10k potentiometer angle sensing via ESP32 ADC.
// ADC is 12-bit (0–4095) and maps 0–3.3V to 0–360°.

#define JOINT_COUNT 3

// ADC full scale (12-bit)
#define POT_ADC_MAX         4095
// Degrees per ADC count (360° / 4096 counts)
#define POT_DEG_PER_COUNT   (360.0f / 4096.0f)

void     hall_sensor_init(void);
uint16_t hall_read_raw(int joint_id);
float    hall_read_angle(int joint_id);
void     hall_calibrate_zero(int joint_id);
float    hall_get_zero_offset(int joint_id);
bool     hall_is_connected(int joint_id);
bool     hall_check_connection(int joint_id);

#endif // HALL_SENSOR_H
