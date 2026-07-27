// boards/esp32c3_motionsimbot/board_config.h
// ============================================================================
//  Board: ESP32-C3 MotionSimBot Controller
//  Chip:  ESP32-C3
//  Motors: 3× IBT-2 H-Bridges (24V 250W)
//  Sensors: 3× 12-Bit Hall Angle Sensors (0.088° resolution)
// ============================================================================
#pragma once

#define FW_VERSION          "v1.0-bot"

#define _XSTR(x) #x
#define _STR(x) _XSTR(x)

#ifndef DEVICE_NUMBER
#define DEVICE_NUMBER 1
#endif

#define MDNS_HOSTNAME       "motionsimbot" _STR(DEVICE_NUMBER)
#define MDNS_INSTANCE       "MotionSim Bot v" _STR(DEVICE_NUMBER)

#define WEB_SERVER_PORT     80
#define UDP_MOTION_PORT     20777

#include "secrets.h"

#define NTP_SERVER          "pool.ntp.org"
#define TIMEZONE            "EST5EDT,M3.2.0,M11.1.0"

// ── Status LED ──────────────────────────────────────────────────────────────
#define LED_GPIO            GPIO_NUM_8
#define LED_ACTIVE_LOW      1

// ── Emergency Stop Input Button ──────────────────────────────────────────────
#define ESTOP_BTN_GPIO      GPIO_NUM_9

// ── IBT-2 Motor Driver Pin Assignments ───────────────────────────────────────
#define MOTOR1_RPWM_GPIO    GPIO_NUM_2
#define MOTOR1_LPWM_GPIO    GPIO_NUM_3

#define MOTOR2_RPWM_GPIO    GPIO_NUM_5
#define MOTOR2_LPWM_GPIO    GPIO_NUM_10

#define MOTOR3_RPWM_GPIO    GPIO_NUM_20
#define MOTOR3_LPWM_GPIO    GPIO_NUM_21

// Hardware Enable Line (tied to all 3 IBT-2 R_EN + L_EN)
#define MOTOR_ENABLE_GPIO   GPIO_NUM_4

// ── 12-Bit Hall Angle Sensor ADC Channels (0-3.3V) ───────────────────────────
#define HALL_ADC_UNIT       ADC_UNIT_1
#define HALL_JOINT1_ADC_CH  ADC_CHANNEL_0  // GPIO 0
#define HALL_JOINT2_ADC_CH  ADC_CHANNEL_1  // GPIO 1

// ── OLED Display (I2C) ───────────────────────────────────────────────────────
#define OLED_SDA_PIN        7
#define OLED_SCL_PIN        6
#define OLED_ADDR           0x3C

#define MAX_SCHEDULED_ACTIONS  8
