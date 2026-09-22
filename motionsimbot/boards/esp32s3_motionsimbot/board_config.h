// boards/esp32s3_motionsimbot/board_config.h
// ESP32-S3 MotionSimBot controller pin configuration.
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

// Status LED: GPIO 48 WS2812 / NeoPixel on common ESP32-S3 DevKit boards.
#define LED_GPIO            GPIO_NUM_48
#define LED_ACTIVE_LOW      0
#define BOARD_HAS_NEOPIXEL  1

// Physical emergency-stop button: active low with internal pull-up.
#define ESTOP_BTN_GPIO      GPIO_NUM_18

// IBT-2 motor driver PWM and shared enable lines.
#define MOTOR1_RPWM_GPIO    GPIO_NUM_4
#define MOTOR1_LPWM_GPIO    GPIO_NUM_5
#define MOTOR2_RPWM_GPIO    GPIO_NUM_6
#define MOTOR2_LPWM_GPIO    GPIO_NUM_7
#define MOTOR3_RPWM_GPIO    GPIO_NUM_15
#define MOTOR3_LPWM_GPIO    GPIO_NUM_16
#define MOTOR_ENABLE_GPIO   GPIO_NUM_17

// Hall sensors: GPIO 1/2/3 = ADC1 channels 0/1/2 on ESP32-S3.
#define HALL_ADC_UNIT       ADC_UNIT_1
#define HALL_JOINT1_ADC_CH  ADC_CHANNEL_0
#define HALL_JOINT2_ADC_CH  ADC_CHANNEL_1
#define HALL_JOINT3_ADC_CH  ADC_CHANNEL_2

// I2C bus pins (shared by AS5600 encoder and display).
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9
#define AS5600_ADDR         0x36
#define TCA9548A_ADDR       0x70

// SSD1306 I2C display.
#define OLED_SDA_PIN        I2C_SDA_PIN
#define OLED_SCL_PIN        I2C_SCL_PIN
#define OLED_ADDR           0x3C

#define MAX_SCHEDULED_ACTIONS  8
