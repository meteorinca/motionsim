// boards/esp32s3_motionsimbot/board_config.h
// ESP32-S3 MotionSimBot controller pin configuration.
#pragma once

#define FW_VERSION          "v0.5"

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

// Analog potentiometer inputs (10kΩ single-turn, 0–3.3V → 0–360°).
// ADC1 is used; GPIO maps: ch0=GPIO1, ch1=GPIO2, ch2=GPIO3.
#define POT_ADC_UNIT        ADC_UNIT_1
#define POT_JOINT1_ADC_CH   ADC_CHANNEL_0  // GPIO 1
#define POT_JOINT2_ADC_CH   ADC_CHANNEL_1  // GPIO 2
#define POT_JOINT3_ADC_CH   ADC_CHANNEL_2  // GPIO 3

#define POT_JOINT1_GPIO     GPIO_NUM_1
#define POT_JOINT2_GPIO     GPIO_NUM_2
#define POT_JOINT3_GPIO     GPIO_NUM_3

// SSD1306 I2C display (uses its own dedicated I2C bus init inside oled.c).
#define OLED_SDA_PIN        8
#define OLED_SCL_PIN        9
#define OLED_ADDR           0x3C

#define MAX_SCHEDULED_ACTIONS  8
