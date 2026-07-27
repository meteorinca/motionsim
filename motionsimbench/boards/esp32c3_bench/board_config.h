// boards/esp32c3_bench/board_config.h
// ============================================================================
//  Board: ESP32-C3 MotionSim Benchmarking Device
//  Chip:  ESP32-C3 (SuperMini / NodeMCU style)
//  Pins:  OLED SDA=7, SCL=6 | Built-in LED=8 | BTN1=0, BTN2=1, BOOT=9 | Buzzer=3
// ============================================================================
#pragma once

// ── Firmware identity ───────────────────────────────────────────────────────
#define FW_VERSION          "v1.0-bench"

#define _XSTR(x) #x
#define _STR(x) _XSTR(x)

#ifndef DEVICE_NUMBER
#define DEVICE_NUMBER 1
#endif

#define MDNS_HOSTNAME       "motionsimbench" _STR(DEVICE_NUMBER)
#define MDNS_INSTANCE       "MotionSim Bench v" _STR(DEVICE_NUMBER)

#define WEB_SERVER_PORT     80
#define DISABLE_OTA         0

// UDP Benchmarking Port (20778 to avoid conflict with DR2.0 on 20777)
#define UDP_BENCH_PORT      20778

// ── Network ─────────────────────────────────────────────────────────────────
#include "secrets.h"           // provides WIFI_SSID, WIFI_PASS

// ── Time / NTP ──────────────────────────────────────────────────────────────
#define NTP_SERVER          "pool.ntp.org"
#define TIMEZONE            "EST5EDT,M3.2.0,M11.1.0"   // US Eastern

// ── LED (Built-in GPIO LED) ─────────────────────────────────────────────────
#define LED_GPIO                GPIO_NUM_8
#define LED_ACTIVE_LOW          1       // HIGH = OFF

// ── Buzzer (Passive) ──────────────────────────────────────────────────────────
#define BUZZER_PIN              GPIO_NUM_3

// ── Status LEDs & Buttons ────────────────────────────────────────────────────
#define LED_GRN_PIN         GPIO_NUM_20
#define LED_RED_PIN         GPIO_NUM_21
#define BTN_1_GPIO          GPIO_NUM_0
#define BTN_2_GPIO          GPIO_NUM_1
#define BTN_BOOT_GPIO       GPIO_NUM_9    // boot button

// ── OLED Display (I2C) ───────────────────────────────────────────────────────
#define OLED_SDA_PIN        7
#define OLED_SCL_PIN        6
#define OLED_ADDR           0x3C

// ── Scheduler ─────────────────────────────────────────────────────────────────
#define MAX_SCHEDULED_ACTIONS  8
