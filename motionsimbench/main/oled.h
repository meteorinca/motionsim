#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OLED_MODE_BENCH_STATS = 0,    // Live RTT, Pkt Rate, Loss %, RSSI
    OLED_MODE_BENCH_GRAPH = 1,    // Live sparkline latency / jitter chart
    OLED_MODE_DR2_TELEMETRY = 2,  // Live Dirt Rally 2.0 telemetry (Sway/Surge/Heave)
    OLED_MODE_WIFI_INFO = 3,      // SSID, IP, MAC, Device Number
    OLED_MODE_EYES = 4,           // Robot animated eyes
    OLED_MODE_MARIO_DANCE = 5,    // Mario animation
    OLED_MODE_MATRIX_RAIN = 6,    // Matrix rain
    OLED_MODE_FIREWORKS = 7,      // Fireworks
    OLED_MODE_MENU = 8,           // Menu
    OLED_MODE_SAFETY_STOP = 9,    // Safety stop alert screen
    OLED_MODE_COUNT
} oled_mode_t;

typedef enum {
    EYE_EMOTION_NORMAL = 0,
    EYE_EMOTION_MAD,
    EYE_EMOTION_SAD,
    EYE_EMOTION_SLEEPY,
    EYE_EMOTION_SURPRISED,
    EYE_EMOTION_COUNT
} eye_emotion_t;

void oled_init(void);
void oled_set_text(const char* msg, int duration_ms);
void oled_set_mode(oled_mode_t mode);
oled_mode_t oled_get_mode(void);
void oled_cycle_mode(void);
void oled_set_emotion(eye_emotion_t emotion);
eye_emotion_t oled_get_emotion(void);
void oled_update_rtt_sample(float rtt_ms);

#ifdef __cplusplus
}
#endif
