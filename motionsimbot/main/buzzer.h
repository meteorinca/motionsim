#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void buzzer_init(void);
void buzzer_play_tone(uint32_t freq_hz, uint32_t duration_ms);
void buzzer_demo_wifi_connected(void);

#endif
