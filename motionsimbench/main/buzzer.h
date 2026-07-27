#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void buzzer_init(void);
void buzzer_play_tone(uint32_t freq_hz, uint32_t duration_ms);
void buzzer_demo_coin(void);
void buzzer_demo_gameover(void);
void buzzer_demo_siren(void);
void buzzer_demo_laser(void);
void buzzer_demo_startup(void);
void buzzer_demo_wifi_connected(void);
void buzzer_demo_xfiles(void);
void buzzer_demo_mario(void);
void buzzer_demo_1up(void);

#endif // BUZZER_H
