#ifndef OLED_H
#define OLED_H

#include <stdbool.h>

typedef enum {
    OLED_MODE_DIAGNOSTICS = 0,
    OLED_MODE_WIFI = 1,
    OLED_MODE_SAFETY = 2,
    OLED_MODE_COUNT
} oled_mode_t;

void oled_init(void);
void oled_set_mode(oled_mode_t mode);
void oled_cycle_mode(void);
oled_mode_t oled_get_mode(void);

#endif // OLED_H
