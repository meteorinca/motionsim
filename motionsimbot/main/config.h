#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/soc_caps.h"

#ifdef BOARD_CONFIG_HEADER
  #include BOARD_CONFIG_HEADER
#else
  #error "No BOARD selected. Run: idf.py -DBOARD=esp32c3_motionsimbot set-target esp32c3 build"
#endif

#if LED_ACTIVE_LOW
  #define LED_ON  0
  #define LED_OFF 1
#else
  #define LED_ON  1
  #define LED_OFF 0
#endif

#endif // CONFIG_H
