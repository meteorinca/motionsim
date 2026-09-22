#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_bus_init(void);
i2c_master_bus_handle_t i2c_bus_get_handle(void);
esp_err_t i2c_bus_probe(uint16_t address);

#ifdef __cplusplus
}
#endif
