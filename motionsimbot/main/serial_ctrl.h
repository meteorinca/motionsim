#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct {
    bool ready, control_enabled, telemetry_enabled, timed_out;
    uint16_t telemetry_hz;
    uint32_t received, rejected, coalesced, telemetry_sent, telemetry_dropped;
} serial_ctrl_stats_t;
esp_err_t serial_ctrl_init(void);
esp_err_t serial_ctrl_configure(bool control, bool telemetry, unsigned hz);
bool serial_ctrl_owns_motion(void);
serial_ctrl_stats_t serial_ctrl_get_stats(void);
// PID-task mailbox consumer. No USB I/O occurs here.
void serial_ctrl_poll(void);
