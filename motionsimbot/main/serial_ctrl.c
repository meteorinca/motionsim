#include "serial_ctrl.h"
#include "usb_protocol.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static serial_ctrl_stats_t s_stats = {.telemetry_hz = 100};
static usb_frame_t s_latest;
static bool s_pending, s_have_sequence, s_stop_pending;
static uint32_t s_sequence;
static int64_t s_last_rx_us;

serial_ctrl_stats_t serial_ctrl_get_stats(void) {
    portENTER_CRITICAL(&s_lock);
    serial_ctrl_stats_t copy = s_stats;
    portEXIT_CRITICAL(&s_lock);
    return copy;
}
bool serial_ctrl_owns_motion(void) { return serial_ctrl_get_stats().control_enabled; }

esp_err_t serial_ctrl_configure(bool control, bool telemetry, unsigned hz) {
    if (hz < 1 || hz > 100) return ESP_ERR_INVALID_ARG;
    if (!serial_ctrl_get_stats().ready) return ESP_ERR_INVALID_STATE;
    if (pid_is_autotuning() && control) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_lock);
    if (control || control != s_stats.control_enabled || s_stats.timed_out) {
        s_stop_pending = true;
        s_pending = false;
        s_have_sequence = false;
        s_last_rx_us = 0;
        s_stats.timed_out = false;
    }
    s_stats.control_enabled = control;
    s_stats.telemetry_enabled = telemetry;
    s_stats.telemetry_hz = hz;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

static void receive_frame(const usb_frame_t *frame) {
    if (frame->magic == USB_CONFIG_MAGIC) {
        bool config_valid = frame->value[0] >= 0 && frame->value[0] <= 3 && frame->value[2] == 0;
        if (config_valid && serial_ctrl_configure(frame->value[0] & 1, frame->value[0] & 2, frame->value[1]) == ESP_OK) return;
        portENTER_CRITICAL(&s_lock); s_stats.rejected++; portEXIT_CRITICAL(&s_lock);
        return;
    }
    bool valid = true;
    for (int i = 0; i < 3; i++) if (frame->value[i] < 0 || frame->value[i] > 3600) valid = false;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if (!valid || !s_stats.control_enabled || s_stats.timed_out ||
        (s_have_sequence && !usb_sequence_newer(frame->sequence, s_sequence))) {
        s_stats.rejected++;
    } else {
        if (s_pending) s_stats.coalesced++;
        s_latest = *frame;
        s_pending = s_have_sequence = true;
        s_sequence = frame->sequence;
        s_last_rx_us = now;
        s_stats.received++;
    }
    portEXIT_CRITICAL(&s_lock);
}

void serial_ctrl_poll(void) {
    usb_frame_t command;
    int64_t now = esp_timer_get_time();
    bool estop = motor_get_estop();
    portENTER_CRITICAL(&s_lock);
    bool stop = s_stop_pending;
    s_stop_pending = false;
    if (s_stats.control_enabled && !s_stats.timed_out &&
        (estop || (s_have_sequence && now - s_last_rx_us > 100000))) {
        s_stats.timed_out = true;
        s_pending = false;
        stop = true;
    }
    bool apply = s_pending && s_stats.control_enabled && !s_stats.timed_out;
    if (apply) command = s_latest;
    s_pending = false;
    portEXIT_CRITICAL(&s_lock);
    if (stop) { motor_arm(false); pid_reset_integrals(); }
    if (apply && !pid_is_autotuning() && !motor_get_estop()) {
        pid_set_mode(PID_MODE_FOLLOW);
        for (int i = 0; i < 3; i++) pid_set_target_angle(i + 1, command.value[i] / 10.0f);
    }
}

static void rx_task(void *arg) {
    usb_parser_t parser = {0};
    uint8_t bytes[256];
    while (1) {
        // Only this task waits. PID never reads the transport.
        int count = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), pdMS_TO_TICKS(20));
        uint32_t errors = 0;
        for (int i = 0; i < count; i++) {
            usb_frame_t frame;
            if (usb_parser_push(&parser, bytes[i], &frame, &errors)) receive_frame(&frame);
        }
        portENTER_CRITICAL(&s_lock); s_stats.rejected += errors; portEXIT_CRITICAL(&s_lock);
        if (count == sizeof(bytes)) vTaskDelay(1);
    }
}

static void tx_task(void *arg) {
    char line[768];
    while (1) {
        serial_ctrl_stats_t stats = serial_ctrl_get_stats();
        if (!stats.telemetry_enabled) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        pid_live_snapshot_t s;
        pid_get_live_snapshot(&s);
        int n = snprintf(line, sizeof(line),
            "{\"type\":\"telemetry\",\"us\":%lld,\"seq\":%lu,\"dt_us\":%lu,\"work_us\":%lu,\"late\":%lu,\"estop\":%s,\"armed\":%s,\"usb_timeout\":%s,\"dropped\":%lu,\"joints\":["
            "{\"j\":1,\"act\":%.3f,\"tgt\":%.3f,\"err\":%.3f,\"duty\":%d,\"raw\":%u,\"age_ms\":%lu},"
            "{\"j\":2,\"act\":%.3f,\"tgt\":%.3f,\"err\":%.3f,\"duty\":%d,\"raw\":%u,\"age_ms\":%lu},"
            "{\"j\":3,\"act\":%.3f,\"tgt\":%.3f,\"err\":%.3f,\"duty\":%d,\"raw\":%u,\"age_ms\":%lu}]}\n",
            (long long)s.us, (unsigned long)s.sequence, (unsigned long)s.dt_us,
            (unsigned long)s.work_us, (unsigned long)s.late_ticks,
            s.estop ? "true" : "false", s.armed ? "true" : "false", stats.timed_out ? "true" : "false", (unsigned long)stats.telemetry_dropped,
            s.joint[0].actual, s.joint[0].target, s.joint[0].error, s.joint[0].duty, s.joint[0].raw, (unsigned long)s.joint[0].age_ms,
            s.joint[1].actual, s.joint[1].target, s.joint[1].error, s.joint[1].duty, s.joint[1].raw, (unsigned long)s.joint[1].age_ms,
            s.joint[2].actual, s.joint[2].target, s.joint[2].error, s.joint[2].duty, s.joint[2].raw, (unsigned long)s.joint[2].age_ms);
        // This IDF driver queues the entire write or returns zero.
        bool sent = n > 0 && n < sizeof(line) && usb_serial_jtag_write_bytes(line, n, 0) == n;
        portENTER_CRITICAL(&s_lock);
        if (sent) s_stats.telemetry_sent++; else s_stats.telemetry_dropped++;
        portEXIT_CRITICAL(&s_lock);
        vTaskDelay(pdMS_TO_TICKS(1000 / stats.telemetry_hz));
    }
}

esp_err_t serial_ctrl_init(void) {
    usb_serial_jtag_driver_config_t config = {.tx_buffer_size = 2048, .rx_buffer_size = 2048};
    esp_err_t err = usb_serial_jtag_driver_install(&config);
    if (err != ESP_OK) return err;
    TaskHandle_t rx = NULL;
    if (xTaskCreate(rx_task, "usb_rx", 4096, NULL, 12, &rx) != pdPASS) { usb_serial_jtag_driver_uninstall(); return ESP_ERR_NO_MEM; }
    if (xTaskCreate(tx_task, "usb_tx", 4096, NULL, 5, NULL) != pdPASS) { vTaskDelete(rx); usb_serial_jtag_driver_uninstall(); return ESP_ERR_NO_MEM; }
    portENTER_CRITICAL(&s_lock); s_stats.ready = true; portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}
