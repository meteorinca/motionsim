#include "motion_udp.h"
#include "config.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "led.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "MOTION_UDP"
#define SAFETY_TIMEOUT_US (100 * 1000) // 100 ms threshold

static motion_stats_t s_stats = {0};
static SemaphoreHandle_t s_mutex = NULL;

static void motion_rx_task(void *pvParameters) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    int rcvbuf = 2048;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in serv_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_MOTION_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d", UDP_MOTION_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Motion UDP Receiver bound on port %d", UDP_MOTION_PORT);

    motion_cmd_t cmd;
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int rate_count = 0;
    int64_t rate_start = esp_timer_get_time();

    while (1) {
        int len = recvfrom(sock, &cmd, sizeof(cmd), 0, (struct sockaddr *)&client_addr, &addr_len);
        int64_t now_us = esp_timer_get_time();

        if (len == sizeof(motion_cmd_t) && cmd.magic == MOTION_MAGIC) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_stats.rx_count++;
            s_stats.last_rx_us = now_us;

            if (s_stats.rx_count > 1) {
                if (cmd.seq > s_stats.seq_last + 1) {
                    s_stats.lost_count += (cmd.seq - s_stats.seq_last - 1);
                }
            }
            s_stats.seq_last = cmd.seq;

            if (s_stats.safety_stopped) {
                s_stats.safety_stopped = false;
                ESP_LOGI(TAG, "Motion command stream resumed!");
            }

            rate_count++;
            if (now_us - rate_start >= 1000000LL) {
                s_stats.pkt_rate_hz = (float)rate_count * 1000000.0f / (float)(now_us - rate_start);
                rate_count = 0;
                rate_start = now_us;
            }
            xSemaphoreGive(s_mutex);

            // Update PID target angles (deg * 10 converted to float degrees)
            pid_set_target_angle(1, (float)cmd.target_deg1 / 10.0f);
            pid_set_target_angle(2, (float)cmd.target_deg2 / 10.0f);
            pid_set_target_angle(3, (float)cmd.target_deg3 / 10.0f);

            led_action_set(s_stats.rx_count % 2 == 0);
        }
    }
}

static void motion_watchdog_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
        int64_t now_us = esp_timer_get_time();

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (s_stats.rx_count > 0 && !s_stats.safety_stopped) {
            if (now_us - s_stats.last_rx_us > SAFETY_TIMEOUT_US) {
                s_stats.safety_stopped = true;
                s_stats.pkt_rate_hz = 0.0f;
                ESP_LOGW(TAG, "SAFETY WATCHDOG TRIPPED! Ramping motors to neutral position.");
                // Return motors to center 180° neutral position
                pid_set_target_angle(1, 180.0f);
                pid_set_target_angle(2, 180.0f);
                pid_set_target_angle(3, 180.0f);
            }
        }
        xSemaphoreGive(s_mutex);
    }
}

void motion_udp_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    xTaskCreate(motion_rx_task, "motion_rx", 4096, NULL, 20, NULL);
    xTaskCreate(motion_watchdog_task, "motion_wd", 2048, NULL, 15, NULL);
}

motion_stats_t motion_udp_get_stats(void) {
    motion_stats_t copy;
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        copy = s_stats;
        xSemaphoreGive(s_mutex);
    } else {
        memset(&copy, 0, sizeof(copy));
    }
    return copy;
}
