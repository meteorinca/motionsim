#include "bench_udp.h"
#include "config.h"
#include "led.h"
#include "wifi_mgr.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "BENCH_UDP"
#define SAFETY_TIMEOUT_US (100 * 1000) // 100 ms safety threshold

static bench_stats_t s_stats = {0};
static SemaphoreHandle_t s_stats_mutex = NULL;

static void bench_rx_task(void *pvParameters) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    // Set small socket receive buffer (reduce kernel queuing delay)
    int rcvbuf = 2048;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    int sndbuf = 2048;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    struct sockaddr_in serv_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_BENCH_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind to port %d", UDP_BENCH_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP Bench Server listening on port %d", UDP_BENCH_PORT);

    bench_packet_t pkt;
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int64_t last_rx_t = 0;
    int rate_window_count = 0;
    int64_t rate_window_start = esp_timer_get_time();

    while (1) {
        int len = recvfrom(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&client_addr, &addr_len);
        int64_t now_us = esp_timer_get_time();

        if (len == sizeof(bench_packet_t) && pkt.magic == BENCH_PACKET_MAGIC) {
            // Fill ESP receive timestamp
            pkt.esp_ts_us = (uint32_t)now_us;

            // Immediately echo back to sender
            sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&client_addr, addr_len);

            xSemaphoreTake(s_stats_mutex, portMAX_DELAY);
            
            s_stats.rx_count++;
            s_stats.tx_count++;

            // Measure interval between UDP receives (tests for stalls/blocks)
            if (last_rx_t > 0) {
                uint32_t interval = (uint32_t)(now_us - last_rx_t);
                if (interval > s_stats.max_rx_interval_us) {
                    s_stats.max_rx_interval_us = interval;
                }
            }
            last_rx_t = now_us;
            s_stats.last_rx_us = now_us;

            // Sequence tracking
            if (s_stats.rx_count == 1) {
                s_stats.seq_last = pkt.seq;
            } else {
                if (pkt.seq == s_stats.seq_last + 1) {
                    s_stats.seq_last = pkt.seq;
                } else if (pkt.seq > s_stats.seq_last + 1) {
                    s_stats.lost_count += (pkt.seq - s_stats.seq_last - 1);
                    s_stats.seq_last = pkt.seq;
                } else if (pkt.seq == s_stats.seq_last) {
                    s_stats.dup_count++;
                } else if (pkt.seq < s_stats.seq_last) {
                    s_stats.out_of_order_count++;
                }
            }

            if (s_stats.safety_stopped) {
                s_stats.safety_stopped = false;
                ESP_LOGI(TAG, "Packets resumed! Safety stop cleared.");
            }

            // Packet rate calculation
            rate_window_count++;
            if (now_us - rate_window_start >= 1000000LL) {
                s_stats.pkt_rate_hz = (float)rate_window_count * 1000000.0f / (float)(now_us - rate_window_start);
                rate_window_count = 0;
                rate_window_start = now_us;
                s_stats.rssi = wifi_mgr_get_rssi();
            }

            xSemaphoreGive(s_stats_mutex);

            // Brief toggle for LED
            led_action_set(s_stats.rx_count % 2 == 0);
        }
    }
}

static void bench_watchdog_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20)); // Check every 20 ms
        int64_t now_us = esp_timer_get_time();

        xSemaphoreTake(s_stats_mutex, portMAX_DELAY);
        if (s_stats.rx_count > 0 && !s_stats.safety_stopped) {
            if (now_us - s_stats.last_rx_us > SAFETY_TIMEOUT_US) {
                s_stats.safety_stopped = true;
                s_stats.safety_trips++;
                s_stats.pkt_rate_hz = 0.0f;
                ESP_LOGW(TAG, "SAFETY STOP TRIPPED! No packet for %lld ms", (now_us - s_stats.last_rx_us) / 1000);
            }
        }
        xSemaphoreGive(s_stats_mutex);
    }
}

void bench_udp_init(void) {
    s_stats_mutex = xSemaphoreCreateMutex();
    xTaskCreate(bench_rx_task, "bench_rx", 4096, NULL, 20, NULL);
    xTaskCreate(bench_watchdog_task, "bench_wd", 2048, NULL, 15, NULL);
}

bench_stats_t bench_udp_get_stats(void) {
    bench_stats_t copy;
    if (s_stats_mutex) {
        xSemaphoreTake(s_stats_mutex, portMAX_DELAY);
        copy = s_stats;
        xSemaphoreGive(s_stats_mutex);
    } else {
        memset(&copy, 0, sizeof(copy));
    }
    return copy;
}

void bench_udp_reset_stats(void) {
    if (s_stats_mutex) {
        xSemaphoreTake(s_stats_mutex, portMAX_DELAY);
        s_stats.rx_count = 0;
        s_stats.tx_count = 0;
        s_stats.lost_count = 0;
        s_stats.out_of_order_count = 0;
        s_stats.dup_count = 0;
        s_stats.safety_trips = 0;
        s_stats.max_rx_interval_us = 0;
        s_stats.pkt_rate_hz = 0.0f;
        xSemaphoreGive(s_stats_mutex);
    }
}
