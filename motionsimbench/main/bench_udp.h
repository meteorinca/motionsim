#ifndef BENCH_UDP_H
#define BENCH_UDP_H

#include <stdint.h>
#include <stdbool.h>

#define BENCH_PACKET_MAGIC 0x4D53  // "MS"

typedef struct __attribute__((packed)) {
    uint16_t magic;         // 0x4D53
    uint16_t flags;         // reserved/test mode flags
    uint32_t seq;           // Sequence number (PC increments)
    uint32_t pc_ts_us;      // PC send timestamp (us)
    uint32_t esp_ts_us;     // ESP32 fill-in: esp_timer_get_time() (us)
} bench_packet_t;

typedef struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t seq_last;
    uint32_t lost_count;
    uint32_t out_of_order_count;
    uint32_t dup_count;
    uint32_t safety_trips;
    uint32_t max_rx_interval_us;  // Tracks max execution delay between UDP frames (proves OLED thread doesn't block UDP!)
    int64_t  last_rx_us;
    bool     safety_stopped;
    float    pkt_rate_hz;
    int      rssi;
} bench_stats_t;

void bench_udp_init(void);
bench_stats_t bench_udp_get_stats(void);
void bench_udp_reset_stats(void);

#endif // BENCH_UDP_H
