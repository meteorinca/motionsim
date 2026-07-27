#ifndef MOTION_UDP_H
#define MOTION_UDP_H

#include <stdint.h>
#include <stdbool.h>

#define MOTION_MAGIC 0x4D53494D  // "MSIM"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    int16_t  target_deg1;  // deg * 10 (e.g. 1800 = 180.0 deg)
    int16_t  target_deg2;
    int16_t  target_deg3;
    uint16_t checksum;
} motion_cmd_t;

typedef struct {
    uint32_t rx_count;
    uint32_t lost_count;
    uint32_t seq_last;
    float    pkt_rate_hz;
    bool     safety_stopped;
    int64_t  last_rx_us;
} motion_stats_t;

void motion_udp_init(void);
motion_stats_t motion_udp_get_stats(void);

#endif // MOTION_UDP_H
