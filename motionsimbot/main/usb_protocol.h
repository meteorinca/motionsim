#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define USB_MOTION_MAGIC 0x4D53494Du
#define USB_CONFIG_MAGIC 0x4643534Du
#define USB_FRAME_SIZE 16
typedef struct { uint8_t bytes[USB_FRAME_SIZE]; size_t used; } usb_parser_t;
typedef struct { uint32_t magic, sequence; int16_t value[3]; } usb_frame_t;
static inline uint16_t usb_u16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t usb_u32(const uint8_t *p) { return usb_u16(p) | ((uint32_t)usb_u16(p + 2) << 16); }
// Recover framing after noise, corrupt packets, partial reads or dropped bytes.
static inline bool usb_parser_push(usb_parser_t *p, uint8_t byte, usb_frame_t *frame, uint32_t *errors) {
    p->bytes[p->used++] = byte;
    while (p->used >= 4) {
        uint32_t magic = usb_u32(p->bytes);
        if (magic == USB_MOTION_MAGIC || magic == USB_CONFIG_MAGIC) {
            if (p->used < USB_FRAME_SIZE) return false;
            uint16_t sum = 0;
            for (size_t i = 0; i < 14; i++) sum += p->bytes[i];
            if (sum == usb_u16(p->bytes + 14)) {
                frame->magic = magic;
                frame->sequence = usb_u32(p->bytes + 4);
                for (int i = 0; i < 3; i++) frame->value[i] = (int16_t)usb_u16(p->bytes + 8 + i * 2);
                p->used = 0;
                return true;
            }
            (*errors)++;
        }
        for (size_t i = 1; i < p->used; i++) p->bytes[i - 1] = p->bytes[i];
        p->used--;
    }
    return false;
}
static inline bool usb_sequence_newer(uint32_t value, uint32_t previous) {
    uint32_t delta = value - previous;
    return delta != 0 && delta < 0x80000000u;
}
