#include "ld2450_sensor.h"
#include <string.h>

// Hi-Link LD2450 serial protocol V1.03, section 2.3 (2023-10-17).
// AA FF 03 00 + three 8-byte targets + 55 CC; fixed length, no checksum.
static const uint8_t header[] = {0xaa, 0xff, 0x03, 0x00};

static uint16_t little_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t signed_value(const uint8_t *p)
{
    uint16_t raw = little_u16(p);
    int magnitude = raw & 0x7fff;
    // This is sign/magnitude with bit 15 set for POSITIVE, not two's complement.
    return (int16_t)((raw & 0x8000) ? magnitude : -magnitude);
}

static void decode(const uint8_t *frame, ld2450_report_t *report)
{
    *report = (ld2450_report_t){0};
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        const uint8_t *p = frame + 4 + i * 8;
        ld2450_target_t *target = &report->targets[i];
        for (unsigned j = 0; j < 8; ++j) target->present |= p[j] != 0;
        target->x_mm = signed_value(p);
        target->y_mm = signed_value(p + 2);
        target->speed_cm_s = signed_value(p + 4);
        target->resolution_mm = little_u16(p + 6);
        report->count += target->present;
    }
}

void ld2450_parser_discard(ld2450_parser_t *parser)
{
    parser->discarded_bytes += parser->used;
    parser->used = 0;
}

void ld2450_parser_expire(ld2450_parser_t *parser, int64_t now_us)
{
    // A report is ~1.2 ms on the wire, normally every 100 ms. Allow scheduler
    // jitter/fragmentation, but never join a long-abandoned prefix to fresh data.
    if (parser->used && now_us - parser->last_byte_us >= 300000) {
        parser->partial_timeouts++;
        ld2450_parser_discard(parser);
    }
}

unsigned ld2450_parser_feed(ld2450_parser_t *parser, const uint8_t *data,
    size_t length, int64_t now_us, ld2450_report_t *latest)
{
    ld2450_parser_expire(parser, now_us);
    unsigned frames = 0;
    for (size_t i = 0; i < length; ++i) {
        parser->last_byte_us = now_us;
        parser->bytes[parser->used++] = data[i];
        // Sliding resynchronization preserves overlapping headers and headers
        // embedded in a rejected candidate. Storage never exceeds 30 bytes.
        while (parser->used) {
            size_t prefix = parser->used < sizeof(header) ? parser->used : sizeof(header);
            if (!memcmp(parser->bytes, header, prefix)) {
                if (parser->used < LD2450_FRAME_BYTES) break;
                if (parser->bytes[28] == 0x55 && parser->bytes[29] == 0xcc) {
                    decode(parser->bytes, latest);
                    parser->frames++;
                    frames++;
                    parser->used = 0;
                    break;
                }
                parser->framing_errors++;
            }
            parser->discarded_bytes++;
            parser->used--;
            memmove(parser->bytes, parser->bytes + 1, parser->used);
        }
    }
    return frames;
}
