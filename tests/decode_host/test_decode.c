/* Runs profile/raw-data cases produced by tests/decode_host/run.py and prints one result line
 * per case on stdout. All judging happens in Python against independent reference maths.
 * Input file: u32 count, then per case: u32 len + SensorProfile, u32 len + init bytes,
 * u32 len + cycle bytes (all little-endian lengths). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config_manager.h"
#include "pb_decode.h"
#include "sensor_decode.h"

static aai16_SensorProfile g_profile;
static uint8_t g_prof[16384], g_init[256], g_cycle[256];

static uint32_t rd32(FILE *f)
{
    uint8_t b[4];
    if (fread(b, 1, 4, f) != 4) {
        exit(2);
    }
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

static size_t rdblob(FILE *f, uint8_t *dst, size_t cap)
{
    uint32_t n = rd32(f);
    if (n > cap || fread(dst, 1, n, f) != n) {
        exit(2);
    }
    return n;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        return 2;
    }
    const uint32_t cases = rd32(f);
    for (uint32_t i = 0; i < cases; i++) {
        const size_t plen = rdblob(f, g_prof, sizeof(g_prof));
        const size_t ilen = rdblob(f, g_init, sizeof(g_init));
        const size_t clen = rdblob(f, g_cycle, sizeof(g_cycle));

        memset(&g_profile, 0, sizeof(g_profile));
        pb_istream_t is = pb_istream_from_buffer(g_prof, plen);
        if (!pb_decode(&is, aai16_SensorProfile_fields, &g_profile)) {
            puts("DECODEFAIL");
            continue;
        }
        cm_error_t e;
        if (!cm_validate_profile(&g_profile, &e)) {
            printf("INVALID %d %u\n", (int)e.reason, (unsigned)e.detail);
            continue;
        }
        decode_state_t st;
        if (decode_init(&g_profile, g_init, ilen, &st) != DECODE_OK) {
            puts("ERR INIT");
            continue;
        }
        float out[CM_OUTPUT_MAX] = {0};
        const decode_status_t s = decode_cycle(&g_profile, &st, g_cycle, clen, out);
        if (s == DECODE_BAD_FRAME) {
            puts("ERR BAD_FRAME");
        } else if (s == DECODE_CALC_ERROR) {
            puts("ERR CALC");
        } else {
            printf("OK %u", (unsigned)g_profile.output_count);
            for (uint32_t k = 0; k < g_profile.output_count; k++) {
                printf(" %.9g", (double)out[k]);
            }
            putchar('\n');
        }
    }
    fclose(f);
    return 0;
}
