#pragma once
/* Small streaming SHA-256 (FIPS 180-4). Own implementation so the host tests and the firmware run
 * identical code (no mbedtls/PSA API differences between IDF versions); verified against Python's
 * hashlib in tests/ml_host. */
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t h[8];
    uint8_t buf[64];
    uint64_t total;
    size_t fill;
} sha256_ctx_t;

void sha256_init(sha256_ctx_t *c);
void sha256_update(sha256_ctx_t *c, const void *data, size_t len);
void sha256_final(sha256_ctx_t *c, uint8_t out[32]);
