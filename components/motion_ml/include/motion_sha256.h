#pragma once
// SHA-256 for model/package integrity.
//
// ESP-IDF 5.x provides mbedtls/sha256.h. ESP-IDF 6.x (mbedTLS 4) removed that
// header in favour of the PSA crypto API, so IDF >= 6 uses PSA. Host tests do not
// define ESP_IDF_VERSION_MAJOR and keep using the mbedtls header, as before.
// Both back ends compute the same standard SHA-256 and allocate no heap here.
#include <cstddef>
#include <cstdint>
#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 6
#define AAI16_SHA256_PSA 1
#include "psa/crypto.h"
#else
#include "mbedtls/sha256.h"
#endif

namespace aai16 {

class Sha256 {
public:
    Sha256(const Sha256 &) = delete;
    Sha256 &operator=(const Sha256 &) = delete;

#ifdef AAI16_SHA256_PSA
    Sha256() = default;
    ~Sha256() { psa_hash_abort(&op_); }
    // PSA requires psa_crypto_init() first; it is safe to call repeatedly.
    bool start() { return psa_crypto_init() == PSA_SUCCESS && psa_hash_setup(&op_, PSA_ALG_SHA_256) == PSA_SUCCESS; }
    bool update(const void *data, size_t length) {
        return psa_hash_update(&op_, static_cast<const uint8_t *>(data), length) == PSA_SUCCESS;
    }
    bool finish(uint8_t out[32]) {
        size_t produced = 0;
        return psa_hash_finish(&op_, out, 32, &produced) == PSA_SUCCESS && produced == 32;
    }
    static bool compute(const void *data, size_t length, uint8_t out[32]) {
        size_t produced = 0;
        return psa_crypto_init() == PSA_SUCCESS &&
               psa_hash_compute(PSA_ALG_SHA_256, static_cast<const uint8_t *>(data), length, out, 32, &produced) == PSA_SUCCESS &&
               produced == 32;
    }

private:
    psa_hash_operation_t op_ = PSA_HASH_OPERATION_INIT;
#else
    Sha256() { mbedtls_sha256_init(&ctx_); }
    ~Sha256() { mbedtls_sha256_free(&ctx_); }
    bool start() { return mbedtls_sha256_starts(&ctx_, 0) == 0; }
    bool update(const void *data, size_t length) {
        return mbedtls_sha256_update(&ctx_, static_cast<const unsigned char *>(data), length) == 0;
    }
    bool finish(uint8_t out[32]) { return mbedtls_sha256_finish(&ctx_, out) == 0; }
    static bool compute(const void *data, size_t length, uint8_t out[32]) {
        return mbedtls_sha256(static_cast<const unsigned char *>(data), length, out, 0) == 0;
    }

private:
    mbedtls_sha256_context ctx_;
#endif
};

} // namespace aai16
