#pragma once
/* Model transfer + staging + activation. Two flash banks: the ACTIVE model is
 * never written; an upload always goes to the other bank, is verified (size, SHA-256, package
 * contract) and only then becomes STAGED. Activation flips the persisted active-bank pointer after the
 * caller's runtime check passes. A reboot discards partial/staged data and restores the last committed
 * model. Pure C; flash and metadata are reached through model_store_ops_t.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "ml_package.h"
#include "sha256.h"

typedef struct {
    uint32_t magic;
    uint32_t bank;        /* 0 or 1 */
    uint32_t length;      /* package bytes in that bank */
    uint32_t kind;
    uint32_t version;
    uint8_t sha256[32];
} model_meta_t;           /* persisted (NVS) */

typedef struct {
    /* erase (at least) the first `size` bytes of the bank, rounded up to the flash sector size by the backend */
    bool (*erase)(void *ctx, uint8_t bank, size_t size);
    bool (*write)(void *ctx, uint8_t bank, uint32_t offset, const uint8_t *data, size_t len);
    /* contiguous read-only view of the first `len` bytes of a bank (esp_partition_mmap on target) */
    const uint8_t *(*map)(void *ctx, uint8_t bank, size_t len);
    /* meta: return false if none is stored */
    bool (*load_meta)(void *ctx, model_meta_t *out);
    bool (*save_meta)(void *ctx, const model_meta_t *in);
    void *ctx;
    uint32_t bank_size;
} model_store_ops_t;

typedef enum { XFER_IDLE = 0, XFER_RECEIVING, XFER_STAGED } xfer_state_t;

typedef struct {
    model_store_ops_t ops;
    model_meta_t active;
    bool has_active;

    xfer_state_t state;
    uint8_t stage_bank;
    uint32_t transfer_id, generation, version, total_chunks, total_size;
    aai16_ModelKind kind;
    uint8_t digest[32];
    uint32_t next_seq, received;
    sha256_ctx_t hash;
} model_store_t;

#define MODEL_META_MAGIC 0x314C444Du /* "MDL1" */

/* Load the persisted meta and verify the active model still parses (digest, contract); an
 * unreadable/corrupt active model is dropped (has_active = false) rather than trusted. */
void model_store_init(model_store_t *ms, const model_store_ops_t *ops);

/* Handle one chunk; always fills *ack. Never touches the active bank. */
void model_store_on_chunk(model_store_t *ms, const aai16_ModelChunk *c, aai16_ChunkAck *ack);

/* Abort any in-progress or staged transfer (ModelControl ABORT_TRANSFER). */
void model_store_abort(model_store_t *ms, uint32_t transfer_id);

/* Parse the staged package (for the runtime check); NO_STAGED_MODEL if none. */
aai16_ModelReason model_store_staged(model_store_t *ms, uint32_t transfer_id, ml_package_t *out);

/* Activate the staged model. `check` runs the target-specific validation (TFLM load, op/shape check,
 * input-compatibility) and returns NONE to accept. Only on success is the active pointer persisted
 * and switched; on any failure the previous model stays active. */
typedef aai16_ModelReason (*model_check_fn)(const ml_package_t *pkg, void *ctx);
aai16_ModelReason model_store_activate(model_store_t *ms, uint32_t transfer_id, model_check_fn check, void *ctx);

/* The currently active package (after init / activate). */
aai16_ModelReason model_store_active(model_store_t *ms, ml_package_t *out);
