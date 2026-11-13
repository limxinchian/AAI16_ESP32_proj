/* Model store: chunked upload into the inactive flash bank, SHA-256 and contract verification, activation and rollback. */
#include "model_store.h"

#include <string.h>

#define CHUNK_MAX 1024u
#define R(x) aai16_ModelReason_MODEL_REASON_##x

static uint8_t other_bank(const model_store_t *ms) { return ms->has_active ? (uint8_t)(1u - ms->active.bank) : 0u; }

static aai16_ModelReason parse_bank(model_store_t *ms, uint8_t bank, uint32_t len, ml_package_t *out)
{
    if (len == 0 || len > ms->ops.bank_size) {
        return R(BAD_PACKAGE);
    }
    const uint8_t *p = ms->ops.map(ms->ops.ctx, bank, len);
    return p ? ml_package_parse(p, len, out) : R(STORAGE_ERROR);
}

void model_store_init(model_store_t *ms, const model_store_ops_t *ops)
{
    memset(ms, 0, sizeof(*ms));
    ms->ops = *ops;
    model_meta_t m;
    if (ms->ops.load_meta(ms->ops.ctx, &m) && m.magic == MODEL_META_MAGIC && m.bank <= 1) {
        static ml_package_t pkg; /* contract copy only needed transiently */
        if (parse_bank(ms, (uint8_t)m.bank, m.length, &pkg) == R(NONE) &&
            memcmp(pkg.contract.tflite_sha256.bytes, m.sha256, 32) == 0) {
            ms->active = m;
            ms->has_active = true;
        }
    }
}

static void ack_fill(const model_store_t *ms, const aai16_ModelChunk *c, aai16_ChunkAck *a, aai16_ChunkStatus st)
{
    memset(a, 0, sizeof(*a));
    a->transfer_id = c->transfer_id;
    a->generation = c->generation;
    a->seq = c->seq;
    a->status = st;
    a->next_expected = (ms->state == XFER_IDLE) ? 0 : ms->next_seq;
}

static void drop(model_store_t *ms)
{
    ms->state = XFER_IDLE;
    ms->next_seq = ms->received = 0;
}

static bool same_transfer(const model_store_t *ms, const aai16_ModelChunk *c)
{
    return c->transfer_id == ms->transfer_id && c->generation == ms->generation && c->kind == ms->kind &&
           c->model_version == ms->version && c->total_chunks == ms->total_chunks &&
           c->total_size == ms->total_size && c->sha256.size == 32 && memcmp(c->sha256.bytes, ms->digest, 32) == 0;
}

void model_store_on_chunk(model_store_t *ms, const aai16_ModelChunk *c, aai16_ChunkAck *ack)
{
    /* basic field validation applies to every chunk */
    const bool fields_ok =
        (c->kind == aai16_ModelKind_MODEL_MOTION || c->kind == aai16_ModelKind_MODEL_ENVIRONMENT ||
         c->kind == aai16_ModelKind_MODEL_ANOMALY) &&
        c->format == aai16_ModelFormat_FORMAT_TFLITE && c->sha256.size == 32 && c->total_chunks >= 1 &&
        c->total_size >= 1 && c->total_size <= ms->ops.bank_size && c->seq < c->total_chunks &&
        c->data.size >= 1 && c->data.size <= CHUNK_MAX &&
        (uint64_t)c->total_chunks * CHUNK_MAX >= c->total_size && c->total_chunks <= c->total_size;
    if (!fields_ok) {
        ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_BAD_LENGTH);
        return;
    }

    const bool known = ms->state != XFER_IDLE && same_transfer(ms, c) && ms->next_seq > 0;
    if (c->seq == 0 && !known) { /* a retransmitted chunk 0 of the current/staged transfer is a duplicate */
        /* (re)start: a new upload replaces any partial or staged one - never the active bank */
        ms->stage_bank = other_bank(ms);
        if (!ms->ops.erase(ms->ops.ctx, ms->stage_bank, c->total_size)) {
            drop(ms);
            ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_STORAGE_ERROR);
            return;
        }
        ms->state = XFER_RECEIVING;
        ms->transfer_id = c->transfer_id;
        ms->generation = c->generation;
        ms->kind = c->kind;
        ms->version = c->model_version;
        ms->total_chunks = c->total_chunks;
        ms->total_size = c->total_size;
        memcpy(ms->digest, c->sha256.bytes, 32);
        ms->next_seq = ms->received = 0;
        sha256_init(&ms->hash);
    }

    if (ms->state == XFER_IDLE || !same_transfer(ms, c)) {
        ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_BAD_TRANSFER);
        return;
    }
    if (ms->state == XFER_STAGED || c->seq < ms->next_seq) {
        ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_DUPLICATE); /* already have it; nothing is rewritten */
        ack->next_expected = ms->next_seq;
        return;
    }
    if (c->seq > ms->next_seq) {
        ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_OUT_OF_ORDER);
        return;
    }
    if (ms->received + c->data.size > ms->total_size ||
        (c->seq + 1 < c->total_chunks && ms->received + c->data.size >= ms->total_size)) {
        drop(ms);
        ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_BAD_LENGTH);
        return;
    }
    if (!ms->ops.write(ms->ops.ctx, ms->stage_bank, ms->received, c->data.bytes, c->data.size)) {
        drop(ms);
        ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_STORAGE_ERROR);
        return;
    }
    sha256_update(&ms->hash, c->data.bytes, c->data.size);
    ms->received += c->data.size;
    ms->next_seq++;

    if (ms->next_seq == ms->total_chunks) {
        uint8_t got[32];
        sha256_ctx_t h = ms->hash;
        sha256_final(&h, got);
        if (ms->received != ms->total_size) {
            drop(ms);
            ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_BAD_LENGTH);
            return;
        }
        if (memcmp(got, ms->digest, 32) != 0) {
            drop(ms);
            ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_DIGEST_MISMATCH);
            return;
        }
        static ml_package_t pkg;
        const aai16_ModelReason r = parse_bank(ms, ms->stage_bank, ms->received, &pkg);
        if (r != R(NONE) || pkg.contract.kind != ms->kind || pkg.contract.model_version != ms->version) {
            drop(ms);
            ack_fill(ms, c, ack, r == R(DIGEST_MISMATCH) ? aai16_ChunkStatus_CHUNK_DIGEST_MISMATCH
                                                         : aai16_ChunkStatus_CHUNK_BAD_TRANSFER);
            return;
        }
        ms->state = XFER_STAGED;
    }
    ack_fill(ms, c, ack, aai16_ChunkStatus_CHUNK_OK);
}

void model_store_abort(model_store_t *ms, uint32_t transfer_id)
{
    if (ms->state != XFER_IDLE && ms->transfer_id == transfer_id) {
        drop(ms);
    }
}

aai16_ModelReason model_store_staged(model_store_t *ms, uint32_t transfer_id, ml_package_t *out)
{
    if (ms->state != XFER_STAGED) {
        return R(NO_STAGED_MODEL);
    }
    if (ms->transfer_id != transfer_id) {
        return R(TRANSFER_MISMATCH);
    }
    return parse_bank(ms, ms->stage_bank, ms->received, out);
}

aai16_ModelReason model_store_activate(model_store_t *ms, uint32_t transfer_id, model_check_fn check, void *ctx)
{
    static ml_package_t pkg;
    aai16_ModelReason r = model_store_staged(ms, transfer_id, &pkg);
    if (r != R(NONE)) {
        return r;
    }
    if (check) {
        r = check(&pkg, ctx);
        if (r != R(NONE)) {
            return r; /* previous model untouched; the staged copy stays available for a retry */
        }
    }
    model_meta_t m = {.magic = MODEL_META_MAGIC, .bank = ms->stage_bank, .length = ms->received,
                      .kind = (uint32_t)pkg.contract.kind, .version = pkg.contract.model_version};
    memcpy(m.sha256, pkg.contract.tflite_sha256.bytes, 32);
    if (!ms->ops.save_meta(ms->ops.ctx, &m)) {
        return R(STORAGE_ERROR);
    }
    ms->active = m;
    ms->has_active = true;
    drop(ms);
    return R(NONE);
}

aai16_ModelReason model_store_active(model_store_t *ms, ml_package_t *out)
{
    if (!ms->has_active) {
        return R(NO_ACTIVE_MODEL);
    }
    return parse_bank(ms, (uint8_t)ms->active.bank, ms->active.length, out);
}
