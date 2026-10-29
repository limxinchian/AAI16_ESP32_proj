#pragma once
#include "aai16_protocol.h"
#include "motion_candidate.h"
#include "motion_model_control.h"

namespace upload {
constexpr uint32_t kSector = 4096, kTimeoutMs = 60000;
// Partition-relative addresses only. Implementations must bound every access.
// map() remains valid through unmap(); validation never erases a mapped bank.
struct Flash {
    virtual ~Flash() = default;
    virtual uint32_t capacity(unsigned family) = 0;
    virtual bool read(unsigned family, uint32_t offset, void *, size_t) = 0;
    virtual bool erase(unsigned family, uint32_t offset, size_t) = 0;
    virtual bool write(unsigned family, uint32_t offset, const void *, size_t) = 0;
    virtual const uint8_t *map(unsigned family, uint32_t offset, size_t) = 0;
    virtual void unmap() = 0;
};
struct Stored {
    bool valid = false;
    unsigned bank = 0;
    uint64_t serial = 0;
    aai16_UploadManifest manifest = aai16_UploadManifest_init_zero;
};
struct ModelControl {
    virtual ~ModelControl() = default;
    virtual motion_model_state_t snapshot() = 0;
    virtual motion_model_reply_t change(const motion_model_command_t &) = 0;
};
class Store {
public:
    Store(Flash &flash, uint32_t boot, uint8_t *arena, size_t arena_size, ModelControl *runtime=nullptr);
    void recover(); // worker only, including boot-time full package/golden verification
    void tick(uint64_t now_ms);
    // Worker only; caller admits requests through aai16_protocol_decode first.
    void handle(const aai16_Envelope &, uint64_t now_ms, aai16_Envelope &reply);
    void invalid(const aai16_Envelope &request, aai16_ResultCode code, aai16_Envelope &reply);
    uint32_t generation() const { return generation_; }
    const Stored &stored(unsigned family, unsigned bank) const { return slots_[family-1][bank]; }
    const Stored *completed_candidate() const {
        return session_ && state_ == aai16_UploadState_UPLOAD_STORED ? &slots_[manifest_.family-1][bank_] : nullptr;
    }
private:
    Flash &flash_;
    ModelControl *runtime_;
    struct Receipt { uint64_t request=0; uint8_t digest[32]{}; aai16_Envelope reply=aai16_Envelope_init_zero; } receipts_[8];
    unsigned receipt_next_=0;
    uint64_t highest_change_request_=0;
    uint32_t boot_, generation_ = 0;
    uint8_t *arena_;
    size_t arena_size_;
    Stored slots_[2][2];
    uint64_t serial_ = 0, last_ms_ = 0, begin_request_ = 0;
    uint32_t begin_generation_ = 0, offset_ = 0;
    unsigned bank_ = 0;
    uint8_t transfer_[16] = {};
    bool session_ = false;
    aai16_UploadState state_ = aai16_UploadState_UPLOAD_UNSPECIFIED;
    aai16_ResultCode terminal_result_ = aai16_ResultCode_RESULT_OK;
    char terminal_detail_[64] = {};
    aai16_UploadManifest manifest_ = aai16_UploadManifest_init_zero;
    uint32_t base(unsigned family, unsigned bank);
    bool layout(unsigned family);
    bool verify(const Stored &, char detail[64]);
    bool commit(const aai16_Envelope &, char detail[64]);
    void terminal(aai16_UploadState, aai16_ResultCode, const char *);
    void ack(const aai16_Envelope &, aai16_Envelope &, aai16_ResultCode, aai16_UploadState, const char *);
    void status(const aai16_Envelope &, aai16_Envelope &, aai16_ResultCode, const char *);
    void change(const aai16_Envelope &, aai16_Envelope &);
    bool resolve(const aai16_ModelSelection &, uint32_t epoch, aai16_ModelControl &, char detail[64]);
};
}
