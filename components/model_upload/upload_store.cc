#include "upload_store.h"
#include "motion_backend.h"
#include "motion_sha256.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace upload {
namespace {
using R = aai16_ResultCode;
using S = aai16_UploadState;
constexpr R OK = R::aai16_ResultCode_RESULT_OK;
uint32_t u32(const uint8_t *p) { return uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16|uint32_t(p[3])<<24; }
void put32(uint8_t *p,uint32_t x) { for(unsigned i=0;i<4;++i) p[i]=x>>(8*i); }
uint64_t u64(const uint8_t *p) { return u32(p)|uint64_t(u32(p+4))<<32; }
void put64(uint8_t *p,uint64_t x) { put32(p,x); put32(p+4,x>>32); }
bool sha(const void *data,size_t n,uint8_t out[32]) { return aai16::Sha256::compute(data,n,out); }
bool same(const aai16_UploadManifest &a,const aai16_UploadManifest &b) {
    return a.family==b.family && !std::strcmp(a.version,b.version) && a.package_size==b.package_size &&
        a.model_size==b.model_size && a.compatibility_profile==b.compatibility_profile &&
        !std::memcmp(a.package_sha256.bytes,b.package_sha256.bytes,32) && !std::memcmp(a.model_sha256.bytes,b.model_sha256.bytes,32);
}
bool manifest_valid(const aai16_UploadManifest &m) {
    aai16_Envelope e=aai16_Envelope_init_zero;
    e.has_protocol_version=true; e.protocol_version=1; e.has_request_id=true; e.request_id=1;
    e.has_boot_id=e.has_generation=true; e.which_body=aai16_Envelope_model_control_tag;
    auto &c=e.body.model_control; c.has_operation=true; c.operation=aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD;
    c.has_transfer_id=true; c.transfer_id.size=16; c.has_upload=true; c.upload=m;
    return aai16_protocol_validate(&e)==OK;
}
void identity(const aai16_UploadManifest &m,unsigned bank,aai16_ModelIdentity &id) {
    id=aai16_ModelIdentity_init_zero;
    id.has_family=id.has_version=id.has_sha256=id.has_source=id.has_bank=true;
    id.family=m.family; std::strcpy(id.version,m.version);
    id.sha256.size=32; std::memcpy(id.sha256.bytes,m.model_sha256.bytes,32);
    id.source=aai16_ModelSource_SOURCE_STORED; id.bank=bank;
}
void identity(const motion_model_identity_t &m,aai16_ModelIdentity &id) {
    id=aai16_ModelIdentity_init_zero;
    id.has_family=id.has_version=id.has_sha256=id.has_source=true;
    id.family=static_cast<aai16_ModelFamily>(m.family); std::strcpy(id.version,m.version);
    id.sha256.size=32; std::memcpy(id.sha256.bytes,m.sha256,32);
    id.source=static_cast<aai16_ModelSource>(m.source); id.has_bank=m.source==2; id.bank=m.bank;
}
motion_model_identity_t identity(const aai16_ModelIdentity &m) {
    motion_model_identity_t id{}; id.family=m.family; id.source=m.source; id.bank=m.bank;
    std::strcpy(id.version,m.version); std::memcpy(id.sha256,m.sha256.bytes,32); return id;
}
}
Store::Store(Flash &flash,uint32_t boot,uint8_t *arena,size_t arena_size,ModelControl *runtime):
    flash_(flash),runtime_(runtime),boot_(boot),arena_(arena),arena_size_(arena_size) {}
bool Store::layout(unsigned family) {
    auto capacity=flash_.capacity(family);
    return capacity% (2*kSector)==0 && capacity/2>=kSector+AAI16_PACKAGE_MAX;
}
uint32_t Store::base(unsigned family,unsigned bank) { return bank*(flash_.capacity(family)/2); }
bool Store::verify(const Stored &slot,char detail[64]) {
    const auto &m=slot.manifest;
    uint8_t chunk[AAI16_CHUNK_MAX],digest[32];
    aai16::Sha256 hash;
    bool ok=hash.start();
    uint32_t address=base(m.family,slot.bank)+kSector;
    for(uint32_t offset=0;ok && offset<m.package_size;) {
        size_t n=std::min<uint32_t>(sizeof(chunk),m.package_size-offset);
        ok=flash_.read(m.family,address+offset,chunk,n) && hash.update(chunk,n);
        offset+=n;
    }
    ok=ok && hash.finish(digest);
    if(!ok) { std::strcpy(detail,"storage_read"); return false; }
    if(std::memcmp(digest,m.package_sha256.bytes,32)) { std::strcpy(detail,"package_sha256"); return false; }
    const uint8_t *p=flash_.map(m.family,address,m.package_size);
    if(!p) { std::strcpy(detail,"storage_map"); return false; }
    motion_candidate_info_t info;
    ok=motion_candidate_validate(p,m.package_size,arena_,arena_size_,&info,detail);
    flash_.unmap();
    if(ok && (info.family!=unsigned(m.family)||info.profile!=m.compatibility_profile||info.model_size!=m.model_size||
        std::strcmp(info.version,m.version)||std::memcmp(info.model_sha256,m.model_sha256.bytes,32))) {
        std::strcpy(detail,"manifest_package_mismatch"); ok=false;
    }
    return ok;
}
void Store::recover() {
    for(unsigned f=1;f<=2;++f) for(unsigned b=0;b<2;++b) {
        auto &slot=slots_[f-1][b]; slot={}; slot.bank=b;
        if(!layout(f)) continue;
        uint8_t record[256],digest[32];
        if(!flash_.read(f,base(f,b),record,sizeof(record)) || std::memcmp(record,"AAI16ST1",8) ||
            u32(record+252)!=AAI16_COMMIT_MARKER || !sha(record,220,digest) || std::memcmp(record+220,digest,32)) continue;
        size_t n=u32(record+16);
        if(!n||n>132||!u64(record+8)) continue;
        auto stream=pb_istream_from_buffer(record+20,n);
        if(!pb_decode(&stream,aai16_UploadManifest_fields,&slot.manifest)||!manifest_valid(slot.manifest)||unsigned(slot.manifest.family)!=f) continue;
        char detail[64];
        if(!verify(slot,detail)) continue;
        slot.serial=u64(record+8); slot.valid=true; serial_=std::max(serial_,slot.serial);
    }
}
void Store::terminal(S state,R result,const char *detail) {
    state_=state; terminal_result_=result; std::snprintf(terminal_detail_,64,"%s",detail);
}
void Store::tick(uint64_t now) {
    if(session_ && state_==S::aai16_UploadState_UPLOAD_RECEIVING && now-last_ms_>=kTimeoutMs)
        terminal(S::aai16_UploadState_UPLOAD_ABORTED,R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"upload_timeout_restart_begin");
}
void Store::ack(const aai16_Envelope &request,aai16_Envelope &reply,R result,S state,const char *detail) {
    reply=aai16_Envelope_init_zero;
    reply.has_protocol_version=true; reply.protocol_version=1; reply.has_request_id=request.request_id!=0; reply.request_id=request.request_id;
    reply.has_boot_id=reply.has_generation=reply.has_sequence=true;
    reply.boot_id=boot_; reply.generation=generation_; reply.sequence=request.sequence;
    reply.which_body=aai16_Envelope_model_upload_ack_tag;
    auto &a=reply.body.model_upload_ack;
    a.has_transfer_id=a.has_result=a.has_state=a.has_next_offset=a.has_max_chunk_bytes=a.has_detail=true;
    a.transfer_id.size=16;
    const auto *id=request.which_body==aai16_Envelope_model_upload_chunk_tag ? request.body.model_upload_chunk.transfer_id.bytes : request.body.model_control.transfer_id.bytes;
    std::memcpy(a.transfer_id.bytes,id,16); a.result=result; a.state=state;
    a.next_offset=offset_; a.max_chunk_bytes=AAI16_CHUNK_MAX; std::snprintf(a.detail,64,"%s",detail);
}
void Store::status(const aai16_Envelope &request,aai16_Envelope &reply,R result,const char *detail) {
    reply=aai16_Envelope_init_zero;
    reply.has_protocol_version=true; reply.protocol_version=1; reply.has_request_id=request.request_id!=0; reply.request_id=request.request_id;
    reply.has_boot_id=reply.has_generation=true; reply.boot_id=boot_; reply.generation=generation_;
    reply.which_body=aai16_Envelope_model_status_tag;
    auto &s=reply.body.model_status;
    s.has_result=s.has_state=s.has_model_epoch=s.has_backend=s.has_detail=true;
    s.result=result; s.state=aai16_ModelState_MODEL_STATE_UNAVAILABLE; s.model_epoch=0;
    // Only a snapshot from the ML owner may claim an active identity/readiness.
    motion_model_state_t current{}; // Keep detail storage alive until it is copied below.
    if(runtime_) {
        current=runtime_->snapshot();
        s.has_active=current.active.family!=0;
        if(s.has_active) identity(current.active,s.active);
        s.has_previous=current.has_previous;
        if(s.has_previous) identity(current.previous,s.previous);
        s.model_epoch=current.epoch;
        s.state=!current.ready?aai16_ModelState_MODEL_STATE_UNAVAILABLE:
            current.rolled_back?aai16_ModelState_MODEL_STATE_ROLLED_BACK:aai16_ModelState_MODEL_STATE_RUNNING;
        if(!detail) detail=current.detail;
    }
    s.backend=motion_inference_backend();
    std::snprintf(s.detail,64,"%s",detail?detail:"boot_bundled_a_volatile_activation");
    for(auto &family:slots_) for(auto &slot:family) if(slot.valid) identity(slot.manifest,slot.bank,s.stored[s.stored_count++]);
}
void Store::invalid(const aai16_Envelope &request,R code,aai16_Envelope &reply) {
    if(request.which_body==aai16_Envelope_model_selection_tag ||
       (request.which_body==aai16_Envelope_model_control_tag &&
        request.body.model_control.operation>=aai16_ModelOperation_MODEL_OP_ACTIVATE))
        status(request,reply,code,"invalid_model_message");
    else ack(request,reply,code,S::aai16_UploadState_UPLOAD_REJECTED,"invalid_model_message");
}
bool Store::resolve(const aai16_ModelSelection &selection,uint32_t epoch,aai16_ModelControl &control,char detail[64]) {
    control=aai16_ModelControl_init_zero;
    control.has_operation=control.has_expected_model_epoch=true;
    control.operation=selection.operation; control.expected_model_epoch=epoch;
    if(selection.operation==aai16_ModelOperation_MODEL_OP_ROLLBACK) return true;
    control.has_target=true;
    if(selection.family==aai16_ModelFamily_MODEL_A) {
        motion_model_identity_t bundled;
        motion_model_bundled_a_identity(&bundled);
        identity(bundled,control.target);
        return true;
    }
    const auto &banks=slots_[aai16_ModelFamily_MODEL_B-1];
    if(banks[0].valid && banks[1].valid && banks[0].serial==banks[1].serial) {
        std::strcpy(detail,"ambiguous_stored_b_use_model_control"); return false;
    }
    const Stored *newest=nullptr;
    for(const auto &bank:banks) if(bank.valid && (!newest || bank.serial>newest->serial)) newest=&bank;
    if(!newest) { std::strcpy(detail,"no_committed_model_b_upload_first"); return false; }
    identity(newest->manifest,newest->bank,control.target);
    return true;
}
void Store::change(const aai16_Envelope &r,aai16_Envelope &out) {
    auto reject=[&](R code,const char *why) { status(r,out,code,why); };
    if(!runtime_) { reject(R::aai16_ResultCode_RESULT_NOT_SUPPORTED,"runtime_control_unavailable"); return; }
    const bool selection=r.which_body==aai16_Envelope_model_selection_tag;
    if(!selection && r.boot_id!=boot_) { reject(R::aai16_ResultCode_RESULT_STALE_GENERATION,"boot_changed_get_status"); return; }
    uint8_t wire[AAI16_FRAME_MAX],digest[32]; size_t n=0;
    if(aai16_protocol_encode(&r,wire,sizeof(wire),&n)!=OK || !sha(wire,n,digest)) {
        reject(R::aai16_ResultCode_RESULT_INTERNAL_ERROR,"request_digest"); return;
    }
    for(auto &receipt:receipts_) if(receipt.request==r.request_id) {
        if(std::memcmp(receipt.digest,digest,32)) reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"request_id_reused");
        else out=receipt.reply;
        return;
    }
    if(r.request_id<=highest_change_request_) {
        reject(R::aai16_ResultCode_RESULT_STALE_GENERATION,"request_expired_use_new_id"); return;
    }
    highest_change_request_=r.request_id;
    // Hash/cache the ORIGINAL intent before resolving identity or epoch. A retry
    // must not select a newly uploaded bank or roll back a second time.
    aai16_ModelControl c=aai16_ModelControl_init_zero;
    if(!selection) c=r.body.model_control;
    auto current=runtime_->snapshot();
    char detail[64]{};
    if(!selection && (r.generation!=generation_ || c.expected_model_epoch!=current.epoch))
        reject(R::aai16_ResultCode_RESULT_STALE_GENERATION,"generation_or_epoch_changed_get_status");
    else if(state_==S::aai16_UploadState_UPLOAD_RECEIVING)
        reject(R::aai16_ResultCode_RESULT_BUSY,"upload_in_progress_finish_or_abort");
    else if(selection && !resolve(r.body.model_selection,current.epoch,c,detail))
        reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,detail);
    else {
        motion_model_command_t command{};
        command.rollback=c.operation==aai16_ModelOperation_MODEL_OP_ROLLBACK;
        command.expected_epoch=c.expected_model_epoch;
        command.target=identity(c.target);
        bool ready=true;
        if(!command.rollback && c.target.source==aai16_ModelSource_SOURCE_STORED) {
            const auto &slot=slots_[c.target.family-1][c.target.bank];
            const auto &m=slot.manifest;
            char detail[64];
            if(!slot.valid || std::strcmp(m.version,c.target.version) || std::memcmp(m.model_sha256.bytes,c.target.sha256.bytes,32)) {
                reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"stored_identity_not_found"); ready=false;
            } else if(!verify(slot,detail)) {
                reject(R::aai16_ResultCode_RESULT_CHECKSUM_MISMATCH,detail); ready=false;
            } else {
                command.package=flash_.map(m.family,base(m.family,slot.bank)+kSector,m.package_size);
                command.package_size=m.package_size;
                if(!command.package) { reject(R::aai16_ResultCode_RESULT_INTERNAL_ERROR,"storage_map"); ready=false; }
            }
        }
        if(ready) {
            auto changed=runtime_->change(command);
            R result=changed.result==MOTION_CHANGE_OK?OK:
                changed.result==MOTION_CHANGE_STALE?R::aai16_ResultCode_RESULT_STALE_GENERATION:
                changed.result==MOTION_CHANGE_BUSY?R::aai16_ResultCode_RESULT_BUSY:
                changed.result==MOTION_CHANGE_INVALID?R::aai16_ResultCode_RESULT_OUT_OF_RANGE:R::aai16_ResultCode_RESULT_INTERNAL_ERROR;
            status(r,out,result,changed.detail);
        }
        if(command.package) flash_.unmap();
    }
    auto &receipt=receipts_[receipt_next_++%8];
    receipt.request=r.request_id; std::memcpy(receipt.digest,digest,32); receipt.reply=out;
}
bool Store::commit(const aai16_Envelope &request,char detail[64]) {
    if(serial_==UINT64_MAX) { std::strcpy(detail,"storage_serial_exhausted"); return false; }
    uint8_t record[256]={},check[256]; std::memcpy(record,"AAI16ST1",8); put64(record+8,serial_+1);
    auto stream=pb_ostream_from_buffer(record+20,132);
    if(!pb_encode(&stream,aai16_UploadManifest_fields,&manifest_)) { std::strcpy(detail,"manifest_encode"); return false; }
    put32(record+16,stream.bytes_written); put64(record+152,request.request_id);
    std::memcpy(record+160,transfer_,16); put32(record+176,generation_);
    if(!sha(record,220,record+220)) { std::strcpy(detail,"record_sha256"); return false; }
    put32(record+252,AAI16_COMMIT_MARKER);
    uint32_t address=base(manifest_.family,bank_);
    if(!flash_.write(manifest_.family,address,record,252)||
       !flash_.read(manifest_.family,address,check,252)||std::memcmp(record,check,252)||
       !flash_.write(manifest_.family,address+252,record+252,4)||
       !flash_.read(manifest_.family,address,check,256)||std::memcmp(record,check,256)) {
        std::strcpy(detail,"storage_commit"); return false;
    }
    auto &slot=slots_[manifest_.family-1][bank_]; slot.valid=true; slot.bank=bank_; slot.manifest=manifest_; slot.serial=++serial_;
    return true;
}
void Store::handle(const aai16_Envelope &r,uint64_t now,aai16_Envelope &out) {
    tick(now);
    if(r.which_body==aai16_Envelope_model_selection_tag) { change(r,out); return; }
    const bool control=r.which_body==aai16_Envelope_model_control_tag;
    const auto op=control?r.body.model_control.operation:aai16_ModelOperation_MODEL_OP_UNSPECIFIED;
    auto reject=[&](R code,const char *why) { ack(r,out,code,S::aai16_UploadState_UPLOAD_REJECTED,why); };
    if(control && (op==aai16_ModelOperation_MODEL_OP_GET_STATUS||op==aai16_ModelOperation_MODEL_OP_LIST)) {
        status(r,out,OK,state_==S::aai16_UploadState_UPLOAD_RECEIVING?"upload_in_progress_active_unchanged":nullptr); return;
    }
    if(control && (op==aai16_ModelOperation_MODEL_OP_ACTIVATE||op==aai16_ModelOperation_MODEL_OP_ROLLBACK)) {
        change(r,out); return;
    }
    if(r.boot_id!=boot_) { reject(R::aai16_ResultCode_RESULT_STALE_GENERATION,"boot_changed_get_status"); return; }
    const auto *id=control?r.body.model_control.transfer_id.bytes:r.body.model_upload_chunk.transfer_id.bytes;
    bool match=session_&&!std::memcmp(transfer_,id,16);
    if(op==aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD) {
        const auto &m=r.body.model_control.upload;
        if(match) {
            if(r.request_id!=begin_request_||r.generation!=begin_generation_||!same(m,manifest_)) { reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"transfer_id_reused"); return; }
            ack(r,out,terminal_result_,state_,state_==S::aai16_UploadState_UPLOAD_RECEIVING?"duplicate_begin":terminal_detail_); return;
        }
        if(state_==S::aai16_UploadState_UPLOAD_RECEIVING) { reject(R::aai16_ResultCode_RESULT_BUSY,"another_transfer_in_progress"); return; }
        if(r.generation!=generation_||generation_==UINT32_MAX) { reject(R::aai16_ResultCode_RESULT_STALE_GENERATION,"get_status_before_begin"); return; }
        if(!manifest_valid(m)) { reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"invalid_manifest"); return; }
        if(!layout(m.family)) { reject(R::aai16_ResultCode_RESULT_INTERNAL_ERROR,"partition_layout"); return; }
        auto &banks=slots_[m.family-1];
        bank_=!banks[0].valid?0:!banks[1].valid?1:banks[0].serial<banks[1].serial?0:1;
        if(runtime_) {
            auto current=runtime_->snapshot();
            auto protected_bank=[&](unsigned bank) {
                auto match=[&](const motion_model_identity_t &id) { return id.source==2 && id.family==unsigned(m.family) && id.bank==bank; };
                return match(current.active) || (current.has_previous && match(current.previous));
            };
            if(protected_bank(bank_)) bank_=1-bank_;
            if(protected_bank(bank_)) { reject(R::aai16_ResultCode_RESULT_BUSY,"active_and_previous_banks_protected"); return; }
        }
        // Never erase either runtime bank; a failed replacement retains active.
        banks[bank_].valid=false;
        manifest_=m; offset_=0; begin_request_=r.request_id; begin_generation_=generation_++;
        std::memcpy(transfer_,id,16); session_=true; last_ms_=now;
        terminal(S::aai16_UploadState_UPLOAD_RECEIVING,OK,"receiving");
        uint32_t end=kSector+((m.package_size+kSector-1)/kSector)*kSector;
        for(uint32_t pos=0;pos<end;pos+=kSector) if(!flash_.erase(m.family,base(m.family,bank_)+pos,kSector)) {
            terminal(S::aai16_UploadState_UPLOAD_REJECTED,R::aai16_ResultCode_RESULT_INTERNAL_ERROR,"storage_erase"); break;
        }
        ack(r,out,terminal_result_,state_==S::aai16_UploadState_UPLOAD_RECEIVING?S::aai16_UploadState_UPLOAD_ACCEPTED:state_,terminal_detail_); return;
    }
    if(!match||r.generation!=generation_) { reject(R::aai16_ResultCode_RESULT_STALE_GENERATION,"unknown_transfer_or_generation"); return; }
    if(state_!=S::aai16_UploadState_UPLOAD_RECEIVING) { ack(r,out,terminal_result_,state_,terminal_detail_); return; }
    last_ms_=now;
    if(op==aai16_ModelOperation_MODEL_OP_ABORT_UPLOAD) {
        terminal(S::aai16_UploadState_UPLOAD_ABORTED,OK,"aborted_active_unchanged"); ack(r,out,OK,state_,terminal_detail_); return;
    }
    if(!control) {
        const auto &c=r.body.model_upload_chunk;
        uint32_t n=c.data.size;
        if(c.offset>=manifest_.package_size||c.offset%AAI16_CHUNK_MAX||r.sequence!=c.offset/AAI16_CHUNK_MAX||
           n!=std::min<uint32_t>(AAI16_CHUNK_MAX,manifest_.package_size-c.offset)||c.offset>offset_) {
            reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"chunk_offset_sequence_size"); return;
        }
        uint32_t address=base(manifest_.family,bank_)+kSector+c.offset;
        uint8_t existing[AAI16_CHUNK_MAX];
        if(c.offset<offset_) {
            if(!flash_.read(manifest_.family,address,existing,n)) terminal(S::aai16_UploadState_UPLOAD_REJECTED,R::aai16_ResultCode_RESULT_INTERNAL_ERROR,"storage_read");
            else if(std::memcmp(existing,c.data.bytes,n)) terminal(S::aai16_UploadState_UPLOAD_REJECTED,R::aai16_ResultCode_RESULT_CHECKSUM_MISMATCH,"conflicting_duplicate_chunk");
            ack(r,out,terminal_result_,state_,state_==S::aai16_UploadState_UPLOAD_RECEIVING?"duplicate_chunk":terminal_detail_); return;
        }
        if(!flash_.write(manifest_.family,address,c.data.bytes,n)||!flash_.read(manifest_.family,address,existing,n)||std::memcmp(existing,c.data.bytes,n))
            terminal(S::aai16_UploadState_UPLOAD_REJECTED,R::aai16_ResultCode_RESULT_INTERNAL_ERROR,"storage_write_readback");
        else offset_+=n;
        ack(r,out,terminal_result_,state_,state_==S::aai16_UploadState_UPLOAD_RECEIVING?"chunk_stored":terminal_detail_); return;
    }
    if(op==aai16_ModelOperation_MODEL_OP_FINISH_UPLOAD) {
        if(offset_!=manifest_.package_size||!r.has_sequence||r.sequence!=(offset_+AAI16_CHUNK_MAX-1)/AAI16_CHUNK_MAX) {
            reject(R::aai16_ResultCode_RESULT_OUT_OF_RANGE,"incomplete_upload_or_sequence"); return;
        }
        Stored candidate; candidate.manifest=manifest_; candidate.bank=bank_;
        char detail[64];
        if(!verify(candidate,detail)) terminal(S::aai16_UploadState_UPLOAD_REJECTED,
            std::strncmp(detail,"storage_",8)==0?R::aai16_ResultCode_RESULT_INTERNAL_ERROR:R::aai16_ResultCode_RESULT_CHECKSUM_MISMATCH,detail);
        else if(!commit(r,detail)) terminal(S::aai16_UploadState_UPLOAD_REJECTED,R::aai16_ResultCode_RESULT_INTERNAL_ERROR,detail);
        else terminal(S::aai16_UploadState_UPLOAD_STORED,OK,"validated_4_of_4_stored_active_unchanged");
        ack(r,out,terminal_result_,state_,terminal_detail_); return;
    }
    reject(R::aai16_ResultCode_RESULT_NOT_SUPPORTED,"unsupported_upload_operation");
}
}
