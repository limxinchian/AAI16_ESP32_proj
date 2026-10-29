#include "upload_store.h"
#include "motion_engine.h"
#include "motion_backend.h"
#include "model_runtime.h"
#include <cstdlib>
#include <malloc.h>
#include "upload_ingress.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "mbedtls/sha256.h"
#include <vector>
#include <fstream>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <memory>
#include <iostream>
#include <thread>
#include <atomic>

namespace {
constexpr auto OK=aai16_ResultCode_RESULT_OK;
constexpr auto RECEIVING=aai16_UploadState_UPLOAD_RECEIVING;
constexpr auto STORED=aai16_UploadState_UPLOAD_STORED;
std::vector<uint8_t> package;
uint32_t le32(const uint8_t *p) { return uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16|uint32_t(p[3])<<24; }
void hash(const void *p,size_t n,uint8_t *out) { assert(!mbedtls_sha256(static_cast<const uint8_t*>(p),n,out,0)); }
bool fail_allocation;
unsigned allocated;
void *allocate(size_t n) {
    if(fail_allocation) return nullptr;
#ifdef _WIN32
    auto *p=_aligned_malloc(n,16);
#else
    auto *p=std::aligned_alloc(16,(n+15)&~size_t(15));
#endif
    if(p) ++allocated; return p;
}
void release(void *p) {
    assert(allocated); --allocated;
#ifdef _WIN32
    _aligned_free(p);
#else
    std::free(p);
#endif
}
struct RuntimeControl:upload::ModelControl {
    motion::ModelRuntime runtime{allocate,release};
    bool race_before_change=false;
    RuntimeControl() { assert(runtime.start()); }
    motion_model_state_t snapshot() override { return runtime.state(); }
    motion_model_reply_t change(const motion_model_command_t &c) override {
        if(race_before_change) { race_before_change=false; assert(runtime.recover_invoke()); }
        return runtime.change(c);
    }
};
struct MemoryFlash:upload::Flash {
    std::vector<uint8_t> data[2]={std::vector<uint8_t>(0x300000,255),std::vector<uint8_t>(0x300000,255)};
    bool fail_read=false,fail_erase=false,fail_map=false;
    int writes=0,fail_write=0,torn_bytes=0,erases=0;
    uint32_t capacity(unsigned f) override { return f>=1&&f<=2?data[f-1].size():0; }
    bool bounds(unsigned f,uint32_t o,size_t n) { return f>=1&&f<=2&&o<=capacity(f)&&n<=capacity(f)-o; }
    bool read(unsigned f,uint32_t o,void *p,size_t n) override {
        if(fail_read||!bounds(f,o,n)) return false;
        std::memcpy(p,data[f-1].data()+o,n); return true;
    }
    bool erase(unsigned f,uint32_t o,size_t n) override {
        assert(o%4096==0&&n==4096); ++erases;
        if(fail_erase||!bounds(f,o,n)) return false;
        std::memset(data[f-1].data()+o,255,n); return true;
    }
    bool write(unsigned f,uint32_t o,const void *p,size_t n) override {
        assert(n<=512&&bounds(f,o,n)); ++writes;
        bool failure=writes==fail_write; size_t count=failure?std::min<size_t>(torn_bytes,n):n;
        for(size_t i=0;i<count;++i) {
            uint8_t x=static_cast<const uint8_t*>(p)[i];
            assert((data[f-1][o+i]&x)==x); data[f-1][o+i]&=x;
        }
        return !failure;
    }
    const uint8_t *map(unsigned f,uint32_t o,size_t n) override { return fail_map||!bounds(f,o,n)?nullptr:data[f-1].data()+o; }
    void unmap() override {}
};
aai16_UploadManifest manifest(const std::vector<uint8_t> &bytes) {
    aai16_UploadManifest m=aai16_UploadManifest_init_zero;
    m.has_family=m.has_version=m.has_package_size=m.has_package_sha256=m.has_model_size=m.has_model_sha256=m.has_compatibility_profile=true;
    m.family=static_cast<aai16_ModelFamily>(le32(bytes.data()+16)); m.compatibility_profile=1;
    std::memcpy(m.version,bytes.data()+64,40); m.package_size=bytes.size(); m.model_size=le32(bytes.data()+24);
    m.package_sha256.size=m.model_sha256.size=32;
    hash(bytes.data(),bytes.size(),m.package_sha256.bytes); std::memcpy(m.model_sha256.bytes,bytes.data()+104,32); return m;
}
struct Harness {
    MemoryFlash flash;
    alignas(16) uint8_t arena[MOTION_CANDIDATE_ARENA];
    std::unique_ptr<upload::Store> store;
    std::unique_ptr<RuntimeControl> runtime;
    uint32_t boot=42,generation=0,offset=0;
    uint64_t request=100,time=1;
    uint8_t transfer[16]={1};
    aai16_UploadManifest m=manifest(package);
    Harness() { reboot(); }
    void reboot() { ++boot; store.reset(); runtime.reset(new RuntimeControl); store.reset(new upload::Store(flash,boot,arena,sizeof(arena),runtime.get())); store->recover(); generation=store->generation(); }
    aai16_Envelope envelope() {
        aai16_Envelope e=aai16_Envelope_init_zero;
        e.has_protocol_version=e.has_request_id=e.has_boot_id=e.has_generation=e.has_sequence=true;
        e.protocol_version=1; e.request_id=++request; e.boot_id=boot; e.generation=generation; return e;
    }
    aai16_Envelope control(aai16_ModelOperation op) {
        auto e=envelope(); e.which_body=aai16_Envelope_model_control_tag;
        auto &c=e.body.model_control; c.has_operation=true; c.operation=op;
        if(op==aai16_ModelOperation_MODEL_OP_GET_STATUS) { e.has_sequence=false; return e; }
        c.has_transfer_id=true; c.transfer_id.size=16; std::memcpy(c.transfer_id.bytes,transfer,16);
        if(op==aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD) { c.has_upload=true; c.upload=m; }
        if(op==aai16_ModelOperation_MODEL_OP_FINISH_UPLOAD) e.sequence=(m.package_size+511)/512;
        return e;
    }
    aai16_Envelope chunk(uint32_t pos,const std::vector<uint8_t> &bytes=package) {
        auto e=envelope(); e.which_body=aai16_Envelope_model_upload_chunk_tag; e.sequence=pos/512;
        auto &c=e.body.model_upload_chunk; c.has_transfer_id=c.has_offset=c.has_data=true;
        c.transfer_id.size=16; std::memcpy(c.transfer_id.bytes,transfer,16); c.offset=pos;
        c.data.size=std::min<size_t>(512,bytes.size()-pos); std::memcpy(c.data.bytes,bytes.data()+pos,c.data.size); return e;
    }
    aai16_Envelope send(const aai16_Envelope &e) {
        uint8_t wire[768]; size_t n=0; aai16_Envelope decoded,out,checked;
        assert(aai16_protocol_encode(&e,wire,sizeof(wire),&n)==OK);
        assert(aai16_protocol_decode(wire,n,&decoded)==OK);
        store->handle(decoded,time++,out);
        assert(aai16_protocol_encode(&out,wire,sizeof(wire),&n)==OK);
        assert(aai16_protocol_decode(wire,n,&checked)==OK);
        assert(checked.request_id==e.request_id&&checked.boot_id==boot);
        if(checked.which_body==aai16_Envelope_model_status_tag) {
            assert(checked.body.model_status.has_backend && checked.body.model_status.backend==TEST_EXPECTED_BACKEND);
            assert(checked.body.model_status.backend==motion_inference_backend());
        }
        return checked;
    }
    aai16_ModelUploadAck begin() {
        offset=0; auto out=send(control(aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD)); generation=out.generation; return out.body.model_upload_ack;
    }
    void chunks(const std::vector<uint8_t> &bytes=package) {
        while(offset<bytes.size()) { auto a=send(chunk(offset,bytes)).body.model_upload_ack; assert(a.result==OK&&a.state==RECEIVING); offset=a.next_offset; }
    }
    aai16_ModelUploadAck finish() { return send(control(aai16_ModelOperation_MODEL_OP_FINISH_UPLOAD)).body.model_upload_ack; }
    aai16_Envelope activate(bool b=true,unsigned bank=0,bool rollback=false) {
        auto r=envelope(); r.has_sequence=false; r.which_body=aai16_Envelope_model_control_tag;
        auto &c=r.body.model_control; c.has_operation=c.has_expected_model_epoch=true;
        c.operation=rollback?aai16_ModelOperation_MODEL_OP_ROLLBACK:aai16_ModelOperation_MODEL_OP_ACTIVATE;
        c.expected_model_epoch=runtime->snapshot().epoch;
        if(!rollback) {
            c.has_target=true; auto &id=c.target;
            id.has_family=id.has_version=id.has_sha256=id.has_source=true;
            id.family=b?aai16_ModelFamily_MODEL_B:aai16_ModelFamily_MODEL_A;
            id.source=b?aai16_ModelSource_SOURCE_STORED:aai16_ModelSource_SOURCE_BUNDLED;
            id.has_bank=b; id.bank=bank; id.sha256.size=32;
            const auto &a=motion::bundled_model(motion::ModelFamily::A)->view;
            std::strcpy(id.version,b?m.version:a.version); std::memcpy(id.sha256.bytes,b?m.model_sha256.bytes:a.sha256,32);
        }
        return r;
    }
    aai16_Envelope select(unsigned family=2) {
        auto r=envelope(); r.has_boot_id=r.has_generation=r.has_sequence=false;
        r.which_body=aai16_Envelope_model_selection_tag;
        auto &s=r.body.model_selection; s.has_operation=true;
        s.operation=family?aai16_ModelOperation_MODEL_OP_ACTIVATE:aai16_ModelOperation_MODEL_OP_ROLLBACK;
        s.has_family=family!=0; s.family=static_cast<aai16_ModelFamily>(family);
        return r;
    }
    void predictions() {
        auto &engine=runtime->runtime.engine(); const auto &model=runtime->runtime.model();
        bool classes[2]={};
        for(size_t i=0;i<model.golden_count;++i) {
            float features[60],logits[2]; int8_t input[60],output[2]; uint32_t clipped;
            assert(motion_preprocess(model.golden[i].raw,features));
            assert(motion_quantize(features,engine.input_scale(),engine.input_zero_point(),input,&clipped));
            assert(engine.invoke(input,output));
            int cls=motion_decode(output,engine.output_scale(),engine.output_zero_point(),logits);
            assert(cls==model.golden[i].prediction); classes[cls]=true;
        }
        assert(classes[0]&&classes[1]);
    }
    unsigned count() {
        auto e=send(control(aai16_ModelOperation_MODEL_OP_GET_STATUS));
        assert(e.body.model_status.active.family==aai16_ModelFamily_MODEL_A&&e.body.model_status.active.source==aai16_ModelSource_SOURCE_BUNDLED);
        if(std::strcmp(e.body.model_status.detail,"upload_in_progress_active_unchanged"))
            assert(!std::strcmp(e.body.model_status.detail,runtime->snapshot().detail));
        return e.body.model_status.stored_count;
    }
};
unsigned passed;
template<class F> void test(const char *name,F fn) { fn(); ++passed; std::printf("PASS %s\n",name); }
void fallback() {
    alignas(16) uint8_t arena[16384]; motion::Engine engine; auto &a=*motion::bundled_model(motion::ModelFamily::A);
    motion_runtime_enter(); assert(engine.init(a.view,arena,sizeof(arena))); size_t passed=0;
    assert(motion::model_self_test(engine,a,passed)&&passed==4); motion_runtime_leave();
}
}
int main(int argc,char **argv) {
    assert(argc==2||argc==3); std::ifstream file(argv[1],std::ios::binary); assert(file);
    package=std::vector<uint8_t>(std::istreambuf_iterator<char>(file),{});
    assert(package.size()>256); fallback();
    if(argc==3&&!std::strcmp(argv[2],"--bridge")) {
        Harness h; std::string line;
        while(std::getline(std::cin,line)) {
            if(line=="REBOOT") { h.reboot(); std::puts("OK"); std::fflush(stdout); continue; }
            assert(line.size()<=1536&&line.size()%2==0); uint8_t wire[768]; size_t n=line.size()/2;
            for(size_t i=0;i<n;++i) { unsigned value=0; assert(std::sscanf(line.c_str()+2*i,"%2x",&value)==1); wire[i]=value; }
            aai16_Envelope request,reply; assert(aai16_protocol_decode(wire,n,&request)==OK);
            h.store->handle(request,h.time++,reply); assert(aai16_protocol_encode(&reply,wire,sizeof(wire),&n)==OK);
            for(size_t i=0;i<n;++i) std::printf("%02x",wire[i]); std::puts(""); std::fflush(stdout);
        }
        return 0;
    }
    test("bounded MQTT fragment assembly, retained/order/size/topic/disconnect",[] {
        upload::Ingress rx; upload::Frame frame; uint8_t data[768]={};
        auto start=[&](int total=768,bool retain=false) { return rx.feed(MODEL_CHUNK_TOPIC,std::strlen(MODEL_CHUNK_TOPIC),data,100,0,total,7,retain,frame); };
        assert(!start()); assert(rx.feed(nullptr,0,data+100,668,100,768,7,false,frame)); assert(frame.length==768&&!frame.control);
        assert(!start(769)); assert(!start(768,true)); assert(!rx.feed(nullptr,0,data,668,100,768,7,false,frame));
        assert(!start()); assert(!rx.feed(nullptr,0,data,600,101,768,7,false,frame));
        assert(!start()); assert(!rx.feed(nullptr,0,data,668,100,768,8,false,frame));
        assert(!start()); assert(!rx.feed(MODEL_CONTROL_TOPIC,std::strlen(MODEL_CONTROL_TOPIC),data,668,100,768,7,false,frame));
        assert(!start()); rx.reset(); assert(!rx.feed(nullptr,0,data,668,100,768,7,false,frame));
        assert(!start()); assert(rx.feed(nullptr,0,data,668,100,768,7,false,frame));
    });
    test("actual B package, output quantization, persist + recover, active A",[] {
        Harness h; assert(h.begin().result==OK); h.chunks(); auto a=h.finish();
        if(a.result!=OK) std::fprintf(stderr,"validation failure: %s\n",a.detail);
        assert(a.result==OK&&a.state==STORED); assert(h.count()==1); assert(h.finish().state==STORED);
        h.reboot(); assert(h.count()==1);
        assert(h.store->stored(2,0).manifest.model_size==2384); fallback();
    });
    test("selection resolves bundled A and stored B; rollback retry never toggles twice",[] {
        Harness h;
        auto a=h.send(h.select(1)).body.model_status;
        assert(a.result==OK&&a.active.family==aai16_ModelFamily_MODEL_A&&a.model_epoch==0);
        auto missing=h.select(); assert(h.send(missing).body.model_status.result!=OK);
        h.begin(); h.chunks(); h.finish();
        assert(h.send(missing).body.model_status.result!=OK); // Cached rejection, not re-resolved after upload.
        auto b=h.send(h.select()).body.model_status;
        assert(b.result==OK&&b.active.source==aai16_ModelSource_SOURCE_STORED&&b.active.bank==0&&b.model_epoch==1); h.predictions();
        assert(h.send(h.select()).body.model_status.model_epoch==1); // already active
        a=h.send(h.select(1)).body.model_status;
        assert(a.result==OK&&a.active.source==aai16_ModelSource_SOURCE_BUNDLED&&a.model_epoch==2); h.predictions();
        auto rollback=h.select(0); auto first=h.send(rollback);
        assert(first.body.model_status.result==OK&&first.body.model_status.active.family==aai16_ModelFamily_MODEL_B);
        for(unsigned i=0;i<3;++i) assert(h.send(rollback).body.model_status.model_epoch==3);
        assert(h.runtime->snapshot().epoch==3); h.predictions();
        auto back=h.activate(false); assert(h.send(back).body.model_status.model_epoch==4);
        h.send(rollback); assert(h.runtime->snapshot().epoch==4&&h.runtime->snapshot().active.family==1);
    });
    test("selection uses commit serial, not bank number, and caches original intent",[] {
        Harness h; h.begin(); h.chunks(); h.finish();
        ++h.transfer[0]; h.begin(); h.chunks(); h.finish();
        auto request=h.select(); auto b=h.send(request).body.model_status;
        assert(b.result==OK&&b.active.bank==1&&b.model_epoch==1);
        ++h.transfer[0]; h.begin(); h.chunks(); h.finish(); // newest is now bank 0; active bank 1 protected
        assert(h.store->stored(2,0).serial>h.store->stored(2,1).serial);
        assert(h.send(request).body.model_status.active.bank==1&&h.runtime->snapshot().epoch==1);
        b=h.send(h.select()).body.model_status;
        assert(b.result==OK&&b.active.bank==0&&b.model_epoch==2); h.predictions();
        h.reboot(); b=h.send(h.select()).body.model_status;
        assert(b.result==OK&&b.active.bank==0&&b.model_epoch==1); h.predictions();
    });
    test("selection fails closed for newest corrupt storage and allocation failure",[] {
        Harness h; h.begin(); h.chunks(); h.finish(); ++h.transfer[0]; h.begin(); h.chunks(); h.finish();
        h.flash.data[1][0x180000+4096+300]^=1;
        auto bad=h.send(h.select()).body.model_status;
        assert(bad.result==aai16_ResultCode_RESULT_CHECKSUM_MISMATCH&&bad.active.family==aai16_ModelFamily_MODEL_A&&bad.model_epoch==0);
        // Never silently activate the older bank 0 after newest bank 1 fails.
        h.predictions(); h.flash.data[1][0x180000+4096+300]^=1;
        fail_allocation=true; bad=h.send(h.select()).body.model_status; fail_allocation=false;
        assert(bad.result!=OK&&bad.model_epoch==0&&bad.active.family==aai16_ModelFamily_MODEL_A); h.predictions();
        assert(h.send(h.select()).body.model_status.result==OK); h.predictions();
    });
    test("selection rejects ambiguous recovered commit serials; exact target still works",[] {
        Harness h; h.begin(); h.chunks(); h.finish(); ++h.transfer[0]; h.begin(); h.chunks(); h.finish();
        auto *first=h.flash.data[1].data(), *second=first+0x180000;
        std::memcpy(second+8,first+8,8); hash(second,220,second+220);
        h.reboot(); assert(h.count()==2);
        auto reply=h.send(h.select()).body.model_status;
        assert(reply.result==aai16_ResultCode_RESULT_OUT_OF_RANGE&&reply.active.family==aai16_ModelFamily_MODEL_A);
        assert(!std::strcmp(reply.detail,"ambiguous_stored_b_use_model_control"));
        assert(h.send(h.activate(true,1)).body.model_status.result==OK); h.predictions();
    });
    test("selection admission preserves upload BUSY and worker epoch race checks",[] {
        Harness h; h.begin(); h.chunks(); h.finish(); ++h.transfer[0]; h.begin();
        auto busy=h.select(); assert(h.send(busy).body.model_status.result==aai16_ResultCode_RESULT_BUSY);
        h.send(h.control(aai16_ModelOperation_MODEL_OP_ABORT_UPLOAD));
        assert(h.send(busy).body.model_status.result==aai16_ResultCode_RESULT_BUSY);
        h.runtime->race_before_change=true;
        auto racing=h.select(); auto reply=h.send(racing).body.model_status;
        assert(reply.result==aai16_ResultCode_RESULT_STALE_GENERATION&&reply.active.family==aai16_ModelFamily_MODEL_A);
        assert(h.runtime->snapshot().epoch==1); h.send(racing); assert(h.runtime->snapshot().epoch==1);
        assert(h.send(h.select()).body.model_status.model_epoch==2); h.predictions();
    });
    test("selection and advanced commands share conflicting-ID and expiry protection",[] {
        Harness h; h.begin(); h.chunks(); h.finish();
        auto simple=h.select(); assert(h.send(simple).body.model_status.result==OK);
        auto advanced=h.activate(false); advanced.request_id=simple.request_id;
        assert(h.send(advanced).body.model_status.result==aai16_ResultCode_RESULT_OUT_OF_RANGE);
        auto changed=simple; changed.body.model_selection.family=aai16_ModelFamily_MODEL_A;
        assert(h.send(changed).body.model_status.result==aai16_ResultCode_RESULT_OUT_OF_RANGE);
        advanced=h.activate(false); --advanced.body.model_control.expected_model_epoch;
        assert(h.send(advanced).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        for(unsigned i=0;i<8;++i) assert(h.send(h.select()).body.model_status.result==OK);
        assert(h.send(simple).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        assert(h.runtime->snapshot().epoch==1); h.predictions();
    });
    test("selection routes only on control topic and malformed selection returns status",[] {
        Harness h; auto selection=h.select(); upload::Frame frame{};
        assert(!upload::frame_accepts(frame,selection)); frame.control=true;
        assert(upload::frame_accepts(frame,selection));
        auto chunk=h.chunk(0); assert(!upload::frame_accepts(frame,chunk));
        frame.control=false; assert(upload::frame_accepts(frame,chunk));
        aai16_Envelope reply; h.store->invalid(selection,aai16_ResultCode_RESULT_MALFORMED,reply);
        assert(reply.which_body==aai16_Envelope_model_status_tag&&reply.request_id==selection.request_id&&reply.body.model_status.result==aai16_ResultCode_RESULT_MALFORMED);
    });
    test("duplicate BEGIN/chunk/FINISH retries do not re-erase/write",[] {
        Harness h; auto begin=h.control(aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD); auto b=h.send(begin); h.generation=b.generation;
        auto c=h.chunk(0); h.send(c); int writes=h.flash.writes,erases=h.flash.erases;
        assert(h.send(c).body.model_upload_ack.next_offset==512);
        assert(h.send(begin).body.model_upload_ack.next_offset==512);
        assert(h.flash.writes==writes&&h.flash.erases==erases); h.offset=512; h.chunks(); assert(h.finish().state==STORED);
    });
    test("out of order, missing, invalid sequence, wrong transfer/generation/boot",[] {
        Harness h; h.begin(); assert(h.send(h.chunk(512)).body.model_upload_ack.result!=OK);
        assert(h.finish().result!=OK); auto c=h.chunk(0); ++c.sequence; assert(h.send(c).body.model_upload_ack.result!=OK);
        c=h.chunk(0); c.body.model_upload_chunk.transfer_id.bytes[0]^=1; assert(h.send(c).body.model_upload_ack.result!=OK);
        c=h.chunk(0); ++c.generation; assert(h.send(c).body.model_upload_ack.result!=OK);
        c=h.chunk(0); ++c.boot_id; assert(h.send(c).body.model_upload_ack.result!=OK);
        h.chunks(); assert(h.finish().state==STORED);
    });
    test("conflicting duplicate aborts candidate",[] {
        Harness h; h.begin(); auto c=h.chunk(0); h.send(c); c.body.model_upload_chunk.data.bytes[9]^=1;
        assert(h.send(c).body.model_upload_ack.result==aai16_ResultCode_RESULT_CHECKSUM_MISMATCH); assert(h.count()==0); h.reboot(); assert(h.count()==0); fallback();
    });
    test("invalid package SHA",[] {
        Harness h; h.m.package_sha256.bytes[0]^=1; h.begin(); h.chunks(); assert(h.finish().result==aai16_ResultCode_RESULT_CHECKSUM_MISMATCH);
        h.reboot(); assert(h.count()==0); fallback();
    });
    test("oversized model/package semantic admission",[] {
        Harness h; auto b=h.control(aai16_ModelOperation_MODEL_OP_BEGIN_UPLOAD);
        b.body.model_control.upload.package_size=AAI16_PACKAGE_MAX+1; assert(aai16_protocol_validate(&b)!=OK);
        b.body.model_control.upload.package_size=h.m.package_size; b.body.model_control.upload.model_size=AAI16_MODEL_MAX+1; assert(aai16_protocol_validate(&b)!=OK);
        assert(h.flash.erases==0&&h.count()==0); fallback();
    });
    test("manifest identity/version mismatch",[] {
        Harness h; h.m.version[0]='X'; h.begin(); h.chunks(); assert(h.finish().result!=OK); assert(h.count()==0);
    });
    test("abort and repeated abort",[] {
        Harness h; h.begin(); h.send(h.chunk(0)); auto r=h.control(aai16_ModelOperation_MODEL_OP_ABORT_UPLOAD);
        assert(h.send(r).body.model_upload_ack.state==aai16_UploadState_UPLOAD_ABORTED);
        assert(h.send(r).body.model_upload_ack.state==aai16_UploadState_UPLOAD_ABORTED); h.reboot(); assert(h.count()==0); fallback();
    });
    test("timeout then retry using new transfer",[] {
        Harness h; h.begin(); h.time+=upload::kTimeoutMs+1; h.store->tick(h.time);
        assert(h.finish().state==aai16_UploadState_UPLOAD_ABORTED); ++h.transfer[0]; h.begin(); h.chunks(); assert(h.finish().state==STORED);
    });
    test("disconnect/reconnect retransmit retains progress",[] {
        Harness h; h.begin(); auto c=h.chunk(0); h.send(c); h.time+=10000; h.store->tick(h.time);
        assert(h.send(c).body.model_upload_ack.next_offset==512); h.offset=512; h.chunks(); assert(h.finish().state==STORED);
    });
    test("erase failure",[] { Harness h; h.flash.fail_erase=true; assert(h.begin().result!=OK); h.reboot(); assert(h.count()==0); fallback(); });
    test("write/read/map failures",[] {
        { Harness h; h.begin(); h.flash.fail_write=1; assert(h.send(h.chunk(0)).body.model_upload_ack.result!=OK); h.reboot(); assert(h.count()==0); }
        { Harness h; h.begin(); h.flash.fail_read=true; assert(h.send(h.chunk(0)).body.model_upload_ack.result!=OK); h.flash.fail_read=false; h.reboot(); assert(h.count()==0); }
        { Harness h; h.begin(); h.chunks(); h.flash.fail_map=true; assert(h.finish().result!=OK); h.flash.fail_map=false; h.reboot(); assert(h.count()==0); }
        fallback();
    });
    test("interrupted upload reboot discards incomplete data",[] {
        Harness h; h.begin(); auto c=h.chunk(0); h.send(c); h.reboot(); assert(h.count()==0); assert(h.send(c).body.model_upload_ack.result!=OK);
        ++h.transfer[0]; h.begin(); h.chunks(); assert(h.finish().state==STORED); fallback();
    });
    test("all chunks without FINISH never become a valid candidate",[] {
        Harness h; h.begin(); h.chunks(); assert(h.count()==0); h.reboot(); assert(h.count()==0); fallback();
    });
    test("wrong partition capacity rejected before erase",[] {
        Harness h; h.flash.data[1].resize(8192); assert(h.begin().result!=OK); assert(h.flash.erases==0); fallback();
    });
    test("A to stored B to bundled A 100 times; REST/MOVING and quantization",[] {
        Harness h; h.begin(); h.chunks(); h.finish(); h.predictions();
        const auto writes=h.flash.writes,erases=h.flash.erases;
        unsigned memory=0;
        for(unsigned i=0;i<100;++i) {
            auto b=h.send(h.activate()).body.model_status;
            assert(b.result==OK&&b.active.source==aai16_ModelSource_SOURCE_STORED&&b.active.family==aai16_ModelFamily_MODEL_B&&b.model_epoch==2*i+1);
            assert(b.previous.family==aai16_ModelFamily_MODEL_A); h.predictions();
            auto a=h.send(h.activate(false)).body.model_status;
            assert(a.result==OK&&a.active.source==aai16_ModelSource_SOURCE_BUNDLED&&a.model_epoch==2*i+2);
            h.predictions(); if(!i) memory=allocated; else assert(memory==allocated);
            assert(h.flash.writes==writes&&h.flash.erases==erases); // Power cut cannot tear a selection record: none is written.
        }
    });
    test("activation receipts: duplicate, conflicting ID, epoch, boot, generation, expiry",[] {
        Harness h; h.begin(); h.chunks(); h.finish(); auto r=h.activate();
        auto first=h.send(r); auto retry=h.send(r);
        assert(first.body.model_status.result==OK&&retry.body.model_status.model_epoch==1);
        assert(h.runtime->snapshot().epoch==1);
        auto changed=r; changed.body.model_control.target.bank=1;
        assert(h.send(changed).body.model_status.result!=OK);
        auto stale=h.activate(false); stale.body.model_control.expected_model_epoch=0;
        assert(h.send(stale).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        stale=h.activate(false); stale.boot_id--;
        assert(h.send(stale).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        stale=h.activate(false); stale.generation++;
        assert(h.send(stale).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        for(unsigned i=0;i<10;++i) assert(h.send(h.activate()).body.model_status.result==OK);
        assert(h.send(r).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        assert(h.runtime->snapshot().epoch==1); h.predictions();
    });
    test("invalid identity, corrupted storage and memory failure retain A",[] {
        Harness h; assert(h.send(h.activate()).body.model_status.result!=OK);
        h.begin(); h.chunks(); h.finish();
        auto wrong=h.activate(); wrong.body.model_control.target.sha256.bytes[0]^=1;
        assert(h.send(wrong).body.model_status.result!=OK);
        h.flash.data[1][4096+300]^=1;
        assert(h.send(h.activate()).body.model_status.result!=OK); h.flash.data[1][4096+300]^=1;
        fail_allocation=true; auto failed=h.send(h.activate()); fail_allocation=false;
        assert(failed.body.model_status.result!=OK&&failed.body.model_status.active.family==aai16_ModelFamily_MODEL_A);
        assert(failed.body.model_status.model_epoch==0); h.predictions();
        assert(h.send(h.activate()).body.model_status.result==OK); h.predictions();
    });
    test("runtime independently rejects incompatible graph, SHA and goldens",[] {
        Harness h;
        for(unsigned offset:{20U,104U,256U,2640U+300U}) {
            auto damaged=package; damaged[offset]^=1;
            motion_model_command_t command{};
            auto r=h.activate(); const auto &id=r.body.model_control.target;
            command.target.family=2; command.target.source=2;
            std::strcpy(command.target.version,id.version); std::memcpy(command.target.sha256,id.sha256.bytes,32);
            command.package=damaged.data(); command.package_size=damaged.size();
            auto reply=h.runtime->runtime.change(command);
            assert(reply.result!=MOTION_CHANGE_OK&&reply.state.ready&&reply.state.active.family==1&&reply.state.epoch==0);
            h.predictions();
        }
    });
    test("rollback, automatic invoke recovery and volatile reboot after activation",[] {
        Harness h; h.begin(); h.chunks(); h.finish();
        assert(h.send(h.activate()).body.model_status.result==OK);
        assert(h.send(h.activate(false,0,true)).body.model_status.active.family==aai16_ModelFamily_MODEL_A); h.predictions();
        h.send(h.activate()); assert(h.runtime->runtime.recover_invoke());
        assert(h.runtime->snapshot().active.family==1&&h.runtime->snapshot().rolled_back); h.predictions();
        auto old=h.activate(); h.send(old); h.reboot();
        assert(h.count()==1&&h.runtime->snapshot().epoch==0);
        assert(h.send(old).body.model_status.result==aai16_ResultCode_RESULT_STALE_GENERATION);
        assert(h.send(h.activate()).body.model_status.result==OK); h.predictions();
    });
    test("active and previous banks protected; activation blocked during upload",[] {
        Harness h; h.begin(); h.chunks(); h.finish(); h.send(h.activate());
        ++h.transfer[0]; assert(h.begin().result==OK); assert(h.store->stored(2,0).valid);
        assert(h.send(h.activate(false)).body.model_status.result==aai16_ResultCode_RESULT_BUSY);
        h.chunks(); h.finish(); h.send(h.activate(true,1));
        unsigned erases=h.flash.erases; ++h.transfer[0];
        assert(h.begin().result==aai16_ResultCode_RESULT_BUSY&&unsigned(h.flash.erases)==erases);
        h.predictions(); h.send(h.activate(false)); ++h.transfer[0]; assert(h.begin().result==OK);
        assert(h.store->stored(2,1).valid); h.predictions();
    });
    test("older candidate survives every torn replacement write",[] {
        for(int failure=1;failure<=10;++failure) {
            Harness h; h.begin(); h.chunks(); assert(h.finish().state==STORED); int original=h.flash.writes;
            ++h.transfer[0]; assert(h.begin().result==OK); h.flash.fail_write=original+failure; h.flash.torn_bytes=2;
            bool failed=false;
            while(h.offset<package.size()) {
                auto a=h.send(h.chunk(h.offset)).body.model_upload_ack;
                if(a.result!=OK) { failed=true; break; } h.offset=a.next_offset;
            }
            if(!failed) assert(h.finish().result!=OK);
            h.reboot(); assert(h.count()==1&&h.store->stored(2,0).valid); fallback();
        }
    });
    test("corrupt persisted bytes rejected on reboot",[] {
        Harness h; h.begin(); h.chunks(); assert(h.finish().state==STORED); h.flash.data[1][4096+300]^=1;
        h.reboot(); assert(h.count()==0); fallback();
    });
    test("complete commit marker before lost completion recovers valid candidate",[] {
        Harness h; h.begin(); h.chunks();
        h.flash.fail_write=h.flash.writes+2; h.flash.torn_bytes=4;
        assert(h.finish().result!=OK); // Driver reports failure after all marker bytes landed.
        assert(h.count()==0); h.reboot(); assert(h.count()==1); fallback();
    });
    test("package contract/golden/INT8 metadata reject with corrected outer SHA",[] {
        for(unsigned index:{20U,44U,56U,136U,168U,256U,2640U+240U,2640U+300U}) {
            auto bad=package; bad[index]^=1; Harness h; h.m=manifest(bad); h.begin(); h.chunks(bad);
            assert(h.finish().result!=OK); h.reboot(); assert(h.count()==0); fallback();
        }
    });
    test("invalid model SHA and unsafe tensor shape with correct package SHA",[] {
        auto bad=package; bad[104]^=1;
        { Harness h; h.m=manifest(bad); h.begin(); h.chunks(bad); assert(h.finish().result!=OK); }
        bad=package; auto *model=tflite::GetModel(bad.data()+256);
        const auto *shape=model->subgraphs()->Get(0)->tensors()->Get(model->subgraphs()->Get(0)->inputs()->Get(0))->shape();
        uint8_t *dimension=const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(shape->Data()))+4;
        dimension[0]=255; dimension[1]=255; dimension[2]=255; dimension[3]=127;
        hash(bad.data()+256,le32(bad.data()+24),bad.data()+104);
        Harness h; h.m=manifest(bad); h.begin(); h.chunks(bad); assert(h.finish().result!=OK); fallback();
    });
    test("candidate validation/storage with concurrent bundled A interpreter",[] {
        std::atomic<unsigned> invokes{0}; std::atomic<bool> stop{false};
        std::thread live([&] {
            alignas(16) uint8_t arena[16384]; motion::Engine engine;
            const auto &a=*motion::bundled_model(motion::ModelFamily::A);
            motion_runtime_enter(); assert(engine.init(a.view,arena,sizeof(arena))); motion_runtime_leave();
            while(!stop) {
                int8_t output[2]; motion_runtime_enter(); assert(engine.invoke(a.golden[0].input,output)); motion_runtime_leave();
                assert(!std::memcmp(output,a.golden[0].output,2)); ++invokes;
            }
        });
        while(!invokes) std::this_thread::yield();
        Harness h; h.begin(); h.chunks(); assert(h.finish().state==STORED); h.reboot(); assert(h.count()==1);
        ++h.transfer[0]; h.begin(); h.flash.fail_write=h.flash.writes+1; assert(h.send(h.chunk(0)).body.model_upload_ack.result!=OK);
        stop=true; live.join(); assert(invokes>0); fallback();
    });
    std::printf("PASS %u storage/validation scenarios (including 10 torn-write positions); bundled A still passes 4 golden vectors\n",passed);
    assert(allocated==0);
}
