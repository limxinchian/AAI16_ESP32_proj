#include "model_upload.h"
#include "upload_store.h"
#include "upload_ingress.h"
#include "motion_ml.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pb_decode.h"
#include <cstring>
#include <new>
#include <inttypes.h>

namespace {
constexpr char TAG[]="MODEL_UPLOAD";
class PartitionFlash final: public upload::Flash {
    const esp_partition_t *part_[2]={};
    esp_partition_mmap_handle_t mapping_=0;
    bool mapped_=false;
    bool range(unsigned f,uint32_t offset,size_t n) {
        return f>=1&&f<=2&&part_[f-1]&&offset<=part_[f-1]->size&&n<=part_[f-1]->size-offset;
    }
    bool timed(esp_err_t result,int64_t start,bool erase) {
        uint32_t us=esp_timer_get_time()-start;
        auto &peak=erase?max_erase_us:max_write_us;
        if(us>peak) peak=us;
        if(us>20000) ESP_LOGW(TAG,"flash_%s_us=%" PRIu32 "; inspect GY-511 gaps",erase?"erase":"write",us);
        // Sector erase may suspend BOTH CPUs' caches. Splitting calls and yielding
        // bounds contiguous work, but is not a hardware timing guarantee.
        vTaskDelay(1);
        return result==ESP_OK;
    }
public:
    uint32_t max_erase_us=0,max_write_us=0;
    void init() {
        for(unsigned f=1;f<=2;++f) {
            auto *p=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,static_cast<esp_partition_subtype_t>(f==1?0x40:0x41),f==1?"model_a":"model_b");
            if(p && p->size==0x300000 && p->address==(f==1?0x410000U:0x710000U) && !p->encrypted) part_[f-1]=p;
            if(part_[f-1]) ESP_LOGI(TAG,"partition=%s address=0x%" PRIx32 " bytes=%" PRIu32 " bank_bytes=%" PRIu32,p->label,p->address,p->size,p->size/2);
            else ESP_LOGE(TAG,"Model %c staging disabled: partition layout/encryption mismatch",f==1?'A':'B');
        }
    }
    uint32_t capacity(unsigned f) override { return f>=1&&f<=2&&part_[f-1]?part_[f-1]->size:0; }
    bool read(unsigned f,uint32_t o,void *p,size_t n) override { return range(f,o,n)&&esp_partition_read(part_[f-1],o,p,n)==ESP_OK; }
    bool erase(unsigned f,uint32_t o,size_t n) override {
        if(!range(f,o,n)||o%4096||n!=4096) return false;
        int64_t start=esp_timer_get_time(); auto err=esp_partition_erase_range(part_[f-1],o,n); return timed(err,start,true);
    }
    bool write(unsigned f,uint32_t o,const void *p,size_t n) override {
        if(!range(f,o,n)||n>512) return false;
        int64_t start=esp_timer_get_time(); auto err=esp_partition_write(part_[f-1],o,p,n); return timed(err,start,false);
    }
    const uint8_t *map(unsigned f,uint32_t o,size_t n) override {
        if(mapped_||!range(f,o,n)) return nullptr;
        const void *p=nullptr;
        if(esp_partition_mmap(part_[f-1],o,n,ESP_PARTITION_MMAP_DATA,&p,&mapping_)!=ESP_OK) return nullptr;
        mapped_=true; return static_cast<const uint8_t*>(p);
    }
    void unmap() override { if(mapped_) esp_partition_munmap(mapping_); mapped_=false; }
} flash;
using upload::Frame;
StaticQueue_t queue_control;
uint8_t queue_storage[4*sizeof(Frame)];
QueueHandle_t queue;
model_upload_reply_fn publish_reply;
uint32_t boot;
uint8_t *arena;
portMUX_TYPE rx_lock=portMUX_INITIALIZER_UNLOCKED;
upload::Ingress rx;
class WorkerControl final: public upload::ModelControl {
public:
    motion_model_state_t snapshot() override { motion_model_state_t s; motion_ml_model_state(&s); return s; }
    motion_model_reply_t change(const motion_model_command_t &command) override {
        motion_model_reply_t reply{};
        if(!motion_ml_request_model(&command)) {
            reply.result=MOTION_CHANGE_BUSY; reply.state=snapshot(); std::strcpy(reply.detail,"ml_worker_busy_or_unavailable"); return reply;
        }
        // The accepted command borrows our flash map until this completion.
        // Never time out and unmap bytes while the worker may still copy them.
        while(!motion_ml_take_model_reply(&reply)) vTaskDelay(1);
        return reply;
    }
} runtime;

void run(void *) {
    flash.init();
    // Large protocol objects and frames live in worker-owned static storage.
    static upload::Store store(flash,boot,arena,MOTION_CANDIDATE_ARENA,&runtime);
    static Frame frame;
    static aai16_Envelope request,reply;
    static uint8_t encoded[AAI16_FRAME_MAX];
    store.recover();
    unsigned recovered=0;
    for(unsigned f=1;f<=2;++f) for(unsigned b=0;b<2;++b) if(store.stored(f,b).valid) {
        ++recovered;
        ESP_LOGI(TAG,"recovered model=%c version=%s bank=%u golden=4/4; candidate only",f==1?'A':'B',store.stored(f,b).manifest.version,b);
    }
    ESP_LOGI(TAG,"staging ready recovered=%u max_package=%u chunk=%u timeout_ms=%u; active model unchanged",recovered,AAI16_PACKAGE_MAX,AAI16_CHUNK_MAX,upload::kTimeoutMs);
    motion_model_state_t reported{};
    bool have_reported=false;
    while(true) {
        if(xQueueReceive(queue,&frame,pdMS_TO_TICKS(250))!=pdTRUE) {
            store.tick(esp_timer_get_time()/1000);
            auto current=runtime.snapshot();
            if(current.active.family && (!have_reported || reported.epoch!=current.epoch ||
                reported.ready!=current.ready || reported.rolled_back!=current.rolled_back || std::strcmp(reported.detail,current.detail))) {
                request=aai16_Envelope_init_zero;
                request.which_body=aai16_Envelope_model_control_tag;
                request.body.model_control.operation=aai16_ModelOperation_MODEL_OP_GET_STATUS;
                store.handle(request,esp_timer_get_time()/1000,reply);
                size_t n=0;
                if(aai16_protocol_encode(&reply,encoded,sizeof(encoded),&n)==aai16_ResultCode_RESULT_OK && publish_reply(true,encoded,n)) { reported=current; have_reported=true; }
            }
            continue;
        }
        auto code=aai16_protocol_decode(frame.bytes,frame.length,&request);
        if(code!=aai16_ResultCode_RESULT_OK) {
            // Decode only for a rejection correlation, never for execution. The
            // production strict decoder above remains the admission boundary.
            request=aai16_Envelope_init_zero;
            auto stream=pb_istream_from_buffer(frame.bytes,frame.length);
            if(!pb_decode(&stream,aai16_Envelope_fields,&request)||!request.has_request_id||!request.request_id) continue;
            if(!upload::frame_accepts(frame,request)) continue;
            bool id_ok=(request.which_body==aai16_Envelope_model_control_tag&&request.body.model_control.has_transfer_id&&request.body.model_control.transfer_id.size==16)||
                (request.which_body==aai16_Envelope_model_upload_chunk_tag&&request.body.model_upload_chunk.has_transfer_id&&request.body.model_upload_chunk.transfer_id.size==16);
            bool model_control=request.which_body==aai16_Envelope_model_control_tag && request.body.model_control.has_operation &&
                request.body.model_control.operation>=aai16_ModelOperation_MODEL_OP_ACTIVATE && request.body.model_control.operation<=aai16_ModelOperation_MODEL_OP_LIST;
            if(!id_ok && !model_control && request.which_body!=aai16_Envelope_model_selection_tag) continue;
            store.invalid(request,code,reply);
        } else {
            if(!upload::frame_accepts(frame,request)) continue;
            store.handle(request,esp_timer_get_time()/1000,reply);
        }
        bool status=reply.which_body==aai16_Envelope_model_status_tag;
        size_t n=0;
        if(aai16_protocol_encode(&reply,encoded,sizeof(encoded),&n)==aai16_ResultCode_RESULT_OK) {
            if(!publish_reply(status,encoded,n)) ESP_LOGW(TAG,"reply mailbox full; client must retry request");
        } else ESP_LOGE(TAG,"response encoding failed");
        if(status) {
            const auto &s=reply.body.model_status;
            ESP_LOGI(TAG,"request=%" PRIu64 " result=%d active=%s source=%d epoch=%" PRIu32 " detail=%s",
                reply.request_id,int(s.result),s.has_active?s.active.version:"none",int(s.active.source),s.model_epoch,s.detail);
        } else {
            const auto &a=reply.body.model_upload_ack;
            ESP_LOGI(TAG,"request=%" PRIu64 " generation=%" PRIu32 " state=%d result=%d next=%" PRIu32 " detail=%s",reply.request_id,reply.generation,int(a.state),int(a.result),a.next_offset,a.detail);
            if(a.state==aai16_UploadState_UPLOAD_STORED) {
                const auto *candidate=store.completed_candidate();
                if(candidate) ESP_LOGI(TAG,"stored model=%c version=%s bank=%u model_bytes=%" PRIu32 " golden=4/4; candidate only",
                    candidate->manifest.family==aai16_ModelFamily_MODEL_A?'A':'B',candidate->manifest.version,candidate->bank,candidate->manifest.model_size);
                ESP_LOGI(TAG,"candidate STORED; active unchanged; flash_max_erase_us=%" PRIu32 " flash_max_write_us=%" PRIu32 " stack_min_bytes=%u heap_min_bytes=%u",
                    flash.max_erase_us,flash.max_write_us,(unsigned)uxTaskGetStackHighWaterMark(nullptr),
                    (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
            }
        }
    }
}
}
extern "C" bool model_upload_start(uint32_t boot_id,model_upload_reply_fn callback) {
    if(queue||!callback) return false;
    arena=static_cast<uint8_t*>(heap_caps_aligned_alloc(16,MOTION_CANDIDATE_ARENA,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
    if(!arena) return false;
    queue=xQueueCreateStatic(4,sizeof(Frame),queue_storage,&queue_control);
    boot=boot_id; publish_reply=callback;
    if(xTaskCreatePinnedToCore(run,"model_upload",12288,nullptr,2,nullptr,1)!=pdPASS) {
        vQueueDelete(queue); queue=nullptr; heap_caps_free(arena); arena=nullptr; return false;
    }
    return true;
}
extern "C" void model_upload_disconnected() {
    portENTER_CRITICAL(&rx_lock); rx.reset(); portEXIT_CRITICAL(&rx_lock);
    // Keep a complete upload session for bounded reconnect/retry. No flash here.
}
extern "C" void model_upload_receive(const char *topic,size_t topic_length,const void *data,int length,int offset,int total,int id,bool retained) {
    if(!queue) return;
    Frame completed;
    portENTER_CRITICAL(&rx_lock);
    bool ready=rx.feed(topic,topic_length,data,length,offset,total,id,retained,completed);
    portEXIT_CRITICAL(&rx_lock);
    // A dropped frame produces no false success. Same-byte client retry handles
    // bounded queue congestion. Never call queue APIs while holding rx_lock.
    if(ready) xQueueSend(queue,&completed,0);
}
