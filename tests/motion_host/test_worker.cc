#include "worker_platform.h"
#include <cassert>
#include <csetjmp>
#include <cstdarg>
#include <cstring>
#include "motion_ml.h"
bool init_fail, golden_fail, invoke_fail, config_during_invoke;
bool trial_fail;
unsigned invoke_calls;
#include "model_runtime_under_test.cc"
bool motion::CandidateProvider::parse(const uint8_t *,size_t,char[64]) {
    descriptor=*motion::bundled_model(motion::ModelFamily::B); return true;
}
#include "motion_worker_under_test.cc"

static std::jmp_buf stopped;
static const char *scenario;
static int64_t now_us=1000000;
static unsigned produced, predictions, received_results, allocation_calls;
static unsigned head, count, qsize, capacity;
static uint8_t *qstorage;
static bool heap_fail, task_fail, queue_fail;
static void (*task_fn)(void*);
static motion_result_t outputs[100];
static unsigned change_replies, successful_changes;
static uint32_t last_switch_sequence;
static bool is(const char *name) { return !std::strcmp(scenario,name); }
static bool switching() { return !std::strncmp(scenario,"switch_",7); }
static bool failed_switch() { return is("switch_init_fail")||is("switch_golden_fail")||is("switch_trial_fail")||is("switch_heap_fail"); }
void on_engine_init(const motion::ModelView &view) {
    if(!is("switch_timing") || !std::strstr(view.version,"model-b")) return;
    // Acquisition runs while candidate initialization occupies the ML worker.
    // All five zero-wait submissions must be discarded at the commit boundary.
    for(unsigned i=0;i<5;++i) {
        now_us+=100000;
        motion_sample_t s{}; s.boot_id=7; s.sequence=++produced;
        s.generation=control.generation; s.interval_ms=100; s.acquired_us=now_us; s.valid=true; s.xyz[2]=1;
        assert(motion_ml_submit(&s));
    }
}
int64_t esp_timer_get_time() { return now_us; }
static bool backend_logged;
void test_log(const char *format,...) {
    if(!std::strstr(format,"backend=%s")) return;
    char line[512]; va_list args; va_start(args,format);
    std::vsnprintf(line,sizeof(line),format,args); va_end(args);
    assert(std::strstr(line,"backend=BACKEND_TFLM_REFERENCE backend_id=1"));
    backend_logged=true;
}
void *heap_caps_aligned_alloc(size_t alignment,size_t size,unsigned caps) {
    alignas(16) static uint8_t bytes[2][16384];
    alignas(16) static uint8_t packages[2][256];
    assert(alignment==16 && (size==sizeof(bytes[0])||size==256) && caps==3);
    ++allocation_calls;
    if(heap_fail) return nullptr;
    static unsigned arenas, copies;
    return size==256?packages[copies++%2]:bytes[arenas++%2];
}
void heap_caps_free(void *) {}
size_t heap_caps_get_free_size(unsigned) { return 50000; }
size_t heap_caps_get_minimum_free_size(unsigned) { return 45000; }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t) { return 3000; }
QueueHandle_t xQueueCreateStatic(unsigned length,unsigned size,uint8_t* data,StaticQueue_t*) {
    assert(length==32); capacity=length; qsize=size; qstorage=data;
    return queue_fail ? nullptr : data;
}
void vQueueDelete(QueueHandle_t) { count=0; }
BaseType_t xQueueSend(QueueHandle_t,const void* value,TickType_t wait) {
    assert(wait==0);
    if(count==capacity) return pdFALSE;
    std::memcpy(qstorage+((head+count)%capacity)*qsize,value,qsize); ++count;
    return pdTRUE;
}
BaseType_t xTaskCreatePinnedToCore(void(*fn)(void*),const char* name,unsigned stack,void*,unsigned priority,TaskHandle_t* handle,int core) {
    assert(!std::strcmp(name,"motion_ml") && stack==6144 && priority==3 && core==1);
    task_fn=fn; *handle=(void*)1; return task_fail ? pdFALSE : pdPASS;
}
static void result_callback(const motion_result_t *r) {
    const auto &view=motion::bundled_model(r->model_identity.family==1?motion::ModelFamily::A:motion::ModelFamily::B)->view;
    assert(!std::strcmp(r->model_version,view.version));
    assert(received_results<100); outputs[received_results++]=*r;
    if(r->status==MOTION_PREDICTION) {
        ++predictions;
        assert(r->model_ready && r->class_id==0 && r->last_sequence-r->first_sequence==19);
        assert(r->window_end_us-r->window_start_us<=19*150000);
        assert(r->generation==control.generation);
        assert(r->logits[0] == (1 - view.output_zero_point) * view.output_scale);
        assert(r->logits[1] == (0 - view.output_zero_point) * view.output_scale);
        if(is("live") || is("default")) assert(r->last_sequence==20+(predictions-1)*10);
        if(is("default")) assert(r->generation==0 && r->interval_ms==100);
        if(is("interval_restore")) assert(r->generation==2 && r->first_sequence>=30);
        if(is("invalid")) assert(r->first_sequence>10);
        if(is("gap") || is("reboot") || is("sequence") || is("configure")) assert(r->first_sequence>=10);
        if(switching()) {
            assert(r->model_epoch==successful_changes);
            if(successful_changes) assert(r->first_sequence>last_switch_sequence);
            if(is("switch_old_sample")) assert(r->first_sequence>=12);
        }
    }
}
static void request_switch(bool b=true) {
    static const uint8_t package[256]={};
    motion_model_command_t command{};
    command.target=motion::bundled_identity(*motion::bundled_model(b?motion::ModelFamily::B:motion::ModelFamily::A));
    if(b) { command.target.source=2; command.package=package; command.package_size=sizeof(package); }
    motion_model_state_t current; motion_ml_model_state(&current); command.expected_epoch=current.epoch;
    assert(motion_ml_request_model(&command));
    assert(!motion_ml_request_model(&command)); // Bounded single pending handoff.
    motion_model_reply_t reply; assert(!motion_ml_take_model_reply(&reply));
}
void change_config_in_invoke() {
    if(is("switch_invoke")) {
        request_switch();
        motion_model_state_t s; motion_ml_model_state(&s);
        assert(s.active.family==1 && s.epoch==0); // No swap inside active Invoke.
    } else motion_ml_configure(100,2,true);
}
BaseType_t xQueueReceive(QueueHandle_t,void* value,TickType_t wait) {
    assert(wait==10);
    if(is("switch_auto_rollback") && published_model.epoch==2 && successful_changes==1) {
        assert(published_model.active.family==1&&published_model.rolled_back);
        successful_changes=2; last_switch_sequence=produced;
    }
    motion_model_reply_t reply;
    if(motion_ml_take_model_reply(&reply)) {
        ++change_replies;
        assert(reply.result==(failed_switch()?MOTION_CHANGE_FAILED:MOTION_CHANGE_OK));
        if(reply.result==MOTION_CHANGE_OK) { ++successful_changes; last_switch_sequence=produced; }
        assert(reply.state.epoch==successful_changes);
        init_fail=golden_fail=trial_fail=heap_fail=false;
    }
    if(!count) {
        if(++produced>(switching()?100U:60U)) std::longjmp(stopped,1);
        if(switching() && !is("switch_invoke") && produced==10) {
            request_switch();
            init_fail=is("switch_init_fail"); golden_fail=is("switch_golden_fail");
            trial_fail=is("switch_trial_fail"); heap_fail=is("switch_heap_fail");
        }
        if(is("switch_repeat") && (produced==40||produced==70)) request_switch(produced==70);
        if(is("switch_auto_rollback") && produced==40) invoke_fail=true;
        if(is("interval_restore") && produced==10) motion_ml_configure(500,1,true);
        if(is("interval_restore") && produced==30) motion_ml_configure(100,2,true);
        now_us+=(int64_t)control.interval_ms*1000;
        if(is("no_data")) return pdFALSE;
        motion_sample_t s={};
        s.boot_id=7; s.sequence=produced; s.generation=control.generation;
        s.interval_ms=control.interval_ms; s.acquired_us=now_us; s.valid=true; s.xyz[2]=1;
        if(produced==10) {
            if(is("invalid")) s.valid=false;
            if(is("gap")) { now_us+=50001; s.acquired_us=now_us; }
            if(is("configure")) { motion_ml_configure(100,1,true); s.generation=1; }
            if(is("disabled")) motion_ml_configure(100,1,false);
        }
        if(is("reboot") && produced==10) s.boot_id=8;
        if(is("sequence") && produced>=10) ++s.sequence;
        if(is("stale")) s.acquired_us-=150001;
        if(is("switch_old_sample") && produced==11) s.acquired_us-=100000; // same timestamp as commit: conservatively discard
        assert(motion_ml_submit(&s));
    }
    std::memcpy(value,qstorage+head*qsize,qsize); head=(head+1)%capacity; --count;
    return pdTRUE;
}
int main(int argc,char**argv) {
    assert(argc==2); scenario=argv[1];
    task_fail=is("task_fail"); queue_fail=is("queue_fail"); heap_fail=is("heap_fail");
    init_fail=is("init_fail"); golden_fail=is("golden_fail"); invoke_fail=is("invoke_fail");
    config_during_invoke=is("config_invoke")||is("switch_invoke");
    if(task_fail || queue_fail) { assert(motion_ml_start(7,result_callback)==ESP_ERR_NO_MEM); assert(!queue); return 0; }
    assert(motion_ml_start(7,result_callback)==ESP_OK);
    assert(motion_ml_start(7,result_callback)==ESP_ERR_INVALID_STATE);
    if(!is("unavailable")) motion_ml_set_sensor_available(true);
    if(is("non_100ms")) motion_ml_configure(500,1,true);
    if(is("overflow")) {
        motion_sample_t s={}; s.boot_id=7; s.interval_ms=100; s.valid=true; s.acquired_us=now_us;
        for(unsigned i=0;i<32;++i) { s.sequence=i; assert(motion_ml_submit(&s)); }
        assert(!motion_ml_submit(&s) && control.drops==1);
    }
    if(!setjmp(stopped)) task_fn(nullptr);
    assert(backend_logged);
    if(switching()) {
        assert(change_replies==(is("switch_repeat")?3U:1U));
        assert(predictions>0 && control.drops==0 && control.interval_ms==100 && control.generation==0);
        assert(successful_changes==(failed_switch()?0U:is("switch_repeat")?3U:is("switch_auto_rollback")?2U:1U));
        std::printf("Worker %s: PASS (%u predictions, %u control replies)\n",scenario,predictions,change_replies);
        return 0;
    }
    assert(allocation_calls==(is("invoke_fail")?2U:1U));
    if(is("live") || is("default") || is("overflow")) assert(predictions==5);
    else if(is("interval_restore")) assert(predictions==2);
    else if(is("invalid") || is("gap") || is("reboot") || is("sequence") || is("configure")) assert(predictions==4);
    else if(is("config_invoke")) assert(predictions==3 && invoke_calls==4);
    else if(is("invoke_fail")) assert(predictions==3 && invoke_calls==4);
    else assert(predictions==0);
    if(is("non_100ms")) assert(outputs[received_results-1].status==MOTION_WAIT_100MS);
    if(is("disabled")) assert(outputs[received_results-1].status==MOTION_SENSOR_DISABLED);
    if(heap_fail || init_fail || golden_fail) assert(outputs[received_results-1].status==MOTION_INIT_FAILED);
    if(is("no_data")) assert(outputs[received_results-1].status==MOTION_NO_DATA);
    if(is("unavailable")) assert(outputs[received_results-1].status==MOTION_SENSOR_UNAVAILABLE);
    std::printf("Worker %s: PASS (%u predictions, %u allocations)\n",scenario,predictions,allocation_calls);
}
