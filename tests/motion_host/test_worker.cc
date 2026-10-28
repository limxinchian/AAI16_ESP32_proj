#include "worker_platform.h"
#include <cassert>
#include <csetjmp>
#include <cstring>
#include "motion_ml.h"
bool init_fail, golden_fail, invoke_fail, config_during_invoke;
unsigned invoke_calls;
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
static bool is(const char *name) { return !std::strcmp(scenario,name); }
int64_t esp_timer_get_time() { return now_us; }
void test_log(const char*,...) {}
void *heap_caps_aligned_alloc(size_t alignment,size_t size,unsigned caps) {
    alignas(16) static uint8_t bytes[16384];
    assert(alignment==16 && size==sizeof(bytes) && caps==3);
    ++allocation_calls;
    return heap_fail ? nullptr : bytes;
}
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
    assert(received_results<100); outputs[received_results++]=*r;
    if(r->status==MOTION_PREDICTION) {
        ++predictions;
        assert(r->model_ready && r->class_id==0 && r->last_sequence-r->first_sequence==19);
        assert(r->window_end_us-r->window_start_us<=19*150000);
        assert(r->generation==control.generation);
        if(is("live") || is("default")) assert(r->last_sequence==20+(predictions-1)*10);
        if(is("default")) assert(r->generation==0 && r->interval_ms==100);
        if(is("interval_restore")) assert(r->generation==2 && r->first_sequence>=30);
        if(is("invalid")) assert(r->first_sequence>10);
        if(is("gap") || is("reboot") || is("sequence") || is("configure")) assert(r->first_sequence>=10);
    }
}
void change_config_in_invoke() { motion_ml_configure(100,2,true); }
BaseType_t xQueueReceive(QueueHandle_t,void* value,TickType_t wait) {
    assert(wait==10);
    if(!count) {
        if(++produced>60) std::longjmp(stopped,1);
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
        assert(motion_ml_submit(&s));
    }
    std::memcpy(value,qstorage+head*qsize,qsize); head=(head+1)%capacity; --count;
    return pdTRUE;
}
int main(int argc,char**argv) {
    assert(argc==2); scenario=argv[1];
    task_fail=is("task_fail"); queue_fail=is("queue_fail"); heap_fail=is("heap_fail");
    init_fail=is("init_fail"); golden_fail=is("golden_fail"); invoke_fail=is("invoke_fail");
    config_during_invoke=is("config_invoke");
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
    assert(allocation_calls==1);
    if(is("live") || is("default") || is("overflow")) assert(predictions==5);
    else if(is("interval_restore")) assert(predictions==2);
    else if(is("invalid") || is("gap") || is("reboot") || is("sequence") || is("configure")) assert(predictions==4);
    else if(is("config_invoke")) assert(predictions==3 && invoke_calls==4);
    else assert(predictions==0);
    if(invoke_fail) assert(invoke_calls==1);
    if(is("non_100ms")) assert(outputs[received_results-1].status==MOTION_WAIT_100MS);
    if(is("disabled")) assert(outputs[received_results-1].status==MOTION_SENSOR_DISABLED);
    if(heap_fail || init_fail || golden_fail) assert(outputs[received_results-1].status==MOTION_INIT_FAILED);
    if(is("no_data")) assert(outputs[received_results-1].status==MOTION_NO_DATA);
    if(is("unavailable")) assert(outputs[received_results-1].status==MOTION_SENSOR_UNAVAILABLE);
    std::printf("Worker %s: PASS (%u predictions, one startup allocation)\n",scenario,predictions);
}
