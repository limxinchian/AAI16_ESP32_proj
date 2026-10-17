#include "platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/ld2450_sensor/ld2450_sensor.c"

static bool installed, delete_fail, length_fail, read_fail, flush_fail, post_read_error;
static unsigned init_step, fail_step, installs, deletes, waits, flushes;
static uint8_t input[2048]; static size_t used;
static uart_event_t events[32]; static unsigned queued;
static const uint8_t idle[30] = {0xaa,0xff,3,0,[28]=0x55,[29]=0xcc};
static void event(uart_event_type_t type) { assert(queued < 32); events[queued++].type = type; }
static void append(const uint8_t *data, size_t length) { assert(used+length <= sizeof(input)); memcpy(input+used,data,length); used+=length; }
bool uart_is_driver_installed(uart_port_t port) { assert(port == 1); return installed; }
esp_err_t uart_driver_install(uart_port_t port, int rx, int tx, int count, QueueHandle_t *queue, int flags)
{
    assert(port == 1 && rx == 1024 && tx == 0 && count == 16 && !flags); installs++;
    if (++init_step == fail_step) return ESP_ERR_NO_MEM;
    installed = true; *queue = (void *)1; return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t port) { assert(port == 1 && installed); deletes++; if (delete_fail) return ESP_FAIL; installed=false; return ESP_OK; }
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *c)
{
    assert(port == 1 && installed && c->baud_rate == 256000 && c->data_bits == 8 && c->stop_bits == 1);
    assert(!c->parity && !c->flow_ctrl && c->source_clk == UART_SCLK_DEFAULT);
    return ++init_step == fail_step ? ESP_FAIL : ESP_OK;
}
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts)
{ assert(port == 1 && tx == 17 && rx == 18 && rts == -1 && cts == -1); return ++init_step == fail_step ? ESP_FAIL : ESP_OK; }
esp_err_t uart_get_buffered_data_len(uart_port_t port, size_t *n)
{ assert(port == 1); if (length_fail) return ESP_FAIL; *n=used; return ESP_OK; }
esp_err_t uart_flush_input(uart_port_t port)
{ assert(port == 1); flushes++; if (flush_fail) return ESP_FAIL; used=0; return ESP_OK; }
int uart_read_bytes(uart_port_t port, void *data, uint32_t length, TickType_t wait)
{
    assert(port == 1 && !wait && length == 256);
    if (read_fail) return -1;
    size_t n = used < length ? used : length;
    memcpy(data,input,n); used-=n; memmove(input,input+n,used);
    if (post_read_error) event(UART_FRAME_ERR);
    test_now_us += 5; return n;
}
unsigned uxQueueMessagesWaiting(QueueHandle_t q) { assert(q == (void*)1); return queued; }
BaseType_t xQueueReceive(QueueHandle_t q, void *out, TickType_t wait)
{
    assert(q == (void*)1 && (wait == 0 || wait == 10));
    if (queued) { *(uart_event_t*)out=events[0]; memmove(events,events+1,--queued*sizeof(*events)); return pdTRUE; }
    if (wait) { waits++; test_now_us+=wait*10000LL; }
    return pdFALSE;
}
static void reset(void)
{
    s_owned=s_initialized=installed=false; s_events=NULL; used=queued=0;
    s_diag=(ld2450_diagnostics_t){0}; s_parser=(ld2450_parser_t){0};
    init_step=fail_step=installs=deletes=waits=flushes=0;
    delete_fail=length_fail=read_fail=flush_fail=post_read_error=false; test_now_us=0;
}
int main(void)
{
    ld2450_report_t r; unsigned frames=123;
    assert(ld2450_sensor_receive(&r,&frames)==ESP_ERR_INVALID_STATE && !frames);
    assert(ld2450_sensor_discard_pending()==ESP_ERR_INVALID_STATE);
    assert(ld2450_sensor_receive(NULL,&frames)==ESP_ERR_INVALID_ARG);
    assert(ld2450_sensor_receive(&r,NULL)==ESP_ERR_INVALID_ARG);
    for (unsigned step=1; step<=3; ++step) {
        reset(); fail_step=step; assert(ld2450_sensor_init()!=ESP_OK);
        assert(!s_initialized && !installed && deletes==(step>1));
    }
    reset(); installed=true; assert(ld2450_sensor_init()==ESP_ERR_INVALID_STATE && !installs && !deletes);
    reset(); fail_step=2; delete_fail=true; assert(ld2450_sensor_init()==ESP_FAIL && s_owned);
    assert(ld2450_sensor_init()==ESP_ERR_INVALID_STATE && installs==1);
    reset(); assert(ld2450_sensor_init()==ESP_OK); assert(ld2450_sensor_init()==ESP_OK && installs==1);
    assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && !frames && waits==1 && test_now_us==100005);
    append(idle,13); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && !frames);
    append(idle+13,17); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && frames==1 && !r.count);
    for (unsigned i=0;i<20;i++) append(idle,30);
    unsigned total=0;
    while (used) { assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && frames<=9); total+=frames; }
    assert(total==20 && s_diag.max_buffered_bytes==600 && s_diag.max_processing_us==5);
    // Noise and bad footer are counted, and a subsequent valid frame survives.
    uint8_t bad[30]; memcpy(bad,idle,30); bad[29]=0;
    append(bad,30); append(idle,30);
    assert(ld2450_sensor_receive(&r,&frames)==ESP_ERR_INVALID_RESPONSE && frames==1);
    assert(s_parser.framing_errors==1 && s_parser.discarded_bytes==30);
    append(idle,12); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && !frames);
    test_now_us+=300000;
    assert(ld2450_sensor_receive(&r,&frames)==ESP_ERR_INVALID_RESPONSE && s_parser.partial_timeouts==1);
    // Every UART transport error rejects the entire batch and drops partial state.
    for (uart_event_type_t type=UART_FIFO_OVF; type<=UART_BREAK; type++) {
        append(idle,10); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && !frames);
        append(idle+10,20); event(type);
        assert(ld2450_sensor_receive(&r,&frames)==ESP_FAIL && !frames && !used && !s_parser.used);
        append(idle,30); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && frames==1);
    }
    assert(s_diag.rx_overflows==2 && s_diag.uart_errors==3);
    append(idle,30); for (unsigned i=0;i<16;++i) event(UART_DATA);
    assert(ld2450_sensor_receive(&r,&frames)==ESP_FAIL && !frames && s_diag.event_overflows==1);
    append(idle,30); post_read_error=true;
    assert(ld2450_sensor_receive(&r,&frames)==ESP_FAIL && !frames); post_read_error=false;
    append(idle,30); read_fail=true;
    assert(ld2450_sensor_receive(&r,&frames)==ESP_FAIL && !frames); read_fail=false;
    length_fail=true; assert(ld2450_sensor_receive(&r,&frames)==ESP_FAIL && !frames); length_fail=false;
    flush_fail=true; event(UART_BUFFER_FULL); assert(ld2450_sensor_receive(&r,&frames)==ESP_FAIL); flush_fail=false;
    append(idle,30); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && frames==1);
    // Resume discards both an incomplete parser frame and queued full frames,
    // accounts for old UART errors, and retains initialization/counter history.
    append(idle,13); assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && !frames);
    append(idle+13,17); append(idle,30); event(UART_FRAME_ERR);
    unsigned old_frames=s_parser.frames, old_errors=s_diag.uart_errors;
    assert(ld2450_sensor_discard_pending()==ESP_OK);
    assert(!used && !s_parser.used && !queued && installed && s_initialized);
    assert(s_parser.frames==old_frames && s_diag.uart_errors==old_errors+1);
    assert(ld2450_sensor_receive(&r,&frames)==ESP_OK && !frames);
    // A frame already on the wire can leave its tail after the flush. It must
    // be discarded, with only the subsequent complete frame returned as valid.
    append(idle+13,17); append(idle,30);
    assert(ld2450_sensor_receive(&r,&frames)==ESP_ERR_INVALID_RESPONSE && frames==1);
    assert(s_parser.frames==old_frames+1 && !r.count);
    flush_fail=true; assert(ld2450_sensor_discard_pending()==ESP_FAIL); flush_fail=false;
    length_fail=true; assert(ld2450_sensor_discard_pending()==ESP_FAIL); length_fail=false;
    ld2450_diagnostics_t d; ld2450_sensor_get_diagnostics(&d);
    assert(d.frames==s_parser.frames && d.discarded_bytes>200 && installs==1 && !deletes);
    assert(ld2450_sensor_deinit()==ESP_OK && !installed && deletes==1);
    assert(ld2450_sensor_deinit()==ESP_OK && deletes==1);
    puts("PASS: LD2450 UART ownership/init failures, bounded receive, silence, fragmentation, overflow and error recovery");
}
