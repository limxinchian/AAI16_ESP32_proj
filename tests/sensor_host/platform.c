#include "platform.h"
#include <assert.h>
#include <stdarg.h>

int64_t test_now_us;
unsigned test_log_calls;
char test_last_log[512];
static unsigned lock_depth;

void test_enter_critical(void) { assert(lock_depth == 0); ++lock_depth; }
void test_exit_critical(void) { assert(lock_depth == 1); --lock_depth; }
int64_t esp_timer_get_time(void) { assert(lock_depth == 0); return test_now_us; }
const char *esp_err_to_name(esp_err_t error)
{
    assert(lock_depth == 0);
    return error == ESP_OK ? "ESP_OK" : "TEST_ERROR";
}
void test_log(const char *tag, const char *format, ...)
{
    assert(lock_depth == 0);
    va_list args;
    va_start(args, format);
    vsnprintf(test_last_log, sizeof(test_last_log), format, args);
    va_end(args);
    test_log_calls++;
}
