#include "platform.h"
#include <assert.h>
#include <stdarg.h>
#include <string.h>

int64_t test_now_us;
unsigned test_log_calls;
char test_last_log[512];
unsigned test_error_logs, test_recovery_logs, test_summary_logs, test_config_logs;
char test_last_level;
static char history[64][512];
static unsigned history_next;
bool test_logs_contain(const char *text)
{
    for (unsigned i = 0; i < 64; ++i) if (strstr(history[i], text)) return true;
    return false;
}
static unsigned lock_depth;

void test_enter_critical(void) { assert(lock_depth == 0); ++lock_depth; }
void test_exit_critical(void) { assert(lock_depth == 1); --lock_depth; }
int64_t esp_timer_get_time(void) { assert(lock_depth == 0); return test_now_us; }
const char *esp_err_to_name(esp_err_t error)
{
    assert(lock_depth == 0);
    return error == ESP_OK ? "ESP_OK" : "TEST_ERROR";
}
void test_log(char level, const char *tag, const char *format, ...)
{
    assert(lock_depth == 0);
    va_list args;
    va_start(args, format);
    int length = vsnprintf(test_last_log, sizeof(test_last_log), format, args);
    va_end(args);
    assert(length >= 0 && (size_t)length < sizeof(test_last_log));
    assert(!strchr(test_last_log, '\n') && !strchr(test_last_log, '\033'));
    if (!strcmp(tag, "SENSOR")) {
        assert(strstr(test_last_log, " | DATA | ") || strstr(test_last_log, " | ERROR | ") ||
               strstr(test_last_log, " | HEALTH | "));
    }
    strcpy(history[history_next++ % 64], test_last_log);
    test_last_level = level;
    test_error_logs += level == 'W' || level == 'E';
    test_recovery_logs += strstr(test_last_log, "recovery=READY") != NULL;
    test_summary_logs += !strcmp(tag, "SENSORS");
    test_config_logs += strstr(test_last_log, "config_period_ms=") != NULL;
    test_log_calls++;
}
