#include "motion_candidate.h"
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
namespace {
StaticSemaphore_t storage;
SemaphoreHandle_t mutex;
portMUX_TYPE creation_lock = portMUX_INITIALIZER_UNLOCKED;
}
extern "C" void motion_runtime_enter()
{
    portENTER_CRITICAL(&creation_lock);
    if (!mutex) mutex = xSemaphoreCreateMutexStatic(&storage);
    portEXIT_CRITICAL(&creation_lock);
    configASSERT(mutex);
    xSemaphoreTake(mutex, portMAX_DELAY);
}
extern "C" void motion_runtime_leave() { xSemaphoreGive(mutex); }
#else
#include <mutex>
namespace { std::mutex mutex; }
extern "C" void motion_runtime_enter() { mutex.lock(); }
extern "C" void motion_runtime_leave() { mutex.unlock(); }
#endif
