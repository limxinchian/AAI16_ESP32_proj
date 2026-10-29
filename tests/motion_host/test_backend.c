#include "motion_backend.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const char *names[] = {"BACKEND_UNSPECIFIED", "BACKEND_TFLM_REFERENCE",
                          "BACKEND_ESP_NN_ANSI", "BACKEND_ESP_NN_OPTIMIZED"};
    assert(motion_inference_backend() == TEST_EXPECTED_BACKEND);
    assert(!strcmp(motion_inference_backend_name(), names[TEST_EXPECTED_BACKEND]));
    printf("PASS backend=%s backend_id=%u\n", motion_inference_backend_name(),
           (unsigned)motion_inference_backend());
    return 0;
}
