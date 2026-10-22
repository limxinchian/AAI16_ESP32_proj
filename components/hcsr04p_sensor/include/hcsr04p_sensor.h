#pragma once

#include "esp_err.h"

// Week 6 wiring: HC-SR04P powered from 3V3 (not the 5V-only HC-SR04).
#define HCSR04P_TRIG_GPIO 4
#define HCSR04P_ECHO_GPIO 5
#define HCSR04P_MIN_CYCLE_MS 60
#define HCSR04P_SAMPLE_MS 200
#define HCSR04P_ECHO_TIMEOUT_MS 30
#define HCSR04P_MIN_CM 2
#define HCSR04P_MAX_CM 400

typedef struct {
    float distance_cm;
    float echo_us;
} hcsr04p_reading_t;

// Single reader task owns initialization and reads. Init success proves only
// GPIO/capture setup; a valid read is required to prove a sensor response.
// Failed initialization is terminal until reboot; no automatic recovery.
esp_err_t hcsr04p_sensor_init(void);
// Blocks only the caller on an ISR queue, with a 30 ms response deadline.
// At 100 Hz the RTOS wait is 4 ticks (30..40 ms plus scheduling latency).
// Output is cleared on failure. Calls <60 ms after a trigger are rejected.
esp_err_t hcsr04p_sensor_read(hcsr04p_reading_t *out);
