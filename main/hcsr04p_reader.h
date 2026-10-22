#pragma once

// Call once from app_main. Fixed 200 ms acquisition with latest-valid MQTT
// handoff; runtime configuration/control notifications belong to a later phase.
void hcsr04p_reader_start(void);
