# Requirements traceability (handoff §9 acceptance matrix)

Evidence levels: **H** host-tested (PC, simulated devices) - **T** target-built (ESP-IDF 6.1, esp32s3) - **Q** ran in QEMU -
**B** board-tested. As of 2026-11-04 **nothing is B**. "Gap" says what is still needed for the row to count as done.

| ID | Requirement | Where | H | T | Q | B | Gap |
|---|---|---|---|---|---|---|---|
| AC-01 | Four generic buses (GPIO/I2C/UART/SPI) | `sensor_runtime`, `plat_bus.c` | yes (simulated devices) | yes | scheduler runs all four, sensorless, no hang | no | real transactions on a board; GPIO timing capture |
| AC-02 | Eight sensors, four active, swap A<->B without reflash | `tools/sensor_profiles.py`, config manager, node core | yes: config A->swap one bus->config B | yes | config A applied | no | real sensors; profiles unverified vs datasheets |
| AC-03 | Generic command/read: change bytes/length/address -> different transaction | profiles are data (`Op` list) | yes: mutated profiles change executed ops | yes | - | no | observe on a bus analyzer / real device |
| AC-04 | Independent periods; requested vs actual | `sensor_runtime` scheduler | yes: 20 ms/2 s/100 ms/200 ms give 500/5/100/50 readings per 10 s, `last_period_us` measured, overruns counted | yes | - | no | measure achieved rates on the board; >1 Hz sensor |
| AC-05 | Protobuf change requirement | `protocol/aai16.proto`, `.options` | yes: Python-encoded fixtures decode in nanopb and re-encode byte-identically; oversize/malformed rejected; no JSON on the wire | yes | - | - | none for the core categories; rules/bridge/commands are Protobuf too |
| AC-06 | Reliable config application (stale/invalid rejected with reason, prior config kept, generation/ACK) | `config_manager` | yes (122 checks, 28 single-field negatives, rollback on engine and persist failure) | yes | forbidden pin refused, good config accepted | no | lost-ACK retry policy undecided (STATUS finding 2) |
| AC-07 | Real Motion Model A: GY-511 window -> **on-target** inference -> MQTT | `ml/`, `ml_core`, `plat_ml.cc` | partial: real model, node pipeline matches recorded TFLite tensors (matrix multiply is a test double) | yes | no (no model loaded) | no | run TFLM on the chip with real GY-511 data |
| AC-08 | Real Environment Model B: DHT22, then BME280, same model | same | partial: same as AC-07, incl. DHT22->BME280 switch | yes | - | no | on-chip inference with real sensors |
| AC-09 | Wrong sensor/model rejected before inference; previous mapping kept | `ml_selector` | yes (DHT22 temperature -> motion model rejected; each incompatibility has its own reason; mapping preserved) | yes | - | - | none beyond board run |
| AC-10 | Actual model replacement over the network, no reflash | `model_store`, `plat_storage.c`, `tools/node_client.py` | yes: chunked upload -> verify -> stage -> activate in the other bank; client-built chunks accepted | yes | - | no | real MQTT transfer; flash bank write/mmap on hardware |
| AC-11 | Upload robustness (out of order, duplicate, lost, invalid, corrupt); active model protected | `model_store`, node core | yes: all of these, plus persistence failure after the runtime accepted the model (rollback) | yes | - | - | interrupted Wi-Fi on hardware |
| AC-12 | Rule engine: runtime update, bounded action / inference trigger | `rule_engine` | yes (147 checks incl. pin policy re-checked at fire time) | yes | - | no | GPIO/PWM outputs on a board |
| AC-13 | Serial REPL RUN/STOP/SHOW through the same managers | `repl`, `node_core` | yes | yes | yes (all commands on the real image) | no | - |
| AC-14 | Node-to-node bridge, time-bounded virtual input | `bridge` | yes: virtual sensor 128..255, freshness, loop guards | yes | - | no | two boards via a broker |
| AC-15 | Heartbeat / diagnostics / persistence and restore | `diag`, `config_manager`, `plat_storage.c` | yes: heartbeat fields; restore after reboot on simulated storage | yes | NVS **write** observed; restore **not demonstrable** (QEMU drops flash on restart; control experiment in `tests/qemu_nvs_probe`) | no | persistence across reboot on a board (rules, bridge, ML mapping and model generation are persisted and host-tested) |
| AC-16 | Hardware optimisation (if confirmed): reference vs optimised kernels genuinely used and measured | `sdkconfig`: `CONFIG_NN_OPTIMIZED=y`, `CONFIG_ESP_TFLITE_MICRO_USE_ESP_NN=y` | no | optimised build only | - | no | reference image (ESP-NN off) now builds (385,712 B vs 393,856 B); still needed: compare outputs and timing on the board, supervisor must confirm scope |
| AC-17 | Benchmarks with raw data and method | `docs/demo_and_benchmarks.md` | no | - | - | no | all measurements pending a board |

## Handoff §12 integrity questions

- Existing files reused: none (this folder held only the handoff and the Pico reference); everything is new, FlexiEdgePico informed the rule/REPL/bridge design only.
- Toolchain actually used: ESP-IDF v6.1 (`C:\esp\v6.1`), Xtensa GCC 15.2.0, target esp32s3; TFLM = `espressif/esp-tflite-micro` + ESP-NN via the component manager; nanopb 0.4.9.1.
- Sensor-name logic in application code: none; acquisition is the generic `Op` interpreter. (`plat_bus.c` implements generic bus primitives only.)
- Real inference on hardware: **not yet** (see AC-07/08).
- Open supervisor questions (handoff §10): all still open.
