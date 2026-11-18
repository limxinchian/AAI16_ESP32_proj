# Demo procedure and benchmarks

No number in the "measured" tables below is invented: the only figures are ones measured in this repository
(build sizes, host test counts, model sizes). Everything that needs a board is marked **pending**.

## Demo script (needs a board + an MQTT broker, e.g. mosquitto, on the same LAN)

1. Set the node id, Wi-Fi and broker: `idf.py menuconfig` -> "AAI16 node"; `idf.py -p COMx flash monitor`.
2. Configuration A: `python tools/node_client.py --broker <ip> --node node1 config A --gen 1` -> expect ADMITTED then APPLIED;
   `listen` shows `reading/<id>` messages. Swap one bus: build a `ConfigApply` with only the new profile (generation 2).
3. Configuration B: `... config B --gen 3`.
4. Model upload: `... upload ml/artifacts/motion/motion.amdl --kind motion --transfer 1` then
   `... activate --kind motion --transfer 1 --gen 1` then `... inputs --kind motion --gen 2 1:0:0 1:1:1 1:2:2`.
   Then `listen`: `inference` messages. Wrong mapping: `... inputs --kind motion --gen 3 2:1:0 1:1:1 1:2:2` is refused.
5. Model swap: upload `environment.amdl` (transfer 2), activate (gen 4), `inputs --kind environment --gen 5 2:1:0 2:0:1`
   (DHT22 temp, humidity); after swapping the I2C/GPIO sensors, point it at the BME280: `5:0:0 5:2:1`.
6. Corrupt upload: flip a byte inside the model payload of a copy of the package and upload it: the node answers `DIGEST_MISMATCH` and keeps its model.
7. Serial console (115200 8N1 on the console UART/USB): `HELP`, `SHOW`, `STOP`, `RUN`, `CLEAR`, `REBOOT`, `CFG <hex>`.
8. Reboot (`REBOOT`, or power-cycle) and check that the node comes back with its configuration, dormant profiles, rules, bridge and model. This was observed on node1; sensors that a rule started are asleep again after a reboot by design.

Offline rehearsal without hardware: `python tools/node_client.py selftest`, `python tools/run_all_tests.py`,
`python tools/qemu_session.py` (see STATUS.md).

## Measured in this repository

| Item | Value | How |
|---|---|---|
| Firmware image | 393,856 B (0x60280), 91 % of the 4 MB app partition free | `idf.py build` (clean build, ESP-IDF 6.1, esp32s3) |
| Static internal RAM (DIRAM) | ~82 KB of 341 KB | `idf.py size` |
| Motion / environment model | 3,600 B / 3,304 B int8 TFLite; ops FULLY_CONNECTED + SOFTMAX | `ml/artifacts/*/report.json` |
| Host verification | 5,155 checks in 6 suites (+ client self-test) | `tools/run_all_tests.py` |
| QEMU session | 17 checks (boot, config, REPL, NVS write, no crash) | `tools/qemu_session.py` |
| ESP-NN | optimised kernels enabled in the main build (`CONFIG_ESP_TFLITE_MICRO_USE_ESP_NN=y`); reference build (`sdkconfig.ref.defaults`, ESP-NN off) is 385,712 B vs 393,856 B | `sdkconfig`, `build_ref` |

## Benchmarks - all pending a board

Record raw data to CSV (one row per sample) and keep the script that produced it. Proposed method per row.

| Metric (AC-17) | Method | Result |
|---|---|---|
| Achieved vs requested sampling rate per sensor | log `Reading.timestamp_us` deltas over >= 5 min | pending |
| Inference latency (`inference_us`) per model | median/p95 of `InferenceResult.inference_us` over >= 1000 runs | pending |
| Same, ESP-NN **off** vs on | second image (`sdkconfig.ref.defaults`, build in `build_ref`); compare outputs (tolerance) and timing | pending (needs supervisor to confirm scope) |
| TFLM arena actually used | log line `arena used X of Y B` at model load | pending |
| Model swap time | client: `upload` start -> `ModelStatus ACTIVE` | pending |
| Chunk size 256 / 512 / 1024 | upload time and retries per size | pending |
| MQTT round trip | publish `command` -> observed effect / ACK latency | pending |
| Free heap / PSRAM | `Heartbeat.free_heap` over a long run | pending |
| Behaviour under Wi-Fi loss / reboot | disconnect during upload; power-cycle; check restore | pending |
