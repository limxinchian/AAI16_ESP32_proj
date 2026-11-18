# Architecture

```
 laptop (tools/node_client.py)                              other nodes
        |  MQTT, binary Protobuf                                  |
        v                                                         v
 plat_net.c  --(queue, reassembled messages)-->  node task (single owner of node_t)
                                                    |
        +-------------------------------------------+--------------------------------------------+
        v                v                 v                 v                  v                  v
  config_manager    rule_engine         bridge         model_store        ml_selector          repl
  validate/gen/ACK  RPN rules/actions   virtual 128+   chunks/banks       compat + windows     console
        |                 ^                 |                 |                  ^
        v                 |                 |                 v                  |
  sensor_runtime ---- Reading fan-out ------+------------> plat_ml.cc (TFLM) ----+--> InferenceResult
  executor + scheduler  (telemetry, rules, selector, bridge)
        |
        v
  sensor_decode (bit fields, checksums, RPN calibration)       plat_bus.c: I2C / UART / SPI / GPIO drivers
```

Layers (each is plain C with a hardware interface, host-tested; `platform_esp` is the only ESP-IDF-specific code):

| Component | Responsibility |
|---|---|
| `aai16_protocol` | `protocol/aai16.proto` + nanopb runtime; all messages bounded (`.options`) |
| `config_manager` | decode, strict validation, generations, ADMITTED->APPLIED ACKs, rollback, persist/restore |
| `sensor_decode` | data-driven decoding: bit fields, sign modes, checksums, UART sync, calibration, RPN formulas |
| `sensor_runtime` | executes `Op` lists on a `bus_backend_t`, per-bus periods, fault isolation, re-init |
| `ml_core` | SHA-256, feature extraction (mirrors `ml/common.py`), int8 glue, package parser, input selector, staged model store |
| `node_services` | rule engine, node bridge, heartbeat builder, REPL command layer |
| `node_core` | wires everything behind `node_platform_t`; handlers for every inbound topic |
| `platform_esp` | drivers, NVS + flash banks, Wi-Fi/MQTT, GPIO/PWM outputs, TFLite Micro wrapper |
| `main` | `app_main`: NVS init, PSRAM allocation of the node, node task, console task |

## Flows (handoff §7) and where they live

| Flow | Path |
|---|---|
| A reconfiguration | MQTT `in/config` -> `cm_handle_bytes` -> hooks `rt_apply`/`rt_stop` -> persist -> ACK `out/config_ack` |
| B acquisition + rules | scheduler -> `rt_run_ops` -> `decode_cycle` -> Reading -> telemetry + `re_on_reading` + `mlsel_push` |
| C dynamic ML input | `in/model/control` SET_INPUTS -> `mlsel_set_inputs` (compat check) -> `out/model_status` |
| D model replacement | `in/model/chunk` -> `model_store_on_chunk` (other bank) -> ACTIVATE -> `plat_ml_load` check -> metadata saved -> status |
| E rules | `in/rules` -> `re_apply` (atomic) -> actions through platform hooks |
| F bridge | other node's `out/reading/<id>` -> `bridge_on_remote` -> virtual sensor -> rules / ML |
| G serial + recovery | console -> `repl_exec` -> same handlers; boot -> `cm_restore` + active model reload |
| H dormant sensors | `ConfigApply` with `dormant` -> stored (not running); rule action `RACT_SENSOR_START` -> `rh_sensor` -> `cm_dormant_by_id` -> `rt_apply` (RAM only; optional timer checked in `node_tick`); `RACT_SENSOR_STOP` or a config for the bus ends it |

## Concurrency and memory

- One task owns `node_t` (~150 KB, PSRAM). MQTT and console tasks only enqueue messages (`plat_msg_t`).
- Buses are serviced one after another in that task: a blocking op delays other buses' timestamps by its duration.
- Static internal RAM use of the image is ~82 KB (DIRAM); the 8 MB PSRAM holds the node, message reassembly buffers and model copies.
- No JSON anywhere; no unbounded allocation during decode (nanopb static structs).

## Added after the first version

- **Dormant sensors** (flow H above): a profile can be stored but not running and started by a rule. A bus holds either a running or
  a dormant profile; dormant profiles reserve their pins and are persisted, but never start by themselves after a reboot.
- **Model C (optional anomaly detector)**: a third model kind with slot features, a presence mask and per-slot error attribution
  (`ml_core/ml_features.c`, `ml_selector.c`, `ml/ANOMALY_SPEC.md`).
- **Laptop helpers**: `tools/json_bridge*.py` mirror node Protobuf as JSON for MQTTX (laptop side only; the node never sees JSON),
  `tools/record_slot_data.py` and `tools/analyze_sampling_timing.py` record and analyse real data, `tools/build_all.ps1` builds the nodes.
