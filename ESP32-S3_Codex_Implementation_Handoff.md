# AAI Group 16 — ESP32-S3 Configurable Edge Node
## Claude code Implementation Handoff / Working Specification

**Document date:** 4 November 2026  
**Target board:** ESP32-S3-N16R8 (16 MB flash, 8 MB integrated PSRAM)  
**Reference platform:** `FlexiEdgePico` (Raspberry Pi Pico W / Pico 2 W project)  
**Target firmware:** ESP-IDF / FreeRTOS on ESP32-S3  
**Status:** Proposed implementation plan, **not yet approved by Prof Fauzi**  
**Important user decision:** Use **exactly two proposed ML models**: **Model A — Motion Classification** and **Model B — Environmental Classification**. **Do not add a third anomaly-detection model or an autoencoder/SVDD implementation** unless the user explicitly changes this decision.

---

## 0. Instructions to Claude code: read before changing code

You are assisting with a **real embedded-firmware implementation**, not writing an imaginary demonstration. Your job is to inspect the actual repository, port/reuse working pieces, implement missing parts incrementally, and show verifiable evidence for each completed step.

1. **Inspect first; never assume existing code matches this specification.** Identify the actual repository root, Git status, source tree, CMake build files, `sdkconfig`, components, generated nanopb files, unit tests, and current implemented features. The earlier repository was called `AAI16_ESP32_proj-main`; confirm the actual current name/path. Search for existing work on Tasks 1–10 before editing anything.
2. If `FlexiEdgePico` is available, inspect it to identify **verified code** and transferable behaviour. The legacy feature list in this document originated from a summary supplied by the team; **it is not proof that all legacy features actually work**. Record confirmed functionality, gaps, and portability issues.
3. Preserve existing working code and tests. Avoid broad rewrites or replacing a working component solely for style. Do not lose previously implemented Protobuf generation/ACK rules, configuration validation, or host tests.
4. **Do not claim target builds, hardware tests, model inference, OTA transfers, or benchmark results unless they actually ran.** If ESP-IDF, libraries, internet access, a board, or a trained model is missing, say exactly what is blocked. Host GCC/Zig logic tests are useful but **not a substitute** for `idf.py build` or on-device validation.
5. Use **real ESP-IDF components and ML libraries**; no fake headers, dummy ML predictions masquerading as inference, or hand-written stand-ins presented as genuine TFLite Micro/ESP-DL integration. Prefer ESP-IDF Component Manager for large dependencies. Small, bounded test doubles are fine **only** in explicitly labelled host tests.
6. Keep all required runtime MQTT messages Protobuf-encoded and bounded. Do not reintroduce JSON into the five change-requirement categories. Optional human-readable *serial logs* are acceptable; on-wire rule/control protocol details still require a decision.
7. Separate **confirmed requirements**, **team choices**, and **questions for the supervisor**. If a choice is unresolved, implement the stable interface first and document the blocker rather than silently inventing project requirements.
8. Work in reviewable milestones. After each milestone: list files changed, tests/build commands run, actual results, any unresolved issue, and next steps. Never modify production firmware and declare it complete on the strength of pseudocode alone.

### Source-of-truth priority

1. **Supervisor's latest direct feedback and written project specification** (the team must supply any newer material and decisions).
2. **Separate Protocol Buffers change requirement:** replaces JSON for the specified final message categories; bounded fields and strict validation.
3. **`2026-11-03-cargo-integrity-node-design(1).md`:** architecture/design notes, including four buses, the eight proposed sensors, generation/ACK concepts and the ML input selector.
4. **`ESP32-S3_Supervisor_Validation_Proposal_Expanded.docx`:** broadened team proposal to retain selected FlexiEdgePico capabilities.
5. **This handoff:** consolidates the conversation and records the team's latest choice of **two ML models only**. Where it differs from the expanded proposal's suggestion to additionally port anomaly detection, the **latest explicit user decision overrides that suggestion**.
6. Actual repository source code and observable results determine **what is implemented**, not what earlier notes claim has been completed.

The supervisor's 3 November 2026 transcript clearly supports: (a) changing the **entire ML model over the network**; (b) independently selecting sensor data to feed inference; (c) configured command bytes and response lengths rather than simply choosing sensor names; (d) a believable use case; and (e) explaining exact components and how they connect. **The provided transcript does not explicitly confirm the accelerated-vs-non-accelerated requirement**; that requirement appears in a separately relayed supervisor summary and the design document. Treat it as a documented requirement **pending confirmation of scope**, not as something to erase.

---

## 1. Project objective and demonstration story

**Objective:** Port and improve the previous group's **FlexiEdgePico reconfigurable IoT edge-node platform** to an ESP32-S3. The resulting system should receive remote configuration, communicate with heterogeneous sensors through generic bus engines, run configurable rules and actual ML inference, swap ML models over MQTT, and publish readings/predictions/statuses without firmware reflashing for routine changes.

**Plausible use case (demonstration only): Configurable Cargo Integrity Monitoring.** Different cargo setups need different measurements. A team member can remotely switch active sensors, configure sampling rates and rules, select ML inputs, and upload a model suitable for motion or environmental monitoring. Multiple ESP32-S3 nodes can exchange readings. This is **not a certified cargo-safety product**, and the ML outputs must not be claimed to be calibrated risk assessments.

### Explicit deliverable boundaries

- **Eight proposed sensors** across **four buses**; demonstrate **one active sensor per bus at a time** (four active choices total) and replace each choice using runtime MQTT configuration. Where electrically safe, wire both alternatives in advance; **do not require hot-unplugging a powered board**.
- Generic configuration defines **how to communicate**: bus, address/pins, command bytes, read lengths, operation sequence, acquisition interval. Do not implement a top-level fixed switch calling `read_dht22()` / `read_gy511()` / etc. as the only acquisition route.
- **Two trained ML model purposes only:** motion and environment (details in §5). Do not retain a separate legacy autoencoder/SVDD inference task in the planned implementation.
- Models must be **actually transferred over the network**, staged, validated and activated. Merely toggling between binaries built into firmware does **not** demonstrate model upload/replacement.
- Sensor selection and model selection are **independent controls**. The ML Manager **rejects incompatible sensor/model mappings before inference** and preserves the last valid active model/input configuration.
- The system should retain useful FlexiEdgePico-style facilities: Rule Engine, serial REPL, node-to-node sensor bridging, heartbeat, diagnostics, configurable outputs and persistence, subject to source-code inspection and supervisor acceptance.
- Use **Protobuf/nanopb**, not JSON, for the mandated communication types; benchmark and improve the original reported ~1 Hz polling limitation.

---

## 2. Hardware and four communication buses

| Bus | Initial sensor (A) | Alternate sensor (B) | Example data | Integration concern |
|---|---|---|---|---|
| **GPIO** | DHT22 | HC-SR04 | Temperature/humidity; distance | DHT22 requires precise pulse timing; HC-SR04 needs timed TRIG/ECHO and safe voltage-level handling. |
| **I2C** | GY-511 | BME280 | Acceleration/magnetic field; temperature/humidity/pressure | Different addresses/registers; BME280 needs calibration/compensation for meaningful physical units. |
| **UART** | LD2450 | MH-Z19(B) | Radar/presence; CO₂ | Different baud rates, command/reply or unsolicited framed messages; asynchronous receive buffering. |
| **SPI** | RC522 | MAX6675 | RFID ID/presence; thermocouple temperature | Different CS pins, transaction lengths, modes and sensor-specific conversions. |

**The sensor set above is from the design notes, not a claim that all eight are already connected or tested.** Before the demo, confirm power, wiring, level shifting, bus configuration, pin conflicts, supported data rates and each sensor's actual product datasheet.

### Existing design-note pin assumptions (verify against real boards/wiring)

- DHT22 GPIO6; HC-SR04 TRIG/ECHO GPIO4/5 (level-shift 5 V ECHO before ESP32-S3 input if applicable).
- I2C SDA/SCL GPIO8/9 for both GY-511 and BME280.
- LD2450 UART1 TX/RX GPIO17/18; alternate MH-Z19(B) UART2 GPIO15/16 according to the earlier design (confirm hardware UART assignment and flow).
- SPI SCLK GPIO12, MOSI GPIO11, MISO GPIO13; RC522 CS GPIO14, MAX6675 CS GPIO10.
- N16R8 PSRAM is module-integrated. Configure the target/PSRAM correctly; do not propose separate PSRAM wiring.

### Correct meaning of sensor swapping

`Configuration A` activates DHT22 + GY-511 + LD2450 + RC522.  
`Configuration B` activates HC-SR04 + BME280 + MH-Z19(B) + MAX6675.

The ESP32-S3 receives a validated `ConfigApply`, stops affected acquisitions, applies the new generic profiles and starts the requested sensors **without reboot, code edit or firmware reflash**. Ideally support switching **one bus only**, not requiring all four to change at once. If only one physical device can be safely connected for a particular setup, physical rewiring occurs **with power off**; this is not software hot-swap.

**Existing open requirement risk:** Earlier analysis of the old Pico project suggests only DHT22 and GY-511 may match its exact original sensor parts. Supervisor must confirm whether “four previous sensor types” means exact part numbers or functional categories.

---

## 3. Generic configuration-driven sensor acquisition — mandatory architecture

### 3.1 Required behaviour

The professor's example was effectively: **send three specified command bytes, then read two response bytes**. The device must follow configured operations, not select an opaque hardcoded driver based on sensor name. Target a **bounded operation interpreter** with reusable bus backends:

- Addressing and electrical setup: bus type, pins/device address, UART baud, SPI mode/clock/CS, GPIO mode.
- Bounded ordered operations: `WRITE`, `READ`, `WRITE_READ`, delay, GPIO set/pulse and the additional pulse capture/one-wire/counter primitives **actually needed** by the proposed sensors.
- Explicit maximum transmit and receive byte lengths; timeout/error handling on every operation.
- Sensor-specific acquisition periods (`period_ms`) independently scheduled, without the previous reported global 1-second ceiling.
- Decode valid outputs to a common sensor reading (timestamp, generation, sensor ID, validity, values and optional raw bytes), making them available to MQTT, ML and rules.

**Important distinction:** generic **transport** is not the same as generic **meaningful decoding**. Reading BME280 raw registers is not the same as producing calibrated °C, humidity and pressure. The team proposes configurable simple decoding primitives (byte positions, endian, signedness, scale) plus reusable timing/compensation mechanisms as needed. Whether **all complex calibration formulae** must be configuration-defined remains a supervisor question. Do not silently substitute a list of preprogrammed sensor-name handlers for the requested generic communication engine.

### 3.2 Suggested responsibility split (adapt to existing repo)

| Responsibility | Intended contract |
|---|---|
| `Config Manager` | Decode/validate Protobuf, reject invalid profiles, apply generation-tagged changes atomically or with safe rollback. |
| `Sensor Manager` | Own current active profiles and sensor lifecycle; coordinate replacement per bus. |
| `Acquisition Scheduler` | Respect each active sensor period independently; monitor achieved rates. |
| `GPIO / I2C / UART / SPI Engines` | Execute generic bounded operations; no sensor-name decision at this layer. |
| `Decode/Normalize Layer` | Turn raw bytes/timings into typed, calibrated/normalized values where supported; report limitations/errors explicitly. |
| `Reading Store/Fan-out` | Store last valid reading with generation and time, and deliver to telemetry, rules and ML. |

### 3.3 Preserve what works; fix gaps

- Existing ESP32 drivers mentioned in the design: `dht22`, `gy511_sensor`, `ld2450_sensor`, `rc522_sensor` and `spi_bus_manager` may be starting points. **Inspect actual repository before referring to these as present or complete.**
- Original design note flags `spi_bus_manager` as limited to four bytes and one in-flight transfer; confirm before rewriting and extend to supported variable-length operations if still true.
- One-wire and pulse/echo require reliable timing capture, not merely `GPIO_SET` / `GPIO_PULSE`; document ESP-IDF peripherals/interrupt mechanisms used and their test results.
- A raw-read demonstration can prove generic communication even when some elaborate decoding is unresolved, but **do not report uncalibrated bytes as reliable engineering units**.

---

## 4. MQTT, Protobuf, lifecycle and reliability

### 4.1 Communication architecture

- Laptop (and additional ESP32-S3 nodes) communicates through an MQTT broker on the local network.
- Reuse **ESP-IDF `esp-mqtt`** via the existing `mqtt_manager` wrapper if it is functional. Do not reinvent MQTT transport.
- MQTT transports bytes; **nanopb** encodes/decodes Protobuf payloads. Serial REPL may remain human-readable but must feed the **same validation/application logic** as MQTT.
- The separate change requirement mandates Protobuf for **configuration, APPLY/ACK, model control, model upload, telemetry and committed configuration storage**. Model chunks must include bounded data and sequence/generation identifiers; every decoded field must be range-checked before use. Required strings, byte arrays and repeated fields have fixed maxima; do not heap-allocate unbounded fields during message processing.
- Legacy JSON/text formats are **not** the final MQTT wire contract for those categories. Decide and document bounded Protobuf schemas for rule updates, node-bridge control and operational commands if they are required on the network.

### 4.2 Earlier design message catalogue (baseline, verify against code)

| Protobuf message | Function |
|---|---|
| `SensorProfile`, `Op`, `ConfigApply` | Define the active bus operations, device IDs and sampling periods. |
| `ApplyAck` | Report admit/applied phase and acceptance/rejection with reason for the relevant generation. |
| `Reading`, `CommittedConfig` | Publish validated sensor readings and persist committed configuration. |
| `ModelChunk`, `ChunkAck` | Transfer bounded model blocks and acknowledge individual sequences. |
| `ModelControl`, `InputSource` | Select/activate models and map source sensor/value indices to model inputs. |
| `ModelStatus`, `InferenceResult` | Report model state and actual inference output. |

Use the uploaded design's generation scheme: increasing `generation` on configuration/model requests, `ApplyAck` echoing generation and sensor ID, `ChunkAck` keyed by transfer ID/sequence/generation, and `ModelStatus` tied to the requested model version. ACK **after** data/state is actually accepted/stored according to the chosen contract. Reject duplicate, stale, out-of-range and malformed messages safely.

**Existing draft is not automatically a final wire protocol.** Before freezing `.proto`, review ambiguous fields (e.g. `secondary_pin` overloaded for baud/SPI speed; delay duration stored in `read_len`), complete GPIO/UART/SPI support, needed semantic input metadata and node/rule messages. If modifying schema, document backwards-compatibility/migration and update tests **without breaking the already-agreed generation/ACK semantics**.

### 4.3 Model upload and persistence

- Use bounded `ModelChunk` uploads (the earlier `.proto` suggested **up to 1024 payload bytes per chunk**; benchmark payload sizes like 256/512/1024, but don't claim an unconstrained max).
- Validate transfer ID, model ID, generation, size, chunk sequence, total count and final integrity (e.g. an agreed digest/checksum). Handle duplicates, retries, lost chunks and aborted transfers.
- Store to **inactive/staging storage** first; never overwrite the currently working model before validating the new one. Consider flash partition capacity and erase/write endurance on 16 MB flash.
- At a safe inference boundary, validate model format/runtime/operators/input contract; activate only if compatible; publish explicit success/failure.
- Reboot/reconnection should restore the **last committed valid** configuration and model where available, not partially uploaded state.
- Bounded Protobuf message buffers and a fixed transfer-chunk buffer do not preclude carefully controlled ML tensor arenas or model allocations **outside** message decode. Record actual heap/PSRAM usage.

---

## 5. Machine Learning — exactly TWO model tasks

> **Latest team decision (overrides earlier expanded proposal):** Do **not** add FlexiEdgePico's autoencoder/SVDD anomaly-detection model as a third model. Retain the **reconfigurable model-management concept**, not an additional required ML task. The existing Pico anomaly code may be studied for porting ideas, but it is outside this selected two-model scope unless the supervisor later directs otherwise.

### 5.1 Model A — Motion Classification

- **Primary source:** GY-511 accelerometer over I2C.
- **Purpose:** classify a short window of motion samples into **`STATIONARY`, `MOVING`, `VIBRATING`**.
- **Input:** timestamped X/Y/Z acceleration history for a fixed sampling window; explicit sensor units, preprocessing, normalization and feature order. **A single instantaneous acceleration value is not enough** for the proposed vibration-pattern example.
- **Training:** Python on the laptop. Start with labelled stationary/moving/shaking data from the real device if practical; controlled synthetic data can bootstrap integration but must be labelled as synthetic. Keep a reproducible training script, train/test split, saved preprocessing metadata and model export.
- **Output:** three class probabilities (or equivalent model scores), the chosen predicted label and model identifier/version. Existing `InferenceResult.outputs` max count 4 can represent the three scores.

### 5.2 Model B — Environmental Classification

- **Primary source:** **either** DHT22 **or** BME280. Both can provide **temperature and humidity**, enabling a meaningful interchangeable input contract.
- **Purpose:** classify a time window as **`STABLE`** or **`CHANGING`** environmental conditions, using temporal behaviour rather than an obvious fixed temperature threshold. These are **proposed demo labels**, not scientific safety categories.
- **Input:** fixed-length, time-ordered **temperature + humidity** readings with consistent units, sample spacing, normalization and validity checks. Both DHT22 and BME280 must be converted to the same feature schema. The BME280's pressure channel is *not required* for the initial model input.
- **Training:** Python on the laptop, preferably with records from **both sensor types** to reduce sensor-domain mismatch. Synthetic data may be used only as a clearly labelled development aid; do not claim field accuracy.
- **Output:** two class scores/probabilities and predicted label; publish via Protobuf/MQTT.

### 5.3 The three separate choices in the ML system

1. **Select input source (without changing model):** e.g. keep Model B active and change its compatible temperature/humidity source from DHT22 to BME280 using MQTT `MODEL_SET_INPUTS`. The firmware remains unchanged; inference uses the same input shape and preprocessing contract.
2. **Swap entire model (without reflashing):** upload actual Model A or Model B file/weights through `ModelChunk`, validate/activate via `ModelControl`, and prove the new binary/version is the active one. This is more than changing an enum between two built-in binaries.
3. **Select inference implementation (if required):** standard/reference operations versus ESP32-S3-optimized operations. This is **not** the same as choosing Model A vs Model B and does **not** require training a third ML task.

### 5.4 Input compatibility validation — settled team decision

The **ML Manager must reject incompatible input mappings before inference**. A usable model metadata/contract mechanism is needed beyond a bare `sensor_id` + `value_index`:

- Model identity/version, runtime/model format, input tensor type/shape, feature count and order.
- Expected feature semantics (e.g. accelerometer X/Y/Z vs temperature/humidity), units and preprocessing/normalization version.
- Required history window, sample period/tolerance and minimum freshness.
- Output shape/labels, compatible runtime/operator set.
- Sensor enabled state, validity, generation, time freshness and any per-bus exclusivity.

**Example:** GY-511 → Model A is valid; DHT22 temperature → Model A is invalid even if an underlying tensor dimension could be made to match. DHT22 or BME280 temperature+humidity → Model B can be valid **if** conversion and input shape are consistent. If the user changes the active I2C sensor from GY-511 to BME280 while Model A is active, the system must prevent stale GY-511 inputs; choose a clearly documented safe policy (reject incompatible configuration, or suspend inference with a reported status) while preserving the last valid model/mapping whenever possible.

Reject errors through MQTT **with an identifiable reason**, do not silently accept garbage, do not erase the valid configuration, and do not mislabel invalid readings as legitimate ML predictions.

### 5.5 Multiple sensor inputs

The professor's requirement is to **select data from any supported sensor**; this means the selector infrastructure should support bounded multiple `InputSource` entries and validate each one. **It does not mean that each of the two trained models must produce meaningful output for all eight unrelated sensors.** For this handoff, demonstrate Model A with GY-511 and Model B with DHT22/BME280. Any later multi-sensor model or expanded compatibility needs an explicit model contract and user/supervisor approval; **do not create a third model by default**.

### 5.6 Inference runtime integration: real libraries, real verification

- Preferred initial milestone: run **real** `espressif/esp-tflite-micro` on ESP32-S3 using a quantized model with supported operators; prove actual inference before attempting remote model activation.
- Accelerated kernels: **ESP-NN** supplies optimized operations for ESP32-S3. Espressif's `ESP_TFLITE_MICRO_USE_ESP_NN` setting is a **build-time** setting; one MQTT boolean alone does **not** switch standard vs optimized kernels in a single firmware. Do **not** claim runtime selection unless two genuine execution implementations are compiled and selectable.
- Alternative optimized stack: **ESP-DL**, whose deployed models require Espressif `.espdl` format, ordinarily prepared through ESP-PPQ from an ONNX/PyTorch-compatible model. `.tflite` is **not directly** an `.espdl` model. A second export format is **not** a third prediction task.
- Confirm with the supervisor whether accelerated and non-accelerated execution must be runtime-swappable over MQTT or may be benchmarked in separate builds. Keep the runtime interface abstract enough to allow either resolution without rewriting the ML Manager.
- Never create “correct-looking” includes against libraries that are not present and mark the milestone as verified. Install ESP-IDF and fetch real component dependencies before target integration claims.
- Compare predictions using equivalent inputs within appropriate numerical tolerance; benchmark `invoke()` timing and memory use on board. Optimized inference may or may not materially speed up these tiny models; **record actual measurements**.

### 5.7 Suggested model-related repository artifacts (names illustrative)

```text
ml/
  train_motion.py             # GY-511 model training and export
  train_environment.py        # Temperature/humidity temporal model
  requirements.txt            # Pinned, reproducible training dependencies
  README.md                   # Data, units, class definitions, conversion/testing
  metadata/
    motion_model_contract.*   # Input shape/meaning/normalization/labels/version
    environment_contract.*
  examples/ or test_data/     # Tiny non-sensitive fixtures for smoke tests

components/ (adapt to actual repo):
  ml_manager/                 # Model/control state and rollback
  ml_input_selector/          # Mapping + compatibility and preprocessing
  model_transfer/             # Chunking/ACK/staging/integrity
  inference_runtime/          # Real TFLM baseline; optional optimized backend
```

Use the repository's existing module structure where possible. Paths above are suggestions, **not claims that these directories currently exist**.

---

## 6. Other FlexiEdgePico capabilities to retain in the port

These are **planned platform capabilities**, not additional ML models. Verify what the Pico source genuinely supports, then reuse design/logic where sensible.

### 6.1 Configurable Rule Engine

- Runtime rules (legacy description: RPN/stack evaluation), bounded expressions, sliding-window statistics and actions.
- Support simple examples like “when a configured movement statistic exceeds its rule threshold, log/notify/activate an output,” as well as triggering inference.
- Keep **rule thresholds** separate from **ML learned weights**. The earlier expanded proposal's “dynamic anomaly thresholds” are **not** a mandate to create an anomaly-detection ML model; implement configurable rule/action thresholds where useful.
- Actions: local GPIO/PWM/log/MQTT notification, ML trigger, and allowed remote command to another node. Validate output pin permissions and bounds. Ensure no recursive/infinite rule-action loops.
- Decide whether remote rule updates are Protobuf rather than copying legacy JSON/text wire packets.

### 6.2 Serial REPL

- Local USB serial commands corresponding to `RUN`, `STOP`, `CLEAR`, `SHOW`, configuration entry and diagnostics, as supported by the actual reference implementation.
- Route serial and MQTT control to the **same** validation/application services. The REPL is for local control/debug and may use human-readable text; do not duplicate sensor/ML business logic.

### 6.3 Node-to-node sensor bridging

- Node A publishes Protobuf readings; Node B subscribes and stores readings as **virtual inputs** with node ID, sensor ID, generation, receive time, source timestamp/validity and timeout/freshness policy.
- Make virtual inputs available to compatible rules and ML mappings. Avoid ID collisions and feedback loops. A two-board demo is sufficient as an initial acceptance target pending supervisor confirmation.
- If nodes do not have synchronized clocks, use a local **receive timestamp** to enforce freshness and treat source timestamps as supplementary metadata.

### 6.4 Reading fan-out, diagnostics, heartbeat and recovery

- Each valid physical/virtual reading can go to serial diagnostics (raw + interpreted as available), Rule Engine, MQTT telemetry and ML input store.
- Periodic heartbeat (legacy summary suggests around 10 s): node ID, uptime/connection status, current generation, enabled sensors and health/fault counters as feasible.
- Persist and restore committed configuration and active model reference safely, with schema/version validation. Reconnect MQTT without blocking local data acquisition indefinitely.
- Provide bounded errors/telemetry for invalid sensor commands, sensor timeout, corrupt model upload and rejected ML mapping.

### 6.5 Legacy features intentionally *not* replicated verbatim

- **JSON payloads:** replaced by Protobuf for required message types (and consider Protobuf for added remote control/rules).
- **Pico-specific driver code / scheduling:** port concepts, not microcontroller-specific register logic.
- **Autoencoder/SVDD anomaly detection:** **out of scope as a third ML model** by the user's latest decision. Inspect only if useful for understanding reference OTA/runtime logic.

---

## 7. End-to-end flows Claude code must implement and test

### Flow A — Remote sensor reconfiguration

`Laptop → MQTT broker → ESP-IDF MQTT handler → nanopb decode → strict validation → Config Manager (generation) → Sensor Manager → scheduler/bus engine → physical sensor → Reading Store → Protobuf telemetry + ACK`

For a rejected update, keep active configuration unchanged and publish a reason.

### Flow B — Ordinary acquisition and rule processing

`Active sensor / remote virtual sensor → generic operations → decode/validate/time-stamp → Reading Store → serial diagnostics + Rule Engine + MQTT + optional ML Input Selector`

No blocking 1 Hz super-loop. Every configured period must be measured against actual device capability.

### Flow C — Dynamic ML input selection

`Laptop ModelControl(MODEL_SET_INPUTS) → MQTT/nanopb → ML Manager checks model contract + active sensor values/history → accept/reject → inference on selected compatible data → InferenceResult → MQTT`

Demonstrate **DHT22 → BME280** while **keeping Environmental Model B unchanged**.

### Flow D — Actual ML model replacement

`Laptop model file → ordered ModelChunk uploads → MQTT/nanopb → ChunkAck per accepted chunk → staging storage + size/integrity checks → ML Manager validates binary and input contract → safe activation → ModelStatus → inference → MQTT`

Demonstrate the reverse direction too if practical (B → A) and a failed upload that **does not destroy the previous working model**.

### Flow E — Rule update and action

`Laptop rule configuration → validated control path → Rule Engine updates rule atomically → reading triggers rule → bounded GPIO/PWM/log/MQTT or inference action`

### Flow F — Multi-node virtual sensor

`ESP32-S3 Node A reading → MQTT broker → ESP32-S3 Node B virtual reading store → freshness check → rule or compatible ML input`

### Flow G — Serial fallback and recovery

`USB serial command → shared command handlers → valid config/operation → state report` and `reboot → validate stored committed state → resume safe operation`.

---

## 8. Incremental execution plan for Claude code (do not skip audit)

### Phase 0 — Audit and feasibility report (first deliverable)

- Identify what is **already implemented** in the current ESP32 repo and what remains; cross-reference any prior Tasks 1–10 and pending Tasks 12–13.
- Identify ESP-IDF/toolchain versions available; board target, managed components, CMake wiring, PSRAM settings, flash partitions, MQTT and nanopb availability.
- Identify transferable FlexiEdgePico features **based on actual source**, not description alone.
- Create a traceability matrix: **requirement → component/files → current status → build/test evidence → gap**.
- Flag constraints that need supervisor decisions, without silently changing them.
- **Do not first replace major existing implementation.** Start by reporting the audit and proposing the smallest safe edit sequence.

### Phase 1 — Baseline hardware build and messaging

- Get a clean ESP-IDF ESP32-S3 build/flash/monitor working on a real board, or document exact missing prerequisites.
- Confirm MQTT connection and bounded Protobuf encode/decode for required messages.
- Add/verify validation, transaction generations, APPLY/ACK and model-chunk ACK unit tests.
- Confirm safe build setup for 16 MB flash/8 MB PSRAM and room for two model generations/staging.

### Phase 2 — Generic sensor path

- Implement/refactor bounded operation executor for each of four buses, including timing-aware GPIO and variable-length SPI read support.
- Implement sample period scheduling independent for each active bus profile.
- Prepare real eight sensor profiles (data-driven, not `switch(sensor_name)` in application logic).
- Ensure valid/invalid sensor readings are time-stamped and fan out to shared services.
- Verify single-bus sensor change without affecting other active buses.

### Phase 3 — Platform parity services

- Implement/port Rule Engine and safe action dispatch (bounded evaluation).
- Implement serial REPL using common validation/application code.
- Implement heartbeat, diagnostics and committed-state recovery.
- Implement node bridge / virtual sensor store with freshness and source identity.
- Cover each with host tests, then on-device tests as hardware permits.

### Phase 4 — Two-model ML preparation and local inference

- Implement reproducible laptop-side training for **motion** and **environment** models only.
- Define feature contracts, normalization, sample windows, class labels and fixtures; record provenance of any synthetic data.
- Export compatible quantized model binaries; verify output on desktop with known fixtures.
- Integrate **real** target ML library and run each model locally on ESP32-S3 with explicit test inputs. No “ML manager complete” status without real inference.
- Connect selected real sensor readings to input history/preprocessing and publish actual inference results via Protobuf.

### Phase 5 — Network-driven input selection and model hot-swap

- Implement dynamic `MODEL_SET_INPUTS` plus compatibility checks, including DHT22/BME280 interchangeability for Model B.
- Add reliable chunked model upload, staging, integrity checking, activation, status reporting, rollback on failure and reboot recovery.
- Prove real file replacement over MQTT; do not rely solely on switching preembedded model pointers.
- Preserve correct generation/ACK relationships under retries and interruptions.

### Phase 6 — Acceleration resolution (conditional supervisor scope)

- Confirm whether accelerated and non-accelerated paths are mandatory and whether runtime switching in one firmware is required.
- If required, implement **genuine** reference and optimized inference execution with clear backend selection, runtime/model-format validation and target measurement.
- Do not pretend toggling a flag changes ESP-NN's build-time kernel selection.

### Phase 7 — Integrated demo and benchmarks

- Demonstrate configuration-driven sensor swaps; all four buses; one active sensor per bus; live sensor readout; independent rates.
- Demonstrate Rule Engine, serial REPL, heartbeat, node bridge and recovery.
- Demonstrate Motion Model A and Environment Model B, compatible sensor switching and invalid mapping rejection.
- Demonstrate actual network model upload and activation, plus failure fallback.
- Record data for model swap time, inference latency, MQTT round-trip, model-chunk sizes, achieved vs requested sampling rates, memory usage and resilience to disconnection/reboot.

**Prioritization if time is limited:** do not secretly remove legacy features. Present dependencies and a scoped fallback to the team/supervisor. Core: generic bus engine + Protobuf + MQTT + two real ML models + network model swap + dynamic input validation. Preserve additional baseline features according to confirmed scope.

---

## 9. Acceptance matrix: observable proof, not just source code

| ID | Requirement / behaviour | Minimal observable evidence |
|---|---|---|
| AC-01 | Four generic communication buses | ESP32-S3 executes actual configured GPIO/I2C/UART/SPI operations on target. |
| AC-02 | Eight sensor choices, four active | Sensor profile swap A↔B per bus, with telemetry/status and **no reflash**. |
| AC-03 | Generic command/read behaviour | Modify configured bytes/length/address and observe different actual transaction/response without code edit. |
| AC-04 | Independent acquisition periods | Requested vs actual timestamps/rates, with suitable sensor >1 Hz if supported. |
| AC-05 | Protobuf change requirement | Golden encode/decode tests, max-size rejection, malformed-input handling; no JSON in specified final message paths. |
| AC-06 | Reliable config application | Invalid/stale update rejected with reason, prior config preserved, correct generation/ACK. |
| AC-07 | Real Motion Model A | Real GY-511 window → true target inference → documented class scores → MQTT. |
| AC-08 | Real Environment Model B | DHT22 data → true target inference; switch to compatible BME280 data while keeping **same model**. |
| AC-09 | Wrong sensor/model rejection | DHT22→Motion mapping rejected before inference; previous valid mapping retained. |
| AC-10 | Actual model replacement | Transfer complete model file through MQTT, load and activate a different model, verify changed ID/generation and real inference, no reflash. |
| AC-11 | Upload robustness | Out-of-order/duplicate/lost/invalid chunk and corrupt model scenarios handled; active model protected. |
| AC-12 | Rule Engine and actions | Update rule at runtime; observe bounded action or inference trigger. |
| AC-13 | Serial REPL | RUN/STOP/SHOW and config handling through same manager; no conflicting state paths. |
| AC-14 | Node-to-node bridge | Node A reading becomes validated time-bounded virtual input on Node B. |
| AC-15 | Diagnostics/heartbeat/persistence | Heartbeat/status and valid state restoration after reboot; explicit errors. |
| AC-16 | Hardware optimization (if confirmed) | Reference and optimized kernels/runtimes genuinely used, outputs checked and measured on board. |
| AC-17 | Benchmarking | Raw CSV/structured evidence and methodology for latency, rate, size/memory, retries and swap timing. |

**Evidence rule:** distinguish **unit tested**, **host integrated**, **target compiled**, **flashed**, and **physically demonstrated**. These levels are not interchangeable.

### Negative tests worth automating

- Unknown bus/operation enum, forbidden GPIO, invalid address/baud/CS, illegal period/length and excess repeated fields.
- Duplicate/stale generation, incompatible apply, invalid model ID, wrong model format/shape, nonexistent/disabled sensor ID, stale reading.
- Bad chunk sequence/transfer ID, missing/duplicate chunk, length overflow, CRC/digest mismatch, partial reboot, unexpected Wi-Fi loss.
- Incorrect physical bus response, timeout, mis-sized UART frame, GPIO pulse timeout, input history short/invalid.
- Node bridge stale data, duplicate source IDs, cyclic forwarding, rule stack underflow/overflow, unauthorized output pin/action.

---

## 10. Open decisions / questions for Prof Fauzi (do not silently resolve)

1. **Legacy parity:** Must *all* FlexiEdgePico features be retained, including Rule Engine, serial REPL, node bridge, GPIO modes and heartbeat, or is a selective port expected?
2. **Generic decoding boundary:** Must raw-byte decoding and complex sensor calibration formulas also be fully specified by Protobuf configuration, or may reusable generic primitives/calibration helpers be used?
3. **Previous sensor types:** Must four sensor types exactly match Pico part numbers, or can equivalent types count? Is four-at-a-time logical switching between eight preconnected sensors acceptable?
4. **Two-model ML scope:** Is Model A (GY-511 motion: stationary/moving/vibrating) and Model B (DHT22/BME280 environmental: stable/changing) sufficient, **without retaining the Pico autoencoder/SVDD as a third model**?
5. **Dynamic ML inputs:** Is rejection of sensor/model-incompatible combinations correct? Must either trained model meaningfully process every sensor or arbitrary multi-sensor combinations, or is general routing with compatibility enforcement sufficient?
6. **Acceleration:** Is optimized + non-optimized inference mandatory? If so, must runtime switching via MQTT work inside a single firmware, or is separate target benchmarking allowed?
7. **Protocol scope:** Does the Protobuf change also cover rule definitions, bridging/control commands and all remote operations, beyond the specified core message categories?
8. **Acceptance scope:** Which baseline parity features, reliability tests and timing benchmarks are essential for grading?

**Until answered:** implement interfaces that preserve future options; don't assert disputed behaviour is compulsory or signed off.

---

## 11. Suggested repository documents / artifacts Claude code should maintain

- `docs/requirements_traceability.md` — each requirement, source, implemented files, test evidence and supervisor decision status.
- `docs/architecture.md` — real module diagram and data/control flows; updated when code changes.
- `docs/sensor_profiles.md` — all eight profiles, protocol notes, electrical warnings and decoding capability/status.
- `docs/ml_model_contracts.md` — exactly two models, supported input sources, shape/window/units/normalization, outputs, format and runtime.
- `docs/mqtt_protobuf_protocol.md` — topics, message IDs, fixed maxima, generations, ACK/retry/transfer contracts.
- `docs/demo_and_benchmarks.md` — precise demo commands, actual target evidence, failures and numerical results; **no fabricated numbers**.
- Versioned `.proto`, nanopb `.options` and generation scripts; real model-training scripts/metadata; reproducible build instructions.

Optional: give the team a short `STATUS.md` documenting `Implemented / Host-tested / Target-built / Board-tested / Blocked` after each Claude code iteration.

---

## 12. Implementation integrity and handoff checklist for Claude code

Before declaring a phase complete, answer:

- Which **actual existing** files/modules were inspected and reused?
- Which files changed, and why? Did tests for earlier tasks remain green?
- What version of ESP-IDF and ML dependencies was **actually** used? Is the chosen toolchain accessible? Is the target `esp32s3`?
- Were the applicable nanopb fixed-size options respected and decoded fields validated before use?
- Is any logic hardcoded by sensor name that contradicts the generic-operation requirement?
- Do model A/B run **real ML inference** and have meaningful input feature contracts, not trivial software `if` statements posing as ML?
- Does remote **model upload** transfer new bytes and activate them safely (not just choose among firmware-embedded constants)?
- Are incompatible ML input mappings rejected while keeping a valid active configuration?
- Are all claims accompanied by actual host-test, target-build and/or hardware-run evidence?
- Were pending supervisor questions left visible rather than guessed away?

### First response expected from Claude code after reading this handoff

1. A short **repository audit** with verified current status and gaps.
2. A **component-by-component implementation roadmap**, prioritized by dependency and risk.
3. A separate **ML readiness report** covering ESP-IDF availability, real TFLM/ESP-NN/ESP-DL dependency status, presence/absence of trained Motion and Environment model files, model metadata contracts, and what target verification is blocked.
4. A recommendation for the **smallest safe first code change** and tests. Then implement incrementally, reporting evidence as work progresses.

---

## 13. External technical references (for Claude code to consult, not proof of current implementation)

- Espressif ESP-IDF Component Manager manifest documentation: <https://docs.espressif.com/projects/idf-component-manager/en/latest/reference/manifest_file.html>
- Espressif `esp-tflite-micro` and ESP-NN integration: <https://github.com/espressif/esp-tflite-micro>
- ESP-TFLite-Micro ESP-NN build-time Kconfig: <https://github.com/espressif/esp-tflite-micro/blob/master/Kconfig>
- ESP-NN optimized kernels: <https://github.com/espressif/esp-nn>
- ESP-DL model conversion and target setup: <https://docs.espressif.com/projects/esp-dl/en/latest/getting_started/readme.html>

**Final instruction:** The aim is a **verifiable port-and-improve ESP32-S3 platform**, with **only the motion and environmental classification ML tasks**, not the appearance of a complete platform created by stubs or unsupported assumptions. Show real evidence and surface unresolved project decisions early.
