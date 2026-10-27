# GY-511 ML Phase 1: raw data collection

Pipeline: GY-511 -> ESP32-S3 -> existing MQTT broker -> Python collector ->
labelled CSV. This phase implements acquisition and recording only.

## Baseline and scope

Verified branch: `feature/ml-gy511-data-collection`.
Baseline HEAD: `ed28c75467f87f61a0368c9d9a70f1eb7353dae8`.
The initial working tree contained only a modification to `dependencies.lock`:
IDF `6.1.0` -> `5.5.5`, lock format `3.0.0` -> `2.0.0`. That existing change
is preserved. The build selects cached `espressif/cjson 1.7.19~2` and
`espressif/mqtt 1.1.0` with ESP-IDF v5.5.5. The managed MQTT component overrides
the IDF-bundled component; host SDK tests now select the managed component too.

No branches were created/switched by the implementation, and no commit, push,
merge, hardware flash, credential edit or partition change was performed.
Other seven sensor implementations and the shared bus managers are unchanged.
Sensor Status continues to report all eight sensors independently.

The baseline GY-511 reader defaulted to 500 ms, and its MQTT parser accepted
only integer intervals from 500 through 10000 ms. The reader already supported
runtime acquisition scheduling. The driver did not check data readiness or
verify written settings, and MQTT `uptime_ms` was generated at submission,
after both acceleration and magnetic reads. There was no dataset collector.

## Acquisition and telemetry contract

- GY-511 accepts **integer 100..10000 ms, inclusive**. The default remains
  **500 ms** after each boot. Configuration is in RAM, so set 100 ms again
  after reboot. Every previous valid interval remains valid.
- Actual periods round **up** to the RTOS tick: this project's tick is 10 ms,
  so 100 -> 100 ms and 101 -> 110 ms. No other sensor interval limit changes.
- The owning GY-511 task changes its I2C read deadline, then issues the applied
  ACK. Publication follows successful reads; there is no separate 10 Hz replay
  timer. Configuration remains ordered, with bounded command processing.
- The accelerometer remains at **100 Hz**, high resolution, +/-2 g, BDU on:
  `CTRL_REG1_A=0x57`, `CTRL_REG4_A=0x88`. The magnetometer remains continuous at
  **30 Hz**, `CRA_REG_M=0x14`, +/-1.3 gauss. Initialization reads back the two
  acceleration settings and magnetic ODR; mismatch or I2C failure fails init.
- Before each acceleration burst, `STATUS_REG_A.ZYXDA` must indicate new XYZ
  data. No-ready returns `ESP_ERR_INVALID_STATE` immediately. No busy polling,
  cached replay, value-based freshness test or retries are added. Equal
  numerical values at REST remain valid fresh samples. Internal 100 Hz
  conversions between 10 Hz reads are intentionally not all collected.
- `gy511_data_t.acquired_us` is captured immediately after the acceleration
  burst, before the magnetic read. It is an ESP timer acquisition timestamp,
  **not UTC and not a sensor conversion timestamp**. The sensor provides no
  conversion timestamp here; BDU/register latch timing and I2C latency remain.
- `jitter_us` is signed read-start time minus the scheduled deadline. It
  includes RTOS tick quantization. Acquisition deltas also include read/bus
  latency, which is why CSV timestamps must never be synthesized as `n*100`.
  HEALTH reports cumulative maximum absolute jitter, maximum read duration,
  read failures and overruns.
- One task owns GY-511 reads, so reads cannot overlap. Existing notification
  waits sleep until a deadline/configuration event. If a read overruns a full
  period, missed periods are skipped and the task waits another full period.
- Combined-read behavior is retained: a magnetic read error or saturation
  suppresses that complete sample, even if acceleration succeeded. The 30 Hz
  magnetic ODR exceeds 10 Hz and requires no conversion wait in this driver.
  Its I2C errors can still add latency/gaps. Caller output remains unchanged on
  any failure; Sensor Status records it and later successful reads recover.
- GPIO8/9, 100 kHz GY-511 transactions, shared BME280 bus and per-transaction
  100 ms I2C timeout remain unchanged. ESP-IDF serializes synchronous master
  transactions through its bus semaphore. No shared bus reset or extra
  application lock was added. An electrically wedged shared bus can still
  affect both devices; software fault isolation cannot prevent that.

These settings and data-ready semantics follow the
[ST LSM303DLHC datasheet](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf),
particularly tables 20, 27, 35 and the magnetic rate table. Shared transaction
serialization is described in the
[ESP-IDF v5.5.5 I2C thread-safety documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32s3/api-reference/peripherals/i2c.html#thread-safety).

Topic stays `aai16/sensors/gy511/data`. JSON retains `sensor`, `boot_id`, `seq`,
`uptime_ms`, `acceleration_g.{x,y,z}` and `magnetic_field_ut.{x,y,z}`; it adds
`jitter_us`. Acceleration remains in g with the existing three-decimal JSON
precision; no model preprocessing is applied.

In the updated reader, `uptime_ms = acquired_us / 1000`, so it measures
acceleration acquisition rather than submission. Expiry uses that same time.
The legacy six-float publication API remains available with submission-time
semantics and jitter=0; the production reader uses the new timestamped API.

`seq` is the per-boot uint32 publication-attempt counter, including rejected
handoffs/transport drops. Failed I2C reads do **not** increment it. Therefore
use both sequence discontinuities and timestamp gaps, along with Sensor Status.
MQTT remains non-retained QoS 0 with a bounded queue and its existing expiry
policy. Queue admission and successful SDK writes do not prove delivery.
Subscribing at QoS 1 cannot upgrade a QoS 0 publisher.

JSON is temporary collection compatibility. Production MQTT must migrate to
Tribunal-required Protobuf. `decode_json(bytes) -> Sample` is the replaceable
adapter; `Recording` writes the same CSV independently of the wire format.

## Install and connect

Use Python **3.10+** and `paho-mqtt==2.1.0`. Tested here with Python 3.14 and
Paho 2.1.0. The client uses Paho's
[version 2 callbacks](https://eclipse.dev/paho/clients/python/docs/).
Run from the repository root, optionally inside your own virtual environment:

```powershell
python -m pip install -r tools/ml_data_collection/requirements.txt
$env:MQTT_HOST = Read-Host 'Existing broker hostname or IP (no mqtt:// prefix)'
$env:MQTT_USERNAME = Read-Host 'Existing broker username'
```

Use the same broker host, port and account as your working MQTTX connection.
The default port is 1883. For a TLS broker add `--tls` (default port 8883),
`--port` if needed, and optionally `--ca-file` for its private CA. Certificate
verification remains enabled. No code reads or changes firmware credentials.

`--password-prompt` asks without echo. Alternatively set `MQTT_PASSWORD` in
your process environment via your normal secret-management workflow. There is
no password CLI argument or credential file generated by this tool. Anonymous
brokers can omit username/password. No broker credentials are stored in CSV or
summary files. Do not put secrets into scripts or shell command history.

## Exact MQTTX verification procedure

These are **manual checks for hardware running this build**. The implementation
did not flash or access a physical device. Deployment is a separate user action.

1. Connect MQTTX using your existing verified broker connection and a unique
   client ID (not the firmware's `aai16-esp32-s3`). Subscribe to
   `aai16/config/ack`, `aai16/sensors/gy511/data`, and
   `aai16/sensors/+/data`; subscribe separately to
   `aai16/sensors/rc522/event` to check RFID. Use QoS 1 subscriptions if desired.
2. Verify boot logs show GY-511 initialized at 100 Hz/+/-2 g and magnetic 30 Hz.
   Before configuration, acquisition/telemetry deltas should be near 500 ms.
   Check all eight sensor statuses; allow the CO2 sensor's existing warm-up.
3. Publish the following to **`aai16/config/set`**, QoS 1, **Retain OFF**:

   ```json
   {"sensor":"gy511","enabled":true}
   ```

   Wait for `{"sensor":"gy511","status":"applied","enabled":true}`.
   Then publish, separately on the same topic:

   ```json
   {"sensor":"gy511","interval_ms":100}
   ```

   Require `{"sensor":"gy511","status":"applied","interval_ms":100}`.
   Wait for old queued samples to drain (at least a few seconds), then start a
   fresh recording. The dispatcher can take time to handle a command; receipt
   alone is not an applied ACK. Disabled or unavailable sensors reject changes.
4. Inspect a stream of at least 100 messages. Require a stable `boot_id`, forward
   `seq`, increasing `uptime_ms`, plausible raw `acceleration_g`, and integer
   `jitter_us`. Calculate acquisition deltas from `uptime_ms`, not MQTTX arrival
   times. Expect values near 100 ms, not a perfect network-arrival schedule.
   Cross-check the HEALTH `period=100 ms`, `max_jitter`, failures and overruns.
5. Check invalid interval commands one at a time: `0`, `-1`, `99`, `10001`,
   `100.5`, `true`, `null`, and `"100"` must be rejected; acquisition should
   continue at its previous valid interval. `100`, `101`, `499`, `500`, `501`
   and `10000` are valid. Observe 101 rounding to about 110 ms; restore 100
   after tests. A 10000 ms command naturally slows acquisition for this check.
6. Publish `{"sensor":"gy511","enabled":false}` and wait for applied ACK.
   Allow already queued telemetry to drain; fresh reads stop. An interval
   command while disabled is rejected with `sensor_disabled`. Publish
   `{"sensor":"gy511","enabled":true}`; readings resume at the retained
   interval, without a catch-up burst. Repeated enable does not postpone reads.
7. Verify the other seven sensors still run at their own configured rates,
   BME280 reads remain healthy, RFID events still appear when presented, and
   ordinary config ACKs work. Their minimum interval rules are unchanged.
8. In a separate diagnostic session, interrupt the broker connection. The
   collector must exit with a partial CSV/summary and nonzero status. Reconnect
   and start a new session; do not join recordings across the outage. Observe
   firmware MQTT drop counters. Do not mistake gaps for fabricated zero-g data.
9. If checking missing-sensor behavior, power down before changing wiring,
   then start a separate diagnostic run. Missing/init-failed GY-511 must produce
   no fake rows; other available sensors continue. On a running reader, I2C
   errors should raise failure counters and leave gaps, with no cached replay.
   Electrical faults on shared SDA/SCL can affect BME280 as described above.

The collector never sends configuration commands itself. Repeat step 3 after
every ESP32 reboot before recording ML data.

## Record the first REST and MOVING sessions

After setting 100 ms and checking telemetry, place the stationary sensor in one
orientation. Keep it stationary for the entire REST recording:

```powershell
python tools/ml_data_collection/collector.py --label REST --duration 60 --session-id rest_orientation_01 --password-prompt
```

Start moving before the MOVING recording begins, then maintain controlled,
continuous movement in different directions at moderate speeds for the whole
recording. Avoid impacts and full-scale saturation:

```powershell
python tools/ml_data_collection/collector.py --label MOVING --duration 60 --session-id moving_directions_01 --password-prompt
```

Append broker-specific `--port`, `--tls`, or `--ca-file` to both commands when
needed. Recording duration begins after successful subscription acknowledgement.
Use new session IDs for further runs, or omit `--session-id` for a generated
UTC+random ID. Existing files are never overwritten. For REST, change
orientation **between** sessions. For MOVING, use additional independent runs
with different directions and moderate speeds. Keep starts/stops and label
transitions outside recorded intervals. Labels are never inferred from axes.

Output defaults to `tools/ml_data_collection/recordings/`, which is ignored by
Git. Every run writes `<session_id>.csv` and `<session_id>.summary.json`.
`--output-dir` can select another location; its Git exclusion is your choice.
Ctrl+C closes the partial recording and writes a summary. Disconnect, boot
change, another device boot ID, no valid data, and disk errors end recording
with a nonzero status. Reconnect/reboot requires a new session. Only one ESP32
publisher should use the subscribed topic during a recording.

## CSV format, validation and summaries

```csv
timestamp_ms,boot_id,seq,uptime_ms,ax_g,ay_g,az_g,label,session_id,jitter_us
```

`timestamp_ms` and `uptime_ms` are the **same real ESP acquisition timestamp**
from the updated firmware, included under both names for compatibility with the
requested dataset fields. They reset each boot and are not host wall time.
`jitter_us` is the real firmware value; it is blank for older JSON payloads
without that field (reported as `missing_jitter`, not invented as zero).
Session creation/finish UTC times live in the summary, separate from samples.

No actual hardware samples were collected during implementation, so no example
measurement rows or training dataset are supplied. To display **real CSV output**
after the commands above:

```powershell
Get-Content tools/ml_data_collection/recordings/rest_orientation_01.csv -TotalCount 6
Get-Content tools/ml_data_collection/recordings/rest_orientation_01.summary.json
```

Each row preserves decoded X/Y/Z in g, exactly as represented by the existing
telemetry precision. There is **no filtering, scaling, mean subtraction,
gravity removal, normalization, clipping, interpolation or axis remapping**.
Non-numeric/missing/non-finite axes and values outside the representable
rounded +/-2.048 g register range are rejected, never clipped. For example,
a representable 2.048 g value is retained, not changed to 2.0 g. Retained
messages, malformed JSON, duplicate fields and invalid identities/times are
also rejected. A repeated numeric vector with a new sequence/time is valid.

The summary reports received, accepted and dropped samples, invalid/retained
counts, duplicates, out-of-order messages, timestamp errors, boot changes,
sequence gaps, excessive acquisition gaps, queue overflow, shutdown drops,
write errors, minimum/maximum acquisition deltas and maximum absolute jitter.
Under ordinary completion `received = accepted + dropped`. Missing messages
were never received and are not added to that total. Sequence-gap counts are
missing positions in the **accepted** stream; they can include locally rejected
messages as well as firmware/network drops. They cannot identify the exact
location of loss. Loss before the first or after the final received sequence
cannot be inferred. uint32 sequence wrap is handled.

Default excessive-gap threshold is **greater than 150 ms**, configurable using
`--max-gap-ms`. Gapped valid rows are retained with their actual timestamps;
the tool warns and reports gaps without filling them. A sensor still at the
500 ms default therefore produces obvious gap warnings. Recent duplicates are
recognized within a bounded 4096-sequence history; older repeats are rejected
as out-of-order. The MQTT callback queue holds at most 1024 messages; overflow
and unprocessed messages on early shutdown are counted as drops.

Expect roughly 600 samples per uninterrupted 60-second 10 Hz run, but assess
timestamps and counters rather than treating QoS 0 as a guarantee of 600 rows.
Do not use sessions with unexplained gaps, unexpected firmware metadata,
mixed labels or write errors for later training without reviewing them. A disk
failure may leave an incomplete final row/file; the summary/exit status marks
failure rather than promising transactional durability. Each row is flushed
to the OS; abrupt power loss is not covered by that guarantee.

## Future training contract (not implemented)

Approved design: 10 Hz; 20 samples x 3 axes; stride 10; model input `[1,60]`.
The documented model preprocessing is **per-window mean removal -> clamp to
+/-2 g -> divide by 2 g**. Implement it identically in future training and
on-device inference, using the untouched raw CSV as the source.

Flatten ordering is **not yet approved**. Document a consistent decision later,
along with the exact mean definition/axis handling, numeric precision and
discontinuity rules; do not claim those unspecified details were approved.
Split data by session (and respect boot/discontinuity boundaries) **before**
forming overlapping windows, so the 10-sample overlap cannot leak between
training, validation and test sets. No windows, model, training, inference,
TFLM, ESP-DL, model switching, fall detector or full Protobuf migration are
included here.

## Validation and known limits

Baseline before source edits:

- ESP-IDF v5.5.5 build passed.
- MQTT, RC522, LD2450 and MH-Z1911A host suites passed.
- Sensor suite had four application-summary failures expecting 39 lines
  (six sensors) instead of 51 (eight), plus six CO2 scenarios treating slot 6
  as nonexistent. A continue-through-failures baseline run exposed all ten.
- HC-SR04P host suite failed its pull-down assertion: the test omitted
  `ESP_IDF_VERSION_MAJOR`, so neither SDK-specific GPIO setup branch ran.

After implementation:

- All six C host suites passed. Sensor suite: **129 passing cases**, including
  seven new collection runtime scenarios. The eight-reader 65-second simulation
  includes 632 GY reads (500 ms initially, 100 ms after configuration), 32 DHT,
  325 RFID, 590 radar, 325 distance, 1 warmed-up CO2, 65 BME and 260 MAX reads.
- Driver tests cover ODR/range readback, data-ready behavior, identical fresh
  values, acquisition timestamp placement, I2C failures, magnetic saturation,
  unchanged failed output and cleanup.
- Runtime tests exercise 100 ms I2C starts, 101 ms rounding, overruns, missing
  data/failures, recovery, pause/resume, repeated commands, tick wrap, a late
  wake with recorded jitter, and other-sensor isolation.
- MQTT tests cover old/new config limits, invalid types, unchanged unrelated
  floors, timestamp preservation/expiry, axes/units, BME/MAX telemetry,
  sequence/drop accounting, queue pressure and ACK/event behavior. Managed SDK
  tests model both 500 ms and 100 ms GY producers over 15 simulated minutes.
  This is a deterministic model, not measured broker or ESP32 throughput.
- Python suite: **22 tests**, including real Paho 2.1.0 over a local MQTT socket
  fixture for QoS 0 delivery, retained/invalid/duplicate/late data, gaps, CSV
  output and disconnect. Other tests cover labels/sessions, raw values,
  sequence wrap, boot change, bounded queues, Ctrl+C and file errors.
- ESP-IDF v5.5.5 reconfigure/build passed. Image `0xff030` bytes in the existing
  `0x400000` app partition, with `0x300fd0` bytes (75%) free. Registry update
  checks were unavailable in the sandbox; cached locked components built.
  Process-local SDK environment settings resolved the initial ROM-symbol
  helper warning; no global Git settings or SDK files were edited.
- `git diff --check` passed. Physical 10 Hz timing, electrical bus behavior,
  Wi-Fi/broker delivery and representative labelled data remain hardware checks.

Reproduce from the repository root:

```powershell
python tests/sensor_host/run.py
python tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
python tests/rc522_host/run.py
python tests/ld2450_host/run.py
python tests/hcsr04p_host/run.py
python tests/mhz1911a_host/run.py
python -m unittest discover -s tools/ml_data_collection -p 'test_*.py' -v
git diff --check
```

Host C tests require GCC with pthread support. Install the collector requirements
for both real-Paho tests; they explicitly skip when Paho is unavailable.
In your activated ESP-IDF **v5.5.5** terminal, run `idf.py --version` and
`idf.py build`. These commands do not flash hardware. The local validation
installed Paho into ignored `build/ml_python`; that environment used
`$env:PYTHONPATH='build/ml_python'` for Python tests.

## Files changed

| Files | Purpose |
| --- | --- |
| `components/gy511_sensor/gy511_sensor.c`, `include/gy511_sensor.h`, `CMakeLists.txt` | Readback, fresh-data guard, acquisition timestamp, atomic successful output, timer dependency |
| `main/main.c` | Read-owner jitter accounting and timestamped MQTT handoff; existing schedule/default retained |
| `components/mqtt_manager/mqtt_manager.c`, `include/mqtt_manager.h` | GY-only 100 ms floor, acquisition-time API, additive jitter field |
| `tools/ml_data_collection/collector.py`, `requirements.txt`, `README.md` | Standalone recorder, pinned dependency, collection and verification guide |
| `tools/ml_data_collection/test_collector.py`, `test_mqtt_loopback.py` | Offline validation and real Paho loopback integration tests |
| `tests/sensor_host/test_gy511.c`, `test_runtime.c`, `test_app.c`, `test_logging.c`, `run.py` | Driver/scheduler regressions, eight real reader loops, corrected status expectations |
| `tests/mqtt_host/test_mqtt.c`, `test_sdk.c`, `run.py`, `sdk_platform.h` | Payload/config regressions, 10 Hz SDK model and actual managed-component selection |
| `tests/mhz1911a_host/test_runtime.inc` | Correct registry boundary for eight sensors |
| `tests/hcsr04p_host/platform.h` | Explicit IDF 5 host-test version guard |
| `.gitignore` | Exclude default recordings and optional local `.env` (the collector does not load `.env`) |

`dependencies.lock` remains the user's original uncommitted change.
