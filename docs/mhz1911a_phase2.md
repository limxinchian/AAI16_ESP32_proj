# MH-Z1911A Phase 2: MQTT CO2 telemetry

This is the Phase 2 implementation record. [Phase 3](mhz1911a_phase3.md) adds
runtime interval and enable controls, retaining this telemetry schema and UART
driver. Statements below about unsupported configuration describe Phase 2 only.

## Baseline and scope

The user reports initial continuous Phase 1 hardware verification: UART2
TX15/RX16, 9600/8N1, a 60-second warm-up, approximately five-second acquisition,
READY status, readings around 1429-1435 ppm, and no UART/checksum errors in those
initial logs. The model-specific UART protocol remains unconfirmed by the
manufacturer; accuracy and long-term reliability are not established.

Phase 2 preserves that working driver, range/checksum validation, acquisition
schedule, priority 3, 4096-byte reader stack, status transitions and existing
SENSOR logs. The only reader behavior added is a successful-read MQTT handoff.
No calibration, runtime interval/enable controls, diagnostic modes or ML were
added. MQTT configuration for MH-Z1911A remains unsupported.

## Topic, schema and conventions

Topic: **`aai16/sensors/mhz1911a/data`**, **QoS 0**, **retain=false**.

Example MQTTX payload (illustrative boot ID/timestamp/sequence, not a new
hardware capture):

```json
{"sensor":"mhz1911a","boot_id":"12345678","seq":1,"uptime_ms":65000,"co2_ppm":1433}
```

| Field | Meaning |
| --- | --- |
| `sensor` | Literal `mhz1911a` |
| `boot_id` | Existing platform `esp_random()` boot identifier, eight lowercase hexadecimal characters |
| `seq` | Per-sensor consumed publication-attempt counter, uint32, first attempt 1; wraps through 0 |
| `uptime_ms` | Successful acquisition completion time in monotonic microseconds divided by 1000 |
| `co2_ppm` | Validated integer concentration, 400..5000 inclusive |

The serializer follows the existing radar/distance fixed-buffer `snprintf`
convention. It uses a 128-byte payload buffer, checks truncation, allocates no
JSON tree and introduces no fields beyond the proposed schema. Identical ppm
on successive acquisitions is valid; distinct acquisition timestamps identify
distinct samples.

## Producer and publisher

After `mhz1911a_sensor_read` succeeds, the reader records its normal status and
calls `mqtt_manager_update_mhz1911a_snapshot(reading.co2_ppm, completed_us)` once.
Warm-up, timeout, checksum/range/structure failures and incomplete reads never
call it. MQTT results do not feed into sensor health or trigger extra reads.

The API copies the scalar ppm and 64-bit timestamp together under the existing
MQTT stats/snapshot spinlock. It performs no allocation, SDK call, logging or
queue wait. Valid new input replaces one pending sample. The lock also guards
the last accepted timestamp: duplicate or older handoffs are rejected even if
their original sample has already been consumed. Invalid values/timestamps do
not replace pending data.

The existing `mqtt_tx` worker consumes this mailbox after radar/distance service
and only when the shared ACK/telemetry and RFID queues have no waiting work.
It takes a local copy and clears pending under the lock, then releases the lock
before formatting or calling ESP-MQTT. Producer updates can continue while an
SDK call holds that local copy. There is one pending sample plus at most one
in-flight copy, no CO2 FIFO, no new lock, task, client or broker connection.

Publication is sample-driven at the reader's nominal 5000 ms acquisition rate;
there is no separate five-second publisher timer that could age a new sample
while waiting for another deadline. The existing publisher's idle wait is at
most one second, so a handoff is normally discovered within that interval.
Queued work or SDK stalls can delay/drop data. Repeated worker passes without a
new handoff do not publish anything. Delayed work never accumulates historical
samples for a catch-up replay; arrival spacing can vary under transport stalls.

## Freshness, outages and sequences

This path uses the same **2000 ms maximum age** as existing latest-snapshot
telemetry, independently of the five-second acquisition interval. Age is checked
after consumption and again just before calling the SDK. Samples aged **>=2000
ms** are dropped. The timestamp is never replaced with publication time.

When disconnected, a consumed sample is dropped and counted offline. If the
publisher is stalled or higher-priority queue work is pending, only the latest
sample remains in RAM and newer acquisitions replace it. On reconnect:

- Consumed offline, stale or failed attempts are never resent.
- An unconsumed sample younger than two seconds can still be published once,
  retaining its original measurement timestamp.
- An older pending sample is dropped; the next valid acquisition supplies new
  data. There is no persistent storage or historical replay.

The reader's next failed acquisition is at least five seconds later, so a
previously pending success is already beyond this freshness window. It cannot
be refreshed or repeatedly published by failures.

`seq` increments when the publisher consumes a pending sample, matching the
radar/distance snapshot attempt semantics. Offline, stale, encoding-failed and
SDK-failed consumed attempts all advance it. Producer updates, replacements,
invalid/duplicate handoffs and busy deferrals do not. Consequently, sequence gaps
can occur and are not exclusively evidence of network loss. Replacement of an
unpublished sample is tracked separately and need not produce a sequence gap.
Reboot creates the platform's new boot ID and resets RAM counters.

As with other QoS 0 telemetry, `publish_ok` means the SDK reported local publish
success, **not broker acknowledgement or subscriber receipt**. SDK failures are
counted and not retried by the application. The existing SDK mutex can stall the
publisher; no hard upper bound on delivery latency is promised. A sample admitted
while fresh may become old while inside the SDK, and cannot be cancelled there.
Consumers should retain `(sensor, boot_id, seq)` and measurement uptime when
assessing identity/freshness; uptime is not UTC wall-clock time.

## Logging

Existing SENSOR DATA/ERROR/HEALTH logging and its minute summaries are preserved.
There is no per-sample JSON dump. Example SENSOR output (values illustrative):

```text
SENSOR: mhz1911a | HEALTH | state=UNINITIALIZED warmup=60000 ms UART2 TX=15 RX=16 9600/8N1 period=5000 ms
SENSOR: mhz1911a | DATA | co2=1433 ppm read=30.0 ms
SENSOR: mhz1911a | HEALTH | state=READY ok_total=13 failed_total=0 consecutive=0 period=5000 ms timeouts_total=0 checksum_failed_total=0
```

The existing minute `AAI16_MQTT` summary now also reports:

```text
AAI16_MQTT:   mhz1911a_snapshot_updates=13 coalesced=0 invalid_updates=0 nonnew_updates=0 busy_deferrals=0 pending=0 max_age_ms=2000
AAI16_MQTT: mhz1911a attempts=13 queued=0 publish_ok=13 enqueued=0 max_tx_call_us=20000
AAI16_MQTT:   drops: offline=0 queue_full=0 invalid=0 stale=0 outbox_full=0 enqueue_failed=0 publish_failed=0 cancelled=0
```

Counters/timing above are examples, not observations from your new firmware.
`updates` counts accepted handoffs; `coalesced` counts replacements;
`invalid_updates` counts bad values/timestamps; `nonnew_updates` counts duplicate
or older timestamps; `busy_deferrals` counts service passes delayed by queued
work. `queued`/`enqueued` stay zero for this path, since it uses neither the shared
TX FIFO nor the QoS 1 outbox. SDK failure warnings use the existing MQTT style,
throttled to once per ten seconds. Sensor READY/health is independent of MQTT.

## Files changed

| Files | Phase 2 change |
| --- | --- |
| `components/mqtt_manager/mqtt_manager.c` | Topic, latest snapshot, validation/deduplication, bounded JSON, existing worker service and diagnostics |
| `components/mqtt_manager/include/mqtt_manager.h` | Documented snapshot API and attempt/drop contract |
| `components/mqtt_manager/CMakeLists.txt` | Private MH-Z1911A component dependency for range constants |
| `main/mhz1911a_reader.c` | Include MQTT header and hand off only successful readings |
| `tests/mqtt_host/test_mhz1911a.c` (new) | Payload, lifecycle, real reader/manager integration and concurrent snapshot tests |
| `tests/mqtt_host/test_mqtt.c`, `platform.h`, `run.py`, `test_hcsr04p.c` | CO2 fixture/SDK hooks, pthread critical-section double and required include path; preserve existing cases |
| `tests/mhz1911a_host/run.py`, `test_reader.c`, `test_runtime.inc` | Success-only handoff, exact completion timestamp, failed handoff and six-sensor isolation checks |
| `docs/mhz1911a_phase2.md` (new), `mhz1911a_phase1.md` | Current telemetry guide and link from historical Phase 1 notes |

The Phase 1 driver component and Sensor Status files are byte-for-byte unchanged
from the start of Phase 2. Earlier uncommitted Phase 1 changes remain in the
working tree. Credentials, local sdkconfig/defaults and partition configuration
are unchanged. Work remains on `feature/mhz1911a-uart`; no branch operation,
commit, push, merge or flash was performed.

## Validation

All six host suites passed:

```powershell
python tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
python tests/mhz1911a_host/run.py
python tests/sensor_host/run.py
python tests/rc522_host/run.py
python tests/ld2450_host/run.py
python tests/hcsr04p_host/run.py
```

New MQTT coverage includes exact topic/QoS/retain/schema, integer encoding,
buffer limits, sequence/drop/wrap behavior, invalid/duplicate/stale data,
disconnect/reconnect, SDK failure, bounded coalescing, queued-work priority,
all existing sensor publications and the actual CO2 reader during a synthetic
SDK stall. Two parallel producers issue 20,000 updates while a third thread
checks coherent ppm/timestamp snapshots. Host locks model critical sections;
these are not physical ESP32 SMP, Wi-Fi, broker or sensor verification.

The permanent sensor suite passes 45 driver and 14 reader scenarios. The platform
suite retains seven 125-second six-sensor/configuration scenarios, now also
checking continued acquisition/status despite rejected MQTT handoffs. Existing
installed-SDK publication/outbox tests also pass. Logs are retained under
`build/mhz1911a_phase2_*.log`.

ESP-IDF 5.5.5 `idf.py build size` completed successfully without compiler warnings.
The padded application binary is 1,034,992 bytes, within the existing 4 MiB
factory partition; the partition-table binary hash is unchanged. Linked-symbol
inspection confirms the reader and new handoff API are present. `git diff
--check`, whitespace checks on new files and driver/configuration hash checks
passed. Phase 2 has not yet been flashed or hardware-tested.

## Manual ESP32 and MQTTX verification

1. Keep the verified Phase 1 wiring/power/mode selection unchanged. Open an
   ESP-IDF 5.5.5 terminal in the project and run:

   ```powershell
   idf.py --version
   idf.py build
   ```

2. In MQTTX, connect to the same configured broker using your existing broker
   credentials/TLS settings. Use a distinct client ID such as `aai16-mqttx-test`,
   not the ESP32's `aai16-esp32-s3`. Subscribe at QoS 0 to:

   ```text
   aai16/sensors/mhz1911a/data
   ```

   Subscribe to the existing sensor topics as well to check coexistence.
3. Power the sensor before ESP32 warm-up starts. Manually flash and monitor,
   replacing `COM7` with your actual port:

   ```powershell
   idf.py -p COM7 flash monitor
   ```

   No new menuconfig setting is needed. Enable Monitor logging with Ctrl+T,
   then Ctrl+L using the default shortcuts.
4. Verify no new CO2 message during warm-up. After a validated measurement,
   expect one JSON message per successful acquisition, normally about five
   seconds apart. Compare integer ppm with Monitor, `boot_id` with the MQTT
   summary, and `uptime_ms` with acquisition completion uptime. Verify QoS 0,
   retain=false, and no repeated `(boot_id, seq)` from this firmware. Adjacent
   samples may legitimately have equal ppm.
5. Keep both devices powered and, if your wiring permits an isolated safe signal
   change, disconnect only sensor TX from ESP32 GPIO16. Observe timeout/status
   changes and absence of new CO2 messages; keep power, ground, R/T and HD intact.
   Restore the signal. Valid measurements should restore READY and publications
   without replaying failed measurements. Check all other sensors/MQTT continue.
6. Exercise an actual ESP32 broker outage on your test network for at least
   30 seconds (for example, stop your local test broker, then restart it).
   Closing MQTTX alone does not disconnect the ESP32. During the outage, verify
   CO2 DATA and five-second acquisition continue and sensor health remains based
   on UART results. After restoring the broker, expect fresh data without a
   historical burst; an unconsumed sample younger than two seconds is eligible.
   Look for offline/stale/failure/coalescing counters and possible sequence gaps.
7. Run for at least 15 minutes, preserve both Monitor and MQTTX output, and check
   the other five sensors, MQTT diagnostics, stack watermark and acquisition
   jitter/errors. A subsequent intentional reboot should repeat warm-up and
   create a new boot ID with a new attempt sequence. Exit Monitor with Ctrl+].

These checks establish initial integration behavior only. QoS 0 can lose data;
snapshot replacement/drop policies intentionally favor fresh observations over
history. Long-term stability, reference measurement accuracy and the exact
manufacturer UART specification remain separate validation work before relying
on the measurements or adding Phase 3 runtime configuration.
