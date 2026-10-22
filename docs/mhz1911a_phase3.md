# MH-Z1911A Phase 3: MQTT runtime configuration

Implemented on `feature/mhz1911a-uart`, preserving the existing uncommitted
Phase 1/2 work. No branch operations, commits, pushes, merges or hardware flashing
were performed. UART driver source/header, credentials, sdkconfig and the
partition table were preserved. No new UART commands, clients, tasks or queues.

The user reports initial hardware verification of Phase 1/2: UART2 TX15/RX16,
9600/8N1, valid nine-byte responses, 60-second warm-up, five-second reads,
READY, and three consecutive real MQTTX readings. Phase 3 validation below is
host simulation and firmware compilation; its runtime controls still need
device verification. Accuracy, long-term stability and physical outage/recovery
behavior remain unverified. Manufacturer confirmation of this exact module's
UART protocol is still pending.

## Files changed in Phase 3

| Files | Change |
| --- | --- |
| `main/mhz1911a_reader.c`, `.h` | Live control handle, notification waits, interval/enable commits, warm-up-aware scheduling, disabled health reporting |
| `main/main.c`, `main/sensor_config.h` | Sixth-sensor dispatch route; notification encoding comment updated to 60000 ms |
| `components/mqtt_manager/mqtt_manager.c`, `include/mqtt_manager.h` | Configuration rule/limits, snapshot gate/generation, cancellation at admission and diagnostics |
| `components/sensor_status/sensor_status.c`, `include/sensor_status.h` | Probe-owner pause/resume API; existing five-sensor enable behavior retained |
| `tests/mhz1911a_host/test_reader.c`, `test_runtime.inc`, new `test_config_runtime.inc` | Registry/lifecycle guards and real six-sensor scheduling/control tests |
| `tests/mqtt_host/test_mqtt.c`, `test_mhz1911a.c`, new `test_mhz1911a_config.c` | Task handle double, Phase 3 parser/FIFO/ACK/admission/concurrency tests |
| `tests/sensor_host/test_app.c`, `test_runtime.c`, `run.py` | Sixth-owner notification/ACK assertions and 13 configuration scenarios |
| `docs/mhz1911a_phase2.md`, this guide | Historical scope pointer and Phase 3 behavior/verification record |

The broader working-tree diff includes earlier Phase 1/2 integration; it is not
all new Phase 3 work. Other five sensor reader logic and the CO2 UART driver were
not rewritten. Registry capacity remains eight, with MH-Z1911A in slot 5.

## Commands, ACKs and ownership

Send non-retained JSON on `aai16/config/set`, one operation per message:

```json
{"sensor":"mhz1911a","interval_ms":10000}
{"sensor":"mhz1911a","enabled":false}
{"sensor":"mhz1911a","enabled":true}
```

Interval default: **5000 ms**. Allowed inclusive range: **5000–60000 ms**.
The configuration table already supports per-sensor bounds; 10000 ms is not a
global encoding limit. 60000 fits the existing uint32 notification word below
its high-bit enable marker. This floor respects the existing approximately
five-second measurement cycle; the upper bound is an application scheduling
policy, not a sensor range/configuration command. Numeric integral values such
as `5000.0` follow existing parser conventions; fractions are rejected.

The MQTT callback validates and appends to the existing four-entry FIFO with
zero wait. It does not apply controls or send a success ACK. `app_main` forwards
the FIFO head via `eSetValueWithoutOverwrite`; if the notification is occupied,
the head remains queued. The live CO2 reader commits controls between complete
UART transactions, at most four per loop pass, then submits its applied ACK.
No callback waits for the reader. Commands for the same sensor retain FIFO
application/owner-ACK order; independent owners can finish in different orders.
Immediate parser/queue rejections may precede ACKs for older pending commands.

The handle becomes available under a short lock only after UART initialization
succeeds. A command reaching dispatch before that, or after init/task creation
failure, rejects with `sensor_unavailable`. Startup warm-up and READ-stage
UNAVAILABLE have a live owner and accept controls.

Exact existing applied-ACK format on `aai16/config/ack`, QoS 1, retain=false:

```json
{"sensor":"mhz1911a","status":"applied","interval_ms":10000}
{"sensor":"mhz1911a","status":"applied","enabled":false}
{"sensor":"mhz1911a","status":"applied","enabled":true}
```

ACK submission follows registry, schedule and snapshot-gate commit. The
existing protocol has no separate `phase` field, acceptance ACK, request ID or
configuration version field. Representative rejections:

```json
{"sensor":"mhz1911a","status":"rejected","reason":"sensor_disabled","interval_ms":10000}
{"sensor":"mhz1911a","status":"rejected","reason":"interval_ms must be an integer from 5000 to 60000","interval_ms":4999}
{"sensor":"mhz1911a","status":"rejected","reason":"queue_full","enabled":false}
```

| Input/failure | Existing reason/behavior |
| --- | --- |
| Zero, negative, fraction, out of range/non-finite number | `interval_ms must be an integer from 5000 to 60000` |
| Wrong interval type | `missing_or_invalid_interval_ms` |
| Non-Boolean enable | `enabled must be a boolean` |
| Both operations or repeated operation key | `ambiguous_configuration` |
| No operation | `missing_configuration_operation` |
| Missing/duplicate/invalid sensor | `missing_or_invalid_sensor` |
| Malformed JSON | `invalid_json`, sensor null |
| Oversized/fragmented payload | `invalid_payload_size_or_fragmented` |
| Retained command | Ignored, without ACK |
| Control FIFO unavailable/full | `queue_full` |
| No live initialized driver/task | `sensor_unavailable` |
| Interval applied for consideration while disabled | `sensor_disabled`; old interval retained |

ACK transport retains the existing bounded shared TX queue and QoS 1 outbox.
Queue saturation, disconnection or outbox failure can prevent ACK delivery even
after a successful commit; counters report these outcomes and configuration is
not rolled back. Broker acceptance is not subscriber receipt. Do not infer
failure solely from a missing ACK, or flood retries. QoS 1 can redeliver ACKs;
separate duplicate commands are processed separately because the protocol has
no request deduplication.

## Acquisition, pause and health

The sole UART owner remains priority 3 / stack 4096 bytes. UART2, pins, 9600/8N1,
request/checksum/range checks and bounded transaction timeouts are unchanged.

- Without controls, first read follows the original 60000 ms warm-up; subsequent
  reads use the default 5000 ms actual-start cadence.
- Each interval command commits only after any current read finishes. It clears
  old pending telemetry and sets the next read to `max(commit time + interval,
  original warm-up deadline)`. Even repeating the interval restarts this
  deadline. Regular cadence then follows actual starts, without catch-up bursts.
- Disable lets the current bounded transaction and its success-only handoff
  finish before committing. Commit stops new reads and handoffs, clears pending
  telemetry, and sets DISABLED. It does not power off, deinitialize, flush for
  configuration, calibrate or otherwise command the physical sensor.
- Enable retains the last applied interval, clears old data and waits one full
  interval, also respecting the original warm-up deadline. It sets UNINITIALIZED
  until a new result establishes communication health. It does not restart the
  60-second warm-up because the sensor remains powered.
- Repeated `enabled:false` or `enabled:true` is idempotent: it does not postpone
  a pending read, clear a fresh sample, or erase counters/history.
- All cumulative successes/failures, consecutive failures, last error and last
  success timestamp survive pause/resume. A fresh valid read establishes READY
  and resets consecutive failures. After a past success, one/two failures are
  DEGRADED and three consecutive failures are UNAVAILABLE. Before any success,
  a failure remains UNAVAILABLE. READ failures can recover; init/task failures
  are not repaired by configuration.
- Notification waits end at the next read or minute HEALTH deadline, and wake
  on controls. HEALTH continues about every 60 seconds while disabled. No busy
  polling is added. Existing error/recovery throttling stays in place.

Driver reads remain non-overlapping. A late wake starts one read; an overrun
skips missed periods and waits a full period after completion. Scheduler/tick
rounding and other task latency can make deadlines later, never intentionally
earlier. Commands during a read wait for that bounded read plus dispatch and
scheduling latency. Repeated interval commands can deliberately keep moving
the deadline forward, consistent with HC-SR04P's reset-on-commit convention.

## Telemetry and concurrent control

Topic/payload remain `aai16/sensors/mhz1911a/data`, QoS 0, retain=false:

```json
{"sensor":"mhz1911a","boot_id":"12345678","seq":1,"uptime_ms":60030,"co2_ppm":1433}
```

Only successful new reads hand off a timestamped sample. Publication follows
acquisition, with no separate rate timer. A single replaceable snapshot remains
bounded even offline or behind other MQTT work. Timestamp is never refreshed
by configuration/reconnect, and data aged **>=2000 ms** at SDK admission drops.
Failed/offline/consumed samples are not retried or replayed.

The existing snapshot lock now also protects an enabled gate and uint32
generation. Every interval commit and enable-state edge clears pending data and
increments the generation. A publisher that already copied an old sample must
check gate/generation again immediately before SDK admission. Mismatch counts
as `cancelled`. This also handles disable followed rapidly by enable: merely
seeing enabled again cannot resurrect an older copied sample.

Admission is a short, locked decision; no SDK call occurs under a spinlock.
No new admission can succeed after disable commits. Work admitted before commit
may enter/finish the SDK call and reach the broker afterward, including when
preempted between admission and the call. It cannot be recalled, and a late
arrival alone does not prove a fresh post-disable submission. Likewise, an
interval change cannot retract already admitted work. This matches the existing
distance/radar admission model without blocking the UART owner on the SDK.

`seq` still counts consumed publication attempts (including stale, offline,
cancelled or failed attempts), starts at 1 and wraps as uint32. Clearing an
unconsumed snapshot, replacements, invalid/disabled/duplicate handoffs and busy
deferrals do not consume sequence numbers. Transport outcomes never affect
sensor health. Preserve `(sensor, boot_id, seq)` when comparing captures.

## Validation performed

All six runners passed, with 248 `PASS:` reports (some reports group assertions):

| Runner | Reports |
| --- | ---: |
| `tests/mhz1911a_host/run.py` | 59 |
| `tests/sensor_host/run.py` | 120 |
| `tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf` | 24 |
| `tests/rc522_host/run.py` | 17 |
| `tests/ld2450_host/run.py` | 14 |
| `tests/hcsr04p_host/run.py` | 14 |

Coverage includes range endpoints/types/duplicates, real actual-read schedules,
60-second maximum intervals, controls during warm-up, idle and successful/
timeout in-flight disable, retained intervals, no pre-success READY on resume,
counters/history, disabled HEALTH, rapid commands with busy notifications,
owner ACK timing/order, task/init failure, recovery, and all five other owners
continuing over simulated 125-second runs. MQTT tests use the real parser,
mailbox and publisher with bounded queues, SDK timing hooks for generation
races, real parallel producer/config threads (10000 controls), offline/reconnect,
two-second freshness boundaries, in-flight SDK completion and allocation/drop
accounting. Existing dual-UART driver and installed ESP-MQTT API tests also pass.

Host doubles do not establish ESP32 SMP timing, physical UART/electrical
behavior, broker/network delivery, stack margin or long-term reliability.

`idf.py --version`: **ESP-IDF v5.5.5**. `idf.py build size`: **passed** (exit 0).
App binary: **1,036,112 bytes**, `0xfcf50`, in the unchanged `0x400000` app
partition, **75% free**. No compiler warnings/errors in this build. PowerShell
labels the size tool's stderr size-summary text `NativeCommandError`; the tool
and build succeeded. `git diff --check` and whitespace checks of new untracked
source/tests/docs passed. Hash comparisons confirmed preserved files and the
unchanged partition binary. Logs: `build/mhz1911a_phase3_*host.log` and
`build/mhz1911a_phase3_build.log`.

## Manual MQTTX/device verification

Perform this after you choose to install the Phase 3 firmware; no flashing or
broker/device commands were executed during this implementation. Keep the
existing verified wiring and power supply. Connect MQTTX to your configured
broker without changing credential files. Subscribe to `aai16/config/ack`,
`aai16/sensors/mhz1911a/data`, and the existing five sensors' topics. Publish all
commands to `aai16/config/set`, **retain off**, preferably QoS 1, one at a time
unless deliberately testing a burst. Save both Monitor and MQTTX timestamps.

1. On boot, verify UNINITIALIZED/60000 ms warm-up, then READY and fresh CO2
   messages about every five seconds. Allow publisher/network latency; use
   `uptime_ms` differences to assess acquisition, not MQTTX arrival time alone.
2. Send `{"sensor":"mhz1911a","interval_ms":10000}`. Require its exact applied
   ACK and CONFIG log. The next UART read starts at least ten seconds after
   commit, then continues about every ten seconds. Check fresh acquisition
   timestamps and continued other-sensor activity.
3. Send `{"sensor":"mhz1911a","enabled":false}`. After the applied ACK/CONFIG
   log, verify DISABLED and no new UART requests/DATA. Existing admitted MQTT
   work may finish once. Wait over a minute: HEALTH must still report the
   retained 10000 ms interval and unchanged read counters.
4. While disabled, send `{"sensor":"mhz1911a","interval_ms":60000}`. Require
   `sensor_disabled`. Then send `{"sensor":"mhz1911a","enabled":true}`. The
   interval remains 10000 ms; no immediate/cached sample and no READY until a
   new successful read. Repeat enabled=true during that wait: it must not move
   the deadline again.
5. Test limits 5000 and 60000 while enabled, allowing enough observation time.
   Try 0, -1, 4999, 60001, 5000.5, a quoted number and mixed enabled/interval
   fields. Require the rejection reasons above and no schedule change.
6. On a separate boot, disable about ten seconds into warm-up and enable about
   thirty seconds in. No request before the original sixty-second deadline.
   Disable at about 55 seconds and enable near 58 seconds on another boot: the
   next read is at least one retained interval after enable (about 63 seconds
   at the default), not immediately at the original deadline.
7. With the sensor safely disconnected while power is off, boot to exercise
   missing-response timeouts; request disable while a bounded read is pending.
   Confirm applied ACK only after the read finishes and no further requests.
   For recovery, restore verified wiring only under appropriate power-off
   conditions; do not hot-plug pins to create errors. Host tests cover precise
   in-flight timings that may be difficult to reproduce manually.
8. Interrupt broker connectivity while leaving the device running. Check
   continued UART acquisition/other-sensor health, then reconnect and verify
   fresh timestamps without backlog. Repeat with a committed disabled state:
   reconnect must not resume CO2 telemetry until explicitly enabled and read.
9. Optionally send the ordered burst disable -> interval -> enable -> interval.
   Check disabled rejection and ordered owner ACKs. A larger burst can yield
   `queue_full`; that immediate rejection can arrive before pending applied
   ACKs. Finish with enabled=true and interval_ms=5000, waiting for each ACK.

Illustrative Monitor excerpts, not new hardware captures:

```text
SENSOR: mhz1911a | CONFIG | interval_ms=10000
SENSOR: mhz1911a | CONFIG | enabled=false
SENSOR: mhz1911a | HEALTH | state=DISABLED ok_total=3 failed_total=0 consecutive=0 period=10000 ms enabled=false timeouts_total=0 checksum_failed_total=0
SENSOR: mhz1911a | CONFIG | enabled=true
SENSOR: mhz1911a | DATA | co2=1433 ppm read=30.0 ms
SENSOR: mhz1911a | HEALTH | state=READY ok_total=4 failed_total=0 consecutive=0 period=10000 ms enabled=true timeouts_total=0 checksum_failed_total=0
```

Health lines remain minute-based; the READY summary is not emitted for every
sample. No automatic fallback commands or calibration were added. If repeated
reads fail, diagnose the existing wiring/power/protocol path or seek manufacturer
confirmation; runtime configuration does not prove protocol compatibility or
recover broken initialization. Settings are RAM-only and reset to enabled /
5000 ms on the next boot, following the rest of the platform.
