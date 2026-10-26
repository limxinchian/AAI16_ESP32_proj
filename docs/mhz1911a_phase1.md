# MH-Z1911A Phase 1: continuous UART acquisition

Phase 2 adds MQTT telemetry to the unchanged acquisition design. See the
[Phase 2 guide](mhz1911a_phase2.md) for current publication behavior and hardware
verification. The Phase 1-only scope and validation below describe that earlier
implementation stage.

## Evidence and scope

The user reported one successful physical-module exchange at UART2, TX GPIO15,
RX GPIO16, 9600/8N1: request `FF 01 86 00 00 00 00 00 79`, response
`FF 86 06 EA 43 00 00 00 47`. The latter passes the checksum and decodes to
1770 ppm (`0x06EA`). It is retained as a host-test fixture. This supports this
request/response method on that module; it does not prove continuous stability,
accuracy, every firmware revision's compatibility, or undocumented side effects.
Manufacturer confirmation of the exact UART protocol remains unavailable.

The [third-party library](https://github.com/tobiasschuerg/MH-Z-CO2-Sensors)
provided the original request and decoding evidence. The
[Winsen V1.3 manual](https://www.sensor-test.de/assets/Fairs/2025/ProductNews/PDFs/MH-Z1911A-Infrared-CO2-Sensor-Manual-V1.3.pdf)
documents one-minute preheat and a 400-5000 ppm product option. Its T90 response
specification concerns gas response, not the UART transaction timeout.
This implementation uses the user's stated 400-5000 ppm module range and
requested 5000 ms sampling cycle. Early measurements may still be stabilizing;
neither plausible numbers nor a checksum establish absolute accuracy.

Phase 1 provides local acquisition, DATA/ERROR/HEALTH logs and the sixth registry
slot. It adds no CO2 MQTT publications, runtime configuration, enable/disable,
calibration, reset, range-changing commands or ML behavior. Existing MQTT and
the other five sensors are unchanged.

## Architecture and changes

| Files | Responsibility |
| --- | --- |
| `components/mhz1911a_sensor/CMakeLists.txt`, `include/mhz1911a_sensor.h`, `mhz1911a_sensor.c` | New UART owner, exact-frame decoder, bounded transactions and cumulative UART diagnostics |
| `main/mhz1911a_reader.c`, `.h` | New independent acquisition task, warm-up, scheduling, logs and registry reporting |
| `main/main.c`, `main/CMakeLists.txt` | Start and build the permanent reader unconditionally |
| `components/sensor_status/include/sensor_status.h`, `sensor_status.c` | Add `SENSOR_MHZ1911A` in slot 5 of 8 and an opt-in readiness/recovery acquisition API; retain other owners' behavior |
| `tests/mhz1911a_host/run.py`, `mhz_platform.h`, `test_driver.c`, `test_reader.c`, `test_runtime.inc` | Permanent driver/reader/status and platform coverage, reusing diagnostic test doubles and cases |
| `tests/sensor_host/run.py`, `test_app.c`, `test_runtime.c` | Sixth reader in application tests; longer histories for 125-second isolation runs |
| `docs/mhz1911a_phase1.md` | Current implementation and manual verification instructions |

Removed the obsolete `main/mhz1911a_uart_diag.c/.h`,
`main/mhz1911a_uart_experiment.c/.h`, `main/Kconfig.projbuild`, conditional build
hooks, two old activation documents, and temporary tests `test_diag.c`,
`test_experiment.c`, `runtime.inc`, `experiment_runtime.inc`. Checksum, strict
frame validation, bounded waits, UART ownership/error handling, stale-data and
fragmentation tests were retained or adapted. The earlier speculative byte-5
status check is replaced by treating bytes 4..7 as opaque; their model-specific
meaning has not been established. The concentration range is now 400..5000.

Old diagnostic/experiment selections were removed from local `sdkconfig` and
`sdkconfig.old`. There are no remaining source/build references that can activate
either mode, even if an old configuration copy contains those symbols. There is
no replacement menu switch. **A normal build now sends periodic read requests
after warm-up.** Credentials, `sdkconfig.defaults`, unrelated local settings and
the partition layout are preserved.

## UART transaction

The reader alone owns the driver. Init reserves UART2 exclusively and refuses an
already installed driver. UART1 TX17/RX18 remains owned by LD2450. UART2 uses a
512-byte RX ring, no TX ring and an eight-entry SDK event queue to detect UART
errors/overflow. No extra application queue or communication task is created.

Each acquisition:

1. Checks queued UART faults, counts/discards buffered stale input, and waits at
   most 100 ms for an empty TX FIFO. A 20 ms quiet guard rejects input arriving
   across the request boundary. Preflight faults skip transmission.
2. Enqueues exactly `FF 01 86 00 00 00 00 00 79` once using `uart_tx_chars`.
   A short/failed write fails this read without appending bytes or retrying.
   The SDK TX mutex is uncontended under the single-owner contract.
3. Waits at most 100 ms for TX completion, within a two-second response deadline
   measured from enqueue. Reads nonblocking fragments, yielding a tick between
   polls. An exact nine-byte frame must be followed by a 20 ms trailing guard.
4. Rejects noise prefixes, echo, extra bytes, duplicates in the guard, wrong
   header/command, bad checksum, incomplete responses and transport faults.
   It does not extract plausible subframes from corrupt input. Checks require
   `FF 86`, bytes 1..8 summing to zero modulo 256, and bytes 2/3 decoding within
   400..5000 inclusive. Other fields remain uninterpreted.
5. Returns output only on success. Failures preserve the caller's previous
   output, discard pending input and update counters. The reader logs no stale
   measurement; it waits for the next scheduled acquisition.

Normal software waits total at most about 2120 ms plus scheduling/tick rounding;
the response window itself is two seconds. Neither is a manufacturer-confirmed
UART reply deadline. There is no request identifier in this observed format:
flushing, a quiet boundary and a five-second cadence reject known stale bytes,
but cannot prove that an arbitrarily late old response arriving just after a new
request belongs to that request. Hardware testing must check response timing.
Unsolicited streams or bytes arriving after the trailing guard are not interpreted
as additional measurements and are discarded at the next boundary.

## Scheduling, status and logging

The task uses provisional **priority 3 / 4096-byte stack**, matching the other
independent peripheral readers. Its measured stack watermark appears in HEALTH
logs. It waits at least 60000 ms after UART setup, assuming the sensor is already
powered, with `vTaskDelay`; no requests or readings are produced during warm-up.
Two concise startup HEALTH lines explain warm-up and the evidence limitation.

Acquisition starts are anchored to the actual start plus 5000 ms. A delayed wake
starts a new schedule; an acquisition/logging overrun schedules the next read
5000 ms after completion. It never overlaps requests or catches up missed reads
with a burst. Timing uses monotonic microseconds, with delays rounded to ticks.

| Condition | Registry behavior |
| --- | --- |
| Warm-up or waiting for first acquisition | UNINITIALIZED, no invented successes |
| First valid response | READY, initialized=true, successful completion timestamp |
| Failure before any valid response | UNAVAILABLE, initialized=false, counted failure |
| First/second consecutive failure after success | DEGRADED, last successful timestamp retained |
| Third consecutive failure after success | UNAVAILABLE; scheduled probes continue |
| Valid response after acquisition failure | READY; consecutive count resets, cumulative history retained |
| UART setup or task creation failure | UNAVAILABLE at INIT/TASK stage; other readers continue; no automatic resource reinitialization |

The three-failure threshold is a platform policy. `sensor_status_record_probe`
allows recovery only from acquisition unavailability; it cannot revive failed
init/tasks or DISABLED slots. Existing `sensor_status_record_read` behavior is
unchanged. No MQTT path enables/disables or changes the period of MH-Z1911A.

Valid readings produce one DATA line. Registry failure/recovery logs are
throttled to at most one warning and one recovery per ten seconds; identical
repeated errors are silent. Approximately once a minute the reader reports state,
success/failure/consecutive counts, TX/RX timeouts, checksum/invalid/partial
failures, UART/overflow/API faults, request and discarded-byte counts, maximum
acquisition time, wake lateness, jitter, overruns and stack watermark. The normal
registry summary also includes slot 5. Raw frames are not logged per sample.

## Expected Monitor output

ESP-IDF adds severity and uptime prefixes. Startup includes:

```text
SENSOR: mhz1911a | HEALTH | state=UNINITIALIZED warmup=60000 ms UART2 TX=15 RX=16 9600/8N1 period=5000 ms
SENSOR: mhz1911a | HEALTH | UART method tested on one module; manufacturer confirmation pending; early readings may stabilize
```

After warm-up, illustrative output using the supplied capture (30.0 ms here is
the host reader fixture's duration, not a claimed hardware timing):

```text
SENSOR: mhz1911a | DATA | co2=1770 ppm read=30.0 ms
SENSOR: mhz1911a | HEALTH | state=READY ok_total=1 failed_total=0 consecutive=0 period=5000 ms timeouts_total=0 checksum_failed_total=0
```

Actual ppm, duration and counters are computed at runtime. A disconnection after
one success, followed by three timeouts and recovery, can produce:

```text
SENSOR: mhz1911a | ERROR | state=DEGRADED error=ESP_ERR_TIMEOUT failed_total=1 consecutive=1
SENSOR: mhz1911a | HEALTH | recovery=READY ok_total=2 failed_total=3
```

The state becomes UNAVAILABLE on the third consecutive failure even when the
repeated warning is suppressed; the next periodic HEALTH summary reflects the
current state. If missing from startup, the first timeout reports UNAVAILABLE.

## First continuous hardware test (manual)

No hardware was flashed or operated while implementing this change.

1. Use the same verified wiring and UART-mode selection as the successful
   capture. With power off, check GPIO15 to sensor RX, GPIO16 from sensor TX and
   common ground. Use stable sensor power, 5.0 +/- 0.1 V with at least 150 mA
   supply capability, and compatible 3.3 V UART logic. Preserve the verified R/T
   arrangement; do not touch HD/calibration or apply 5 V to ESP32 GPIOs.
2. Open an ESP-IDF 5.5.5 terminal in this repository. Run:

   ```powershell
   idf.py --version
   idf.py build
   ```

   No menuconfig activation is needed. Power the sensor before the ESP32 begins
   its warm-up; firmware cannot detect a later, separate sensor power-on. Avoid
   driving an unpowered sensor through UART.
3. Manually flash and start Monitor, replacing the example port:

   ```powershell
   $mhzPort = 'COM7'
   idf.py -p $mhzPort flash monitor
   ```

   Enable Monitor file logging with Ctrl+T then Ctrl+L (default shortcuts).
   Capture startup and at least 15 minutes of subsequent operation. A reset
   restarts the 60-second guard and resets RAM counters.
4. Confirm no CO2 DATA/READY during warm-up; then readings at roughly five-second
   intervals, READY only after valid communication, and minute HEALTH summaries.
   Check the existing DHT22, GY511, RC522, LD2450, HC-SR04P and MQTT behavior during
   warm-up and acquisition. Record timeout/checksum/overflow counts, maximum
   read/jitter/late values and stack watermark, not just plausible ppm.
5. Once healthy, if the wiring allows a safe isolated signal disconnect, open
   only the sensor TX-to-GPIO16 signal while keeping both devices powered and
   ground connected. Do not move power, R/T or HD. Observe DEGRADED then
   UNAVAILABLE after three failed acquisitions; leave it disconnected through a
   HEALTH summary. Restore that signal and expect READY on a subsequent valid
   scheduled read, with earlier failure totals retained. Verify other sensors
   continue throughout. Skip a live jumper change if it risks shorting pins.
6. Exit Monitor with Ctrl+]. Preserve the log and assess sustained error rates,
   recovery and task timing before MQTT Phase 2. Separately compare against a
   suitable independent CO2 reference before making accuracy claims. This
   initial run is not evidence of long-term reliability or certification.

If acquisition remains silent, inspect power/contacts, TX/RX orientation and
UART-mode selection. Do not add automatic baud searches, calibration or other
commands. Passive PWM waveform observation or a manufacturer protocol request
remains the next investigation if the tested exchange cannot be reproduced.

## Software validation

Passed: 45 driver scenarios (including the actual capture, exact range boundaries,
fragmentation, stale/noisy/late frames, TX/RX faults, checksum and disconnection
recovery); 13 reader/status scenarios; seven 125-second six-sensor scenarios in
the platform suite. The driver scenarios also run the real LD2450 UART1 driver
alongside UART2. Platform tests exercise all six real reader loops and MQTT
configuration dispatch with deterministic acquisition doubles, including
unresponsive CO2 and rejected Phase 3 controls. They do not emulate ESP32 SMP,
electrical behavior or physical measurement accuracy.

All existing host suites were run:

```powershell
python tests/mhz1911a_host/run.py
python tests/sensor_host/run.py
python tests/rc522_host/run.py
python tests/ld2450_host/run.py
python tests/hcsr04p_host/run.py
python tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
```

ESP-IDF 5.5.5 `idf.py build size` passed without compiler warnings. The padded
application binary is 1,033,824 bytes, within the existing 4 MiB factory partition;
the partition-table binary hash is unchanged. Logs are in
`build/mhz1911a_phase1_*.log`. Linked-symbol inspection confirms the permanent
reader and driver are present and old diagnostic tasks are absent.
`git diff --check` and whitespace checks on new files passed. Work remains uncommitted on
`feature/mhz1911a-uart`; no branch switch, commit, push, merge or flash was made.
