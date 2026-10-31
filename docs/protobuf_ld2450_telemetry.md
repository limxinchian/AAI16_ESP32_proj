# LD2450 radar telemetry: binary Protobuf on the existing topic

Fourth sensor-telemetry migration (after [RC522](protobuf_rc522_telemetry.md),
[HC-SR04P](protobuf_hcsr04p_telemetry.md) and [DHT22](protobuf_dht22_telemetry.md)).
`aai16/sensors/ld2450/data` now carries a binary `aai16.Envelope` with a `sensor_telemetry.ld2450` body,
QoS 0, retain false, one message per publication period (default 1000 ms, configurable 500 to 10000 ms).
The JSON text is removed from the firmware. Existing JSON-only subscribers of this topic must use the schema
decoder **and** adapt to the three content changes below.

**Status.** Host tests pass (C encoder, C decoder, independent Python decoder) and the full firmware image
builds for ESP32-S3 on ESP-IDF 6.1. **Verified on the board:** live radar messages were captured from the
broker and decoded with the project schema; see [Hardware status](#hardware-status).

## What subscribers must change

The data is the same, but the shape is different from the JSON:

1. **All three slots are always sent**, each with a `present` flag. The JSON listed only present targets.
   Filter on `present` instead of relying on the list length. An unused slot is `present = false` with
   every other field zero.
2. **`target_count` is gone.** It is the number of slots with `present = true`.
3. **`distance_mm` is gone.** It was derived: `round(sqrt(x_mm^2 + y_mm^2))`. The host test recomputes it from
   the decoded `x_mm`/`y_mm` and checks it against the old values (1883 mm and 5000 mm for the two fixture
   targets).

## Wire contract

The schema is unchanged: `protocol/aai16.proto` (`Envelope` body tag 20, `SensorTelemetry` reading tag 13,
`Ld2450Reading` with exactly three `RadarTarget`s, `max_count:3`).

| Previous JSON field | Protobuf field | Notes |
| --- | --- | --- |
| `"sensor":"ld2450"` | `sensor_telemetry.sensor = SENSOR_LD2450` | Translated through `config_sensor_to_wire()` |
| `"boot_id":"12345678"` | `Envelope.boot_id` | Same `uint32`, no longer hex text |
| `"seq":N` | `Envelope.sequence` | Consumed publication attempts: offline, stale and SDK drops still advance it; deferrals do not |
| `"uptime_ms":N` | `sensor_telemetry.acquired_us` | **Acquisition** time (when the UART report was received), now in microseconds |
| (none) | `sensor_telemetry.uptime_us` | **New.** Time the message was built; always `>= acquired_us` |
| `"target_count":N` | number of `present` slots | See above |
| `"targets":[...]` | `ld2450.targets` (3 entries) | See above |
| `"slot":N` | `RadarTarget.slot` | 1 to 3, always set |
| (none) | `RadarTarget.present` | **New** |
| `"x_mm"`, `"y_mm"`, `"speed_cm_s"`, `"resolution_mm"` | same names | Same integers; `resolution_mm` is the range-gate size, not a distance |
| `"distance_mm":N` | (derived) | Not sent |
| (none) | `Envelope.generation` | **New.** The sensor's configuration generation **when the report was handed over**, not when it was sent |
| (none) | `Envelope.protocol_version = 1` | **New**, required by every message |

`request_id`, `phase`, `health` and `interval_ms` are intentionally absent.

**Generation rule.** A report keeps the generation it was acquired under even if publication is delayed
(design section 1.3). It is captured in the mailbox by `mqtt_manager_update_ld2450_snapshot()`, in a short
separate critical section, never nested with the statistics lock. A host test advances the generation between
hand-off and publication and confirms the message keeps the old value.

## Bounds and validation (before anything is published)

| Field | Valid range | Enforced by |
| --- | --- | --- |
| `x_mm`, `y_mm`, `speed_cm_s` | -32767 to 32767 | `mqtt_manager_update_ld2450_snapshot()` (rejects, counts `invalid`), and again in the codec |
| `resolution_mm` | 0 to 65535 | type |
| `slot` | 1, 2, 3, each exactly once | encoder, and the codec |
| unused slot (`present = false`) | all other fields zero | the update API, and the codec |
| target count | equals the number of present slots | the update API (as before) |
| `acquired_us` | 0 to now, and `<= uptime_us` | the update API, `encode_radar`, codec |
| whole message | worst case 110 bytes; buffer `LD2450_PAYLOAD_SIZE` is now 192 (it was 512 for JSON), guarded by a compile-time check against the nanopb maximum | `_Static_assert` + encode capacity |

The LD2450 parser decodes sign/magnitude (`components/ld2450_sensor/ld2450_parser.c`), so real data is always
within +-32767 and an unused slot is eight zero bytes; the new range checks therefore never reject real radar
frames. They protect the encoder from any other caller. The previous code did not check these ranges. A
rejected update is counted as `invalid` and the previous valid report is kept. A failed encode counts as
`invalid` for the radar's publisher statistics and publishes nothing. Storage is static nanopb: no heap
allocation, no `snprintf`, no JSON.

## Size comparison (JSON vs Protobuf)

Same reports, uptime 5 minutes, `seq=12`, `boot_id=12345678`, generation 0. JSON is the exact format the
previous firmware published (`git show HEAD:components/mqtt_manager/mqtt_manager.c`, `format_radar`).
Targets used: (-782, 1713, -16, 320), (3000, -4000, 12, 640), (-1200, 2500, 0, 320). These are payload bytes,
not MQTT/TCP packet sizes.

| Targets present | JSON | Protobuf | Reduction |
| ---: | ---: | ---: | ---: |
| 0 | 98 B | 78 B | 20.4 % |
| 1 | 188 B | 81 B | 56.9 % |
| 2 | 279 B | 84 B | 69.9 % |
| 3 | 369 B | 87 B | 76.4 % |

The saving is smallest with an empty scene because Protobuf always sends the three slots. The old 3-target
JSON message (369 B) did not fit the shared 320-byte telemetry slot, which is why radar used its own 512-byte
buffer; the Protobuf message does fit.

Protobuf example, 2 targets (84 B), hex:
`08011d785634122500000000350c000000a2014008041080c6868f011880c6868f016a300a0f08011001189b0c20e21a281f30c0020a0f0802100118f02e20bf3e28183080050a0c080310001800200028003000`

## Verification performed

| Check | Result |
| --- | --- |
| `tests/mqtt_host` C suite (all sensors, config, ML, model routing, SDK model) | 30 `PASS` lines, exit 0 |
| Every slot combination (masks 0 to 7): three slots, `present` flags, exact values, zeroed unused slots, `distance_mm` recoverable from x and y, acquisition time kept in microseconds, `uptime_us >= acquired_us` | `PASS` |
| Worst case (+-32767, 65535, 64-bit times, 32-bit counters): 110 B; every buffer capacity below the length fails without writing past it; the exact fit succeeds; `uptime < acquired` and negative `acquired` refused | `PASS` |
| Rejected hand-offs (not replacing the previous valid report): count mismatch, NULL, negative or future time, `-32768` on an axis (two cases), data in an unused slot | `PASS` |
| Generation is the acquisition-time value even if it changes before publication | `PASS` |
| 10 Hz input coalesced to the publication period, no early send, stale/offline/reconnect, full shared queue defers radar without dropping ACK/RFID, blocked SDK call, transport failures, configuration interval/gate races (the existing tests, now checking binary payloads) | `PASS` |
| C/nanopb bytes -> Python decoder (`--ld2450-bridge`): slots 1 and 3 present with values at both extremes (-32767), empty slot 2, exact microsecond times, sequence 1, generation 7, no request id / phase | `PASS` (87 B) |
| Firmware compiled for ESP32-S3, ESP-IDF 6.1; the radar JSON text (`target_count`, `distance_mm`, `{"sensor":"ld2450"`) is absent from the compiled `mqtt_manager` object (the four remaining JSON sensors are still found by the same check) | builds, 0x11cfe0 bytes, 72 % of the app partition free |

Run the suite: `python tests/mqtt_host/run.py --idf-path <ESP-IDF path>` (needs a host C compiler and
`pip install -r tools/protocol/requirements.txt`). The results above were obtained with a Zig-based C compiler
on Windows. `tests/protocol_host` was **not** re-run for this change: the codec was not modified (its LD2450 rule already
required three unique slots, +-32767 axes and zeroed unused slots; the existing codec tests include a
duplicate-slot rejection, but I did not check whether they exercise every axis limit).

Test changes worth knowing about: the old fixture used `-32768` for the third target, which the real parser can
never produce and the wire contract rejects; it is now `-32767`. The JSON checker became a Protobuf decoder.
The worst-case and buffer-boundary tests now test the Protobuf encoder.

## Hardware status

First boot of this build: the radar produced no data at all (`ld2450 | HEALTH | frames_total=0
rx_bytes_total=0`, `No valid report within 5000 ms`, `radar_snapshot_updates=0`), so nothing was published.
That was the radar's power or UART wiring, not the message format: the same error appeared in an earlier log
before this change. After the radar wiring was corrected (what was changed was not recorded), data flowed.

**Observed on the board (2026-10-31):** a Python subscriber on the local Mosquitto broker captured
`aai16/sensors/ld2450/data` for 12 s: **12 messages (1 per second, the default period)**, decoded with
`tools/protocol/generated/aai16_pb2.py`. One of them:

| Field | Value |
| --- | --- |
| `boot_id`, `generation`, `sequence` | 3169609533, 0, 161 |
| `acquired_us`, `uptime_us` | 162057961, 162134936 (message built 77 ms after acquisition) |
| slot 1 | present, x -23, y 246, speed 0, resolution 360 |
| slot 2 | present, x -805, y 1773, speed 24, resolution 360 |
| slot 3 | not present, all zero |

Not recorded: a comparison against the serial `ld2450 | DATA` lines or a physical measurement of where the
targets were, a message with zero or three targets, and how MQTTX displays the signed `x_mm`/`y_mm` values.

## Reading the messages

MQTTX: see [the RC522 document](protobuf_rc522_telemetry.md#reading-the-messages) (schema import, **Run Script**
set to Received, proto name `aai16.Envelope`, and disconnect JSON connections while checking Protobuf topics
because the decoder setting is application-wide). Python: `pb.Envelope.FromString(payload)` with
`tools/protocol/generated/aai16_pb2.py`, then
`[x for x in msg.sensor_telemetry.ld2450.targets if x.present]`.

## Remaining telemetry migration

GY-511, MH-Z19, BME280 and MAX6675 telemetry is still JSON. The schema already has a message for each.
