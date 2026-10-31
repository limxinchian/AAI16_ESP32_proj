# MAX6675 thermocouple telemetry: binary Protobuf on the existing topic

Sixth sensor-telemetry migration (after [RC522](protobuf_rc522_telemetry.md),
[HC-SR04P](protobuf_hcsr04p_telemetry.md), [DHT22](protobuf_dht22_telemetry.md),
[LD2450](protobuf_ld2450_telemetry.md) and [MH-Z19](protobuf_mhz1911a_telemetry.md)).
`aai16/sensors/max6675/data` now carries a binary `aai16.Envelope` with a `sensor_telemetry.max6675` body,
QoS 0, retain false, once per successful read (default 250 ms poll; configurable 250 to 10000 ms). The JSON
text is removed from the firmware. Existing JSON-only subscribers of this topic must use the schema decoder.

**Status.** Host tests pass (C encoder, C decoder, independent Python decoder) and the full firmware image
builds for ESP32-S3 on ESP-IDF 6.1. **Not yet done:** a captured and decoded MAX6675 message from the board;
see [Hardware status](#hardware-status).

## Wire contract

The schema is unchanged: `protocol/aai16.proto` (`Envelope` body tag 20, `SensorTelemetry` reading tag 17,
`Max6675Reading`).

| Previous JSON field | Protobuf field | Notes |
| --- | --- | --- |
| `"sensor":"max6675"` | `sensor_telemetry.sensor = SENSOR_MAX6675` | Translated through `config_sensor_to_wire()` |
| `"boot_id":"12345678"` | `Envelope.boot_id` | Same `uint32`, no longer hex text |
| `"seq":N` | `Envelope.sequence` | Consumed publication attempts: offline, full-queue and invalid submissions still advance it |
| `"uptime_ms":N` | `sensor_telemetry.acquired_us` and `.uptime_us` | Microseconds; divide by 1000 for the old value. Equal here: the reading is published as it is acquired |
| `"temperature_c":123.25` | `max6675.temperature_c` | float32 on the wire; the JSON printed 2 decimals. Every 0.25 degC step the sensor can report is exact in float32 |
| (none) | `Envelope.generation` | **New.** The sensor's committed configuration generation at submission |
| (none) | `Envelope.protocol_version = 1` | **New**, required by every message |

No JSON information is lost. `request_id`, `phase`, `health` and `interval_ms` are intentionally absent.

## Bounds and validation (before anything is published)

| Field | Valid range | Enforced by |
| --- | --- | --- |
| `temperature_c` | finite, 0 to 1023.75 | `mqtt_manager_publish_max6675()` (rejects, counts `invalid`), and again in the codec (`telemetry_validate`) |
| `acquired_us` | `<= uptime_us` | codec |
| `Envelope.protocol_version` | exactly 1 | codec |
| `Envelope.boot_id`, `sequence` | required | codec |
| whole message | `< 320` bytes (shared slot); worst case 50 | `_Static_assert` + encode capacity |

**Behaviour change.** The previous firmware only rejected non-finite values. The range is the full span of
the sensor: the driver converts a 12-bit count at 0.25 degC (`(raw >> 3) * 0.25`), so every real reading is
0 to 1023.75 degC and the new limits never reject real data. A value outside the range (for example a negative
temperature, which this chip cannot report, or 1024) is now treated as a failed read: it returns
`ESP_ERR_INVALID_ARG`, increments the MAX6675 `invalid` counter, and queues nothing. The limits themselves are
accepted. Thermocouple-open and framing faults were already read failures and are never published as 0 degC.

Storage is static nanopb: no heap allocation, no `snprintf`, no JSON.

## Size comparison (JSON vs Protobuf)

Same reading and sensor state, uptime 5 minutes, `seq=12`, `boot_id=12345678`, generation 0. JSON is the exact
format the previous firmware published (`git show HEAD:components/mqtt_manager/mqtt_manager.c`,
`mqtt_manager_publish_max6675`). These are payload bytes, not MQTT/TCP packet sizes.

| Temperature | JSON | Protobuf | Reduction |
| ---: | ---: | ---: | ---: |
| 0.0 C | 90 B | 42 B | 53.3 % |
| 25.25 C | 91 B | 42 B | 53.8 % |
| 123.25 C | 92 B | 42 B | 54.3 % |
| 1023.75 C | 93 B | 42 B | 54.8 % |

JSON example (92 B):
`{"sensor":"max6675","boot_id":"12345678","seq":12,"uptime_ms":300000,"temperature_c":123.25}`

Protobuf example (42 B), hex:
`08011d785634122500000000350c000000a2011608081080c6868f011880c6868f018a01050d0080f642`

Worst case (maximum 32/64-bit counters and times, 1023.75 C): 50 B. The bridge test below produces 48 B because
its uptime is about 2^54 microseconds.

## Verification performed

| Check | Result |
| --- | --- |
| `tests/mqtt_host` C suite (all sensors, config, ML, model routing, SDK model) | 32 `PASS` lines, exit 0 |
| MAX6675 message decoded in C: sequence 1, boot_id `0x12345678`, the sensor's current generation, `acquired_us == uptime_us` (the clock at submission), 123.25 C | `PASS` |
| Invalid inputs: NaN, infinity, -0.25 and 1024: all `ESP_ERR_INVALID_ARG`, four counted `invalid`, nothing queued | `PASS` |
| Boundary values (0 and 1023.75) are published | `PASS` |
| C/nanopb bytes -> Python decoder (`--max6675-bridge`): exact 1023.75, exact `uint64` times (about 2^54), sequence 1, generation 7, no request id / phase | `PASS` (48 B) |
| Codec unit tests (`tests/protocol_host`, 16 tests): 0 and 1023.75 round-trip; -0.25, 1024, NaN and infinity are rejected cleanly | `PASS` |
| Firmware compiled for ESP32-S3, ESP-IDF 6.1; the MAX6675 JSON template is absent from the compiled `mqtt_manager` object (the two remaining JSON sensors are still found by the same check) | builds, 0x11d0c0 bytes, 72 % of the app partition free |

Run the suites: `python tests/mqtt_host/run.py --idf-path <ESP-IDF path>` and
`python tests/protocol_host/run.py` (both need a host C compiler and
`pip install -r tools/protocol/requirements.txt`).

Caveats on how these results were obtained: the C compiler was Zig on Windows. `tests/protocol_host/run.py`
additionally needs `nm` and an Xtensa compiler for its object-size and heap-symbol checks, which that setup does
not provide, so only its 16 unit tests were run (against a bridge built with `-O0`, because the Zig `-O2`
build of that bridge printed nothing). The size report and heap-symbol scan were **not** run for this change.

Test changes worth knowing about: the old assertion parsed the JSON text with cJSON; it now decodes the binary
message. In that test the expected generation is the sensor's live generation (earlier configuration tests in
the same run had already advanced it), and the expected time is the clock captured before publishing, because
the test's `pump()` advances the simulated clock.

## Hardware status

Not yet verified on the board. Earlier logs showed this sensor reading `raw=0x0000` (0.00 C), which meant its
SO/SCK/CS wiring or power was wrong, not the message format; it was reported working afterwards but no reading
was recorded. To record, with the sensor wired (SCK GPIO12, SO GPIO13, CS GPIO10) and powered:

1. Serial: `MAX6675: T: ... C | raw=0x...` with a believable temperature (room temperature is about 20 to
   30 C; a probe held in a hand, a little higher), not `0.00`.
2. A decoded message on `aai16/sensors/max6675/data` in MQTTX (or a Python subscriber) with `temperature_c`
   matching the serial value, and `boot_id` equal to the boot log. Expect about 4 messages per second at the
   default poll, so use a subscription filter or pause when inspecting.

## Reading the messages

MQTTX: see [the RC522 document](protobuf_rc522_telemetry.md#reading-the-messages) (schema import, **Run Script**
set to Received, proto name `aai16.Envelope`, and disconnect JSON connections while checking Protobuf topics
because the decoder setting is application-wide). Python: `pb.Envelope.FromString(payload)` with
`tools/protocol/generated/aai16_pb2.py`.

## Remaining telemetry migration

GY-511 and BME280 telemetry is still JSON. The schema already has a message for each.
