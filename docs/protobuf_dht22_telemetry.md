# DHT22 telemetry: binary Protobuf on the existing topic

Third sensor-telemetry migration (after [RC522](protobuf_rc522_telemetry.md) and
[HC-SR04P](protobuf_hcsr04p_telemetry.md)). `aai16/sensors/dht22/data` now carries a binary
`aai16.Envelope` with a `sensor_telemetry.dht22` body, QoS 0, retain false, once per DHT22 read
(default 2000 ms, configurable 2000 to 10000 ms). The JSON text is removed from the firmware. Existing JSON-only subscribers of this
topic must use the schema decoder.

**Status.** Host tests pass (C encoder, C decoder, independent Python decoder) and the full firmware
image builds for ESP32-S3 on ESP-IDF 6.1. **Not yet done:** a captured and decoded DHT22 message from the
board. The DHT22 has to be plugged into GPIO6 and the board powered; see
[Hardware status](#hardware-status).

## Wire contract

The schema is unchanged: `protocol/aai16.proto` (`Envelope` body tag 20, `SensorTelemetry` reading tag 10,
`Dht22Reading`).

| Previous JSON field | Protobuf field | Notes |
| --- | --- | --- |
| `"sensor":"dht22"` | `sensor_telemetry.sensor = SENSOR_DHT22` | Translated through `config_sensor_to_wire()` |
| `"boot_id":"12345678"` | `Envelope.boot_id` | Same `uint32`, no longer hex text |
| `"seq":N` | `Envelope.sequence` | Consumed publication attempts: offline, full-queue and invalid submissions still advance it |
| `"uptime_ms":N` | `sensor_telemetry.acquired_us` and `.uptime_us` | Microseconds; divide by 1000 for the old value. Equal here: the reading is published as it is acquired |
| `"temperature_c":30.5` | `dht22.temperature_c` | float32 on the wire; the JSON rounded to 0.1 |
| `"humidity_pct":71.8` | `dht22.humidity_pct` | float32 on the wire; the JSON rounded to 0.1 |
| (none) | `Envelope.generation` | **New.** The sensor's committed configuration generation at submission |
| (none) | `Envelope.protocol_version = 1` | **New**, required by every message |

No JSON information is lost, and the values are more precise. `request_id`, `phase`, `health` and
`interval_ms` are intentionally absent.

## Bounds and validation (before anything is published)

| Field | Valid range | Enforced by |
| --- | --- | --- |
| `temperature_c` | finite, -40 to 80 (DHT22 datasheet) | `mqtt_manager_publish_dht22()` (rejects, counts `invalid`), and again in the codec (`telemetry_validate`) |
| `humidity_pct` | finite, 0 to 100 | the same two checks |
| `acquired_us` | `<= uptime_us` | codec |
| `Envelope.protocol_version` | exactly 1 | codec |
| `Envelope.boot_id`, `sequence` | required | codec |
| whole message | `< 320` bytes (shared slot); worst case measured 54 | `_Static_assert` + encode capacity |

**Behaviour change.** The previous firmware only rejected non-finite values. Values outside the DHT22's
physical range (for example 120 C, or a negative humidity) were published. They are now treated as a failed
read: `mqtt_manager_publish_dht22()` returns `ESP_ERR_INVALID_ARG`, increments the DHT22 `invalid`
counter, and queues nothing. The boundary values (-40 C, 80 C, 0 %RH, 100 %RH) are accepted. The
firmware only publishes after a CRC-valid read (`main/main.c`), so this is a second line of defence.

Storage is static nanopb: no heap allocation, no `snprintf`, no JSON.

## Size comparison (JSON vs Protobuf)

Same reading and sensor state, uptime 5 minutes, `seq=12`, `boot_id=12345678`, generation 0. JSON is the
exact format the previous firmware published (`git show HEAD:components/mqtt_manager/mqtt_manager.c`,
`mqtt_manager_publish_dht22`). These are payload bytes, not MQTT/TCP packet sizes, and say nothing about the
size of decoded C structures.

| Reading | JSON | Protobuf | Reduction |
| --- | ---: | ---: | ---: |
| -12.3 C / 5.0 %RH | 108 B | 46 B | 57.4 % |
| 5.0 C / 40.0 %RH | 107 B | 46 B | 57.0 % |
| 25.4 C / 60.2 %RH | 108 B | 46 B | 57.4 % |
| 30.5 C / 71.8 %RH | 108 B | 46 B | 57.4 % |
| 79.9 C / 99.9 %RH | 108 B | 46 B | 57.4 % |

JSON example (108 B):
`{"sensor":"dht22","boot_id":"12345678","seq":12,"uptime_ms":300000,"temperature_c":30.5,"humidity_pct":71.8}`

Protobuf example (46 B), hex:
`08011d785634122500000000350c000000a2011a08011080c6868f011880c6868f01520a0d0000f441159a998f42`

Worst case (maximum 32/64-bit counters and times, -40 C, 100 %RH): 54 B. The bridge test below produces 52 B
because its uptime is about 2^54 microseconds.

## Verification performed

| Check | Result |
| --- | --- |
| `tests/mqtt_host` C suite (all sensors, config, ML, model routing, SDK model) | 29 `PASS` lines, exit 0 |
| DHT22 message decoded in C: sequence 2, boot_id `0x12345678`, generation 0, `acquired_us == uptime_us == 1234000`, 25.0 C / 50.0 %RH | `PASS` |
| Invalid inputs: NaN and infinity in either field, and -40.1 / 80.1 C, -0.1 / 100.1 %RH: all `ESP_ERR_INVALID_ARG`, six counted `invalid`, nothing queued | `PASS` |
| Boundary values (-40 C / 0 %RH and 80 C / 100 %RH) are published | `PASS` |
| C/nanopb bytes -> Python decoder (`--dht22-bridge`): negative temperature (-12.3), exact float32, exact `uint64` times, sequence 1, generation 7, no request id / phase | `PASS` (52 B) |
| Codec unit tests (`tests/protocol_host`, 16 tests): the same boundary values round-trip; six out-of-range or non-finite DHT22 readings are rejected cleanly | `PASS` |
| Firmware compiled for ESP32-S3, ESP-IDF 6.1; the DHT22 JSON template is absent from the compiled `mqtt_manager` object (the other five JSON sensor templates are still found by the same check) | builds, 0x11d040 bytes, 72 % of the app partition free |

Run the suites: `python tests/mqtt_host/run.py --idf-path <ESP-IDF path>` and
`python tests/protocol_host/run.py` (both need a host C compiler and
`pip install -r tools/protocol/requirements.txt`).

Caveats on how these results were obtained, so they can be reproduced honestly:

- The C compiler was Zig (`zig cc`) on Windows. `tests/protocol_host/run.py` additionally needs `nm` and an
  Xtensa compiler for its object-size and heap-symbol checks, which the Zig setup does not provide. Only its 16
  unit tests were run, against a bridge built with `-O0`: with Zig at `-O2` the bridge printed nothing
  (an issue with that build setup, not examined further). The size report and the heap-symbol scan of that
  runner were **not** run for this change.
- The test changes: the old DHT22 assertion parsed the JSON text with cJSON; it now decodes the binary
  message. The Python bridge check compares the first two bytes against `b'\x08\x01'` (protocol_version field).

## Hardware status

Not yet verified on the board. The DHT22 was unplugged or reporting degraded at the last serial logs, so no
message has been captured. To record:

1. DHT22 data pin on GPIO6 (plus power and ground), board powered and on the demo network.
2. Serial: `dht22   | DATA | temp=... humidity=...` lines and `dht22 | HEALTH` with `ok_total` rising.
3. A decoded message on `aai16/sensors/dht22/data` in MQTTX (or `mosquitto_sub`), with `temperature_c` and
   `humidity_pct` close to the serial `DATA` values, and `boot_id` equal to the boot log.

In MQTTX the float32 values appear with their binary-float digits (for example 30.5 is exact, 71.8 shows as
71.80000305175781) and the 64-bit times as strings.

## Reading the messages

MQTTX: see [the RC522 document](protobuf_rc522_telemetry.md#reading-the-messages) (schema import, **Run Script**
set to Received, proto name `aai16.Envelope`, and disconnect JSON connections while checking Protobuf topics
because the decoder setting is application-wide). Python: `pb.Envelope.FromString(payload)` with
`tools/protocol/generated/aai16_pb2.py`.

## Remaining telemetry migration

GY-511, LD2450, MH-Z19, BME280 and MAX6675 telemetry is still JSON. The schema already has a message for each.
