# HC-SR04P telemetry: binary Protobuf on the existing topic

Second sensor-telemetry migration (after [RC522](protobuf_rc522_telemetry.md)).
`aai16/sensors/hcsr04p/data` now carries a binary `aai16.Envelope` with a
`sensor_telemetry.hcsr04p` body, QoS 0, retain false, about once per second. The JSON text is
removed from the firmware. Existing JSON-only subscribers of this topic must use the schema decoder.

**Status.** Host tests pass (C encoder, C decoder, independent Python decoder) and the full
firmware image builds for ESP32-S3 on ESP-IDF 6.1 (0x11cdf0 bytes, 72% of the app partition free).
**Not verified on hardware:** at the time of writing the sensor on the project board returned
`ESP_ERR_INVALID_RESPONSE` on every read (its ECHO input read high before each trigger, so reads end in
about 0.1 ms and no sample is ever handed to MQTT). No HC-SR04P message has been produced on a real
board yet. See [Hardware status](#hardware-status).

## Wire contract

The schema is unchanged: `protocol/aai16.proto` (`Envelope` body tag 20, `SensorTelemetry` reading tag 14,
`Hcsr04pReading`).

| Previous JSON field | Protobuf field | Notes |
| --- | --- | --- |
| `"sensor":"hcsr04p"` | `sensor_telemetry.sensor = SENSOR_HCSR04P` | Translated through `config_sensor_to_wire()` |
| `"boot_id":"12345678"` | `Envelope.boot_id` | Same `uint32`, no longer hex text |
| `"seq":N` | `Envelope.sequence` | Consumed publication attempts: offline, stale and SDK drops still advance it; replacements and busy deferrals do not |
| `"uptime_ms":N` | `sensor_telemetry.acquired_us` | **Acquisition** time, now in microseconds (the JSON value was `acquired_us / 1000`) |
| (none) | `sensor_telemetry.uptime_us` | **New.** Time the message was built; always `>= acquired_us` |
| `"distance_cm":42.1` | `hcsr04p.distance_cm` | float32 on the wire; the JSON rounded to 0.1 |
| `"echo_us":2456.3` | `hcsr04p.echo_us` | float32 on the wire; the JSON rounded to 0.1 |
| (none) | `Envelope.generation` | **New.** The sensor's configuration generation **when the sample was handed over**, not when it was sent |
| (none) | `Envelope.protocol_version = 1` | **New**, required by every message |

No JSON information is lost, and the values are more precise. `request_id`, `phase`, `health` and
`interval_ms` are intentionally absent.

**Generation rule.** A reading keeps the generation it was acquired under even if publication is delayed
(design section 1.3). The generation is captured in the mailbox when the reader calls
`mqtt_manager_update_hcsr04p_snapshot()`, under a short separate critical section, never nested with the
statistics lock. A host test advances the generation between hand-off and publication and confirms the
message keeps the old value.

## Bounds and validation (before anything is published)

| Field | Valid range | Enforced by |
| --- | --- | --- |
| `distance_cm` | finite, 2 to 400 | `mqtt_manager_update_hcsr04p_snapshot()` (rejects, counts `invalid`), and again in the codec (`telemetry_validate`) |
| `echo_us` | finite, greater than 0, at most 30000 | the same API check; codec requires finite and greater than 0 |
| `acquired_us` | 0 to now, and `<= uptime_us` | API check, `encode_hcsr04p`, codec |
| `uptime_us` | `<= INT64_MAX` | codec |
| `Envelope.protocol_version` | exactly 1 | codec |
| whole message | `< 160` bytes (publisher buffer), `< 320` (shared slot); worst case measured 54 | encode capacity and the host test |

An out-of-range sample is rejected at hand-off and never queued. A failed encode counts as `invalid` and
publishes nothing. Storage is static nanopb: no heap allocation, no `snprintf`, no JSON.

## Size comparison (JSON vs Protobuf)

Same distance and sensor state, uptime 5 minutes, `seq=12`. JSON is the exact format the previous firmware
published (`git show HEAD~:components/mqtt_manager/mqtt_manager.c`, `format_hcsr04p`). These are payload
bytes, not MQTT/TCP packet sizes, and say nothing about the size of decoded C structures.

| Distance | JSON | Protobuf | Reduction |
| ---: | ---: | ---: | ---: |
| 5.0 cm | 103 B | 46 B | 55.3 % |
| 42.1 cm | 105 B | 46 B | 56.2 % |
| 150.5 cm | 106 B | 46 B | 56.6 % |
| 399.9 cm | 107 B | 46 B | 57.0 % |

JSON example (105 B):
`{"sensor":"hcsr04p","boot_id":"12345678","seq":12,"uptime_ms":300000,"distance_cm":42.1,"echo_us":2456.3}`

Protobuf example (46 B), hex:
`08011d785634122500000000350c000000a2011a080510e083e88e011880c6868f01720a0d6666284215cd841945`

Worst case (maximum floats, 64-bit times, 32-bit counters): 54 B.

## Verification performed

| Check | Result |
| --- | --- |
| `tests/mqtt_host` C suite (all sensors, config, ML, model routing, SDK model) | all `PASS` |
| Encode/decode: values, 25000999 us acquisition kept in microseconds, maximum values, every buffer capacity below the length fails and an exact fit succeeds, guard byte never overwritten, inconsistent times refused | `PASS` |
| Mailbox behaviour (5:1 coalescing, copy ownership, freshness limit, reconnect, ACK/RFID priority, blocked SDK call, gate/epoch cancellation), now checking binary payloads | `PASS` |
| Generation is the acquisition-time value even if it changes before publication | `PASS` |
| C/nanopb bytes -> Python decoder (`--hcsr04p-bridge`): exact float32, microsecond times, sequence 1, generation 7, no request id / phase | `PASS` (44 B) |
| C bytes identical to Python's serialisation of the same message | identical |
| Firmware compiled for ESP32-S3, ESP-IDF 6.1; the JSON text is absent from the compiled object | builds |

Run the suite: `python tests/mqtt_host/run.py --idf-path <ESP-IDF path>` (needs a host C compiler and
`pip install -r tools/protocol/requirements.txt`).

Test changes worth knowing about: the old tests compared an exact JSON string and tested truncation with a
trailing NUL; they now decode the binary message and test the encoded length. A new test covers the
acquisition-time generation.

## Hardware status

Open item: **the ultrasonic module does not currently produce readings on the board**, so this change has
only been verified on the host. The firmware reports it as `DEGRADED` with `invalid_total` equal to
`failed_total` and `timeouts_total=0`. In `components/hcsr04p_sensor/hcsr04p_sensor.c` a read is rejected
immediately if ECHO (GPIO5) is already high before the trigger pulse, and a timeout would instead take
about 30 ms. The log shows `max_read` of 0.05 to 1.3 ms, so ECHO reads high at rest. The pin has an
internal pull-down on ESP-IDF 6 (set with `gpio_set_pull_mode`), so an unconnected ECHO would read low and
time out, which points at the module or its wiring driving ECHO high: for example a plain HC-SR04 (a 5 V
part) run at 3.3 V, ECHO or TRIG on the wrong pin, or missing supply or ground. Verify by unplugging the
ECHO wire: if the counters change from `invalid` to `timeouts`, the module is driving ECHO.

When it reads, verify as for the RFID: confirm the serial health line shows `ok_total` rising, then
subscribe to `aai16/sensors/hcsr04p/data` on the Protobuf MQTTX connection (decoder applied to Received,
proto name `aai16.Envelope`) and compare `distance_cm` with a tape measure. In MQTTX the float32 values
appear with their binary-float digits (for example 42.1 shows as 42.099998474121094) and the 64-bit times
as strings.

## Reading the messages

MQTTX: see [the RC522 document](protobuf_rc522_telemetry.md#reading-the-messages) (schema import, **Run Script**
set to Received, proto name `aai16.Envelope`, and disconnect JSON connections while checking Protobuf
topics because the decoder setting is application-wide). Python: `pb.Envelope.FromString(payload)` with
`tools/protocol/generated/aai16_pb2.py`.

## Remaining telemetry migration

DHT22, GY-511, LD2450, MH-Z19, BME280 and MAX6675 telemetry is still JSON. The schema already has a
message for each.
