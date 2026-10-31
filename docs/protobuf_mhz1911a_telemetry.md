# MH-Z19 CO2 telemetry: binary Protobuf on the existing topic

Fifth sensor-telemetry migration (after [RC522](protobuf_rc522_telemetry.md),
[HC-SR04P](protobuf_hcsr04p_telemetry.md), [DHT22](protobuf_dht22_telemetry.md) and
[LD2450](protobuf_ld2450_telemetry.md)). `aai16/sensors/mhz1911a/data` now carries a binary `aai16.Envelope`
with a `sensor_telemetry.mhz1911a` body, QoS 0, retain false, one message per successful acquisition
(default every 5000 ms, configurable 5000 to 60000 ms). The JSON text is removed from the firmware.
Existing JSON-only subscribers of this topic must use the schema decoder.

**Status.** Host tests pass (C encoder, C decoder, independent Python decoder) and the full firmware image
builds for ESP32-S3 on ESP-IDF 6.1. **Not yet done:** a captured and decoded CO2 message from the board; see
[Hardware status](#hardware-status).

## Wire contract

The schema is unchanged: `protocol/aai16.proto` (`Envelope` body tag 20, `SensorTelemetry` reading tag 15,
`Mhz1911aReading`).

| Previous JSON field | Protobuf field | Notes |
| --- | --- | --- |
| `"sensor":"mhz1911a"` | `sensor_telemetry.sensor = SENSOR_MHZ1911A` | Translated through `config_sensor_to_wire()` |
| `"boot_id":"12345678"` | `Envelope.boot_id` | Same `uint32`, no longer hex text |
| `"seq":N` | `Envelope.sequence` | Consumed publication attempts: offline, stale, cancelled and SDK failures still advance it; replacements, deferrals and invalid or duplicate hand-offs do not |
| `"uptime_ms":N` | `sensor_telemetry.acquired_us` | **Acquisition** time, now in microseconds (the JSON value was `acquired_us / 1000`) |
| (none) | `sensor_telemetry.uptime_us` | **New.** Time the message was built; always `>= acquired_us` |
| `"co2_ppm":1433` | `mhz1911a.co2_ppm` | Unsigned integer, same value |
| (none) | `Envelope.generation` | **New.** The sensor's configuration generation **when the sample was handed over**, not when it was sent |
| (none) | `Envelope.protocol_version = 1` | **New**, required by every message |

No JSON information is lost. `request_id`, `phase`, `health` and `interval_ms` are intentionally absent.

**Generation rule.** A sample keeps the generation it was acquired under even if publication is delayed
(design section 1.3). It is captured in the mailbox by `mqtt_manager_update_mhz1911a_snapshot()`, in a short
separate critical section, never nested with the statistics lock. A host test advances the generation between
hand-off and publication and confirms the message keeps the old value.

## Bounds and validation (before anything is published)

| Field | Valid range | Enforced by |
| --- | --- | --- |
| `co2_ppm` | 400 to 5000 | `mqtt_manager_update_mhz1911a_snapshot()` (rejects, counts `invalid`), and again in the codec (`telemetry_validate`) |
| `acquired_us` | 0 to now, strictly newer than the previous accepted sample, and `<= uptime_us` | the update API (as before), `encode_mhz1911a`, codec |
| `Envelope.protocol_version` | exactly 1 | codec |
| whole message | worst case 47 bytes; buffer `MHZ1911A_PAYLOAD_SIZE` is 128 (unchanged), guarded by a compile-time check | `_Static_assert` + encode capacity |

The accepted range is unchanged from the JSON firmware; no new rejection rule was added. A rejected update is
counted as `invalid` and the previous valid sample is kept. A failed encode counts as `invalid` in the CO2
publisher statistics and publishes nothing. Storage is static nanopb: no heap allocation, no `snprintf`, no JSON.

## Size comparison (JSON vs Protobuf)

Same reading and sensor state, uptime 5 minutes, `seq=12`, `boot_id=12345678`, generation 0. JSON is the exact
format the previous firmware published (`git show HEAD:components/mqtt_manager/mqtt_manager.c`,
`format_mhz1911a`). These are payload bytes, not MQTT/TCP packet sizes.

| CO2 | JSON | Protobuf | Reduction |
| ---: | ---: | ---: | ---: |
| 400 ppm | 84 B | 39 B | 53.6 % |
| 800 ppm | 84 B | 39 B | 53.6 % |
| 1433 ppm | 85 B | 39 B | 54.1 % |
| 5000 ppm | 85 B | 39 B | 54.1 % |

JSON example (85 B):
`{"sensor":"mhz1911a","boot_id":"12345678","seq":12,"uptime_ms":300000,"co2_ppm":1433}`

Protobuf example (39 B), hex:
`08011d785634122500000000350c000000a2011308061080c6868f011880c6868f017a0308990b`

Worst case (64-bit times, 32-bit counters, 5000 ppm): 47 B. The bridge test below produces 37 B because its
uptime is only 99.9 seconds.

## Verification performed

| Check | Result |
| --- | --- |
| `tests/mqtt_host` C suite (all sensors, config, ML, model routing, SDK model) | 31 `PASS` lines, exit 0 |
| Encode/decode: values, microsecond times, boot id, sequence, generation; worst case 47 B; every buffer capacity below the length fails without writing past it and the exact fit succeeds; `uptime < acquired`, negative time and out-of-range ppm refused | `PASS` |
| Generation is the acquisition-time value even if it changes before publication | `PASS` |
| Existing behaviour tests, now checking binary payloads: duplicates and out-of-order samples, exact 2 s freshness boundary and the admission recheck, offline/reconnect without replay, failed SDK submission, bounded replacement, ACK/RFID priority, real reader warm-up and 5000 ms cadence, blocked SDK call, 20000 parallel updates, sequence wrap, configuration disable/interval races | `PASS` |
| C/nanopb bytes -> Python decoder (`--mhz1911a-bridge`): integer CO2 1433, exact microsecond times, sequence 1, generation 7, no request id / phase | `PASS` (37 B) |
| Firmware compiled for ESP32-S3, ESP-IDF 6.1; the CO2 JSON template is absent from the compiled `mqtt_manager` object (the three remaining JSON sensors are still found by the same check) | builds, 0x11d030 bytes, 72 % of the app partition free |

Run the suite: `python tests/mqtt_host/run.py --idf-path <ESP-IDF path>` (needs a host C compiler and
`pip install -r tools/protocol/requirements.txt`). The results above were obtained with a Zig-based C compiler on
Windows. `tests/protocol_host` was not re-run: the codec was not modified (its MH-Z19 rule already required
400 to 5000 ppm, and its existing test rejects 399 and 5001).

Test changes worth knowing about: the JSON checker became a Protobuf decoder (in `test_mhz1911a.c` and its
configuration test); the exact-JSON-string and truncation tests became encoder tests.

## Hardware status

Not yet verified on the board. The latest serial log I have (an earlier boot) showed the sensor not working:
`mhz1911a | HEALTH | state=UNAVAILABLE`, `ESP_FAIL` then `ESP_ERR_TIMEOUT` reads, `uart_errors_total=7`,
`discarded_bytes_total=117`. That is the UART link (TX GPIO15, RX GPIO16, 9600 baud) or the sensor's power,
not the message format. Also note the sensor has a 60 s warm-up, so the first message appears about a minute
after boot. To record, with the sensor wired and powered:

1. Serial: `mhz1911a | DATA` lines and `mhz1911a | HEALTH` with `ok_total` rising and `state=READY`.
2. A decoded message on `aai16/sensors/mhz1911a/data` in MQTTX (or a Python subscriber), with `co2_ppm`
   matching the serial value (room air is typically 400 to 1000 ppm), and `boot_id` equal to the boot log.

## Reading the messages

MQTTX: see [the RC522 document](protobuf_rc522_telemetry.md#reading-the-messages) (schema import, **Run Script**
set to Received, proto name `aai16.Envelope`, and disconnect JSON connections while checking Protobuf topics
because the decoder setting is application-wide). Python: `pb.Envelope.FromString(payload)` with
`tools/protocol/generated/aai16_pb2.py`.

## Remaining telemetry migration

GY-511, BME280 and MAX6675 telemetry is still JSON. The schema already has a message for each.
