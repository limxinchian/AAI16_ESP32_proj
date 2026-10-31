# RC522 telemetry: binary Protobuf on the existing topic

First sensor-telemetry migration (the Protobuf change requirement). `aai16/sensors/rc522/event`
now carries a binary `aai16.Envelope` with a `sensor_telemetry.rc522` body. The JSON
event text is removed from the firmware; there are no JSON bytes inside the message.
Existing JSON-only subscribers of this topic must use the schema decoder (below).

**Status.** Verified end to end on an ESP32-S3 running ESP-IDF 6.1: a real card tap was published
to a Mosquitto broker and decoded both by an independent Python decoder and by MQTTX
(see [Hardware verification](#hardware-verification)). Host tests also pass (C encoder, C decoder, and
Python decoder), and the full firmware image builds (0x11cdd0 bytes, 72% of the app partition free).
See [IDF 6.1 build](#idf-61-build).

## Wire contract

Topic `aai16/sensors/rc522/event`, QoS 1, retain false, one message per card
presentation (not per poll). The schema is unchanged: `protocol/aai16.proto`
(`Envelope` body tag 20, `SensorTelemetry` reading tag 12, `Rc522Event`).

| Previous JSON field | Protobuf field | Notes |
| --- | --- | --- |
| `"sensor":"rc522"` | `sensor_telemetry.sensor = SENSOR_RC522` | Translated through `config_sensor_to_wire()`, not cast |
| `"event":"card_presented"` | implied | One message type on this topic, one meaning |
| `"uid_hex":"5A2DFD03"` | `sensor_telemetry.rc522.uid` | Raw bytes, not hex text |
| `"uid_length":4` | `len(rc522.uid)` | Exactly 4, 7 or 10 |
| `"boot_id":"12345678"` | `Envelope.boot_id` | Same `uint32`, no longer hex text |
| `"seq":N` | `Envelope.sequence` | Attempt count: rejected/dropped submissions still advance it |
| `"uptime_ms":N` | `sensor_telemetry.acquired_us` and `.uptime_us` | Microseconds; divide by 1000 for the old value |
| (none) | `Envelope.generation` | **New.** The sensor's committed configuration generation at submission |
| (none) | `Envelope.protocol_version = 1` | **New**, required by every message |

No information in the JSON message is lost. `request_id`, `phase`, `health` and
`interval_ms` are intentionally absent: this is an event, not a configuration
transaction or a periodic reading.

## Bounds and validation (before anything is published)

| Field | Valid range | Enforced by |
| --- | --- | --- |
| `rc522.uid` | exactly 4, 7 or 10 bytes; `max_size:10` in `aai16.options` | publisher argument check, then `aai16_protocol_encode` -> `telemetry_validate` |
| `Envelope.protocol_version` | exactly 1 | codec |
| `Envelope.boot_id`, `sequence` | required | codec |
| `acquired_us` <= `uptime_us` | required, equal here | codec |
| whole message | `< 320` bytes (shared TX slot); worst case measured 54 | `_Static_assert` + encode capacity |

An invalid UID length, or a failed encode, increments the RC522 `invalid` counter and
publishes nothing. Storage is static nanopb: no heap allocation, no `snprintf`, no JSON.

## Size comparison (JSON vs Protobuf)

Same card, same boot, `seq=12`, uptime 5 minutes. JSON is the exact format the
previous firmware published (`git show HEAD~:components/mqtt_manager/mqtt_manager.c`).
These are payload bytes, not MQTT/TCP packet sizes, and say nothing about the size of
decoded C structures.

| UID length | JSON | Protobuf | Reduction |
| ---: | ---: | ---: | ---: |
| 4 bytes | 128 B | 42 B | 67.2 % |
| 7 bytes | 134 B | 45 B | 66.4 % |
| 10 bytes | 141 B | 48 B | 66.0 % |

JSON example (128 B):
`{"sensor":"rc522","event":"card_presented","uid_hex":"5A2DFD03","uid_length":4,"boot_id":"12345678","seq":12,"uptime_ms":300000}`

Protobuf example (42 B), hex:
`08011d785634122500000000350c000000a2011608031080c6868f011880c6868f0162060a045a2dfd03`

The smaller event also changes one existing behaviour, deliberately: more events fit in
the 1024-byte event outbox allowance before backpressure starts. The ACK reserve
(4096 bytes) is unaffected.

## Verification performed

| Check | Result |
| --- | --- |
| `tests/mqtt_host` C suite (all sensors, config, ML, model routing) | all `PASS` |
| RFID test: UID 4/7/10 with embedded `0x00`, offline FIFO, expiry, drop accounting, ACK priority, QoS 1 handoff | `PASS`, now decoding Protobuf |
| C/nanopb bytes -> Python decoder (`--rc522-bridge`): UID 4/7/10, exact `uint64` timestamp, sequence, generation, no request id / phase | `PASS` |
| C bytes identical to Python's serialisation of the same message | identical (48 / 51 / 54 B) |
| Installed ESP-MQTT SDK model, ACK/RFID reserve | `PASS` |
| `mqtt_manager.c` compiled for ESP32-S3, ESP-IDF 6.1, `-Wall -Werror` | compiles; object contains no `card_presented` / `uid_hex` |

Run the suite: `python tests/mqtt_host/run.py --idf-path <ESP-IDF path>` (needs a host C compiler and
`pip install -r tools/protocol/requirements.txt`). The results above were obtained with a Zig-based
C compiler on Windows; any gcc should behave the same.

Test changes worth knowing about: the old test used `strcpy`/`strcmp` on the payload, which stops
at the first `0x00` and would silently truncate a binary message; it now uses lengths and `memcmp`.
The outbox-backpressure test pre-fills part of the allowance so it no longer depends on how large
an event happens to be.

## Hardware verification

Performed on the project board (ESP32-S3, ESP-IDF 6.1), broker Mosquitto 2.1.2 on the demo laptop:

| Check | Observed |
| --- | --- |
| Reader start | `rc522 | HEALTH | Ready | CS=14 RST=21`, `state=READY init=1` |
| Card detected | `rc522 | DATA | event=PRESENT uid=F173CE01 uid_len=4` |
| Event built | `rc522_event attempts=2 queued=2` (one event per presentation, not per poll) |
| Message on the broker | 40 bytes on `aai16/sensors/rc522/event`, QoS 1, captured by `mosquitto_sub` |
| Independent decode (Python, project schema) | `sensor_telemetry`, `SENSOR_RC522`, `boot_id=0xC2B09CED` (equal to the boot log), `sequence=2`, `generation=0`, UID `F173CE01`, uptime 99.9 s |
| MQTTX decode (schema `aai16.proto`, `aai16.Envelope`) | Same fields; `uid` shown as Base64 `8XPOAQ==` (= `F1 73 CE 01`), 64-bit times shown as strings |

Not measured: delivery latency, behaviour under load, and a card-removal/re-presentation sequence over
many cycles. A card left on the reader produces one event only; it must be removed and presented again
for the next.

## Reading the messages

- **MQTTX:** create a separate connection for telemetry. Under **Scripts -> Schema** import
  `protocol/aai16.proto`; in the connection's **Use Script** dialog choose Schema Type **Protobuf**,
  Proto name **aai16.Envelope**, applied to **Received**, with **no** function selected. Subscribe to
  `aai16/sensors/rc522/event`. Keep the existing ACK Monitor connection separate, because MQTTX
  applies the script setting application-wide, not per connection, so a JSON connection that is
  connected at the same time shows deserialization errors for its JSON messages
  ([workflow](protobuf_config_workflow.md)); disconnect it while checking Protobuf topics. In MQTTX 1.13.1
  the connection menu entry is **Run Script**, and the decoded `uid` bytes appear as Base64 text.
- **Python:** `pb.Envelope.FromString(payload)` with `tools/protocol/generated/aai16_pb2.py`.

## IDF 6.1 build

Before this change the branch did not build on ESP-IDF 6.1: `components/motion_ml/motion_engine.cc` and
`components/model_upload/upload_store.cc` included `mbedtls/sha256.h`, which ESP-IDF 6.1's mbedTLS 4 no
longer provides (it moved to the PSA crypto API). The ML code was written against ESP-IDF 5.5.5.

Fixed with `components/motion_ml/include/motion_sha256.h`, a small wrapper used by both files: IDF 6 and
later use PSA (`psa_hash_compute` / `psa_hash_setup`, `PSA_ALG_SHA_256`, preceded by `psa_crypto_init()`);
earlier IDF versions and the host tests make exactly the same mbedTLS calls as before.

Verification: the one-shot hash **ran on the device**. At boot the ML engine verifies the bundled model's
SHA-256 before use, and the boot log shows `ML: initializing model=A ... golden+trial=PASS`; a wrong hash
would have been reported as `sha256_mismatch`. Not exercised: the streaming hash used when a stored model
package is verified after upload (no upload was performed), and the host ML/upload tests
(`tests/model_upload_host`, `tests/motion_host`), which need a host TensorFlow Lite build.

## Remaining telemetry migration

DHT22, GY-511, LD2450, HC-SR04P, MH-Z19, BME280 and MAX6675 telemetry is still JSON. The schema
already has a message for each; they follow the same pattern as this one.
