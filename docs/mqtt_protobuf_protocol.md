# MQTT + Protobuf protocol (draft v1)

Source of truth: `protocol/aai16.proto` and `protocol/aai16.options` (regenerate with `tools/generate_proto.py`).
Every payload is a single binary Protobuf message; no JSON. Strings and repeated fields have fixed maxima.

## Topics (`<id>` = node id, <= 15 characters; configured in menuconfig)

| Direction | Topic | Message |
|---|---|---|
| to node | `aai16/<id>/in/config` | `ConfigApply` |
| to node | `aai16/<id>/in/rules` | `RuleSetApply` |
| to node | `aai16/<id>/in/bridge` | `BridgeApply` |
| to node | `aai16/<id>/in/model/chunk` | `ModelChunk` |
| to node | `aai16/<id>/in/model/control` | `ModelControl` |
| to node | `aai16/<id>/in/command` | `NodeCommand` (RUN / STOP / CLEAR; never forwarded) |
| from node | `aai16/<id>/out/config_ack`, `rules_ack`, `bridge_ack` | `ApplyAck` |
| from node | `aai16/<id>/out/model_chunk_ack` | `ChunkAck` |
| from node | `aai16/<id>/out/model_status` | `ModelStatus` |
| from node | `aai16/<id>/out/inference` | `InferenceResult` |
| from node | `aai16/<id>/out/reading/<sensor_id>` | `Reading` (physical sensors 1..127 only) |
| from node | `aai16/<id>/out/heartbeat` (every 10 s), `out/notification` | `Heartbeat`, `Notification` |

QoS 1 for everything except readings (QoS 0, dropped while disconnected). Bridge input: a node subscribes to
`aai16/+/out/reading/+` and ignores readings that match no configured virtual source.

## Maximum encoded sizes (from the generated nanopb headers)

| Message | max bytes | Message | max bytes |
|---|---|---|---|
| `ConfigApply` | 28146 | `ModelChunk` (<=1024 payload) | 1101 |
| `CommittedConfig` (NVS blob) | 28152 (node limit `NODE_BLOB_MAX` = 12288) | `ModelControl` | 176 |
| `SensorProfile` | 7032 | `ModelContract` | 209 |
| `RuleSetApply` | 4702 | `Reading` | 133 |
| `BridgeApply` | 242 | `Heartbeat` | 143 |
| `ApplyAck` | 28 | `InferenceResult` | 57 |

Typical sizes: configuration A 306 B, configuration B 1.7 KB, a motion/environment model package 3.7 KB / 3.4 KB.
The node reassembles MQTT fragments up to 32 KB (`PLAT_MAX_MESSAGE`); larger messages are dropped and logged.

## Generations and ACKs

- `ConfigApply.generation`, `RuleSetApply.generation`, `BridgeApply.generation` must be **greater** than the committed
  one (0 is never valid). Lower -> `REASON_STALE_GENERATION`; equal -> `REASON_DUPLICATE_GENERATION`.
- Config: `ApplyAck{phase=ADMITTED}` after validation, then `APPLIED` after the buses started **and** the new
  committed configuration was stored; any failure after ADMITTED rolls the buses back and sends `REJECTED`
  (`APPLY_FAILED` / `PERSIST_FAILED`). Rejections carry `reason`, `sensor_id`, `profile_index`, `detail`.
- A `ConfigApply` may name any subset of the four buses; the others keep running. `enabled=false` stops a bus (and deletes its profile). `enabled=false` with `dormant=true` also stops the bus but keeps the complete profile stored (and persisted) as a *dormant* sensor: it reserves its pins, never starts by itself after a reboot, and can be started by a rule. A bus holds either a running or a dormant profile.
- Rule actions `RACT_SENSOR_START` (`sensor_id`, `duration_ms`: 0 = until stopped) and `RACT_SENSOR_STOP` (`sensor_id`) start/stop a dormant sensor. A rule can only start dormant sensors and only stop sensors a rule started; a config for the bus always wins. Ids 1..127 only (physical sensors).
- `ModelChunk`: `transfer_id`, `generation`, `kind`, `format` (TFLITE only), `model_version`, `seq`, `total_chunks`,
  `total_size`, `sha256` (identical in every chunk), `data` (1..1024 B). `ChunkAck` carries `status` and `next_expected`:
  `OK`, `DUPLICATE` (not rewritten), `OUT_OF_ORDER` (resend from `next_expected`), `BAD_LENGTH`, `BAD_TRANSFER`,
  `STORAGE_ERROR`, `DIGEST_MISMATCH`. Seq 0 starts (or restarts) a transfer in the **inactive** bank; the active model is never written.
- `ModelControl.generation` must also increase (except QUERY / ABORT_TRANSFER). ACTIVATE needs the staged transfer id; the
  runtime must accept the model (shapes + quantization equal to its contract) before the pointer flips. SET_INPUTS is all-or-nothing.
- `ModelStatus.reason` values are listed in `ModelReason`; `REJECTED` is an event about a request, followed by the real state.

## Persistence and open items

- The retry policy for a lost APPLIED ACK (see STATUS finding 2) and authentication/encryption of MQTT (plain TCP in this build).
- Persisted in NVS besides the sensor configuration: a `NodeState` (rules, bridge, model generation, ML input mapping; max 5044 B), restored through the same validators at boot.
