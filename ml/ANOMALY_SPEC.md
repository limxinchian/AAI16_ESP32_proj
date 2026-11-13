# Model C - cargo anomaly detector (kind `MODEL_ANOMALY`)

Status: design for implementation (nothing here is hardware-verified until the board test in `docs/`).

## Question it answers
"Do the readings of the sensors that are running right now, taken together, look unusual?" It does **not** name the
cause. It reports one anomaly score, a verdict, and which sensor *slot* contributed most.

## Inputs: 8 role slots, filled by any sensor of the right kind
The contract lists 10 value channels (a mapping message assigns a sensor value to each channel; unmapped channels are
simply absent). Channel -> slot:

| channel | `ValueKind` | slot | role | typical source |
|---|---|---|---|---|
| 0,1,2 | ACCEL_X/Y/Z_G | 0 motion (magnitude of the 3) | handling | GY-511 |
| 3 | TEMPERATURE_C | 1 air temperature | climate | DHT22 / BME280 |
| 4 | HUMIDITY_PCT | 2 humidity | climate | DHT22 / BME280 |
| 5 | PRESSURE_PA | 3 pressure | altitude / door | BME280 |
| 6 | CO2_PPM | 4 CO2 | air | MH-Z19B |
| 7 | DISTANCE_CM | 5 distance | door / load | HC-SR04 |
| 8 | RADAR_DISTANCE_MM | 6 radar | presence | LD2450 (y) |
| 9 | TEMPERATURE_C | 7 equipment temperature | cooling unit | MAX6675 |

Motion needs all three of channels 0..2 mapped. Any other combination is allowed (at least one slot).

## Per-slot features (PREPROC_SLOT_STATS_V1): 4 per slot, 32 in total
Rate independent: updated on every reading with `alpha = 1 - exp(-dt / tau)` (dt in seconds, doubles).

State per slot: `fast` (tau 3 s), `mid` (tau 15 s), `slow` (tau 90 s) of the slot value, `jit` (tau 8 s) of
`|x - fast|` (fast already updated). The first sample initialises fast = mid = slow = x, jit = 0.

| feature | formula | clip |
|---|---|---|
| f0 level | (fast - center) / s0 | [-4, 4] |
| f1 baseline deviation | (fast - slow) / s1 | [-4, 4] |
| f2 jitter | jit / s2 | [0, 4] |
| f3 short trend | (fast - mid) / s3 | [-4, 4] |

Constants (center, s0, s1, s2, s3) per slot:

| slot | center | s0 | s1 | s2 | s3 |
|---|---|---|---|---|---|
| 0 motion (|a| in g) | 1.0 | 0.5 | 0.5 | 0.2 | 0.5 |
| 1 temperature C | 25 | 15 | 2.0 | 0.5 | 1.0 |
| 2 humidity % | 60 | 40 | 10 | 2.0 | 4.0 |
| 3 pressure Pa | 101300 | 3000 | 100 | 30 | 60 |
| 4 CO2 ppm | 700 | 1000 | 300 | 60 | 150 |
| 5 distance cm | 100 | 100 | 20 | 5 | 20 |
| 6 radar mm | 1500 | 1500 | 400 | 150 | 400 |
| 7 equipment C | 40 | 40 | 3.0 | 0.5 | 1.5 |

A slot is **present** when its newest reading is not older than `max_age_ms` and it has at least 5 readings.
Absent slots have all four features = 0 and are excluded from the score.

## Running it
Inference every `sample_period_ms` (1000 ms) of wall time, triggered by an incoming reading, when at least one slot is
present. No fixed window: the contract has `window_samples = 1`, `stride_samples = 1`.

## Scoring (on the node, from the int8 tensors only)
`xq[i] = (in_q[i] - in_zp) * in_scale`, `yq[i] = (out_q[i] - out_zp) * out_scale`, `err[i] = (xq[i] - yq[i])^2`.
`score = sum(err over present features) / (4 * present_slots)`; `slot_error[s] = mean(err over slot s)`;
`anomaly = score > contract.anomaly_threshold`.

## Training (synthetic, honest)
Autoencoder 32-24-12-24-32 (ReLU, linear output), trained on NORMAL cargo operation only, with random slots removed
(masked loss). Threshold = 99.7th percentile of the int8 score on held-out normal data x 1.25. Evaluation also injects
shakes, humidity/temperature/CO2 jumps, pressure drops, door/distance changes, radar targets and equipment overheating,
and reports detection and false-alarm rates. All data is synthetic: this proves the mechanism, not field accuracy.
