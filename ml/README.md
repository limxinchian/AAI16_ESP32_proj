# ML: motion, environment and anomaly models

> Model C (anomaly detector, `MODEL_ANOMALY`) was added after the two models below: 8 role slots (motion, temperature,
> humidity, pressure, CO2, distance, radar distance, equipment temperature), an autoencoder trained on normal data only,
> a presence mask so any subset of sensors works, and per-slot error attribution. Spec: `ANOMALY_SPEC.md`. Train with
> `train_anomaly.py` (synthetic) or `train_anomaly.py --real data/anomaly/normal_01.csv` (real-data mix); score a recording
> exactly like the node does with `eval_real_anomaly.py`. The real-data version covers temperature, humidity, pressure and
> equipment temperature; the other slots stay synthetic. Recordings come from `tools/record_slot_data.py`.

The two original models (handoff §5):

| | Model A - Motion | Model B - Environment |
|---|---|---|
| Sensor | GY-511 accelerometer (x, y, z in g) | DHT22 **or** BME280 (temperature °C, humidity %RH) |
| Classes | STATIONARY, MOVING, VIBRATING | STABLE, CHANGING |
| Window | 64 samples @ 50 Hz (20 ms), 25 % tolerance | 20 samples @ 2 s, 30 % tolerance |
| Preprocessing | `PREPROC_MOTION_STATS_V1` → 11 features | `PREPROC_ENV_TREND_V1` → 6 features |
| Network | Dense 16-8-3, softmax | Dense 12-8-2, softmax |
| TFLite | 3.6 KB, full int8, ops `FULLY_CONNECTED`, `SOFTMAX` | 3.3 KB, same ops |
| Inference stride | every 32 new samples | every 5 new samples |

Train / export: `.venv\Scripts\python.exe ml\train_motion.py` and `ml\train_environment.py`
(pinned versions in `requirements.txt`). Output goes to `ml/artifacts/<task>/`:
`model_int8.tflite`, `<task>.amdl` (the uploadable **model package**), `golden.json`, `contract.json`,
`report.json`, `SHA256SUMS`.

## DATA PROVENANCE - read this before quoting accuracy

Both models are trained on **synthetic** windows generated in `ml/common.py` (gravity + noise, 0.4-3 Hz sway,
7-22 Hz shake; DHT22-/BME280-like noise and quantization with drift/ramp/step/half-sine changes).
They score 100 % on held-out *synthetic* data because the synthetic classes are cleanly separable. That
proves the pipeline (feature maths → int8 model → package → firmware logic), **not** field accuracy.
The CHANGING/STABLE labels are defined outside a gap (stable < 0.15 °C & < 0.6 %RH over the window,
changing ≥ 0.8 °C or ≥ 4 %RH); windows inside the gap are never sampled and are ambiguous by design.
To get real numbers: record labelled GY-511 / DHT22 / BME280 windows, replace `generate_*`, retrain.

## Model package and contract

`"AMDL" | u32 LE contract_len | ModelContract (nanopb) | .tflite`. The `ModelContract` (see
`protocol/aai16.proto`) carries kind, version, input channel *meanings* (`VALUE_ACCEL_X_G` ...), window,
sample period + tolerance, max sample age, preprocessing id, feature count, int8 scales/zero-points,
class labels, stride, arena size and the SHA-256 of the `.tflite`. The firmware refuses a mapping unless
each sensor output's declared `ValueKind` equals the channel kind (so DHT22 temperature → motion model is
rejected, DHT22 → BME280 temperature/humidity for the environment model is accepted).

`tensor_arena_bytes = 16384` is a **declared guess**; it has not been measured with TFLM on a target.

## What is verified where

| Claim | Evidence |
|---|---|
| Python features == C features | `tests/ml_host`: 25 golden windows, tolerance 1e-4 |
| int8 quantization == TFLite's | 225/225 feature values identical |
| int8 output decode + argmax == TFLite interpreter | all 25 vectors |
| Package parse/digest/limits, selector compat, streaming, suspension, chunked upload/ACK/stage/activate/rollback/reboot | `tests/ml_host` (4315 checks) |
| TFLM runs these `.tflite` files on the ESP32-S3 | **NOT verified** (no ESP-IDF here) |
| Same logits on the board as on the PC | **NOT verified** |
