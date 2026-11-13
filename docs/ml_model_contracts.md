# ML model contracts (exactly two models)

Per handoff §5 and the team decision: **Model A - Motion classification** and **Model B - Environmental classification**.
No anomaly/autoencoder model. Details, data provenance and how to retrain: `ml/README.md`.

> **Both models are trained on SYNTHETIC data and score 100 % on held-out synthetic data. That proves the pipeline,
> not field accuracy. Collect real labelled windows (below) and retrain before quoting any accuracy.**

| | Model A (kind `MODEL_MOTION`) | Model B (kind `MODEL_ENVIRONMENT`) |
|---|---|---|
| Input channels (`ValueKind`) | `ACCEL_X_G`, `ACCEL_Y_G`, `ACCEL_Z_G` | `TEMPERATURE_C`, `HUMIDITY_PCT` |
| Valid sources | GY-511 (any sensor whose outputs are declared with those kinds) | DHT22 **or** BME280 (same kinds) |
| Window | 64 samples, 20 ms apart (+-25 %), newest sample <= 500 ms old | 20 samples, 2000 ms apart (+-30 %), newest <= 6000 ms old |
| Inference stride | every 32 new samples | every 5 new samples |
| Preprocessing | `PREPROC_MOTION_STATS_V1` -> 11 features (per-axis std, mean abs diff, zero-crossing rate; total std; roughness) | `PREPROC_ENV_TREND_V1` -> 6 features (per channel: trend, residual std, range) |
| Network | Dense 16-8-3, softmax, int8 | Dense 12-8-2, softmax, int8 |
| Classes | STATIONARY, MOVING, VIBRATING | STABLE, CHANGING |
| TFLite ops | `FULLY_CONNECTED`, `SOFTMAX` | same |
| Size | 3.6 KB model, 3.7 KB package | 3.3 KB model, 3.4 KB package |

The feature definitions are specified once in `ml/common.py` (reference) and implemented in `components/ml_core/ml_features.c`;
golden vectors (`ml/artifacts/*/golden.json`) pin them together (features within 1e-4, int8 inputs identical 225/225).

## What the node enforces before inference (`ml_selector`)

A `SET_INPUTS` request is accepted only if **all** of these hold, otherwise nothing changes and a `ModelStatus` with the
reason is published: the number of inputs equals the channel count; slots are in range and unique; every sensor is part of
the running configuration (or a live virtual sensor); `value_index` exists; the value's declared `ValueKind` equals the
channel kind (DHT22 temperature -> motion model is refused); the first channel's sensor period matches the model period.
While running, a configuration change that breaks the mapping (e.g. GY-511 replaced by BME280 while Model A is active)
**suspends** inference (`SUSPENDED` + reason), keeps the model and the mapping, and never feeds samples from before the
suspension. Invalid readings, stale inputs and timing gaps restart the window.

## Model package and activation

`"AMDL" | u32 contract length | ModelContract | .tflite` with SHA-256 of the `.tflite` inside the contract. Upload goes to the
inactive flash bank; the node checks size, digest and contract, then (on ACTIVATE) lets TFLite Micro load it and compares
tensor shapes and quantization with the contract; only then is the active pointer saved. A model with other ops or shapes
is rejected, never run. `tensor_arena_bytes` (16 KB) is a guess; the loader logs the real use on first load.

## Collecting real data and retraining

Full commands are in `docs/STEP_BY_STEP_GUIDE.md` (Phase 8). In short: `tools/collect_readings.py` saves the node's readings as one CSV per
recording (one label per file, several separate sessions per class); `ml/train_motion.py --data data/motion --out ml/artifacts_real/motion --version 2`
(and the same for `train_environment.py`) builds windows from consecutive samples only (a gap or wrong sample period rejects a window),
splits **by recording session** so overlapping windows never leak into the test set, and writes a new model package. `report.json` states
the provenance (REAL recordings), the split mode and the honest accuracy; fewer than 3 sessions per class falls back to an optimistic
chronological split and says so.