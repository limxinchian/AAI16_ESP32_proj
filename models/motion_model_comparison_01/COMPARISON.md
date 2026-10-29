# Model A / Model B comparison — fixed demo scenario only

Fixed demo only: flat stationary desk REST vs continuous hand movement/shaking MOVING.

Two held-out sessions with correlated overlapping windows; no broad generalization claim. Holdout scores were already known for Model A; Model B uses the specified architecture and identical fixed training settings, with no holdout-based tuning or model selection.

| Measure | Model A | Model B |
|---|---:|---:|
| Architecture | 60 -> Dense(16, ReLU) -> Dense(2, linear logits) | 60 -> Dense(8, ReLU) -> Dense(2, linear logits) |
| Trainable parameters | 1010 | 506 |
| model.keras bytes | 33360 | 27310 |
| model_float32.tflite bytes | 5500 | 3484 |
| model_int8.tflite bytes | 3104 | 2384 |
| keras independent accuracy | 99.1071% | 97.3214% |
| tflite_float32 independent accuracy | 99.1071% | 97.3214% |
| tflite_int8 independent accuracy | 99.1071% | 93.7500% |
| Keras vs tflite_float32 independent agreement | 112/112 | 112/112 |
| Keras vs tflite_int8 independent agreement | 112/112 | 108/112 |
| INT8 clipped inputs: training / independent | 0/10080 / 157/6720 | 0/10080 / 157/6720 |
| INT8 input scale / zero point | 0.0031422548927366734 / 12 | 0.0031422548927366734 / 12 |
| INT8 output scale / zero point | 0.046926043927669525 / -12 | 0.0462011881172657 / -29 |

Model B reduces parameters by 49.90% and INT8 file size by 23.20%.

## Independent confusion matrices

Rows = true REST/MOVING; columns = predicted REST/MOVING.

| Runtime | Model A | Model B |
|---|---|---|
| keras | `[[56, 0], [1, 55]]` | `[[56, 0], [3, 53]]` |
| tflite_float32 | `[[56, 0], [1, 55]]` | `[[56, 0], [3, 53]]` |
| tflite_int8 | `[[56, 0], [1, 55]]` | `[[56, 0], [7, 49]]` |

Model B INT8 changes four Keras classifications (held-out indices 102, 104, 110, 111), all MOVING to REST. Clipping is 157/6720 = 2.34% of held-out input elements for both models. The measured regression is retained; neither recalibration nor retuning used these sessions.

## Data and reproducibility

Training/calibration: exactly rest_orientation_01.csv, moving_directions_01.csv, moving_directions_02.csv; 168 windows (58 REST, 110 MOVING), exactly the same hashes/windows as A. Independent evaluation: rest_orientation_02.csv (session rest_orientation_02_retry) and moving_directions_03.csv; 112 windows (56 REST, 56 MOVING), identical hashes/windows to A's saved evaluation.

Both use 20 samples at 10 Hz, stride 10, exclude gaps >150 ms, and share the byte-identical input contract: chronological float32 per-axis mean subtraction, clamp ±2g, divide by 2g, sample-major XYZ flattening. Training uses Adam 0.001, 150 epochs, batch 16, seed 20261028, inverse-frequency class weights, deterministic CPU operations and a fixed final epoch; no validation split or early stopping. Both achieve 168/168 training accuracy and Keras/float/INT8 classification agreement. These are resubstitution results.

A second B run before held-out evaluation reproduced canonical weights, both TFLite binaries, training history and golden/test vectors byte-for-byte. Exact environment, provenance, hashes, quantization and per-session rejection reports are recorded in each bundle and comparison.json.

## Runtime compatibility

Both exports use static INT8 [1,60] input and [1,2] output, two FULLY_CONNECTED v4 operators (fused RELU then NONE), per-output-channel INT8 weights and INT32 biases. A uses 16/2 weight scales; B uses 8/2. No float fallback or custom operator is required.

The unmodified production Engine loaded both models and executed the installed esp-tflite-micro 1.4.1 reference and ESP-NN 1.3.0 ANSI kernels on the laptop. Both host backends produced identical output bytes.

| Model / vectors | Host class agreement with desktop INT8 | Different output elements | Max difference (LSB) |
|---|---:|---:|---:|
| A / training | 168/168 | 2/336 | 1 |
| A / independent | 112/112 | 0/224 | 0 |
| A / golden | 4/4 | 0/8 | 0 |
| B / training | 168/168 | 0/336 | 0 |
| B / independent | 112/112 | 0/224 | 0 |
| B / golden | 4/4 | 0/8 | 0 |

Model B is compatible with the checked operator/engine contract. Future deployment must supply B's bytes, hash, output quantization and golden vectors; the current firmware still embeds Model A. This phase implements no firmware loading, upload or switching. ESP32-S3 assembly-kernel parity, actual inference time, tensor arena/stack/RAM and live sensor/MQTT behavior must be measured on hardware later. No Model B hardware timing or RAM estimate is claimed.

See ../motion_model_b/ for exports, ../motion_model_b_evaluation_01/ for the frozen first evaluation, and ../../tools/ml_training/README.md for reproduction commands.
