# GY-511 Phase 2: preliminary REST/MOVING Model A

This is a trained TensorFlow/Keras neural network for a narrow demonstration:
flat stationary desk **REST=0**, and continuous hand movement/shaking
**MOVING=1**. It is ready for independent laptop evaluation. Its performance
on new sessions and its ESP32-S3 runtime behavior remain unverified.

Model version: `aai16-motion-model-a-0.1.0`. Artifacts are in
[`models/motion_model_a/`](../../models/motion_model_a/).
The implementation stays on `feature/ml-gy511-data-collection`, HEAD
`ed28c75467f87f61a0368c9d9a70f1eb7353dae8`, with uncommitted work preserved.
Phase 2 adds only laptop training code and model artifacts. The eight sensor
implementations, collector, MQTT behavior, credentials, partitions and existing
`dependencies.lock` changes are unchanged. No branch creation/switch, commit,
push, merge, hardware flash or firmware inference was performed.

## Dataset audit and window admission

The three original measured CSVs remain untouched in
`tools/ml_data_collection/recordings/`. The validator checks required/duplicate
CSV columns, finite numeric XYZ acceleration, labels, session identity,
8-hex-digit boot IDs, uint32 sequence numbers, actual acquisition timestamps,
timestamp/uptime agreement, and chronological order within each boot.
Possible +/-2.048 g register-range saturation and constant data are flagged
for review; constant values alone do not invalidate a fresh REST reading.
Malformed or invalid rows remain barriers at their original row positions.
Mixed valid labels/session IDs in a file are fatal. Nothing is sorted,
interpolated, resampled, fabricated, or written back to the source CSVs.

| Recording | Raw/valid rows | Gaps >150 ms | Candidate windows | Rejected | Usable |
| --- | ---: | ---: | ---: | ---: | ---: |
| rest_orientation_01.csv | 599 / 599 | 0 | 58 | 0 | 58 |
| moving_directions_01.csv | 598 / 598 | 2 | 58 | 4 | 54 |
| moving_directions_02.csv | 599 / 599 | 1 | 58 | 2 | 56 |
| Total | 1,796 / 1,796 | 3 | 174 | 6 | **168** |

Median acquisition interval is 100 ms in every file. REST intervals are
98..102 ms. MOVING 01 has 199 ms at zero-based row 526 and 200 ms at row 579;
MOVING 02 has 201 ms at row 202. CSV physical lines are 528, 581, and 204,
respectively. All three sessions have boot ID `5120e965`, with distinct
session identities and time spans. There are no invalid rows, duplicate or
backward sequences, sequence gaps, reboots, constant axes/windows, or possible
saturation warnings in these recordings.

Candidate starts are original session row indices **0,10,20,...**, each taking
20 consecutive rows. A window containing an invalid row or crossing a gap
greater than 150 ms, sequence discontinuity, or reboot is rejected. A boundary
immediately before a window's first row does not invalidate that window.
The stride grid is not restarted after a gap. This independently produces
168 usable windows. Restarting the stride grid within each continuous segment
would produce 170; that is a different admission policy and is not used here.

Training distribution: **58 REST (34.52%)**, **110 MOVING (65.48%)**. Windows
overlap by 10 samples and are correlated. Only one REST session exists.
Full ranges, standard deviations, timestamps, gaps, source SHA-256 hashes and
window counts are in
[`dataset_report.json`](../../models/motion_model_a/dataset_report.json).

## Exact preprocessing and input/output contract

The collector continues to save raw measured XYZ acceleration in g. Training
and evaluation both call `data.preprocess()` on each admitted window:

1. Convert 20 chronological XYZ samples to float32, shape `[20,3]`.
2. Accumulate each axis in chronological order, with float32 additions. Divide
   each sum by float32(20), then subtract that per-axis mean in float32.
3. Clamp each centered value to `[-2,2]` g.
4. Divide by float32(2).
5. Flatten **sample-major**, `x0,y0,z0,x1,y1,z1,...,x19,y19,z19`.

There is no axis remapping, filtering, separate gravity removal, global
standardization, or learned scaler. Mean removal is part of model preprocessing,
not a modification of the recordings. Sequential float32 accumulation is
specified so a later C implementation can reproduce the reduction order.

Keras accepts batches of 60 float32 features. Both exported TFLite models have
fixed input shape **[1,60]** and output shape **[1,2]**. Outputs are logits in
`[REST,MOVING]` order. Choose argmax; an exact tie selects REST. These logits
are not probabilities, and no confidence threshold has been validated.

The machine-readable contract is
[`input_contract.json`](../../models/motion_model_a/input_contract.json).
`test_vectors.npz` contains every admitted real training window, its float32
features, labels, logits, INT8 inputs and INT8 outputs. `golden_vectors.json`
contains four readable examples with source provenance for later C checks.

## Training configuration and actual results

Architecture: **60 -> Dense(16, ReLU) -> Dense(2, linear logits)**.
Parameters: `(60*16+16) + (16*2+2) = 976+34 = 1,010`.

- Python 3.12.10, TensorFlow 2.20.0, Keras 3.11.3, NumPy 2.2.6.
- Seed 20261028 for Python/NumPy/TensorFlow through Keras; separate seeded
  NumPy generator shuffles training windows each epoch.
- Adam, learning rate 0.001, beta1 0.9, beta2 0.999, epsilon 1e-7.
- Sparse categorical cross entropy with `from_logits=True`.
- 150 epochs, batch size 16, no early stopping, fixed final epoch.
- Inverse-frequency sample weights `N/(2*N_class)`: REST
  1.4482758620689655, MOVING 0.7636363636363637.
- CPU only, one inter-op/intra-op thread, deterministic TensorFlow operations,
  oneDNN disabled. No validation split, random window test split, or heldout
  hyperparameter selection. The two-epoch smoke run only checked conversion.

All 168 usable windows from the three named files train the network. INT8
calibration and RMS threshold selection also use only these training windows.
Final epoch weighted loss is **0.0694552362**. The training history's accuracy
is an unweighted running epoch metric; the following results are computed
again after the final weight update.

**Every score below is training-set resubstitution, not independent evaluation.**

| Method | Correct / windows | Accuracy | Balanced accuracy |
| --- | ---: | ---: | ---: |
| Keras neural network | 168 / 168 | 100% | 100% |
| Float32 TFLite neural network | 168 / 168 | 100% | 100% |
| INT8 TFLite neural network | 168 / 168 | 100% | 100% |
| Movement-RMS threshold baseline | 168 / 168 | 100% | 100% |

For every method, the confusion matrix (rows true, columns predicted, order
REST/MOVING) is `[[58,0],[0,110]]`. REST precision/recall/F1 are each 1.0
(support 58); MOVING precision/recall/F1 are each 1.0 (support 110).

The separate baseline computes
`sqrt(mean_t(sum_axis((a_g - mean_t(a_g))^2)))` in float64 and predicts MOVING
when RMS >= **0.08883622916580566 g**. The threshold maximizes training balanced
accuracy among adjacent-value midpoints and outer bounds; tied best candidates
use the middle index. No inference path substitutes this rule for the network.
Observed RMS ranges are 0.0035784..0.0059184 g for REST and
0.1717541..0.5684183 g for MOVING. This strong separation explains why the
baseline already succeeds. **ML has not demonstrated added classification
value over the baseline on these data.**

One REST recording and overlapping windows cannot establish generalization.
No project accuracy requirement is claimed to be met. New independent sessions,
more sessions per class, and later tests at transition/borderline motion are
needed to assess robustness within the chosen demonstration scope. This is not
human activity recognition, a fall detector, or a safety decision system.

Full metrics are in
[`training_report.json`](../../models/motion_model_a/training_report.json);
`window_predictions.csv` ties predictions back to recording rows/times.

## Export and quantization verification

The original trained model (including optimizer state) is saved as
`model.keras`. TensorFlow converts a fixed-shape inference signature to both
float32 and full INT8 TFLite. The representative generator supplies all 168
preprocessed training vectors; no future evaluation file is read.

| Artifact | Bytes | KiB |
| --- | ---: | ---: |
| model.keras | 33,360 | 32.578 |
| model_float32.tflite | 5,500 | 5.371 |
| model_int8.tflite | **3,104** | **3.031** |

FlatBuffer inspection and laptop interpreter execution confirm two
`FULLY_CONNECTED` operators: fused RELU then NONE. Float32 uses operator
version 1; INT8 uses version **4**, per-output-channel INT8 weights (16 and 2
scales), INT32 biases and INT8 activations/I/O. There are no float tensors,
custom operators or SELECT_TF_OPS in the INT8 model. This establishes the
export's structure; target TFLite Micro kernel compatibility still needs testing.

| INT8 tensor | Shape | Scale | Zero point |
| --- | --- | ---: | ---: |
| Input | [1,60] | 0.0031422548927366734 | 12 |
| Output | [1,2] | 0.046926043927669525 | -12 |

Encode preprocessed float32 features using
`q = clamp(round_ties_to_even(float32(x / input_scale)) + input_zero_point, -128,127)`.
Decode outputs using `float32(q - output_zero_point) * output_scale`.
Both output logits share a positive scale/zero point, so argmax can operate
directly on the INT8 output. C's usual `roundf()` has a different half-tie rule;
reproduce the documented ties-to-even encoding explicitly and check golden vectors.

Calibration observed features in `[-0.4410499930, 0.3602249920]` and clipped
zero training input elements during quantization. The calibrated INT8 range
is narrower than the full preprocessing range `[-1,1]`. Stronger future motion
can saturate the quantized input; independent evaluation reports the count.
Do not recalibrate this frozen Model A using the reserved holdouts before the
first evaluation. Full tensor parameters are in
[`quantization.json`](../../models/motion_model_a/quantization.json).

| Same 168 real vectors versus Keras | Prediction agreement | Mean absolute logit error | Maximum absolute logit error |
| --- | ---: | ---: | ---: |
| Float32 TFLite | 168/168 | 1.7633041e-7 | 1.4305115e-6 |
| INT8 TFLite, dequantized | 168/168 | 0.013064044 | 0.057326078 |

## Artifact identity and reproducibility

`manifest.json` records version, UTC creation time, branch/HEAD, uncommitted
source hashes, data hashes, exact training settings, package versions, canonical
weight hash, sizes and artifact hashes. `environment.lock.txt` freezes all
installed training packages. `SHA256SUMS` covers every other bundle file,
including the manifest. Evaluation verifies these checksums before loading the
model, and checks that preprocessing/runtime source hashes still match.

SHA-256 identities for the delivered model files:

```text
0922fb3deed29858a5d3171d64ef59382047f39b0b5160fe12743346c97aa872  model.keras
e1e795767dba3157ff4ade990e132bd1ece9e1c2579ef24945fb9780a008a723  model_float32.tflite
1885f051bb81daa9f4f8ff12ee39a0706ac4d45999b823ef91ade991e5a4b7aa  model_int8.tflite
```

Canonical learned-weight hash (shape plus little-endian float32 weight bytes):
`ed4614c605ab29c99ee616794ad8cf130899b1ba22a631ebe568b149f00b121a`.

A second full seeded run on this machine reproduced the weights, every saved
test-vector array, training history/results, and both TFLite files bit for bit.
The Keras ZIP has different save timestamps and serialized shared-object IDs;
its `model.weights.h5` is identical. Thus a freshly saved `.keras` archive need
not have the same whole-file hash. Results on a different CPU/OS/TensorFlow
build need verification; the recorded determinism check is local.

From the repository root in PowerShell, use a **standalone Python 3.12**, not
the ESP-IDF Python environment:

```powershell
py -3.12 tools/ml_training/setup_env.py --requirements models/motion_model_a/environment.lock.txt
& tools/ml_training/.venv/Scripts/python.exe -m pip check
& tools/ml_training/.venv/Scripts/python.exe tools/ml_training/train.py --seed 20261028 --epochs 150 --batch-size 16 --learning-rate 0.001 --output-dir build/motion_model_a_reproduction
& tools/ml_training/.venv/Scripts/python.exe -m unittest discover -s tools/ml_training -p test_training.py -v
& tools/ml_training/.venv/Scripts/python.exe tools/ml_training/evaluate.py --mode training-sanity --output-dir build/motion_model_a_sanity_replay
```

If `py` is unavailable, the standalone interpreter used here was
`C:/Users/limxi/AppData/Local/Programs/Python/Python312/python.exe`; invoke that
absolute executable with `&` instead. For a first setup without an existing
bundle, omit `--requirements` to use the pinned direct requirements file.
`setup_env.py` installs only into `tools/ml_training/.venv`; on Windows it uses
an extended interpreter path to handle TensorFlow's long header paths without
changing OS settings. `.venv` is already Git-ignored. The default Python 3.14
and ESP-IDF environment were not used for installing TensorFlow.

Training refuses to overwrite any existing output directory. Use a fresh
directory for each reproduction; keep the delivered `models/motion_model_a`
frozen. Training only reads the three explicit `TRAIN_FILES`, never a glob.
The original CSVs are Git-ignored; reproducing training elsewhere requires
copies with the hashes recorded in `dataset_report.json`.

## First independent evaluation: exact procedure

1. Keep this model bundle, preprocessing, RMS threshold and calibration frozen.
   Use the existing Phase 1 firmware and broker connection. Set GY-511 enabled
   and interval 100 ms through the existing MQTT commands, wait for applied
   ACKs and old queued samples to drain. Repeat after any ESP32 reboot.
   See the [Phase 1 procedure](../ml_data_collection/README.md).
2. Collect a **new** 60-second flat, stationary desk session. Start the MOVING
   session only after continuous movement has begun, and maintain movement
   throughout. Use the collector's existing Python/Paho environment and broker
   environment variables; these commands do not install Paho into the ML venv:

   ```powershell
   python tools/ml_data_collection/collector.py --label REST --duration 60 --session-id rest_orientation_02 --password-prompt
   python tools/ml_data_collection/collector.py --label MOVING --duration 60 --session-id moving_directions_03 --password-prompt
   ```

   Append the same broker `--port`, `--tls` and `--ca-file` options used during
   Phase 1, if needed. Do not copy/rename old recordings into these filenames.
   Keep transitions outside the recordings, inspect collection summaries,
   and keep raw data/metadata intact. Labels come from the collection protocol,
   not model predictions. Do not tune movement or discard sessions based on scores.
3. Run the frozen evaluation once; independent mode is the default:

   ```powershell
   & tools/ml_training/.venv/Scripts/python.exe tools/ml_training/evaluate.py --model-dir models/motion_model_a --data-dir tools/ml_data_collection/recordings --output-dir models/motion_model_a_evaluation_01
   ```

   It requires both exact filenames, both classes with usable windows, new
   session identities and non-overlapping boot/time spans. Reused training
   hashes or session IDs are rejected. Gaps and reboots use the same window
   rejection policy as training. Missing files fail before output is created.
4. Preserve `evaluation.json` and `predictions.npz`. Compare Keras, float and
   INT8 confusion matrices/per-class precision/recall/F1, baseline results,
   Keras/TFLite disagreement, quantization clipping and usable-window counts.
   There is no fitting, recalibration or threshold selection in this command.
   Existing evaluation output directories are never overwritten.
5. Treat even this initial two-session holdout as limited evidence: its
   overlapping windows remain correlated. If results lead to any training,
   threshold, preprocessing or calibration change, version a new model and
   reserve new sessions for its evaluation. The first holdouts then become
   development data and no longer constitute an untouched test set.

No independent scores are available now. The missing-holdout command was
actually exercised: it exited with a clear error naming both missing files and
created no result directory. The separate `--mode training-sanity` command
reloaded and checked the frozen model against the exact original training hashes.

## Validation performed and later ESP32-S3 work

**All 25 ML tests passed.** The suite checks preprocessing order/float32 arithmetic/flattening,
clipping, units, malformed values/headers, exact 150 ms boundary, original
stride, sequence wrap/gaps/duplicates/backward rows, reboot isolation, preserved
CSV bytes, constant/saturation warnings, session mixing, independent-session
guards, RMS/metrics, the actual 168-window dataset, saved golden vectors,
Keras/TFLite replay, tensor types/shapes/operators, quantization rounding,
artifact corruption, and missing holdouts. Test fixtures are synthetic only
inside unit tests; the trained model and exported golden vectors use real data.
Package consistency was checked with `pip check`. Existing collector tests were
also rerun: 20 passed, 2 optional Paho socket tests skipped because Paho is not
installed in the default Python environment. Hash comparison confirmed all 124
pre-existing files remained byte-for-byte unchanged, including the source CSVs,
credentials, firmware and dependency lock. Firmware builds/host C tests were not rerun for this laptop-only
phase; no firmware file changed.

Later firmware deployment should:

1. First review the independent results and freeze the approved model identity.
   Select and verify an ESP32-S3 TFLite Micro runtime/kernel version supporting
   this exact FlatBuffer and both version-4 per-channel INT8 fully connected
   operators, including fused ReLU. No runtime has been integrated in Phase 2.
2. Preserve 100 ms acquisition and raw g axes. Maintain a 20-sample chronological
   buffer, stride 10, rejecting/resetting continuity on sequence discontinuity,
   reboot and >150 ms acquisition gap. Document the streaming cadence after a
   discontinuity; reproduce the offline original-row stride for CSV replay.
3. Implement the exact float32 chronological mean/subtraction, +/-2g clamp, /2,
   sample-major layout and ties-to-even quantization. Compare features and INT8
   bytes against `golden_vectors.json`, then compare all saved vectors with
   the chosen target kernels before live classification.
4. Embed/load the verified model, register the needed fully connected kernel,
   allocate tensors, and measure the actual tensor arena, stack, flash use and
   inference latency. A 3,104-byte model does **not** establish runtime RAM use.
   Preserve other sensors and MQTT scheduling; inference is due every 10 new
   samples (nominally 1 s) after the initial 20-sample buffer is valid.
5. Check raw acquisition gaps, input quantization clipping, logits/class order,
   sustained sensor/MQTT operation and live independent sessions on hardware.
   Kernel rounding can differ by quantized units: record actual differences
   and classification agreement rather than assuming bit-exact MCU results.
   Firmware flash and these hardware checks remain future work.

No ML task, firmware inference, model upload/switching, Protobuf model transfer,
ESP-DL optimization, fall/impact feature or partition change is included.

Implementation references: [TensorFlow pip installation](https://www.tensorflow.org/install/pip),
[deterministic TensorFlow operations](https://www.tensorflow.org/api_docs/python/tf/config/experimental/enable_op_determinism),
and [TensorFlow post-training quantization documentation](https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/g3doc/performance/post_training_quantization.md).
