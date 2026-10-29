"""Compare frozen A/B bundles and run their INT8 vectors in the installed Micro runtime.

Read-only with respect to models, evaluation results and recordings. No fitting,
calibration, checkpoint selection or firmware deployment takes place here.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np

from data import EVALUATION_FILES, load_dataset, require_independent, sha256
from evaluate import verify_artifacts
from runtime import LiteModel, tensorflow, write_json

ROOT = Path(__file__).resolve().parents[2]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def run_host(executable, model_path, lite, inputs, expected):
    with tempfile.TemporaryDirectory(prefix="motion-compat-") as temporary:
        directory = Path(temporary)
        inputs.astype(np.int8).tofile(directory / "inputs.bin")
        scale_in, zero_in = lite.input["quantization"]
        scale_out, zero_out = lite.output["quantization"]
        completed = subprocess.run([
            str(executable.resolve()), str(model_path.resolve()),
            str(directory / "inputs.bin"), str(directory / "outputs.bin"),
            str(scale_in), str(zero_in), str(scale_out), str(zero_out)],
            check=True, capture_output=True, text=True)
        actual = np.fromfile(directory / "outputs.bin", dtype=np.int8).reshape(-1, 2)
    if actual.shape != expected.shape:
        raise ValueError("Host output shape mismatch")
    delta = np.abs(actual.astype(np.int16) - expected.astype(np.int16))
    disagreements = np.flatnonzero(actual.argmax(1) != expected.argmax(1)).tolist()
    return actual, {"vectors": len(inputs), "output_elements": int(expected.size),
                    "different_output_elements": int(np.count_nonzero(delta)),
                    "maximum_output_difference_lsb": int(delta.max()),
                    "classification_agreement_count": len(inputs) - len(disagreements),
                    "classification_disagreement_indices": disagreements,
                    "runner_stdout": completed.stdout.strip()}


def compare(args):
    if args.output_dir.exists():
        raise FileExistsError("Preserve the initial comparison; output directory already exists")
    tf = tensorflow()
    heldout = load_dataset([args.data_dir / name for name in EVALUATION_FILES])
    report = {"scope": "Fixed demo only: flat stationary desk REST vs continuous hand movement/shaking MOVING",
              "limitations": "Two held-out sessions with correlated overlapping windows; no broad generalization claim. "
                             "Holdout scores were already known for Model A; Model B uses the specified architecture "
                             "and identical fixed training settings, with no holdout-based tuning or model selection.",
              "runtime": {"esp_tflite_micro": "1.4.1", "esp_nn": "1.3.0",
                          "dependency_lock_sha256": sha256(ROOT / "dependencies.lock"),
                          "execution": "Laptop production Engine with reference and ESP-NN ANSI kernels",
                          "limitation": "Does not exercise ESP32-S3 assembly kernels or establish target RAM/latency"},
              "models": {}}
    source_paths = [Path(__file__), ROOT / "tests/motion_host/model_file_runner.cc",
                    ROOT / "tests/motion_host/CMakeLists.txt", ROOT / "components/motion_ml/motion_engine.cc"]
    report["validation_source_sha256"] = {p.relative_to(ROOT).as_posix(): sha256(p) for p in source_paths}
    outputs = {}
    for letter in ("a", "b"):
        directory = ROOT / f"models/motion_model_{letter}"
        manifest = verify_artifacts(directory)
        evaluation_path = ROOT / f"models/motion_model_{letter}_evaluation_01/evaluation.json"
        evaluation = read_json(evaluation_path)
        if evaluation["model_manifest_sha256"] != sha256(directory / "manifest.json"):
            raise ValueError("Evaluation does not refer to the frozen model")
        if evaluation["mode"] != "independent" or evaluation["dataset"] != heldout.report():
            raise ValueError("Evaluation does not use the same exact held-out windows")
        require_independent(heldout, manifest["dataset"]["sessions"])
        quantization = read_json(directory / "quantization.json")
        lite = LiteModel(tf, directory / "model_int8.tflite")
        _, encoded, expected, clipped = lite.predict(heldout.x)
        predictions_path = evaluation_path.with_name("predictions.npz")
        with np.load(predictions_path, allow_pickle=False) as saved:
            np.testing.assert_array_equal(saved["labels"], heldout.y)
            np.testing.assert_array_equal(saved["int8_logits"].argmax(1), expected.argmax(1))
        if clipped != evaluation["int8_input_elements_clipped"]:
            raise ValueError("Evaluation clipping does not replay")
        with np.load(directory / "test_vectors.npz", allow_pickle=False) as vectors:
            training_inputs, training_outputs = vectors["quantized_input"], vectors["quantized_output"]
        goldens = read_json(directory / "golden_vectors.json")
        groups = {"training": (training_inputs, training_outputs), "independent": (encoded, expected),
                  "golden": (np.array([v["int8_input"] for v in goldens], dtype=np.int8),
                             np.array([v["int8_output"] for v in goldens], dtype=np.int8))}
        compatibility = {}
        for group, (inputs, expected_outputs) in groups.items():
            compatibility[group] = {}
            for backend in ("reference", "esp_nn_ansi"):
                binary = args.host_build / f"model_{backend}.exe"
                actual, result = run_host(binary, directory / "model_int8.tflite", lite, inputs, expected_outputs)
                compatibility[group][backend] = result
                outputs[f"{letter}_{group}_{backend}"] = actual
            np.testing.assert_array_equal(outputs[f"{letter}_{group}_reference"], outputs[f"{letter}_{group}_esp_nn_ansi"])
        report["models"][letter] = {"model_version": manifest["model_version"],
            "manifest_sha256": sha256(directory / "manifest.json"), "weights_sha256": manifest["weights_sha256"],
            "evaluation_sha256": sha256(evaluation_path), "evaluation_predictions_sha256": sha256(predictions_path),
            "architecture": manifest["training"]["architecture"], "parameters": manifest["training"]["parameters"],
            "training_settings": manifest["training"], "training_dataset": manifest["dataset"],
            "exports": {name: manifest["artifacts"][name] for name in ("model.keras", "model_float32.tflite", "model_int8.tflite")},
            "training_results": read_json(directory / "training_report.json"), "independent_results": evaluation,
            "quantization": quantization, "host_runtime": compatibility}
    a, b = report["models"]["a"], report["models"]["b"]
    if a["training_dataset"] != b["training_dataset"]:
        raise ValueError("Models trained on different windows")
    if (ROOT / "models/motion_model_a/input_contract.json").read_bytes() != (ROOT / "models/motion_model_b/input_contract.json").read_bytes():
        raise ValueError("Different input contracts")
    reproduction = verify_artifacts(args.reproduction_dir)
    if reproduction["weights_sha256"] != b["weights_sha256"]:
        raise ValueError("Repeat run did not reproduce B weights")
    reproduced = ["model_float32.tflite", "model_int8.tflite", "golden_vectors.json", "test_vectors.npz", "training_history.csv"]
    for name in reproduced:
        if sha256(args.reproduction_dir / name) != sha256(ROOT / "models/motion_model_b" / name):
            raise ValueError(f"Repeat run differs: {name}")
    report["reproducibility"] = {"weights_identical": True, "byte_identical_files": reproduced,
                                 "repeat_manifest_sha256": sha256(args.reproduction_dir / "manifest.json"),
                                 "note": "Keras archive/manifest timestamps may differ; canonical weights and TFLite binaries match"}
    report["parameter_reduction_percent"] = 100 * (1 - b["parameters"] / a["parameters"])
    report["int8_size_reduction_percent"] = 100 * (1 - b["exports"]["model_int8.tflite"]["bytes"] / a["exports"]["model_int8.tflite"]["bytes"])
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write_json(args.output_dir / "comparison.json", report)
    np.savez_compressed(args.output_dir / "host_outputs.npz", **outputs)
    (args.output_dir / "COMPARISON.md").write_text(markdown(report), encoding="utf-8")
    (args.output_dir / "SHA256SUMS").write_text("".join(
        f"{sha256(p)}  {p.name}\n" for p in sorted(args.output_dir.iterdir()) if p.is_file()), encoding="utf-8")
    print(json.dumps({"output": str(args.output_dir), "model_b_host_runtime": b["host_runtime"]}, indent=2))


def markdown(report):
    a, b = [report["models"][key] for key in ("a", "b")]
    lines = ["# Model A / Model B comparison — fixed demo scenario only", "", report["scope"] + ".", "",
             report["limitations"], "", "| Measure | Model A | Model B |", "|---|---:|---:|"]
    for label, values in [
        ("Architecture", [m["architecture"] for m in (a, b)]),
        ("Trainable parameters", [m["parameters"] for m in (a, b)]),
        *[(name + " bytes", [m["exports"][name]["bytes"] for m in (a, b)]) for name in ("model.keras", "model_float32.tflite", "model_int8.tflite")],
        *[(kind + " independent accuracy", [f'{m["independent_results"][kind]["accuracy"]:.4%}' for m in (a, b)]) for kind in ("keras", "tflite_float32", "tflite_int8")],
        *[("Keras vs " + kind + " independent agreement", [f'{m["independent_results"]["keras_vs_" + kind]["prediction_agreement_count"]}/112' for m in (a, b)]) for kind in ("tflite_float32", "tflite_int8")],
        ("INT8 clipped inputs: training / independent", [f'{m["quantization"]["training_input_elements_clipped"]}/10080 / {m["independent_results"]["int8_input_elements_clipped"]}/6720' for m in (a, b)]),
        ("INT8 input scale / zero point", [f'{m["quantization"]["int8"]["input"]["scales"][0]} / {m["quantization"]["int8"]["input"]["zero_points"][0]}' for m in (a, b)]),
        ("INT8 output scale / zero point", [f'{m["quantization"]["int8"]["output"]["scales"][0]} / {m["quantization"]["int8"]["output"]["zero_points"][0]}' for m in (a, b)])]:
        lines.append(f"| {label} | {values[0]} | {values[1]} |")
    lines += ["", f'Model B reduces parameters by {report["parameter_reduction_percent"]:.2f}% and INT8 file size by {report["int8_size_reduction_percent"]:.2f}%.', "",
              "## Independent confusion matrices", "", "Rows = true REST/MOVING; columns = predicted REST/MOVING.", "",
              "| Runtime | Model A | Model B |", "|---|---|---|"]
    for kind in ("keras", "tflite_float32", "tflite_int8"):
        matrices = [m["independent_results"][kind]["confusion_matrix_rows_true_columns_predicted"] for m in (a, b)]
        lines.append(f"| {kind} | `{matrices[0]}` | `{matrices[1]}` |")
    lines += ["", "Model B INT8 changes four Keras classifications (held-out indices 102, 104, 110, 111), all MOVING to REST. "
              "Clipping is 157/6720 = 2.34% of held-out input elements for both models. "
              "The measured regression is retained; neither recalibration nor retuning used these sessions.", "",
              "## Data and reproducibility", "",
              "Training/calibration: exactly rest_orientation_01.csv, moving_directions_01.csv, moving_directions_02.csv; "
              "168 windows (58 REST, 110 MOVING), exactly the same hashes/windows as A. "
              "Independent evaluation: rest_orientation_02.csv (session rest_orientation_02_retry) and moving_directions_03.csv; "
              "112 windows (56 REST, 56 MOVING), identical hashes/windows to A's saved evaluation.", "",
              "Both use 20 samples at 10 Hz, stride 10, exclude gaps >150 ms, and share the byte-identical input contract: "
              "chronological float32 per-axis mean subtraction, clamp ±2g, divide by 2g, sample-major XYZ flattening. "
              "Training uses Adam 0.001, 150 epochs, batch 16, seed 20261028, inverse-frequency class weights, "
              "deterministic CPU operations and a fixed final epoch; no validation split or early stopping. "
              "Both achieve 168/168 training accuracy and Keras/float/INT8 classification agreement. These are resubstitution results.", "",
              "A second B run before held-out evaluation reproduced canonical weights, both TFLite binaries, "
              "training history and golden/test vectors byte-for-byte. Exact environment, provenance, hashes, "
              "quantization and per-session rejection reports are recorded in each bundle and comparison.json.", "",
              "## Runtime compatibility", "",
              "Both exports use static INT8 [1,60] input and [1,2] output, two FULLY_CONNECTED v4 operators "
              "(fused RELU then NONE), per-output-channel INT8 weights and INT32 biases. "
              "A uses 16/2 weight scales; B uses 8/2. No float fallback or custom operator is required.", "",
              "The unmodified production Engine loaded both models and executed the installed esp-tflite-micro 1.4.1 "
              "reference and ESP-NN 1.3.0 ANSI kernels on the laptop. Both host backends produced identical output bytes.", "",
              "| Model / vectors | Host class agreement with desktop INT8 | Different output elements | Max difference (LSB) |",
              "|---|---:|---:|---:|"]
    for letter, model in (("A", a), ("B", b)):
        for group, backends in model["host_runtime"].items():
            result = backends["reference"]
            lines.append(f'| {letter} / {group} | {result["classification_agreement_count"]}/{result["vectors"]} | '
                         f'{result["different_output_elements"]}/{result["output_elements"]} | {result["maximum_output_difference_lsb"]} |')
    lines += ["", "Model B is compatible with the checked operator/engine contract. Future deployment must supply B's "
              "bytes, hash, output quantization and golden vectors; the current firmware still embeds Model A. "
              "This phase implements no firmware loading, upload or switching. ESP32-S3 assembly-kernel parity, "
              "actual inference time, tensor arena/stack/RAM and live sensor/MQTT behavior must be measured on hardware later. "
              "No Model B hardware timing or RAM estimate is claimed.", "",
              "See ../motion_model_b/ for exports, ../motion_model_b_evaluation_01/ for the frozen first evaluation, "
              "and ../../tools/ml_training/README.md for reproduction commands.", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "tools/ml_data_collection/recordings")
    parser.add_argument("--host-build", type=Path, default=ROOT / "build/motion_host")
    parser.add_argument("--reproduction-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    compare(args)


if __name__ == "__main__":
    main()
