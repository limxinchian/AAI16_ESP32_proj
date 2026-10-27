"""Train preliminary Model A on the three fixed Phase 1 sessions only."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile

import numpy as np

from data import LABELS, TRAIN_FILES, load_dataset, sha256
from runtime import (build_model, comparison, export_models, fit_rms_baseline, metrics,
                     tensorflow, weights_sha256, write_json)

ROOT = Path(__file__).resolve().parents[2]
VERSION = "aai16-motion-model-a-0.1.0"


def input_contract():
    return {"version": 1, "sensor": "GY-511 accelerometer", "raw_units": "g",
            "sampling_hz": 10, "window_samples": 20, "axes": ["x", "y", "z"], "stride_samples": 10,
            "float_input_shape": [1, 60], "float_dtype": "float32", "class_mapping": LABELS,
            "preprocessing": ["accumulate each axis chronologically in float32 over 20 samples",
                              "divide each sum by float32(20); subtract per-axis mean in float32",
                              "clamp centered acceleration to [-2, 2] g",
                              "divide by float32(2)",
                              "flatten sample-major: x0,y0,z0,x1,y1,z1,...,x19,y19,z19"],
            "window_admission": "20 valid chronological rows; no >150 ms gap, reboot or sequence discontinuity",
            "offline_stride": "starts at original session rows 0,10,20,...; invalid crossing windows discarded",
            "int8_encoding": "q = clamp(round_ties_to_even(float32(x / input_scale)) + input_zero_point, -128, 127)",
            "int8_decoding": "logit = float32(q - output_zero_point) * output_scale",
            "output_shape": [1, 2], "output": "REST and MOVING logits, not probabilities",
            "decision": "argmax(logits); exact ties select REST=0; monotonic scalar quantization permits INT8 argmax",
            "scope": "flat stationary desk REST vs continuous hand movement/shaking MOVING"}


def train(args):
    if args.epochs < 1 or args.batch_size < 1 or not math.isfinite(args.learning_rate) or args.learning_rate <= 0:
        raise ValueError("epochs, batch size and learning rate must be positive")
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}; use a new --output-dir")
    # Deliberately no glob or --include option: reserved future evaluation files
    # cannot silently enter training, threshold fitting, or INT8 calibration.
    dataset = load_dataset([args.data_dir / name for name in TRAIN_FILES])
    if set(dataset.y.tolist()) != {0, 1}:
        raise ValueError("Training needs both REST and MOVING")
    print(json.dumps(dataset.report(), indent=2), flush=True)
    tf = tensorflow(args.seed)
    tf.keras.backend.clear_session()
    tf.keras.utils.set_random_seed(args.seed)
    model = build_model(tf, args.learning_rate)
    counts = np.bincount(dataset.y, minlength=2)
    weights = len(dataset.y) / (2.0 * counts)
    sample_weights = weights[dataset.y].astype(np.float32)
    settings = {"seed": args.seed, "epochs": args.epochs, "batch_size": args.batch_size,
                "optimizer": "Adam", "learning_rate": args.learning_rate,
                "optimizer_config": model.optimizer.get_config(),
                "loss": "SparseCategoricalCrossentropy(from_logits=True)",
                "class_weights": {name: float(weights[value]) for name, value in LABELS.items()},
                "shuffle": "NumPy default_rng(seed).permutation per epoch, training windows only",
                "validation_split": None, "early_stopping": False, "checkpoint_selection": "fixed final epoch",
                "deterministic_ops": True, "cpu_only": True, "oneDNN": False,
                "inter_op_threads": 1, "intra_op_threads": 1,
                "architecture": "60 -> Dense(16, ReLU) -> Dense(2, linear logits)", "parameters": 1010}
    generator = np.random.default_rng(args.seed)
    history = []
    for epoch in range(1, args.epochs + 1):
        model.reset_metrics()
        indices = generator.permutation(len(dataset.y))
        result = None
        for offset in range(0, len(indices), args.batch_size):
            chosen = indices[offset:offset + args.batch_size]
            result = model.train_on_batch(dataset.x[chosen], dataset.y[chosen],
                                          sample_weight=sample_weights[chosen], return_dict=True)
        record = {"epoch": epoch, "weighted_loss": float(result["loss"]),
                  "unweighted_accuracy_during_epoch": float(result["accuracy"])}
        if not math.isfinite(record["weighted_loss"]):
            raise RuntimeError("Non-finite training loss")
        history.append(record)
        if epoch == 1 or epoch % 25 == 0 or epoch == args.epochs:
            print(record, flush=True)
    logits = model(dataset.x, training=False).numpy()
    baseline = fit_rms_baseline(dataset.rms, dataset.y)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".motion-a-", dir=output.parent) as temporary:
        artifacts = Path(temporary)
        float_model, int_model = export_models(tf, model, dataset.x, artifacts)
        float_logits, _, _, _ = float_model.predict(dataset.x)
        int_logits, quantized_input, quantized_output, clipped = int_model.predict(dataset.x)
        if not np.allclose(logits, float_logits, rtol=1e-5, atol=1e-5):
            raise RuntimeError("Float TFLite export disagrees numerically with Keras")
        write_json(artifacts / "input_contract.json", input_contract())
        write_json(artifacts / "dataset_report.json", dataset.report())
        write_json(artifacts / "baseline.json", baseline)
        write_json(artifacts / "quantization.json", {
            "method": "post-training full integer quantization, TFLITE_BUILTINS_INT8",
            "representative_vectors": len(dataset.x), "representative_source": "all training windows only",
            "float32": float_model.metadata, "int8": int_model.metadata,
            "training_feature_min": float(dataset.x.min()), "training_feature_max": float(dataset.x.max()),
            "training_input_elements_clipped": clipped,
            "warning": "Calibration uses observed training range; stronger future movement may clip INT8 inputs"})
        summary = {"evaluation_type": "training-set resubstitution sanity check only",
                   "independent_evaluation": "pending rest_orientation_02 and moving_directions_03",
                   "limitation": "one REST session; overlapping windows are not independent; no generalization or accuracy requirement claim",
                   "keras": metrics(dataset.y, logits.argmax(axis=1)),
                   "tflite_float32": metrics(dataset.y, float_logits.argmax(axis=1)),
                   "tflite_int8": metrics(dataset.y, int_logits.argmax(axis=1)),
                   "rms_baseline": baseline["training_sanity"],
                   "keras_vs_tflite_float32": comparison(logits, float_logits),
                   "keras_vs_tflite_int8": comparison(logits, int_logits)}
        write_json(artifacts / "training_report.json", summary)
        (artifacts / "MODEL_CARD.md").write_text(
            f"# Preliminary motion Model A\n\nVersion: `{VERSION}`.\n\n"
            "Scope: flat stationary desk REST (0) versus continuous hand movement/shaking MOVING (1).\n\n"
            f"Trained on {len(dataset.y)} real windows: {int(counts[0])} REST and {int(counts[1])} MOVING, "
            "from one REST and two MOVING sessions. No independent test data is available yet. "
            "All following scores are training-set resubstitution; overlapping windows are correlated. "
            "These scores do not establish generalization or project accuracy compliance.\n\n"
            f"Keras accuracy: {summary['keras']['accuracy']:.6f}; "
            f"INT8 accuracy: {summary['tflite_int8']['accuracy']:.6f}; "
            f"RMS baseline accuracy: {summary['rms_baseline']['accuracy']:.6f}.\n\n"
            "Architecture: 60 -> Dense(16, ReLU) -> Dense(2, logits), 1,010 parameters. "
            f"Adam, learning rate {args.learning_rate}, {args.epochs} epochs, batch {args.batch_size}, "
            f"seed {args.seed}, inverse-frequency class weights. Final epoch; no validation/tuning split.\n\n"
            "See input_contract.json for the chronological float32 per-axis mean, clamp +/-2g, /2, "
            "sample-major contract. See quantization.json for actual tensor scales/zero points and operators. "
            "Calibration uses training windows only; new movement can exceed its observed range.\n\n"
            "Freeze this bundle before collecting rest_orientation_02.csv and moving_directions_03.csv. "
            "Run tools/ml_training/evaluate.py in its default independent mode, without retraining, "
            "threshold tuning, or recalibration. Keep the first evaluation result.\n\n"
            "This bundle has only been exercised on the laptop. Tensor arena size, target operator "
            "compatibility, latency, and firmware inference remain unverified. "
            "See tools/ml_training/README.md for reproduction, results, and deployment requirements.\n",
            encoding="utf-8")
        with (artifacts / "training_history.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(history[0]))
            writer.writeheader()
            writer.writerows(history)
        with (artifacts / "window_predictions.csv").open("w", newline="", encoding="utf-8") as stream:
            fields = list(dataset.windows[0]) + ["rms_g", "keras_prediction", "float_prediction", "int8_prediction", "baseline_prediction"]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for i, origin in enumerate(dataset.windows):
                writer.writerow(dict(origin, rms_g=float(dataset.rms[i]), keras_prediction=int(logits[i].argmax()),
                                     float_prediction=int(float_logits[i].argmax()), int8_prediction=int(int_logits[i].argmax()),
                                     baseline_prediction=int(dataset.rms[i] >= baseline["threshold_g"])))
        np.savez_compressed(artifacts / "test_vectors.npz", raw_g=dataset.raw, features=dataset.x,
                            labels=dataset.y, keras_logits=logits, float_tflite_logits=float_logits,
                            quantized_input=quantized_input, quantized_output=quantized_output,
                            int8_dequantized_logits=int_logits)
        selected = set()
        for label in (0, 1):
            choices = np.flatnonzero(dataset.y == label)
            selected.update((int(choices[0]), int(choices[np.argmax(dataset.rms[choices])])))
        write_json(artifacts / "golden_vectors.json", [{"vector_index": index, "origin": dataset.windows[index],
                    "raw_g": dataset.raw[index].tolist(), "features": dataset.x[index].tolist(),
                    "int8_input": quantized_input[index].tolist(), "int8_output": quantized_output[index].tolist(),
                    "keras_logits": logits[index].tolist(), "int8_dequantized_logits": int_logits[index].tolist()}
                    for index in sorted(selected)])
        freeze = subprocess.check_output([sys.executable, "-m", "pip", "freeze"], text=True)
        (artifacts / "environment.lock.txt").write_text(freeze, encoding="utf-8")
        source_files = ("data.py", "runtime.py", "train.py", "evaluate.py", "requirements.txt", "setup_env.py")
        manifest = {"model_version": VERSION, "created_utc": datetime.now(timezone.utc).isoformat(),
                    "git_branch": subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip(),
                    "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                    "git_working_tree": "uncommitted implementation; source hashes identify training code",
                    "python": platform.python_version(), "platform": platform.platform(),
                    "tensorflow": tf.__version__, "keras": tf.keras.__version__, "numpy": np.__version__,
                    "training": settings, "dataset": dataset.report(), "weights_sha256": weights_sha256(model),
                    "source_sha256": {name: sha256(Path(__file__).parent / name) for name in source_files},
                    "artifacts": {path.name: {"sha256": sha256(path), "bytes": path.stat().st_size}
                                  for path in sorted(artifacts.iterdir()) if path.is_file()}}
        write_json(artifacts / "manifest.json", manifest)
        (artifacts / "SHA256SUMS").write_text("".join(f"{sha256(path)}  {path.name}\n" for path in sorted(artifacts.iterdir()) if path.is_file()), encoding="utf-8")
        shutil.copytree(artifacts, output)  # Fails rather than replacing an existing model.
    print(json.dumps({"output": str(output), "weights_sha256": weights_sha256(model), "results": summary}, indent=2), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "tools/ml_data_collection/recordings")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "models/motion_model_a")
    parser.add_argument("--seed", type=int, default=20261028)
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    args = parser.parse_args(argv)
    try:
        train(args)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Training stopped: {exc}\n")


if __name__ == "__main__":
    main()
