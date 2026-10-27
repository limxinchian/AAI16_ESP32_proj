"""Evaluate frozen Model A; independent mode is the default. No training/tuning."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from data import EVALUATION_FILES, TRAIN_FILES, load_dataset, require_independent, sha256
from runtime import LiteModel, comparison, metrics, tensorflow, weights_sha256, write_json

ROOT = Path(__file__).resolve().parents[2]


def verify_artifacts(directory):
    checks = directory / "SHA256SUMS"
    if not checks.is_file():
        raise ValueError("Model lacks SHA256SUMS")
    verified = {}
    for line in checks.read_text(encoding="utf-8").splitlines():
        expected, name = line.split("  ", 1)
        if Path(name).name != name or name in verified:
            raise ValueError("Invalid artifact manifest entry")
        path = directory / name
        if sha256(path) != expected:
            raise ValueError(f"Artifact hash mismatch: {name}")
        verified[name] = expected
    required = {"manifest.json", "model.keras", "model_float32.tflite", "model_int8.tflite", "baseline.json", "input_contract.json"}
    if not required.issubset(verified):
        raise ValueError("Missing required model artifacts in SHA256SUMS")
    return json.loads((directory / "manifest.json").read_text(encoding="utf-8"))


def evaluate(args):
    if args.output_dir.exists():
        raise FileExistsError("Evaluation output already exists; preserve the initial result and select a new directory")
    manifest = verify_artifacts(args.model_dir)
    for name in ("data.py", "runtime.py"):
        if sha256(Path(__file__).parent / name) != manifest["source_sha256"][name]:
            raise ValueError(f"{name} differs from the frozen preprocessing/runtime implementation")
    names = EVALUATION_FILES if args.mode == "independent" else TRAIN_FILES
    missing = [name for name in names if not (args.data_dir / name).is_file()]
    if missing:
        raise ValueError("Required recordings are not available: " + ", ".join(missing))
    dataset = load_dataset([args.data_dir / name for name in names])
    if args.mode == "independent":
        require_independent(dataset, manifest["dataset"]["sessions"])
    else:
        expected = {report["file"]: report["sha256"] for report in manifest["dataset"]["sessions"]}
        if any(report["sha256"] != expected.get(report["file"]) for report in dataset.reports):
            raise ValueError("Sanity data differs from the frozen training dataset")
    tf = tensorflow(manifest["training"]["seed"])
    model = tf.keras.models.load_model(args.model_dir / "model.keras", compile=False)
    if weights_sha256(model) != manifest["weights_sha256"]:
        raise ValueError("Loaded Keras weights do not match the training manifest")
    logits = model(dataset.x, training=False).numpy()
    float_model = LiteModel(tf, args.model_dir / "model_float32.tflite")
    int_model = LiteModel(tf, args.model_dir / "model_int8.tflite")
    float_logits, _, _, _ = float_model.predict(dataset.x)
    int_logits, _, _, clipped = int_model.predict(dataset.x)
    baseline = json.loads((args.model_dir / "baseline.json").read_text(encoding="utf-8"))
    baseline_predictions = (dataset.rms >= baseline["threshold_g"]).astype(np.int32)
    result = {"mode": args.mode, "created_utc": datetime.now(timezone.utc).isoformat(),
              "model_version": manifest["model_version"], "model_manifest_sha256": sha256(args.model_dir / "manifest.json"),
              "dataset": dataset.report(), "baseline_threshold_g_frozen": baseline["threshold_g"],
              "keras": metrics(dataset.y, logits.argmax(axis=1)),
              "tflite_float32": metrics(dataset.y, float_logits.argmax(axis=1)),
              "tflite_int8": metrics(dataset.y, int_logits.argmax(axis=1)),
              "rms_baseline": metrics(dataset.y, baseline_predictions),
              "keras_vs_tflite_float32": comparison(logits, float_logits),
              "keras_vs_tflite_int8": comparison(logits, int_logits),
              "int8_input_elements_clipped": clipped,
              "limitations": ("Independent recordings, but only two sessions; overlapping windows are correlated and the demo scope is narrow"
                              if args.mode == "independent" else
                              "Training-set resubstitution only; one REST session; not evidence of generalization or requirement compliance")}
    # All input/admission/model checks precede output creation. Evaluation never
    # changes the model, threshold, representative set, or original recordings.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write_json(args.output_dir / "evaluation.json", result)
    np.savez_compressed(args.output_dir / "predictions.npz", labels=dataset.y, keras_logits=logits,
                        float_logits=float_logits, int8_logits=int_logits, baseline_predictions=baseline_predictions)
    print(json.dumps(result, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=ROOT / "models/motion_model_a")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "tools/ml_data_collection/recordings")
    parser.add_argument("--mode", choices=("independent", "training-sanity"), default="independent")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        evaluate(args)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Evaluation stopped: {exc}\n")


if __name__ == "__main__":
    main()
