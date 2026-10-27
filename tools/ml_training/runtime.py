"""Deterministic Keras setup, metrics, and checked TFLite execution/export."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

import numpy as np

from data import LABELS

_tf = None


def tensorflow(seed=20261028):
    global _tf
    if sys.prefix == sys.base_prefix:
        raise RuntimeError("Use the isolated ML virtual environment; do not install into ESP-IDF")
    if _tf is None:
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0"
        os.environ["TF_DETERMINISTIC_OPS"] = "1"
        os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
        import tensorflow as tf
        tf.config.threading.set_inter_op_parallelism_threads(1)
        tf.config.threading.set_intra_op_parallelism_threads(1)
        tf.config.experimental.enable_op_determinism()
        _tf = tf
    _tf.keras.utils.set_random_seed(seed)
    return _tf


def build_model(tf, learning_rate=0.001):
    model = tf.keras.Sequential([
        tf.keras.Input(shape=(60,), dtype=tf.float32, name="features"),
        tf.keras.layers.Dense(16, activation="relu", name="hidden"),
        tf.keras.layers.Dense(2, activation=None, name="logits"),
    ], name="motion_model_a")
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate),
                  loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                  metrics=[tf.keras.metrics.SparseCategoricalAccuracy(name="accuracy")],
                  jit_compile=False)
    assert model.count_params() == 1010
    return model


def weights_sha256(model):
    digest = hashlib.sha256()
    for array in model.get_weights():
        values = np.asarray(array, dtype="<f4")
        digest.update(str(values.shape).encode("ascii"))
        digest.update(values.tobytes(order="C"))
    return digest.hexdigest()


def metrics(y, predictions):
    matrix = np.zeros((2, 2), dtype=np.int64)
    for truth, predicted in zip(y, predictions, strict=True):
        if int(truth) not in (0, 1) or int(predicted) not in (0, 1):
            raise ValueError("class must be REST=0 or MOVING=1")
        matrix[int(truth), int(predicted)] += 1
    classes = {}
    for label, index in LABELS.items():
        support = int(matrix[index].sum())
        predicted = int(matrix[:, index].sum())
        tp = int(matrix[index, index])
        recall = tp / support if support else None
        precision = tp / predicted if predicted else (0.0 if support else None)
        f1 = 2 * precision * recall / (precision + recall) if support and precision and recall else (0.0 if support else None)
        classes[label] = {"support": support, "precision": precision, "recall": recall, "f1": f1}
    recalls = [value["recall"] for value in classes.values() if value["recall"] is not None]
    return {"samples": len(y), "confusion_matrix_rows_true_columns_predicted": matrix.tolist(),
            "class_order": ["REST", "MOVING"], "accuracy": float(np.trace(matrix) / len(y)) if len(y) else None,
            "balanced_accuracy": float(np.mean(recalls)) if recalls else None,
            "per_class": classes}


def fit_rms_baseline(rms, y):
    values = np.unique(rms)
    candidates = np.r_[np.nextafter(values[0], -np.inf),
                       (values[:-1] + values[1:]) / 2,
                       np.nextafter(values[-1], np.inf)]
    scores = [metrics(y, (rms >= threshold).astype(np.int32))["balanced_accuracy"] for threshold in candidates]
    winners = np.flatnonzero(np.isclose(scores, max(scores), rtol=0, atol=1e-12))
    selected = int(winners[len(winners) // 2])
    threshold = float(candidates[selected])
    return {"statistic": "sqrt(mean_t(sum_axis((a_g-mean_t(a_g))^2)))",
            "units": "g", "moving_if": "rms_g >= threshold_g", "threshold_g": threshold,
            "fit_data": "training sessions only",
            "selection": "maximum training balanced accuracy; median-index candidate among ties",
            "training_sanity": metrics(y, (rms >= threshold).astype(np.int32))}


def tensor_metadata(detail):
    quant = detail["quantization_parameters"]
    return {"name": detail["name"], "index": int(detail["index"]),
            "shape": detail["shape"].tolist(), "shape_signature": detail["shape_signature"].tolist(),
            "dtype": np.dtype(detail["dtype"]).name,
            "scales": quant["scales"].tolist(), "zero_points": quant["zero_points"].tolist(),
            "quantized_dimension": int(quant["quantized_dimension"])}


def operator_metadata(model_bytes):
    from tensorflow.lite.python import schema_py_generated as schema
    model = schema.Model.GetRootAsModel(model_bytes, 0)
    names = {value: key for key, value in vars(schema.BuiltinOperator).items() if isinstance(value, int)}
    activations = {value: key for key, value in vars(schema.ActivationFunctionType).items() if isinstance(value, int)}
    graph = model.Subgraphs(0)
    operations = []
    for index in range(graph.OperatorsLength()):
        op = graph.Operators(index)
        code = model.OperatorCodes(op.OpcodeIndex())
        name = names.get(code.BuiltinCode(), "UNKNOWN")
        record = {"operator": name, "version": int(code.Version()),
                  "inputs": op.InputsAsNumpy().tolist(), "outputs": op.OutputsAsNumpy().tolist()}
        if name == "FULLY_CONNECTED":
            table = op.BuiltinOptions()
            options = schema.FullyConnectedOptions()
            options.Init(table.Bytes, table.Pos)
            record["fused_activation"] = activations[options.FusedActivationFunction()]
        operations.append(record)
    if [op["operator"] for op in operations] != ["FULLY_CONNECTED", "FULLY_CONNECTED"]:
        raise ValueError(f"Unexpected operators: {operations}")
    if [op["fused_activation"] for op in operations] != ["RELU", "NONE"]:
        raise ValueError("Export does not preserve Dense-ReLU / Dense-logits architecture")
    return operations


class LiteModel:
    def __init__(self, tf, path):
        self.interpreter = tf.lite.Interpreter(
            model_path=str(path), num_threads=1,
            experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_WITHOUT_DEFAULT_DELEGATES)
        self.interpreter.allocate_tensors()
        inputs, outputs = self.interpreter.get_input_details(), self.interpreter.get_output_details()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("Expected one input and one output")
        self.input, self.output = inputs[0], outputs[0]
        if self.input["shape"].tolist() != [1, 60] or self.output["shape"].tolist() != [1, 2]:
            raise ValueError("Incorrect TFLite tensor shapes")
        if self.input["shape_signature"].tolist() != [1, 60] or self.output["shape_signature"].tolist() != [1, 2]:
            raise ValueError("TFLite shapes must be fixed for MCU export")
        self.metadata = {"input": tensor_metadata(self.input), "output": tensor_metadata(self.output),
                         "operators": operator_metadata(Path(path).read_bytes()),
                         "tensors": [tensor_metadata(t) for t in self.interpreter.get_tensor_details()]}
        if self.input["dtype"] == np.int8:
            if self.output["dtype"] != np.int8 or any(t["dtype"] not in ("int8", "int32") for t in self.metadata["tensors"]):
                raise ValueError("INT8 export contains floating-point tensors")
            for detail in (self.input, self.output):
                scale, _ = detail["quantization"]
                if not scale > 0:
                    raise ValueError("Missing INT8 quantization scale")
        elif self.input["dtype"] != np.float32 or self.output["dtype"] != np.float32:
            raise ValueError("Expected float32 or fully INT8 I/O")

    def encode(self, features):
        values = np.asarray(features, dtype=np.float32)
        if values.shape != (60,) or not np.isfinite(values).all():
            raise ValueError("Expected 60 finite float32 features")
        if self.input["dtype"] == np.float32:
            return values.reshape(1, 60), 0
        scale, zero_point = self.input["quantization"]
        # Explicit ties-to-even rounding of float32 x/scale, then add the integer
        # zero point and clamp. This is a separate step from g-domain clipping.
        before_clamp = np.rint(values / np.float32(scale)).astype(np.int64) + zero_point
        clipped = int(np.count_nonzero((before_clamp < -128) | (before_clamp > 127)))
        return np.clip(before_clamp, -128, 127).astype(np.int8).reshape(1, 60), clipped

    def predict(self, x):
        logits, encoded, outputs = [], [], []
        clipping = 0
        for features in x:
            value, clipped = self.encode(features)
            clipping += clipped
            self.interpreter.set_tensor(self.input["index"], value)
            self.interpreter.invoke()
            output = self.interpreter.get_tensor(self.output["index"])[0]
            outputs.append(output)
            encoded.append(value[0])
            if self.output["dtype"] == np.int8:
                scale, zero_point = self.output["quantization"]
                output = (output.astype(np.float32) - zero_point) * np.float32(scale)
            logits.append(output)
        return np.asarray(logits), np.asarray(encoded), np.asarray(outputs), clipping


def export_models(tf, model, representative, output: Path):
    model.save(output / "model.keras")
    with tempfile.TemporaryDirectory(prefix="motion-a-export-") as temporary:
        model.export(temporary, format="tf_saved_model",
                     input_signature=[tf.TensorSpec([1, 60], tf.float32, name="features")], verbose=False)
        converter = tf.lite.TFLiteConverter.from_saved_model(temporary)
        (output / "model_float32.tflite").write_bytes(converter.convert())
        converter = tf.lite.TFLiteConverter.from_saved_model(temporary)
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.representative_dataset = lambda: ([row.reshape(1, 60)] for row in representative)
        converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter.inference_input_type = tf.int8
        converter.inference_output_type = tf.int8
        (output / "model_int8.tflite").write_bytes(converter.convert())
    return LiteModel(tf, output / "model_float32.tflite"), LiteModel(tf, output / "model_int8.tflite")


def comparison(reference, candidate):
    if reference.shape != candidate.shape or not np.isfinite(candidate).all():
        raise ValueError("Invalid exported logits")
    difference = np.abs(reference.astype(np.float64) - candidate.astype(np.float64))
    matches = np.argmax(reference, axis=1) == np.argmax(candidate, axis=1)
    return {"vectors": len(reference), "prediction_agreement_count": int(matches.sum()),
            "prediction_agreement_fraction": float(matches.mean()),
            "logit_mean_absolute_error": float(difference.mean()),
            "logit_max_absolute_error": float(difference.max()),
            "disagreement_vector_indices": np.flatnonzero(~matches).tolist()}


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
