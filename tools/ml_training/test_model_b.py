"""Model B provenance, frozen exports, independent replay and compatibility evidence."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np

from data import EVALUATION_FILES, TRAIN_FILES, load_dataset, preprocess, sha256
from evaluate import verify_artifacts
from runtime import LiteModel, metrics, tensorflow, weights_sha256
from train import build_model_b, load_training_dataset, train

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models/motion_model_b"
RECORDINGS = ROOT / "tools/ml_data_collection/recordings"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def training_args(**overrides):
    values = dict(model="b", data_dir=RECORDINGS, output_dir=MODEL,
                  seed=20261028, epochs=150, batch_size=16, learning_rate=.001)
    return argparse.Namespace(**(values | overrides))


class ModelBProvenanceTests(unittest.TestCase):
    def test_extra_csv_and_holdouts_never_enter_training(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for name in TRAIN_FILES:
                shutil.copyfile(RECORDINGS / name, directory / name)
            for name in (*EVALUATION_FILES, "unexpected.csv"):
                (directory / name).write_text("malformed; must not be read")
            dataset = load_training_dataset(training_args(data_dir=directory))
            self.assertEqual(dataset.report(), verify_artifacts(ROOT / "models/motion_model_a")["dataset"])
            self.assertEqual(len(dataset.y), 168)

    def test_changed_training_csv_rejected_before_fitting(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / TRAIN_FILES[0]).write_text("modified")
            with self.assertRaisesRegex(ValueError, "differs from Model A"):
                load_training_dataset(training_args(data_dir=directory))

    def test_phase4a_hyperparameters_are_frozen(self):
        for key, value in dict(seed=1, epochs=149, batch_size=32, learning_rate=.002).items():
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "Phase 4A fixes"):
                load_training_dataset(training_args(**{key: value}))

    def test_existing_model_bundle_cannot_be_overwritten(self):
        for directory in (MODEL, ROOT / "models/motion_model_a"):
            before = sha256(directory / "SHA256SUMS")
            with self.assertRaises(FileExistsError):
                train(training_args(output_dir=directory))
            self.assertEqual(sha256(directory / "SHA256SUMS"), before)


class ModelBArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = verify_artifacts(MODEL)
        cls.tf = tensorflow()
        cls.model = cls.tf.keras.models.load_model(MODEL / "model.keras", compile=False)
        cls.float_model = LiteModel(cls.tf, MODEL / "model_float32.tflite")
        cls.int_model = LiteModel(cls.tf, MODEL / "model_int8.tflite")
        with np.load(MODEL / "test_vectors.npz", allow_pickle=False) as archive:
            cls.vectors = {key: archive[key] for key in archive.files}

    def test_architecture_parameter_count_and_canonical_weights(self):
        self.assertEqual(self.model.count_params(), 506)
        self.assertEqual(sum(int(np.prod(w.shape)) for w in self.model.trainable_weights), 506)
        self.assertEqual([tuple(w.shape) for w in self.model.get_weights()], [(60, 8), (8,), (8, 2), (2,)])
        self.assertEqual([layer.activation.__name__ for layer in self.model.layers], ["relu", "linear"])
        self.assertEqual(build_model_b(self.tf).count_params(), 506)
        self.assertEqual(weights_sha256(self.model), self.manifest["weights_sha256"])

    def test_same_features_contract_and_training_provenance_as_a(self):
        model_a = ROOT / "models/motion_model_a"
        self.assertEqual((MODEL / "input_contract.json").read_bytes(), (model_a / "input_contract.json").read_bytes())
        self.assertEqual(self.manifest["dataset"], verify_artifacts(model_a)["dataset"])
        with np.load(model_a / "test_vectors.npz", allow_pickle=False) as vectors_a:
            for key in ("raw_g", "features", "labels"):
                np.testing.assert_array_equal(self.vectors[key], vectors_a[key])
        for name, expected in self.manifest["source_sha256"].items():
            self.assertEqual(sha256(Path(__file__).parent / name), expected, name)
        self.assertEqual(read_json(MODEL / "quantization.json")["representative_vectors"], 168)

    def test_int8_graph_and_per_channel_weights(self):
        metadata = self.int_model.metadata
        self.assertEqual(metadata["input"]["shape"], [1, 60])
        self.assertEqual(metadata["output"]["shape"], [1, 2])
        self.assertEqual([(op["operator"], op["version"], op["fused_activation"]) for op in metadata["operators"]],
                         [("FULLY_CONNECTED", 4, "RELU"), ("FULLY_CONNECTED", 4, "NONE")])
        tensors = {tensor["index"]: tensor for tensor in metadata["tensors"]}
        for op, width in zip(metadata["operators"], (8, 2), strict=True):
            weight = tensors[op["inputs"][1]]
            self.assertEqual(weight["dtype"], "int8")
            self.assertEqual(len(weight["scales"]), width)
            self.assertEqual(weight["quantized_dimension"], 0)
            self.assertEqual(tensors[op["inputs"][2]]["dtype"], "int32")
        self.assertTrue(all(t["dtype"] in ("int8", "int32") for t in tensors.values()))

    def test_training_and_golden_vectors_replay(self):
        features = np.stack([preprocess(raw) for raw in self.vectors["raw_g"]])
        np.testing.assert_array_equal(features, self.vectors["features"])
        keras = self.model(features, training=False).numpy()
        floats, _, _, _ = self.float_model.predict(features)
        integers, encoded, outputs, clipped = self.int_model.predict(features)
        np.testing.assert_array_equal(keras, self.vectors["keras_logits"])
        np.testing.assert_allclose(floats, keras, rtol=1e-5, atol=1e-5)
        np.testing.assert_array_equal(outputs, self.vectors["quantized_output"])
        np.testing.assert_array_equal(integers.argmax(1), keras.argmax(1))
        self.assertEqual(clipped, 0)
        for golden in read_json(MODEL / "golden_vectors.json"):
            index = golden["vector_index"]
            np.testing.assert_array_equal(preprocess(golden["raw_g"]), features[index])
            np.testing.assert_array_equal(encoded[index], golden["int8_input"])
            np.testing.assert_array_equal(outputs[index], golden["int8_output"])

    def test_first_independent_evaluation_replays_without_selection(self):
        dataset = load_dataset([RECORDINGS / name for name in EVALUATION_FILES])
        result = read_json(ROOT / "models/motion_model_b_evaluation_01/evaluation.json")
        result_a = read_json(ROOT / "models/motion_model_a_evaluation_01/evaluation.json")
        self.assertEqual(result["model_manifest_sha256"], sha256(MODEL / "manifest.json"))
        self.assertEqual(dataset.report(), result["dataset"])
        self.assertEqual(result["dataset"], result_a["dataset"])
        self.assertEqual(dataset.reports[0]["session_id"], "rest_orientation_02_retry")
        keras = self.model(dataset.x, training=False).numpy()
        floats, _, _, _ = self.float_model.predict(dataset.x)
        integers, _, _, clipped = self.int_model.predict(dataset.x)
        for key, logits in (("keras", keras), ("tflite_float32", floats), ("tflite_int8", integers)):
            self.assertEqual(metrics(dataset.y, logits.argmax(1)), result[key])
        self.assertEqual(clipped, 157)
        self.assertEqual(np.flatnonzero(keras.argmax(1) != integers.argmax(1)).tolist(), [102, 104, 110, 111])

    def test_compatibility_evidence_integrity_and_host_replay_outputs(self):
        directory = ROOT / "models/motion_model_comparison_01"
        for line in (directory / "SHA256SUMS").read_text().splitlines():
            expected, name = line.split("  ", 1)
            self.assertEqual(sha256(directory / name), expected)
        report = read_json(directory / "comparison.json")
        for path, digest in report["validation_source_sha256"].items():
            # Phase 4A is frozen historical evidence. Preserve its exact build
            # recipe while current firmware-provider tests extend the harness.
            source = ROOT / path
            if path == "tests/motion_host/CMakeLists.txt":
                source = ROOT / "tests/motion_host/provenance/phase4a_CMakeLists.txt"
            self.assertEqual(sha256(source), digest)
        self.assertTrue(report["reproducibility"]["weights_identical"])
        self.assertEqual(report["models"]["b"]["weights_sha256"], self.manifest["weights_sha256"])
        with np.load(directory / "host_outputs.npz", allow_pickle=False) as outputs:
            np.testing.assert_array_equal(outputs["b_training_reference"], self.vectors["quantized_output"])
            for group in ("training", "independent", "golden"):
                np.testing.assert_array_equal(outputs[f"b_{group}_reference"], outputs[f"b_{group}_esp_nn_ansi"])
                self.assertEqual(report["models"]["b"]["host_runtime"][group]["reference"]["different_output_elements"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
