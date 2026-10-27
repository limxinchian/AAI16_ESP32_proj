"""Data-integrity tests plus checks of the actual frozen trained artifacts.

Fixtures below are synthetic unit-test cases only; training exclusively uses
the three measured CSV recordings. Run with the isolated ML venv interpreter.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from data import (Dataset, EVALUATION_FILES, TRAIN_FILES, load_dataset, load_session,
                  movement_rms, parse_row, preprocess, require_independent, sha256)
from evaluate import evaluate, verify_artifacts
from runtime import LiteModel, build_model, fit_rms_baseline, metrics, tensorflow, weights_sha256

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models/motion_model_a"


def records(n=60):
    return [dict(timestamp_ms=str(1000 + i * 100), uptime_ms=str(1000 + i * 100),
                 boot_id="1234abcd", seq=str(i), ax_g=str(i % 5 * .01), ay_g="0.2", az_g="1.0",
                 label="REST", session_id="fixture") for i in range(n)]


def shift_time(rows, start, delta):
    for row in rows[start:]:
        for name in ("timestamp_ms", "uptime_ms"):
            row[name] = str(int(row[name]) + delta)


class DataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "fixture.csv"

    def write(self, rows):
        with self.path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records(1)[0]))
            writer.writeheader()
            writer.writerows(rows)
        return self.path

    def load(self, rows):
        return load_session(self.write(rows))

    def test_preprocessing_mean_axis_order_and_flatten(self):
        raw = np.tile([1., 2., 3.], (20, 1)).astype(np.float32)
        raw[0] += [1., -2., .5]
        raw[19] -= [1., -2., .5]
        before = raw.copy()
        output = preprocess(raw)
        expected = np.zeros(60, dtype=np.float32)
        expected[:3], expected[-3:] = [.5, -1., .25], [-.5, 1., -.25]
        np.testing.assert_array_equal(output, expected)
        np.testing.assert_array_equal(raw, before)
        self.assertEqual(output.dtype, np.float32)
        np.testing.assert_array_equal(preprocess(raw + np.array([4., 8., 16.], np.float32)), expected)

    def test_clamp_is_after_mean_and_before_division(self):
        raw = np.zeros((20, 3), np.float32)
        raw[0, 0] = 4
        output = preprocess(raw).reshape(20, 3)
        self.assertEqual(output[0, 0], 1.)
        np.testing.assert_array_equal(output[1:, 0], np.full(19, -.1, np.float32))

    def test_reject_bad_preprocessing(self):
        for raw in (np.zeros((19, 3)), np.zeros(60), np.full((20, 3), np.nan), np.full((20, 3), np.inf)):
            with self.subTest(shape=raw.shape), self.assertRaises(ValueError):
                preprocess(raw)

    def test_baseline_rms_units(self):
        raw = np.tile([1., 2., 3.], (20, 1))
        raw[::2, 0] += .5
        raw[1::2, 0] -= .5
        self.assertAlmostEqual(movement_rms(raw), .5)

    def test_row_validation(self):
        bad = {"timestamp_ms": "1.0", "uptime_ms": "1001", "boot_id": "invalid",
               "seq": "4294967296", "ax_g": "NaN", "ay_g": "inf", "az_g": "2.049",
               "label": "walking", "session_id": "../bad"}
        for key, value in bad.items():
            row = records(1)[0]
            row[key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                parse_row(row)
        row = records(1)[0]
        row[None] = ["extra"]
        with self.assertRaises(ValueError):
            parse_row(row)

    def test_gap_threshold_and_original_stride_grid(self):
        rows = records()
        shift_time(rows, 25, 50)  # exactly 150 ms admitted
        self.assertEqual(len(self.load(rows)[0]), 5)
        shift_time(rows, 25, 1)   # 151 ms rejects starts 10,20; no reanchoring
        windows, origins, report = self.load(rows)
        self.assertEqual(len(windows), 3)
        self.assertEqual([item["start_row_index"] for item in origins], [0, 30, 40])
        self.assertEqual(report["sampling_gaps"][0]["delta_ms"], 151)
        self.assertEqual(report["rejected_windows"], {"sampling_gap": 2})

    def test_invalid_row_does_not_compress_positions_or_modify_source(self):
        rows = records()
        rows[25]["ax_g"] = "broken"
        self.write(rows)
        before = sha256(self.path)
        _, origins, report = load_session(self.path)
        self.assertEqual(sha256(self.path), before)
        self.assertEqual([item["start_row_index"] for item in origins], [0, 30, 40])
        self.assertEqual(report["invalid_rows"][0]["row_index"], 25)
        self.assertEqual(report["valid_rows"], 59)
        self.assertEqual(report["rejected_windows"], {"invalid_row": 2})

    def test_reboot_never_crossed(self):
        rows = records()
        for i, row in enumerate(rows[25:]):
            row.update(boot_id="fedcba98", seq=str(i), timestamp_ms=str(i * 100), uptime_ms=str(i * 100))
        _, origins, report = self.load(rows)
        self.assertEqual([item["start_row_index"] for item in origins], [0, 30, 40])
        self.assertEqual(report["reboots"], 1)
        self.assertEqual(report["rejected_windows"], {"reboot": 2})

    def test_sequence_gap_and_wrap(self):
        rows = records()
        for row in rows[25:]:
            row["seq"] = str(int(row["seq"]) + 2)
        _, origins, report = self.load(rows)
        self.assertEqual([item["start_row_index"] for item in origins], [0, 30, 40])
        self.assertEqual(report["sequence_gaps"], 2)
        rows = records()
        for i, row in enumerate(rows):
            row["seq"] = str((2**32 - 10 + i) & 0xFFFFFFFF)
        self.assertEqual(len(self.load(rows)[0]), 5)

    def test_old_boot_remains_rejected_after_an_invalid_row(self):
        rows = records(90)
        for i, row in enumerate(rows[25:50]):
            row.update(boot_id="fedcba98", seq=str(i), timestamp_ms=str(i * 100), uptime_ms=str(i * 100))
        rows[50]["ax_g"] = "bad"
        _, origins, report = self.load(rows)
        self.assertEqual(report["reboots"], 1)
        self.assertEqual(len(report["invalid_rows"]), 40)
        self.assertTrue(all(item["end_row_index"] < 50 for item in origins))

    def test_duplicate_backward_and_timestamp_regression(self):
        for field, value, count_key in (("seq", "24", "duplicates"), ("seq", "20", "out_of_order"),
                                        ("timestamp_ms", "3400", None)):
            rows = records()
            rows[25][field] = value
            if field == "timestamp_ms":
                rows[25]["uptime_ms"] = value
            _, origins, report = self.load(rows)
            self.assertEqual([item["start_row_index"] for item in origins], [0, 30, 40])
            self.assertEqual(len(report["invalid_rows"]), 1)
            if count_key:
                self.assertEqual(report[count_key], 1)

    def test_mixed_session_and_label_are_fatal(self):
        for field, value in (("label", "MOVING"), ("session_id", "different")):
            rows = records()
            rows[25][field] = value
            with self.assertRaises(RuntimeError):
                self.load(rows)

    def test_constant_and_possible_saturation_flagged(self):
        rows = records(20)
        for row in rows:
            row["ax_g"] = "2.047"
        windows, _, report = self.load(rows)
        self.assertEqual(len(windows), 1)
        self.assertEqual(report["constant_windows"], 1)
        self.assertEqual(len(report["possible_saturation_row_indices"]), 20)
        self.assertEqual(len(report["warnings"]), 2)

    def test_constant_axis_detected_despite_float_reduction_roundoff(self):
        report = self.load(records(60))[2]
        self.assertEqual(report["constant_axes"], ["y", "z"])
        self.assertEqual(report["constant_windows"], 0)

    def test_incomplete_session_and_missing_header_rejected(self):
        self.assertEqual(self.load(records(19))[2]["usable_windows"], 0)
        with self.assertRaises(ValueError):
            load_dataset([self.path])
        self.path.write_text("bad,header\n1,2\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            load_session(self.path)

    def test_training_and_reserved_names_disjoint(self):
        self.assertTrue(set(TRAIN_FILES).isdisjoint(EVALUATION_FILES))

    def test_independence_guards(self):
        train = {"sha256": "train", "session_id": "train", "boot_ranges": [dict(boot_id="boot", start_ms=0, end_ms=100)]}
        independent = {"sha256": "test", "session_id": "test", "boot_ranges": [dict(boot_id="boot", start_ms=101, end_ms=200)]}
        dataset = Dataset(None, None, np.array([0, 1]), None, [], [independent])
        require_independent(dataset, [train])
        for changed in (dict(independent, sha256="train"), dict(independent, session_id="train"),
                        dict(independent, boot_ranges=[dict(boot_id="boot", start_ms=100, end_ms=200)])):
            dataset.reports = [changed]
            with self.assertRaises(ValueError):
                require_independent(dataset, [train])
        dataset.reports, dataset.y = [independent], np.array([1])
        with self.assertRaises(ValueError):
            require_independent(dataset, [train])

    def test_baseline_and_confusion_metrics(self):
        result = fit_rms_baseline(np.array([.01, .02, .2, .3]), np.array([0, 0, 1, 1]))
        self.assertAlmostEqual(result["threshold_g"], .11)
        self.assertEqual(result["training_sanity"]["accuracy"], 1.)
        result = metrics(np.array([0, 0, 1]), np.array([0, 1, 1]))
        self.assertEqual(result["confusion_matrix_rows_true_columns_predicted"], [[1, 1], [0, 1]])
        self.assertEqual(result["per_class"]["REST"]["recall"], .5)
        self.assertEqual(result["per_class"]["MOVING"]["precision"], .5)


class TrainedArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # These tests intentionally fail if the real Model A bundle is missing.
        cls.manifest = verify_artifacts(MODEL)
        cls.tf = tensorflow()
        cls.model = cls.tf.keras.models.load_model(MODEL / "model.keras", compile=False)
        cls.float_model = LiteModel(cls.tf, MODEL / "model_float32.tflite")
        cls.int_model = LiteModel(cls.tf, MODEL / "model_int8.tflite")
        with np.load(MODEL / "test_vectors.npz", allow_pickle=False) as archive:
            cls.vectors = {name: archive[name] for name in archive.files}

    def test_real_dataset_counts_integrity_and_provenance(self):
        dataset = load_dataset([ROOT / "tools/ml_data_collection/recordings" / name for name in TRAIN_FILES])
        self.assertEqual([r["rows"] for r in dataset.reports], [599, 598, 599])
        self.assertEqual([r["usable_windows"] for r in dataset.reports], [58, 54, 56])
        self.assertEqual([r["sampling_gap_count"] for r in dataset.reports], [0, 2, 1])
        self.assertEqual(dataset.report()["class_windows"], {"REST": 58, "MOVING": 110})
        self.assertEqual(dataset.report(), self.manifest["dataset"])
        np.testing.assert_array_equal(dataset.x, self.vectors["features"])
        np.testing.assert_array_equal(dataset.raw, self.vectors["raw_g"])
        np.testing.assert_array_equal(dataset.y, self.vectors["labels"])

    def test_preprocessing_replays_saved_vectors(self):
        actual = np.stack([preprocess(raw) for raw in self.vectors["raw_g"]])
        np.testing.assert_array_equal(actual, self.vectors["features"])
        golden = json.loads((MODEL / "golden_vectors.json").read_text())
        for vector in golden:
            features = preprocess(vector["raw_g"])
            np.testing.assert_array_equal(features, vector["features"])
            encoded, _ = self.int_model.encode(features)
            np.testing.assert_array_equal(encoded[0], vector["int8_input"])

    def test_network_and_export_shapes_operators_and_types(self):
        self.assertEqual(self.model.count_params(), 1010)
        self.assertEqual(weights_sha256(self.model), self.manifest["weights_sha256"])
        self.assertEqual(build_model(self.tf).count_params(), 1010)
        self.assertEqual(self.int_model.input["dtype"], np.int8)
        self.assertEqual(self.int_model.output["dtype"], np.int8)
        for model in (self.float_model, self.int_model):
            self.assertEqual(model.metadata["input"]["shape"], [1, 60])
            self.assertEqual(model.metadata["output"]["shape"], [1, 2])
            self.assertEqual([op["operator"] for op in model.metadata["operators"]], ["FULLY_CONNECTED"] * 2)
        self.assertTrue(all(t["dtype"] in ("int8", "int32") for t in self.int_model.metadata["tensors"]))

    def test_saved_predictions_replay_with_float_export_tolerance(self):
        features = self.vectors["features"]
        keras = self.model(features, training=False).numpy()
        floated, _, _, _ = self.float_model.predict(features)
        integer, encoded, output, _ = self.int_model.predict(features)
        np.testing.assert_array_equal(keras, self.vectors["keras_logits"])
        np.testing.assert_allclose(floated, keras, rtol=1e-5, atol=1e-5)
        np.testing.assert_array_equal(integer, self.vectors["int8_dequantized_logits"])
        np.testing.assert_array_equal(encoded, self.vectors["quantized_input"])
        np.testing.assert_array_equal(output, self.vectors["quantized_output"])

    def test_int8_rounding_and_clamp(self):
        # Chosen power-of-two scale makes the exact half cases unambiguous.
        with patch.object(self.int_model, "input", {"dtype": np.int8, "quantization": (.25, -3)}):
            values = np.zeros(60, np.float32)
            values[:8] = np.array([.5, 1.5, 2.5, -.5, -1.5, -2.5, 1000, -1000]) * .25
            output, clipped = self.int_model.encode(values)
            np.testing.assert_array_equal(output[0, :8], [-3, -1, -1, -3, -5, -5, 127, -128])
            self.assertEqual(clipped, 2)

    def test_missing_holdouts_create_no_result_or_model_changes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            before = sha256(MODEL / "SHA256SUMS")
            args = argparse.Namespace(model_dir=MODEL, data_dir=root, mode="independent", output_dir=root / "output")
            with self.assertRaisesRegex(ValueError, "Required recordings"):
                evaluate(args)
            self.assertFalse(args.output_dir.exists())
            self.assertEqual(sha256(MODEL / "SHA256SUMS"), before)

    def test_corrupt_model_bundle_rejected_before_execution(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary)
            (target / "model.keras").write_bytes(b"bad")
            (target / "SHA256SUMS").write_text("0" * 64 + "  model.keras\n")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                verify_artifacts(target)


if __name__ == "__main__":
    unittest.main(verbosity=2)
