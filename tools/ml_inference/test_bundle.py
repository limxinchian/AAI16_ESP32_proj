"""Build-time bundle integrity and model-identity admission tests (no training)."""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from generate_bundle import generate

ROOT = Path(__file__).resolve().parents[2]


class BundleTests(unittest.TestCase):
    def fixture(self, directory, family):
        source = ROOT / f"models/motion_model_{family}"
        for name in ("manifest.json", "input_contract.json", "quantization.json", "golden_vectors.json", "model_int8.tflite"):
            shutil.copyfile(source / name, directory / name)

    def replace_metadata(self, directory, name, value):
        raw = (json.dumps(value) + "\n").encode()
        (directory / name).write_bytes(raw)
        manifest = json.loads((directory / "manifest.json").read_text())
        manifest["artifacts"][name] = {"bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}
        (directory / "manifest.json").write_text(json.dumps(manifest))

    def test_each_frozen_bundle_generates_deterministically(self):
        for family in ("a", "b"):
            with self.subTest(model=family), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp)
                self.fixture(directory, family)
                output = directory / "bundle.h"
                generate(directory, output, family)
                before = output.read_bytes(), output.stat().st_mtime_ns
                generate(directory, output, family)
                self.assertEqual((output.read_bytes(), output.stat().st_mtime_ns), before)

    def test_family_mismatch_is_rejected_before_output(self):
        for family, wrong in (("a", "b"), ("b", "a")):
            with self.subTest(model=family), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp)
                self.fixture(directory, family)
                with self.assertRaisesRegex(ValueError, "identity"):
                    generate(directory, directory / "bundle.h", wrong)
                self.assertFalse((directory / "bundle.h").exists())

    def test_model_and_each_authoritative_metadata_mutation_rejected(self):
        for family in ("a", "b"):
            for name in ("model_int8.tflite", "input_contract.json", "quantization.json", "golden_vectors.json"):
                with self.subTest(model=family, artifact=name), tempfile.TemporaryDirectory() as tmp:
                    directory = Path(tmp)
                    self.fixture(directory, family)
                    raw = bytearray((directory / name).read_bytes())
                    raw[10] ^= 1
                    (directory / name).write_bytes(raw)
                    with self.assertRaisesRegex(ValueError, "integrity"):
                        generate(directory, directory / "bundle.h", family)

    def test_rehashed_unsupported_contract_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            self.fixture(directory, "b")
            contract = json.loads((directory / "input_contract.json").read_text())
            contract["sampling_hz"] = 2
            self.replace_metadata(directory, "input_contract.json", contract)
            with self.assertRaisesRegex(ValueError, "preprocessing contract"):
                generate(directory, directory / "bundle.h", "b")

    def test_rehashed_invalid_quantization_and_missing_goldens_rejected(self):
        for artifact in ("quantization.json", "golden_vectors.json"):
            with self.subTest(artifact=artifact), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp)
                self.fixture(directory, "b")
                value = json.loads((directory / artifact).read_text())
                if artifact == "quantization.json":
                    value["int8"]["output"]["scales"] = [0]
                else:
                    value = []
                self.replace_metadata(directory, artifact, value)
                with self.assertRaises(ValueError):
                    generate(directory, directory / "bundle.h", "b")


if __name__ == "__main__":
    unittest.main()
