"""Run compiled MCU preprocessing and real pinned TFLM against Python TFLite.

Use tools/ml_training/.venv (TensorFlow 2.20.0 / NumPy 2.2.6); never install ML
packages into ESP-IDF. Configure/build tests/motion_host with host GCC first.
"""
import argparse
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

repo = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(repo / "tools/ml_training"))
sys.path.insert(0, str(repo / "tools/ml_inference"))
from data import preprocess
from runtime import LiteModel, tensorflow
from generate_bundle import generate, EXPECTED_SHA

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--build-dir", type=Path, default=repo / "build/motion_host")
a = p.parse_args()
build = a.build_dir.resolve()
model_dir = repo / "models/motion_model_a"
assert hashlib.sha256((model_dir / "model_int8.tflite").read_bytes()).hexdigest() == EXPECTED_SHA
with tempfile.TemporaryDirectory(dir=build) as tmp:
    tmp = Path(tmp)
    for name in ("manifest.json", "input_contract.json", "quantization.json", "golden_vectors.json", "model_int8.tflite"):
        (tmp / name).write_bytes((model_dir / name).read_bytes())
    generate(tmp, tmp / "bundle.h")
    broken = bytearray((tmp / "model_int8.tflite").read_bytes()); broken[100] ^= 1
    (tmp / "model_int8.tflite").write_bytes(broken)
    try:
        generate(tmp, tmp / "bundle.h")
        raise AssertionError("Build generator accepted altered model")
    except ValueError:
        pass

if os.name == "nt":
    # GCC runtime DLLs are discovered through PATH in the host test shell.
    dll = next(build.glob("*core_python.dll"))
else:
    dll = next(build.glob("*core_python.so"))
core = ct.CDLL(str(dll))
fptr, iptr = ct.POINTER(ct.c_float), ct.POINTER(ct.c_int8)
core.motion_preprocess.argtypes = [fptr, fptr]
core.motion_preprocess.restype = ct.c_bool
core.motion_quantize.argtypes = [fptr, ct.c_float, ct.c_int, iptr, ct.POINTER(ct.c_uint32)]
core.motion_quantize.restype = ct.c_bool
tf = tensorflow()
lite = LiteModel(tf, model_dir / "model_int8.tflite")
from tensorflow.lite.python import schema_py_generated as schema
assert schema.Model.GetRootAsModel((model_dir / "model_int8.tflite").read_bytes(), 0).Version() == 3
assert [op["version"] for op in lite.metadata["operators"]] == [4, 4]
assert [op["fused_activation"] for op in lite.metadata["operators"]] == ["RELU", "NONE"]
assert lite.metadata == json.loads((model_dir / "quantization.json").read_text())["int8"]

stored = np.load(model_dir / "test_vectors.npz")
golden = json.loads((model_dir / "golden_vectors.json").read_text())
random = np.random.default_rng(20261028).uniform(-4, 4, (1000, 20, 3)).astype(np.float32)
raws = np.concatenate([stored["raw_g"], np.array([g["raw_g"] for g in golden], dtype=np.float32), random])
features, encoded = [], []
clipped_total = 0
for raw in raws:
    reference = preprocess(raw)
    actual = np.empty(60, dtype=np.float32)
    assert core.motion_preprocess(raw.ctypes.data_as(fptr), actual.ctypes.data_as(fptr))
    np.testing.assert_array_equal(actual, reference)
    q = np.empty(60, dtype=np.int8); clipped = ct.c_uint32()
    scale, zp = lite.input["quantization"]
    assert core.motion_quantize(actual.ctypes.data_as(fptr), scale, zp, q.ctypes.data_as(iptr), ct.byref(clipped))
    python_q, python_clipped = lite.encode(reference)
    np.testing.assert_array_equal(q, python_q[0])
    assert clipped.value == python_clipped
    clipped_total += clipped.value
    features.append(reference); encoded.append(q)
features, encoded = np.array(features), np.array(encoded)
_, ref_inputs, ref_outputs, clipping = lite.predict(features)
np.testing.assert_array_equal(encoded, ref_inputs)
np.testing.assert_array_equal(ref_outputs[:168], stored["quantized_output"])
assert clipped_total == clipping
(build / "reference_inputs.bin").write_bytes(encoded.tobytes())
(build / "python_outputs.bin").write_bytes(ref_outputs.tobytes())
suffix = ".exe" if os.name == "nt" else ""
subprocess.run([str(build / ("test_core" + suffix))], check=True)
worker_cases = ("live", "default", "invalid", "gap", "sequence", "reboot", "configure", "disabled",
                "stale", "no_data", "unavailable", "overflow", "config_invoke", "queue_fail", "task_fail",
                "heap_fail", "init_fail", "golden_fail", "invoke_fail")
for case in worker_cases:
    subprocess.run([str(build / ("test_worker" + suffix)), case], check=True, timeout=10)
comparisons = {}
previous = None
for backend in ("reference", "esp_nn_ansi"):
    out = build / (backend + "_outputs.bin")
    subprocess.run([str(build / ("test_" + backend + suffix)), str(build / "reference_inputs.bin"), str(out)], check=True)
    actual = np.frombuffer(out.read_bytes(), dtype=np.int8).reshape(-1, 2)
    # The four authoritative golden vectors remain byte-exact. On the wider
    # corpus the pinned TFLM runtime differs from TF 2.20 by at most one output
    # quantization unit. Do not label that as bit-exact desktop equivalence.
    np.testing.assert_array_equal(actual[168:172], ref_outputs[168:172])
    delta = np.abs(actual.astype(np.int16) - ref_outputs.astype(np.int16))
    assert delta.max() <= 1
    np.testing.assert_array_equal(actual.argmax(axis=1), ref_outputs.argmax(axis=1))
    if previous is not None:
        np.testing.assert_array_equal(actual, previous)
    previous = actual
    comparisons[backend] = {"different_output_elements": int(np.count_nonzero(delta)),
                            "max_output_lsb_difference": int(delta.max()),
                            "class_agreement": len(raws),
                            "different_vector_indices": np.flatnonzero(np.any(delta, axis=1)).tolist()}
    print(f"{backend}: {comparisons[backend]}")
report = {"model_sha256": EXPECTED_SHA, "schema": 3, "golden_vectors": 4,
          "preprocessing_quantization_vectors_exact": len(raws), "clipped_elements": clipping,
          "tflm_backends_agree_exactly": True, "python_output_comparison": comparisons,
          "stable_invokes_per_backend": 10000,
          "worker_scenarios": len(worker_cases), "hardware_tested": False}
(build / "results.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
