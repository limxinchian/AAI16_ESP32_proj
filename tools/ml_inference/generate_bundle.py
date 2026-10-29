"""Verify frozen A/B artifacts; emit a provider descriptor into the build directory."""
import argparse
import hashlib
import json
import math
from pathlib import Path

EXPECTED_MODELS = {
    "a": (3104, "1885f051bb81daa9f4f8ff12ee39a0706ac4d45999b823ef91ade991e5a4b7aa"),
    "b": (2384, "6ec06da5b5fe3d12fdd99328be1e8adf10bc3d78f3c7191bcb06dcd2f5279aff"),
}
EXPECTED_SHA = EXPECTED_MODELS["a"][1] # Preserve the original Model A audit identity.
CONTRACT_SHA = "e036fdc6e8323334050a7a29daad62687de9a891193f26fde1953fbfbcccc294"

def generate(model_dir, output, family="a"):
    if family not in EXPECTED_MODELS:
        raise ValueError("Unknown model family")
    manifest = json.loads((model_dir / "manifest.json").read_text())
    if manifest["model_version"] != f"aai16-motion-model-{family}-0.1.0":
        raise ValueError("Model identity does not match selected family")
    for name in ("model_int8.tflite", "input_contract.json", "quantization.json", "golden_vectors.json"):
        raw = (model_dir / name).read_bytes()
        record = manifest["artifacts"][name]
        if len(raw) != record["bytes"] or hashlib.sha256(raw).hexdigest() != record["sha256"]:
            raise ValueError(f"Artifact integrity failure: {name}")
    model = (model_dir / "model_int8.tflite").read_bytes()
    expected_size, expected_sha = EXPECTED_MODELS[family]
    if len(model) != expected_size or hashlib.sha256(model).hexdigest() != expected_sha:
        raise ValueError(f"Unexpected Model {family.upper()} bytes; never regenerate the model here")
    if hashlib.sha256((model_dir / "input_contract.json").read_bytes()).hexdigest() != CONTRACT_SHA:
        raise ValueError("Unsupported preprocessing contract")
    contract = json.loads((model_dir / "input_contract.json").read_text())
    assert (contract["window_samples"], contract["stride_samples"], contract["sampling_hz"]) == (20, 10, 10)
    assert contract["class_mapping"] == {"REST": 0, "MOVING": 1}
    quant = json.loads((model_dir / "quantization.json").read_text())["int8"]
    for label, width in (("input", 60), ("output", 2)):
        tensor = quant[label]
        if (tensor["shape"] != [1, width] or tensor["dtype"] != "int8" or
                len(tensor["scales"]) != 1 or len(tensor["zero_points"]) != 1 or
                not math.isfinite(tensor["scales"][0]) or tensor["scales"][0] <= 0 or
                not -128 <= tensor["zero_points"][0] <= 127):
            raise ValueError(f"Invalid {label} quantization contract")
    golden = json.loads((model_dir / "golden_vectors.json").read_text())
    if len(golden) != 4:
        raise ValueError("Frozen bundles require four golden vectors")
    for g in golden:
        if (len(g["raw_g"]) != 20 or any(len(row) != 3 or not all(math.isfinite(v) for v in row) for row in g["raw_g"]) or
                len(g["int8_input"]) != 60 or len(g["int8_output"]) != 2 or
                any(type(v) is not int or not -128 <= v <= 127 for v in g["int8_input"] + g["int8_output"])):
            raise ValueError("Malformed golden vector")
    lines = ["// Generated from verified repository artifacts; do not edit.", "#pragma once",
             '#include "model_provider.h"', f"namespace motion_bundle_{family} {{",
             f'constexpr char version[] = "{manifest["model_version"]}";',
             "alignas(16) const uint8_t model[] = {" + ",".join(map(str, model)) + "};",
             "const uint8_t sha256[] = {" + ",".join(map(str, bytes.fromhex(expected_sha))) + "};"]
    for label in ("input", "output"):
        lines += [f'constexpr float {label}_scale = {quant[label]["scales"][0]:.17g}f;',
                  f'constexpr int {label}_zero_point = {quant[label]["zero_points"][0]};']
    # Exact float32 hexadecimal literals avoid decimal conversion ambiguity.
    def floats(values):
        return ",".join(float(x).hex() + "f" for x in values)
    lines += ["const motion::GoldenVector golden[] = {"]
    for g in golden:
        lines += ["{" + "{" + ",".join("{" + floats(row) + "}" for row in g["raw_g"]) + "}," +
                  "{" + ",".join(map(str, g["int8_input"])) + "}," +
                  "{" + ",".join(map(str, g["int8_output"])) + "}," +
                  str(int(g["int8_output"][1] > g["int8_output"][0])) + "},"]
    lines += ["};", "const motion::ModelDescriptor descriptor = {",
              f"motion::ModelFamily::{family.upper()}, motion::kMotionCompatibilityProfile,",
              "{model, sizeof(model), sha256, version, input_scale, output_scale, input_zero_point, output_zero_point},",
              "golden, sizeof(golden) / sizeof(golden[0])};", "}"]
    output.parent.mkdir(parents=True, exist_ok=True)
    content = "\n".join(lines) + "\n"
    if not output.exists() or output.read_text() != content:
        output.write_text(content)
    print(f"Verified Model {family.upper()}: {len(model)} bytes SHA256={expected_sha}; {len(golden)} golden vectors")

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--model", choices=tuple(EXPECTED_MODELS), default="a")
    a = p.parse_args()
    generate(a.model_dir, a.output, a.model)
