"""Verify immutable Model A artifacts; emit a C++ bundle into the build directory."""
import argparse
import hashlib
import json
from pathlib import Path

EXPECTED_SHA = "1885f051bb81daa9f4f8ff12ee39a0706ac4d45999b823ef91ade991e5a4b7aa"

def generate(model_dir, output):
    manifest = json.loads((model_dir / "manifest.json").read_text())
    for name in ("model_int8.tflite", "input_contract.json", "quantization.json", "golden_vectors.json"):
        raw = (model_dir / name).read_bytes()
        record = manifest["artifacts"][name]
        if len(raw) != record["bytes"] or hashlib.sha256(raw).hexdigest() != record["sha256"]:
            raise ValueError(f"Artifact integrity failure: {name}")
    model = (model_dir / "model_int8.tflite").read_bytes()
    if len(model) != 3104 or hashlib.sha256(model).hexdigest() != EXPECTED_SHA:
        raise ValueError("Unexpected Model A bytes; never regenerate the model here")
    contract = json.loads((model_dir / "input_contract.json").read_text())
    assert (contract["window_samples"], contract["stride_samples"], contract["sampling_hz"]) == (20, 10, 10)
    assert contract["class_mapping"] == {"REST": 0, "MOVING": 1}
    quant = json.loads((model_dir / "quantization.json").read_text())["int8"]
    assert quant["input"]["shape"] == [1, 60] and quant["output"]["shape"] == [1, 2]
    assert quant["input"]["dtype"] == quant["output"]["dtype"] == "int8"
    golden = json.loads((model_dir / "golden_vectors.json").read_text())
    lines = ["// Generated from verified repository artifacts; do not edit.", "#pragma once",
             "#include <cstdint>", "namespace motion_bundle {",
             f'constexpr char version[] = "{manifest["model_version"]}";',
             "alignas(16) const uint8_t model[] = {" + ",".join(map(str, model)) + "};",
             "const uint8_t sha256[] = {" + ",".join(map(str, bytes.fromhex(EXPECTED_SHA))) + "};"]
    for label in ("input", "output"):
        lines += [f'constexpr float {label}_scale = {quant[label]["scales"][0]:.17g}f;',
                  f'constexpr int {label}_zero_point = {quant[label]["zero_points"][0]};']
    # Exact float32 hexadecimal literals avoid decimal conversion ambiguity.
    def floats(values):
        return ",".join(float(x).hex() + "f" for x in values)
    lines += ["struct Golden { float raw[20][3]; int8_t input[60]; int8_t output[2]; int prediction; };",
              "const Golden golden[] = {"]
    for g in golden:
        lines += ["{" + "{" + ",".join("{" + floats(row) + "}" for row in g["raw_g"]) + "}," +
                  "{" + ",".join(map(str, g["int8_input"])) + "}," +
                  "{" + ",".join(map(str, g["int8_output"])) + "}," +
                  str(int(g["int8_output"][1] > g["int8_output"][0])) + "},"]
    lines += ["};", "}"]
    output.parent.mkdir(parents=True, exist_ok=True)
    content = "\n".join(lines) + "\n"
    if not output.exists() or output.read_text() != content:
        output.write_text(content)
    print(f"Verified Model A: {len(model)} bytes SHA256={EXPECTED_SHA}; {len(golden)} golden vectors")

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    generate(a.model_dir, a.output)
