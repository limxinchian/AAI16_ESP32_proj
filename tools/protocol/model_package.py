"""Deterministic staging package from existing artifacts. No training/conversion."""
import hashlib
import json
import math
import re
import struct
from pathlib import Path

CONTRACT_SHA = "e036fdc6e8323334050a7a29daad62687de9a891193f26fde1953fbfbcccc294"
PACKAGE_MAX, MODEL_MAX, HEADER, GOLDEN = 73728, 65536, 256, 320


def package_model(model_dir, family="b"):
    model_dir = Path(model_dir)
    manifest = json.loads((model_dir / "manifest.json").read_text())
    artifacts = {}
    for name in ("model_int8.tflite", "input_contract.json", "quantization.json", "golden_vectors.json"):
        raw = (model_dir / name).read_bytes()
        expected = manifest["artifacts"][name]
        if len(raw) != expected["bytes"] or hashlib.sha256(raw).hexdigest() != expected["sha256"]:
            raise ValueError(f"Artifact integrity failure: {name}")
        artifacts[name] = raw
    if hashlib.sha256(artifacts["input_contract.json"]).hexdigest() != CONTRACT_SHA:
        raise ValueError("Unsupported preprocessing contract")
    version = manifest["model_version"]
    if family not in ("a", "b") or not re.fullmatch(r"[A-Za-z0-9_.-]{1,39}", version):
        raise ValueError("Invalid family/version")
    if not version.startswith(f"aai16-motion-model-{family}-"):
        raise ValueError("Selected family disagrees with artifact version")
    model = artifacts["model_int8.tflite"]
    if not 1 <= len(model) <= MODEL_MAX:
        raise ValueError("Model exceeds firmware size limit")
    q = json.loads(artifacts["quantization.json"])["int8"]
    for label, width in (("input", 60), ("output", 2)):
        t = q[label]
        if (t["shape"] != [1, width] or t["dtype"] != "int8" or len(t["scales"]) != 1 or
                len(t["zero_points"]) != 1 or not math.isfinite(t["scales"][0]) or t["scales"][0] <= 0 or
                not -128 <= t["zero_points"][0] <= 127):
            raise ValueError(f"Unsupported {label} tensor")
    golden = json.loads(artifacts["golden_vectors.json"])
    if len(golden) != 4:
        raise ValueError("Exactly four golden vectors required")
    golden_offset = (HEADER + len(model) + 15) & ~15
    package = bytearray(golden_offset + GOLDEN * 4)
    if len(package) > PACKAGE_MAX:
        raise ValueError("Package exceeds firmware limit")
    package[:8] = b"AAI16MP1"
    struct.pack_into("<9I", package, 8, 1, HEADER, 1 if family == "a" else 2, 1,
                     len(model), 4, HEADER, golden_offset, len(package))
    struct.pack_into("<fifi", package, 44, q["input"]["scales"][0], q["input"]["zero_points"][0],
                     q["output"]["scales"][0], q["output"]["zero_points"][0])
    package[64:64 + len(version)] = version.encode("ascii")
    model_sha = hashlib.sha256(model).digest()
    package[104:136] = model_sha
    package[136:168] = bytes.fromhex(CONTRACT_SHA)
    package[HEADER:HEADER + len(model)] = model
    for i, g in enumerate(golden):
        if len(g["raw_g"]) != 20 or any(len(row) != 3 for row in g["raw_g"]):
            raise ValueError("Golden raw shape")
        raw = [v for row in g["raw_g"] for v in row]
        inputs, outputs = g["int8_input"], g["int8_output"]
        if (not all(math.isfinite(v) for v in raw) or len(inputs) != 60 or len(outputs) != 2 or
                any(type(v) is not int or not -128 <= v <= 127 for v in inputs + outputs)):
            raise ValueError("Malformed golden vector")
        offset = golden_offset + GOLDEN * i
        struct.pack_into("<60f60b2b", package, offset, *raw, *inputs, *outputs)
        struct.pack_into("<I", package, offset + 304, int(outputs[1] > outputs[0]))
    metadata = dict(format_version=1, family=family.upper(), version=version, package_size=len(package),
                    package_sha256=hashlib.sha256(package).hexdigest(), model_size=len(model),
                    model_sha256=model_sha.hex(), compatibility_profile=1)
    return bytes(package), metadata


def validate_package(package, manifest):
    """Laptop preflight only. Device additionally verifies graph + golden vectors."""
    if (manifest.get("format_version") != 1 or manifest.get("family") not in ("A", "B") or
            not re.fullmatch(r"[A-Za-z0-9_.-]{1,39}", manifest.get("version", "")) or
            manifest.get("compatibility_profile") != 1 or
            not HEADER <= len(package) <= PACKAGE_MAX or manifest.get("package_size") != len(package) or
            hashlib.sha256(package).hexdigest() != manifest.get("package_sha256")):
        raise ValueError("Invalid package/manifest/size/SHA-256")
    fields = struct.unpack_from("<9I", package, 8)
    _, _, family, profile, size, count, model_offset, golden_offset, total = fields
    if (package[:8] != b"AAI16MP1" or fields[:2] != (1, HEADER) or family != (1 if manifest["family"] == "A" else 2) or
            profile != 1 or not 1 <= size <= MODEL_MAX or size != manifest.get("model_size") or count != 4 or
            model_offset != HEADER or golden_offset != (HEADER + size + 15) & ~15 or total != len(package) or
            total != golden_offset + 4 * GOLDEN or package[136:168].hex() != CONTRACT_SHA or
            package[64:104].rstrip(b"\0") != manifest["version"].encode("ascii") or
            hashlib.sha256(package[HEADER:HEADER + size]).hexdigest() != manifest.get("model_sha256") or
            package[104:136].hex() != manifest.get("model_sha256")):
        raise ValueError("Package metadata disagrees with manifest")
