"""Generate/check the shared C/Python protocol without changing vendor sources."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "components/aai16_protocol/vendor/nanopb"
SCHEMA = ROOT / "protocol"


def generate(check=False):
    for name, version in (("grpcio-tools", "1.71.2"), ("protobuf", "5.29.5")):
        if importlib.metadata.version(name) != version:
            raise RuntimeError(f"Install tools/protocol/requirements.txt: {name} must be {version}")
    upstream = json.loads((VENDOR / "UPSTREAM.json").read_text())
    for name, digest in upstream["files"].items():
        if hashlib.sha256((VENDOR / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f"Modified pinned nanopb source: {name}")
    with tempfile.TemporaryDirectory(prefix="aai16-protocol-") as directory:
        tmp = Path(directory)
        descriptor = tmp / "aai16.pb"
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{SCHEMA}",
                        f"--python_out={tmp}", f"--descriptor_set_out={descriptor}",
                        "aai16.proto"], check=True)
        env = dict(os.environ, NANOPB_PB2_TEMP_DIR=str(tmp), PYTHONDONTWRITEBYTECODE="1")
        generated = subprocess.run([sys.executable, str(VENDOR / "generator/nanopb_generator.py"),
                                    "-I", str(SCHEMA), "-D", str(tmp), str(descriptor)],
                                   capture_output=True, text=True, env=env)
        if generated.returncode:
            raise RuntimeError(generated.stdout + generated.stderr)
        outputs = {
            "aai16.pb.c": ROOT / "components/aai16_protocol/generated/aai16.pb.c",
            "aai16.pb.h": ROOT / "components/aai16_protocol/generated/aai16.pb.h",
            "aai16_pb2.py": ROOT / "tools/protocol/generated/aai16_pb2.py",
        }
        for name, destination in outputs.items():
            raw = (tmp / name).read_bytes().replace(b"\r\n", b"\n")
            if check:
                if not destination.exists() or destination.read_bytes().replace(b"\r\n", b"\n") != raw:
                    raise RuntimeError(f"Generated file is stale: {destination.relative_to(ROOT)}")
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(raw)
        print("Protocol generation verified" if check else "Generated shared nanopb C and Python bindings")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    generate(parser.parse_args().check)
