"""Host test driver for config_manager (no ESP-IDF needed).

1. regenerates the nanopb C code and the independent Python protobuf module,
2. writes fixture payloads with the *Python* protobuf encoder (so the C decoder
   is checked against a second implementation, not against itself),
3. builds test_config.c with `zig cc` and runs it,
4. parses the ACK bytes produced by C with the Python decoder.

Sensor profiles come from tools/sensor_profiles.py (configuration data, unverified on hardware).

Run:  .venv\\Scripts\\python.exe tests\\config_host\\run.py
"""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
GEN = HERE / "_gen"
sys.path.insert(0, str(ROOT / "tools"))
import sensor_profiles as sp  # noqa: E402
from profile_lib import load_pb  # noqa: E402


def sh(cmd, **kw):
    r = subprocess.run(cmd, **kw)
    if r.returncode != 0:
        sys.exit(f"command failed ({r.returncode}): {' '.join(map(str, cmd))}")


def write_fixtures(pb):
    GEN.mkdir(exist_ok=True)

    def dump(name, msg):
        (GEN / name).write_bytes(msg.SerializeToString())

    dump("config_a.bin", sp.config_a(pb, 1))

    swap = pb.ConfigApply(generation=2)
    swap.profiles.append(sp.bme280(pb))
    dump("swap_i2c_bme280.bin", swap)

    dump("config_b.bin", sp.config_b(pb, 3))

    over = sp.config_a(pb, 9)  # 5 profiles > max 4
    over.profiles.append(sp.bme280(pb))
    dump("too_many.bin", over)

    long_tx = pb.ConfigApply(generation=9)
    p = sp.gy511(pb)
    p.ops[0].tx = bytes(17)  # 17 > max 16
    long_tx.profiles.append(p)
    dump("tx_too_long.bin", long_tx)

    bad_pin = pb.ConfigApply(generation=9)
    p = sp.gy511(pb)
    p.i2c.sda = 26  # SPI flash pin
    bad_pin.profiles.append(p)
    dump("bad_pin.bin", bad_pin)

    raw = (GEN / "config_a.bin").read_bytes()
    (GEN / "truncated.bin").write_bytes(raw[: len(raw) // 2])
    (GEN / "garbage.bin").write_bytes(bytes([0xFF] * 40))
    print(f"config A: {len(raw)} bytes encoded; config B: {len((GEN / 'config_b.bin').read_bytes())} bytes encoded")


def verify_acks(pb):
    def load(name):
        m = pb.ApplyAck()
        m.ParseFromString((GEN / name).read_bytes())
        return m

    ok = load("ack_applied.bin")
    assert (ok.generation, ok.phase, ok.reason) == (1, pb.PHASE_APPLIED, pb.REASON_NONE), ok
    rej = load("ack_rejected.bin")
    assert (rej.generation, rej.phase, rej.reason) == (9, pb.PHASE_REJECTED, pb.REASON_FORBIDDEN_PIN), rej
    assert (rej.sensor_id, rej.profile_index, rej.detail) == (1, 0, 26), rej
    print("python decoder agrees with C-encoded ACKs")


def main():
    sh([sys.executable, str(ROOT / "tools" / "generate_proto.py")], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    pb = load_pb()
    write_fixtures(pb)

    exe = GEN / "test_config.exe"
    srcs = [
        HERE / "test_config.c",
        ROOT / "components/config_manager/config_manager.c",
        ROOT / "components/aai16_protocol/generated/aai16.pb.c",
        ROOT / "components/aai16_protocol/nanopb/pb_common.c",
        ROOT / "components/aai16_protocol/nanopb/pb_decode.c",
        ROOT / "components/aai16_protocol/nanopb/pb_encode.c",
    ]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{ROOT / 'components/config_manager/include'}",
           f"-I{ROOT / 'components/aai16_protocol/generated'}",
           f"-I{ROOT / 'components/aai16_protocol/nanopb'}",
           "-o", str(exe), *map(str, srcs)]
    sh(cmd)
    sh([str(exe), str(GEN)])
    verify_acks(pb)


if __name__ == "__main__":
    main()
