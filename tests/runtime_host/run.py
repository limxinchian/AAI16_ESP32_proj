"""Host tests for sensor_runtime: ConfigApply (Python-encoded) -> config_manager -> executor/scheduler ->
SIMULATED devices -> Readings. Expected values come from independent Python code (Bosch BME280 reference).

Run:  .venv\\Scripts\\python.exe tests\\runtime_host\\run.py
"""
import importlib.util
import pathlib
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
sys.path.insert(0, str(ROOT / "tools"))
import sensor_profiles as sp  # noqa: E402
from profile_lib import load_pb  # noqa: E402

# reuse the reference BME280 maths + byte builders from the decode tests (module import runs no tests)
spec = importlib.util.spec_from_file_location("decode_run", ROOT / "tests" / "decode_host" / "run.py")
dh = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dh)


def main():
    pb = load_pb()
    OUT.mkdir(exist_ok=True)
    (OUT / "config_a.bin").write_bytes(sp.config_a(pb, 1).SerializeToString())
    swap = pb.ConfigApply(generation=2)
    swap.profiles.append(sp.bme280(pb))
    (OUT / "swap_bme280.bin").write_bytes(swap.SerializeToString())
    (OUT / "config_b.bin").write_bytes(sp.config_b(pb, 3).SerializeToString())

    # BME280 register file: calibration @0x88 (24) + H1 @0xA1 + E1..E7 @0xE1 + data @0xF7
    c, adc_t, adc_p, adc_h = dh.BOSCH, 519888, 415148, 30000
    init = dh.bme_init_bytes(c)
    regs = bytearray(256)
    regs[0x88:0x88 + 24] = init[:24]
    regs[0xA1] = init[24]
    regs[0xE1:0xE1 + 7] = init[25:32]
    regs[0xF7:0xF7 + 8] = dh.bme_cycle_bytes(adc_t, adc_p, adc_h)
    (OUT / "bme_regs.bin").write_bytes(bytes(regs))
    t, p, h = dh.bme_ref(c, adc_t, adc_p, adc_h)
    (OUT / "expect.txt").write_text(f"{t!r} {p!r} {h!r}\n")
    print(f"expected BME280: {t:.4f} C, {p:.2f} Pa, {h:.4f} %RH")

    exe = OUT / "test_runtime.exe"
    comps = ROOT / "components"
    srcs = [HERE / "test_runtime.c", comps / "sensor_runtime/sensor_runtime.c", comps / "sensor_decode/sensor_decode.c",
            comps / "config_manager/config_manager.c", comps / "aai16_protocol/generated/aai16.pb.c",
            comps / "aai16_protocol/nanopb/pb_common.c", comps / "aai16_protocol/nanopb/pb_decode.c",
            comps / "aai16_protocol/nanopb/pb_encode.c"]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{comps / 'sensor_runtime/include'}", f"-I{comps / 'sensor_decode/include'}",
           f"-I{comps / 'config_manager/include'}", f"-I{comps / 'aai16_protocol/generated'}",
           f"-I{comps / 'aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs)]
    subprocess.run(cmd, check=True)
    sys.exit(subprocess.run([str(exe), str(OUT)]).returncode)


if __name__ == "__main__":
    main()
