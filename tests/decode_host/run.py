"""Host tests for configuration-driven decoding (no ESP-IDF needed).

Python builds profiles (tools/sensor_profiles.py) and raw sensor bytes, computes the expected
result with INDEPENDENT reference code (Bosch BME280 formulas in plain Python, hand-computed
protocol examples), the C decoder runs the same profile + bytes, and the two are compared.

Run:  .venv\\Scripts\\python.exe tests\\decode_host\\run.py
"""
import math
import pathlib
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools"))
import sensor_profiles as sp  # noqa: E402
from profile_lib import load_pb  # noqa: E402

pb = load_pb()
OUT = HERE / "_gen"
CASES = []  # (name, profile, init, cycle, expect)


def case(name, profile, init=b"", cycle=b"", expect=None):
    CASES.append((name, profile, bytes(init), bytes(cycle), expect))


def ok(*vals, rtol=2e-6, atol=1e-6):
    return ("ok", list(vals), rtol, atol)


# --------------------------------------------------------------- BME280 reference
# Straight from the Bosch BME280 datasheet floating-point reference, typed independently of
# the infix strings in sensor_profiles.py.
def bme_ref(c, adc_t, adc_p, adc_h):
    v1 = (adc_t / 16384.0 - c["T1"] / 1024.0) * c["T2"]
    v2 = (adc_t / 131072.0 - c["T1"] / 8192.0) * (adc_t / 131072.0 - c["T1"] / 8192.0) * c["T3"]
    t_fine = v1 + v2
    temp = t_fine / 5120.0

    v1 = t_fine / 2.0 - 64000.0
    v2 = v1 * v1 * c["P6"] / 32768.0
    v2 = v2 + v1 * c["P5"] * 2.0
    v2 = v2 / 4.0 + c["P4"] * 65536.0
    v1 = (c["P3"] * v1 * v1 / 524288.0 + c["P2"] * v1) / 524288.0
    v1 = (1.0 + v1 / 32768.0) * c["P1"]
    if v1 == 0.0:
        return temp, None, None
    p = 1048576.0 - adc_p
    p = (p - v2 / 4096.0) * 6250.0 / v1
    v1 = c["P9"] * p * p / 2147483648.0
    v2 = p * c["P8"] / 32768.0
    p = p + (v1 + v2 + c["P7"]) / 16.0

    h = t_fine - 76800.0
    h = (adc_h - (c["H4"] * 64.0 + c["H5"] / 16384.0 * h)) * (
        c["H2"] / 65536.0 * (1.0 + c["H6"] / 67108864.0 * h * (1.0 + c["H3"] / 67108864.0 * h)))
    h = h * (1.0 - c["H1"] * h / 524288.0)
    h = min(100.0, max(0.0, h))
    return temp, p, h


BOSCH = dict(T1=27504, T2=26435, T3=-1000, P1=36477, P2=-10685, P3=3024, P4=2855, P5=140,
             P6=-7, P7=15500, P8=-14600, P9=6000, H1=75, H2=362, H3=0, H4=313, H5=50, H6=30)


def bme_init_bytes(c):
    cal = struct.pack("<HhhHhhhhhhhh", c["T1"], c["T2"], c["T3"], c["P1"], c["P2"], c["P3"],
                      c["P4"], c["P5"], c["P6"], c["P7"], c["P8"], c["P9"])
    e4, e6 = (c["H4"] >> 4) & 0xFF, (c["H5"] >> 4) & 0xFF
    e5 = (c["H4"] & 0xF) | ((c["H5"] & 0xF) << 4)
    return cal + bytes([c["H1"]]) + struct.pack("<h", c["H2"]) + bytes([c["H3"], e4, e5, e6, c["H6"] & 0xFF])


def bme_cycle_bytes(adc_t, adc_p, adc_h):
    return (adc_p << 4).to_bytes(3, "big") + (adc_t << 4).to_bytes(3, "big") + adc_h.to_bytes(2, "big")


def bme_case(name, c, adc_t, adc_p, adc_h):
    t, p, h = bme_ref(c, adc_t, adc_p, adc_h)
    expect = ok(t, p, h, rtol=1e-6, atol=1e-4) if p is not None else ("err", "CALC")
    case(name, sp.bme280(pb), bme_init_bytes(c), bme_cycle_bytes(adc_t, adc_p, adc_h), expect)


def build_cases():
    # BME280: Bosch's published example (BMP280 datasheet, same T/P maths) => 25.08 C, 100653.27 Pa
    t, p, _ = bme_ref(BOSCH, 519888, 415148, 30000)
    assert abs(t - 25.08) < 0.01 and abs(p - 100653.27) < 0.05, (t, p)  # validates the reference itself
    bme_case("bme280 datasheet example", BOSCH, 519888, 415148, 30000)
    bme_case("bme280 cold + low pressure", BOSCH, 470000, 380000, 41000)
    # negative H4/H5/H6 exercise the int8*16 + nibble assembly; pick an ADC value whose
    # humidity is NOT clamped, otherwise a decode bug would be hidden by the clamp
    neg = dict(BOSCH, H4=-100, H5=-37, H6=-12)
    adc_h = next(a for a in range(0, 65000, 50) if 5 < bme_ref(neg, 519888, 415148, a)[2] < 95)
    bme_case("bme280 negative H4/H5/H6 (unclamped)", neg, 519888, 415148, adc_h)
    other = dict(BOSCH, H4=-300, H5=1900, H6=127, H3=200)
    adc_h = next(a for a in range(0, 65000, 50) if 5 < bme_ref(other, 519888, 415148, a)[2] < 95)
    bme_case("bme280 extreme H coefficients (unclamped)", other, 519888, 415148, adc_h)
    bme_case("bme280 humidity clamps to 100", BOSCH, 519888, 415148, 65000)
    bme_case("bme280 humidity clamps to 0", BOSCH, 519888, 415148, 1000)
    bme_case("bme280 P1=0 -> divide by zero", dict(BOSCH, P1=0), 519888, 415148, 30000)

    # DHT22: 65.2 %RH, temperatures with sign-magnitude
    def dht(h10, t_raw):
        b = [h10 >> 8, h10 & 255, t_raw >> 8, t_raw & 255]
        return bytes(b + [sum(b) & 255])
    case("dht22 +25.3C", sp.dht22(pb), cycle=dht(652, 253), expect=ok(65.2, 25.3))
    case("dht22 -10.1C (sign-magnitude)", sp.dht22(pb), cycle=dht(652, 0x8000 | 101), expect=ok(65.2, -10.1))
    case("dht22 bad checksum", sp.dht22(pb), cycle=dht(652, 253)[:4] + b"\x00", expect=("err", "BAD_FRAME"))

    # GY-511: 12-bit left-justified two's complement, 1 mg/LSB
    def lj(mg):
        return struct.pack("<H", ((mg & 0xFFF) << 4) & 0xFFFF)
    case("gy511 (1.000, -0.500, 1.020 g)", sp.gy511(pb), cycle=lj(1000) + lj(-500) + lj(1020),
         expect=ok(1.0, -0.5, 1.02))
    case("gy511 full-scale negative", sp.gy511(pb), cycle=lj(-2048) + lj(2047) + lj(0), expect=ok(-2.048, 2.047, 0.0))

    # LD2450: "top bit set = positive" sign convention; datasheet-style example x = -782 mm
    def ld(x, y, v):
        enc = lambda n: struct.pack("<H", (0x8000 | n) if n >= 0 else (-n))
        frame = bytes([0xAA, 0xFF, 0x03, 0x00]) + enc(x) + enc(y) + enc(v) + bytes(18) + bytes([0x55, 0xCC])
        assert len(frame) == 30
        return frame
    case("ld2450 (-782, 1713, -16)", sp.ld2450(pb), cycle=ld(-782, 1713, -16), expect=ok(-782, 1713, -16))
    case("ld2450 zero target", sp.ld2450(pb), cycle=ld(0, 0, 0), expect=ok(0, 0, 0))

    # MH-Z19: reply FF 86 hi lo ... checksum
    def mhz(ppm, bad=False):
        b = [0xFF, 0x86, ppm >> 8, ppm & 255, 0, 0, 0, 0]
        chk = (0x100 - (sum(b[1:8]) & 255)) & 255
        return bytes(b + [chk ^ (1 if bad else 0)])
    case("mhz19 400 ppm", sp.mhz19(pb), cycle=mhz(400), expect=ok(400))
    case("mhz19 1234 ppm", sp.mhz19(pb), cycle=mhz(1234), expect=ok(1234))
    case("mhz19 bad checksum", sp.mhz19(pb), cycle=mhz(400, bad=True), expect=("err", "BAD_FRAME"))
    case("mhz19 short reply", sp.mhz19(pb), cycle=mhz(400)[:5], expect=("err", "BAD_FRAME"))

    case("rc522 VersionReg 0x92", sp.rc522_version(pb), cycle=bytes([0x00, 0x92]), expect=ok(0x92))
    # rc522 card detection: raw = [dummy, FIFOLevel, dummy, ATQA lo, ATQA hi]
    case("rc522 card present (MIFARE 1K ATQA 0x0004)", sp.rc522(pb), cycle=bytes([0, 2, 0, 0x04, 0x00]), expect=ok(1, 4))
    case("rc522 card present (NTAG ATQA 0x0044)", sp.rc522(pb), cycle=bytes([0, 2, 0, 0x44, 0x00]), expect=ok(1, 0x44))
    case("rc522 echo of our own request (1 byte 0x26) is NOT a card", sp.rc522(pb), cycle=bytes([0, 1, 0, 0x26, 0xDD]), expect=ok(0, 0))
    case("rc522 three bytes in the FIFO is NOT a card", sp.rc522(pb), cycle=bytes([0, 3, 0, 0x04, 0x00]), expect=ok(0, 0))
    case("rc522 no card: FIFO garbage is masked", sp.rc522(pb), cycle=bytes([0, 0, 0, 0x55, 0x55]), expect=ok(0, 0))
    case("max6675 25.00C (status bits ignored)", sp.max6675(pb), cycle=struct.pack(">H", (100 << 3) | 1), expect=ok(25.0))
    case("max6675 full scale", sp.max6675(pb), cycle=struct.pack(">H", 0x7FF8), expect=ok(1023.75))
    case("hcsr04 5831 us", sp.hcsr04(pb), cycle=struct.pack(">I", 5831), expect=ok(5831 * 0.01715))

    # ---- admission-time rejection of broken decode configuration ----
    R = pb.Reason
    def bad(name, base, mutate, reason):
        p = base(pb)
        mutate(p)
        case("INVALID: " + name, p, expect=("invalid", R.Name(reason)))

    def step(p, dest, *toks):
        s = p.calc.add(); s.dest_reg, s.token_count = dest, len(toks)
        for op, kw in toks:
            t = p.program.add(); t.op = op
            for k, v in kw.items():
                setattr(t, k, v)
    C = lambda v: (pb.CALC_PUSH_CONST, {"value": v})
    G = lambda r: (pb.CALC_PUSH_REG, {"reg": r})
    O = lambda op: (op, {})

    bad("calc stack underflow", sp.mhz19, lambda p: step(p, 5, O(pb.CALC_ADD)), R.REASON_BAD_CALC)
    bad("calc leaves two values", sp.mhz19, lambda p: step(p, 5, C(1), C(2)), R.REASON_BAD_CALC)
    bad("calc reads undefined register", sp.mhz19, lambda p: step(p, 5, G(9)), R.REASON_BAD_CALC)
    bad("calc stack depth > 8", sp.mhz19, lambda p: step(p, 5, *([C(1)] * 9 + [O(pb.CALC_ADD)] * 8)), R.REASON_BAD_CALC)
    bad("calc NOP token", sp.mhz19, lambda p: step(p, 5, O(pb.CALC_NOP)), R.REASON_BAD_CALC)
    bad("calc orphan tokens", sp.mhz19, lambda p: p.program.add(op=pb.CALC_PUSH_CONST, value=1), R.REASON_BAD_CALC)
    bad("calc bad destination", sp.mhz19, lambda p: step(p, 32, C(1)), R.REASON_BAD_CALC)
    bad("output register never written", sp.mhz19, lambda p: setattr(p, "output_count", 2), R.REASON_BAD_OUTPUT)
    bad("output_count 9", sp.mhz19, lambda p: setattr(p, "output_count", 9), R.REASON_BAD_OUTPUT)
    bad("shift >= field bits", sp.mhz19, lambda p: setattr(p.decode[0], "shift", 16), R.REASON_BAD_DECODE)
    bad("value_bits beyond shifted width", sp.max6675, lambda p: setattr(p.decode[0], "value_bits", 14), R.REASON_BAD_DECODE)
    bad("1-bit sign-magnitude", sp.max6675, lambda p: (setattr(p.decode[0], "value_bits", 1), setattr(p.decode[0], "sign", pb.SIGN_MAGNITUDE)), R.REASON_BAD_DECODE)
    bad("width 5", sp.max6675, lambda p: setattr(p.decode[0], "width", 5), R.REASON_BAD_DECODE)
    bad("register written by two decode fields", sp.dht22, lambda p: setattr(p.decode[1], "dest_reg", 0), R.REASON_BAD_DECODE)
    bad("unknown sign mode", sp.max6675, lambda p: setattr(p.decode[0], "sign", 9), R.REASON_BAD_DECODE)
    bad("init decode without init data", sp.max6675, lambda p: setattr(p.decode[0], "source", pb.SRC_INIT), R.REASON_BAD_INIT)
    bad("init op with read_len 0", sp.gy511, lambda p: setattr(p.init_ops[0], "type", pb.OP_READ), R.REASON_BAD_INIT)
    bad("frame check beyond buffer", sp.mhz19, lambda p: setattr(p.checks[0], "length", 20), R.REASON_BAD_CHECK)
    bad("frame check unknown kind", sp.mhz19, lambda p: setattr(p.checks[0], "kind", 9), R.REASON_BAD_CHECK)
    bad("sync read shorter than its header", sp.ld2450, lambda p: setattr(p.ops[0], "read_len", 3), R.REASON_BAD_LENGTH)
    bad("sync read without timeout", sp.ld2450, lambda p: setattr(p.ops[0], "timeout_us", 0), R.REASON_BAD_TIMEOUT)
    bad("sync read on I2C", sp.gy511, lambda p: (setattr(p.ops[0], "type", pb.OP_UART_SYNC_READ)), R.REASON_OP_NOT_ALLOWED_ON_BUS)


def write_cases():
    OUT.mkdir(exist_ok=True)
    blob = bytearray(struct.pack("<I", len(CASES)))
    for _, prof, init, cyc, _ in CASES:
        for part in (prof.SerializeToString(), init, cyc):
            blob += struct.pack("<I", len(part)) + part
    (OUT / "cases.bin").write_bytes(blob)


def build_and_run():
    exe = OUT / "test_decode.exe"
    srcs = [HERE / "test_decode.c", ROOT / "components/sensor_decode/sensor_decode.c",
            ROOT / "components/config_manager/config_manager.c",
            ROOT / "components/aai16_protocol/generated/aai16.pb.c",
            ROOT / "components/aai16_protocol/nanopb/pb_common.c",
            ROOT / "components/aai16_protocol/nanopb/pb_decode.c",
            ROOT / "components/aai16_protocol/nanopb/pb_encode.c"]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{ROOT / 'components/sensor_decode/include'}", f"-I{ROOT / 'components/config_manager/include'}",
           f"-I{ROOT / 'components/aai16_protocol/generated'}", f"-I{ROOT / 'components/aai16_protocol/nanopb'}",
           "-o", str(exe), *map(str, srcs)]
    subprocess.run(cmd, check=True)
    r = subprocess.run([str(exe), str(OUT / "cases.bin")], capture_output=True, text=True, check=True)
    return r.stdout.splitlines()


def judge(lines):
    assert len(lines) == len(CASES), f"{len(lines)} result lines for {len(CASES)} cases"
    failures = 0
    for (name, _, _, _, expect), line in zip(CASES, lines):
        kind = expect[0]
        good = False
        if kind == "ok":
            parts = line.split()
            if parts[:1] == ["OK"] and int(parts[1]) == len(expect[1]):
                got = [float(x) for x in parts[2:]]
                good = all(math.isclose(g, e, rel_tol=expect[2], abs_tol=expect[3]) for g, e in zip(got, expect[1]))
        elif kind == "err":
            good = line == "ERR " + expect[1]
        elif kind == "invalid":
            parts = line.split()
            good = parts[0] == "INVALID" and pb.Reason.Name(int(parts[1])) == expect[1]
        print(("  ok   " if good else "  FAIL ") + f"{name:46s} {line}")
        failures += not good
    return failures


def main():
    subprocess.run([sys.executable, str(ROOT / "tools" / "generate_proto.py")],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    build_cases()
    write_cases()
    bme = sp.bme280(pb)
    print(f"bme280 profile: {len(bme.program)} program tokens (max 256), {len(bme.calc)} calc steps (max 32), "
          f"{len(bme.decode)} decode fields (max 32), {len(bme.SerializeToString())} bytes encoded")
    failures = judge(build_and_run())
    print(f"\n{len(CASES)} cases, {failures} failures")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
