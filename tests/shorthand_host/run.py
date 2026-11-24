"""Host test: the short config form expands to exactly the full profiles (no board needed).

Run:  .venv\\Scripts\\python.exe tests\\shorthand_host\\run.py
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import config_shorthand as cs  # noqa: E402
import sensor_profiles as sp  # noqa: E402
from google.protobuf import json_format  # noqa: E402

pb = cs.pb
fails = 0


def check(name, cond, extra=""):
    global fails
    fails += 0 if cond else 1
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {extra}")


def full(doc):
    return json_format.ParseDict(cs.expand(doc), pb.ConfigApply())


def one(item):
    return full({"sensors": [item]}).profiles[0]


# 1) a bare sensor name gives exactly the built-in profile (which already uses this project's wiring)
REF = {"gy511": sp.gy511, "dht22": sp.dht22, "ld2450": sp.ld2450, "rc522": sp.rc522, "bme280": sp.bme280,
       "hcsr04": sp.hcsr04, "mhz19b": sp.mhz19, "max6675": sp.max6675}
for name, make in REF.items():
    want = make(pb).SerializeToString()
    check(f"{name}: bare name == built-in profile", one(name).SerializeToString() == want)
    check(f"{name}: dict form identical", one({"sensor": name}).SerializeToString() == want)

# 2) overrides
check("period_ms", one({"sensor": "gy511", "period_ms": 250}).period_ms == 250)
check("i2c addr as hex string", one({"sensor": "bme280", "addr": "0x77"}).i2c.addr == 0x77)
check("dht22 pin moves the data pin AND the op pins", (lambda p: p.gpio.data_pin == 7 and all(o.pin == 7 for o in p.ops))(one({"sensor": "dht22", "pin": 7})))
p = one({"sensor": "hcsr04", "trig": 7, "echo": 8})
check("hcsr04 trig/echo move pins and ops", p.gpio.data_pin == 7 and p.gpio.aux_pin == 8 and p.ops[0].pin == 7 and p.ops[1].pin == 8)
p = one({"sensor": "mhz19b", "baud": 19200, "tx": 15, "rx": 16, "port": 2})
check("uart settings", (p.uart.baud, p.uart.tx, p.uart.rx, p.uart.port) == (19200, 15, 16, 2))
check("spi cs", one({"sensor": "max6675", "cs": 9}).spi.cs == 9)
p = one({"sensor": "gy511", "dormant": True})
check("dormant keeps the complete profile, stopped", p.dormant is True and p.enabled is False and len(p.ops) > 0 and p.sensor_id == 1)
p = one({"sensor": "dht22", "enabled": False, "dormant": True})
check("dormant wins over enabled=false", p.dormant is True and p.enabled is False and len(p.ops) > 0)
p = one({"sensor": "dht22", "enabled": False})
check("enabled=false gives the minimal 'stop this bus' profile", p.enabled is False and p.sensor_id == 2 and len(p.ops) == 0)

# 3) presets and replace-by-sensor
check("preset A = 4 sensors", [x.sensor_id for x in full({"preset": "A"}).profiles] == [2, 1, 3, 4])
check("preset B = 4 sensors", [x.sensor_id for x in full({"preset": "B"}).profiles] == [6, 5, 7, 8])
m = full({"preset": "A", "sensors": [{"sensor": "gy511", "period_ms": 100}]})
check("a later entry replaces the preset's", [x.period_ms for x in m.profiles if x.sensor_id == 1] == [100] and len(m.profiles) == 4)
check("generation and other keys survive", cs.expand({"generation": "auto", "preset": "A"})["generation"] == "auto")
check("full form passes through unchanged", cs.expand({"generation": 5, "profiles": []}) == {"generation": 5, "profiles": []})

# 3b) full form with the sensor NAME instead of sensor_id + bus
d = cs.expand({"generation": "auto", "profiles": [{"sensor": "bme280", "enabled": False}]})
check("name fills sensor_id and bus", d["profiles"][0] == {"sensor_id": 5, "bus": 2, "enabled": False})
d = cs.expand({"profiles": [{"name": "MH-Z19B", "period_ms": 1000}]})
check("'name' works, any spelling", d["profiles"][0]["sensor_id"] == 7 and d["profiles"][0]["bus"] == 3 and "name" not in d["profiles"][0])
d = cs.expand({"profiles": [{"sensor": "dht22", "sensor_id": 2, "bus": 1}]})
check("explicit ids are kept", d["profiles"][0] == {"sensor_id": 2, "bus": 1})
try:
    cs.expand({"profiles": [{"sensor": "nope"}]}); check("unknown name in a full profile refused", False)
except ValueError:
    check("unknown name in a full profile refused", True)

# 4) bad input is refused with a clear message
for bad, why in (({"sensors": [{"sensor": "nope"}]}, "unknown sensor"), ({"sensors": [{"sensor": "dht22", "trig": 4}]}, "setting not valid for the sensor"),
                 ({"preset": "C"}, "unknown preset")):
    try:
        cs.expand(bad); check(why + " refused", False)
    except ValueError as e:
        check(why + " refused", True, f"({e})"[:90])

print(f"\n{'ALL SHORTHAND CHECKS PASS' if not fails else str(fails) + ' FAILED'}")
sys.exit(1 if fails else 0)
