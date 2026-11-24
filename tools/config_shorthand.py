"""Short config form -> full ConfigApply JSON (laptop side only; the node still receives the full binary Protobuf).

A sensor's "driver" (which bytes to send, how to decode, calibration maths) never changes, so you should not have to
write it. Give only what varies: which sensor, and optionally its period, pins or address.

  {"generation": "auto", "sensors": [
      {"sensor": "gy511",   "period_ms": 100},
      {"sensor": "dht22"},
      {"sensor": "hcsr04",  "trig": 6, "echo": 5},
      {"sensor": "bme280",  "addr": "0x77"},
      {"sensor": "dht22",   "enabled": false} ]}

  {"generation": "auto", "preset": "A"}        # A = dht22 gy511 ld2450 rc522 ; B = hcsr04 bme280 mhz19b max6675

Sensors: gy511 dht22 ld2450 rc522 bme280 hcsr04 mhz19b max6675
Optional per sensor (everything else comes from the built-in driver and WIRING below):
  all:            period_ms, enabled (false = stop the sensor on its bus),
                  dormant (true = stop it but KEEP its profile stored; a rule can start it later)
  gy511, bme280:  addr (e.g. "0x76"), sda, scl, clock_hz
  dht22:          pin
  hcsr04:         trig, echo
  ld2450, mhz19b: port, tx, rx, baud
  rc522, max6675: cs, sclk, mosi, miso, clock_hz, mode
The full form ({"profiles": [...]}) is still accepted unchanged for anything not covered here, and a full-form
profile may say  "sensor": "bme280"  instead of "sensor_id": 5, "bus": 2  (the name fills both in).
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from google.protobuf import json_format  # noqa: E402

import sensor_profiles as sp  # noqa: E402
from profile_lib import load_pb  # noqa: E402

pb = load_pb()

# Names people type -> canonical name
ALIASES = {"gy511": "gy511", "dht22": "dht22", "am2302": "dht22", "ld2450": "ld2450", "rc522": "rc522",
           "bme280": "bme280", "hcsr04": "hcsr04", "mhz19": "mhz19b", "mhz19b": "mhz19b", "max6675": "max6675"}
PRESETS = {"A": ["dht22", "gy511", "ld2450", "rc522"], "B": ["hcsr04", "bme280", "mhz19b", "max6675"]}
# Per-sensor pin overrides for this board. Empty: the built-in profiles in sensor_profiles.py already use the
# project wiring (README.md, Hardware). Add an entry here only if a board is wired differently.
WIRING = {}
ALLOWED = {
    "gy511": {"addr", "sda", "scl", "clock_hz"}, "bme280": {"addr", "sda", "scl", "clock_hz"},
    "dht22": {"pin"}, "hcsr04": {"trig", "echo"},
    "ld2450": {"port", "tx", "rx", "baud"}, "mhz19b": {"port", "tx", "rx", "baud"},
    "rc522": {"cs", "sclk", "mosi", "miso", "clock_hz", "mode"}, "max6675": {"cs", "sclk", "mosi", "miso", "clock_hz", "mode"},
}
COMMON = {"sensor", "period_ms", "enabled", "dormant"}


def _int(v):
    return int(v, 0) if isinstance(v, str) else int(v)


def _canon(name):
    key = str(name).lower().replace("-", "").replace("_", "").replace(" ", "")
    if key not in ALIASES:
        raise ValueError(f"unknown sensor {name!r}; use one of: gy511 dht22 ld2450 rc522 bme280 hcsr04 mhz19b max6675")
    return ALIASES[key]


def build_profile(item):
    """item: {"sensor": name, ...overrides} or just a name -> aai16 SensorProfile message."""
    if isinstance(item, str):
        item = {"sensor": item}
    name = _canon(item.get("sensor", ""))
    unknown = set(item) - COMMON - ALLOWED[name]
    if unknown:
        raise ValueError(f"{name}: unknown setting(s) {sorted(unknown)}; allowed: {sorted((COMMON | ALLOWED[name]) - {'sensor'})}")
    p = {"gy511": sp.gy511, "dht22": sp.dht22, "ld2450": sp.ld2450, "rc522": sp.rc522, "bme280": sp.bme280,
         "hcsr04": sp.hcsr04, "mhz19b": sp.mhz19, "max6675": sp.max6675}[name](pb)
    if item.get("enabled") is False and not item.get("dormant"):   # same shape the node uses for "clear this bus": id + bus + enabled=false
        return pb.SensorProfile(sensor_id=p.sensor_id, bus=p.bus, enabled=False)
    o = {**WIRING.get(name, {}), **{k: v for k, v in item.items() if k not in COMMON}}
    if "period_ms" in item:
        p.period_ms = _int(item["period_ms"])
    if name in ("gy511", "bme280"):
        for k in ("addr", "sda", "scl", "clock_hz"):
            if k in o:
                setattr(p.i2c, k, _int(o[k]))
    elif name in ("ld2450", "mhz19b"):
        for k in ("port", "tx", "rx", "baud"):
            if k in o:
                setattr(p.uart, k, _int(o[k]))
    elif name in ("rc522", "max6675"):
        for k in ("cs", "sclk", "mosi", "miso", "clock_hz", "mode"):
            if k in o:
                setattr(p.spi, k, _int(o[k]))
    elif name == "dht22":
        if "pin" in o:
            old, new = p.gpio.data_pin, _int(o["pin"])
            p.gpio.data_pin = new
            for op in p.ops:
                if op.pin == old:
                    op.pin = new
    elif name == "hcsr04":
        old_t, old_e = p.gpio.data_pin, p.gpio.aux_pin
        new_t, new_e = _int(o.get("trig", old_t)), _int(o.get("echo", old_e))
        p.gpio.data_pin, p.gpio.aux_pin = new_t, new_e
        for op in p.ops:
            if op.type == pb.OP_GPIO_PULSE:
                op.pin = new_t
            elif op.type == pb.OP_GPIO_PULSE_WIDTH:
                op.pin = new_e
    if item.get("dormant"):            # stored but not running: a rule action (RACT_SENSOR_START) can start it
        p.enabled, p.dormant = False, True
    return p


def resolve_names(doc):
    """Full form: a profile may name its sensor ("sensor": "bme280", or "name") instead of giving sensor_id and bus.
    The name fills sensor_id and bus (unless you gave them) and is removed before the message is built."""
    if not isinstance(doc, dict) or not isinstance(doc.get("profiles"), list):
        return doc
    profiles = []
    for prof in doc["profiles"]:
        if isinstance(prof, dict) and any(isinstance(prof.get(k), str) for k in ("sensor", "name")):
            prof = dict(prof)
            label = prof.pop("sensor", None) or prof.pop("name", None)
            prof.pop("name", None)
            ref = build_profile({"sensor": label})
            prof.setdefault("sensor_id", ref.sensor_id)
            prof.setdefault("bus", int(ref.bus))
        profiles.append(prof)
    return {**doc, "profiles": profiles}


def is_short(doc):
    return isinstance(doc, dict) and ("sensors" in doc or "preset" in doc)


def expand(doc):
    """Short form -> full ConfigApply dict (same keys the binary message has). Full-form docs pass through."""
    if not is_short(doc):
        return resolve_names(doc)
    items = []
    if "preset" in doc:
        key = str(doc["preset"]).strip().upper()
        if key not in PRESETS:
            raise ValueError(f"unknown preset {doc['preset']!r}; use A or B")
        items += [{"sensor": s} for s in PRESETS[key]]
    for it in doc.get("sensors", []):
        items.append(it if isinstance(it, dict) else {"sensor": it})
    chosen = {}
    for it in items:                      # a later entry for the same sensor replaces the earlier one
        chosen[_canon(it.get("sensor", ""))] = it
    msg = pb.ConfigApply()
    for it in chosen.values():
        msg.profiles.append(build_profile(it))
    full = json_format.MessageToDict(msg, preserving_proto_field_name=True)
    out = {k: v for k, v in doc.items() if k not in ("sensors", "preset")}
    out["profiles"] = full.get("profiles", [])
    return out
