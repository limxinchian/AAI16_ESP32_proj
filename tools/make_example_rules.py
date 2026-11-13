"""Create example rule / bridge configuration files for node_client.py (binary Protobuf).

  python tools/make_example_rules.py rules  --out rules.bin  --sensor 2 --index 1 --above 30 --text hot
  python tools/make_example_rules.py rules  --out rules.bin  --sensor 2 --index 1 --above 30 --pulse-pin 21 --pulse-ms 500
  python tools/make_example_rules.py bridge --out bridge.bin --remote-node node2 --remote-sensor 2 --local 130 \\
         --kinds humidity_pct,temperature_c
  python tools/node_client.py --broker <ip> --node node1 rules rules.bin      (or: bridge bridge.bin)

A rule fires when `value[index] of the sensor > --above` (or `< --below`); LOG/NOTIFY by default, or a GPIO pulse on a
free output pin (the node refuses pins used by running sensors and the reserved pins). Generation numbers must increase.
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from profile_lib import load_pb  # noqa: E402

pb = load_pb()


def build_rules(gen=1, sensor=2, index=1, above=None, below=None, text="rule", cooldown_ms=1000, pulse_pin=None,
                pulse_ms=500, rule_id=1):
    if (above is None) == (below is None):
        raise ValueError("give exactly one of above / below")
    rs = pb.RuleSetApply(generation=gen)
    r = rs.rules.add(rule_id=rule_id, enabled=True, cooldown_ms=cooldown_ms)
    r.vars.add(sensor_id=sensor, value_index=index, stat=pb.STAT_LATEST, window=1)
    r.when.add(op=pb.CALC_PUSH_REG, reg=0)
    r.when.add(op=pb.CALC_PUSH_CONST, value=float(above if above is not None else below))
    r.when.add(op=pb.CALC_GT if above is not None else pb.CALC_LT)
    if pulse_pin is None:
        r.action.type, r.action.text = pb.RACT_LOG, text
    else:
        r.action.type, r.action.pin, r.action.level, r.action.duration_ms = pb.RACT_GPIO_PULSE, pulse_pin, True, pulse_ms
    return rs


def build_bridge(gen=1, remote_node="nodeA", remote_sensor=2, local=130, kinds=("humidity_pct", "temperature_c"),
                 max_age_ms=5000, period_ms=2000):
    b = pb.BridgeApply(generation=gen)
    s = b.sources.add(local_sensor_id=local, remote_node=remote_node, remote_sensor_id=remote_sensor,
                      max_age_ms=max_age_ms, period_ms=period_ms)
    s.kinds.extend(getattr(pb, "VALUE_" + k.upper()) for k in kinds)
    return b


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="what", required=True)
    r = sub.add_parser("rules")
    r.add_argument("--out", required=True); r.add_argument("--gen", type=int, default=1)
    r.add_argument("--sensor", type=int, required=True); r.add_argument("--index", type=int, default=0)
    r.add_argument("--above", type=float); r.add_argument("--below", type=float)
    r.add_argument("--text", default="rule"); r.add_argument("--cooldown-ms", type=int, default=1000)
    r.add_argument("--pulse-pin", type=int); r.add_argument("--pulse-ms", type=int, default=500)
    b = sub.add_parser("bridge")
    b.add_argument("--out", required=True); b.add_argument("--gen", type=int, default=1)
    b.add_argument("--remote-node", required=True); b.add_argument("--remote-sensor", type=int, required=True)
    b.add_argument("--local", type=int, required=True, help="virtual sensor id on this node, 128..255")
    b.add_argument("--kinds", required=True, help="comma list, e.g. humidity_pct,temperature_c (value kinds of the remote sensor, in order)")
    a = ap.parse_args()
    if a.what == "rules":
        msg = build_rules(a.gen, a.sensor, a.index, a.above, a.below, a.text, a.cooldown_ms, a.pulse_pin, a.pulse_ms)
    else:
        msg = build_bridge(a.gen, a.remote_node, a.remote_sensor, a.local, a.kinds.split(","))
    data = msg.SerializeToString()
    pathlib.Path(a.out).write_bytes(data)
    print(f"wrote {len(data)} bytes to {a.out}")


if __name__ == "__main__":
    main()
