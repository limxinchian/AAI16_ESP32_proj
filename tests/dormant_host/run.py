"""Host test for dormant sensors + rule actions RACT_SENSOR_START / RACT_SENSOR_STOP.

A full node runs on simulated devices (GY-511 on I2C, DHT22 on GPIO). Python builds the fixtures with the REAL
profile builders (tools/sensor_profiles.py); the C test (test_dormant.c) drives the node exactly like MQTT would:
configs, rules, taps (readings fed to the rule engine), reboots, CLEAR.

Run:  .venv\\Scripts\\python.exe tests\\dormant_host\\run.py
"""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
sys.path.insert(0, str(ROOT / "tools"))
import node_client as nc  # noqa: E402
import sensor_profiles as sp  # noqa: E402

pb = nc.pb


def cfg(gen, profiles):
    m = pb.ConfigApply(generation=gen)
    m.profiles.extend(profiles)
    return m.SerializeToString()


def dormant(p):
    p.enabled = False
    p.dormant = True
    return p


def rule(rid, sensor, action_type, sensor_id, duration_ms=0, cooldown=1000):
    r = pb.Rule(rule_id=rid, enabled=True, cooldown_ms=cooldown, edge_triggered=True)
    r.vars.add(sensor_id=sensor, value_index=0, stat=pb.STAT_LATEST, window=1)
    r.when.add(op=pb.CALC_PUSH_REG, reg=0)
    r.when.add(op=pb.CALC_PUSH_CONST, value=0.5)
    r.when.add(op=pb.CALC_GT)
    r.action.type = action_type
    r.action.sensor_id = sensor_id
    r.action.duration_ms = duration_ms
    return r


def rules(gen, rs):
    m = pb.RuleSetApply(generation=gen)
    m.rules.extend(rs)
    return m.SerializeToString()


def main():
    OUT.mkdir(exist_ok=True)
    dht = sp.dht22(pb)
    files = {}
    files["cfg_dormant.bin"] = cfg(1, [sp.dht22(pb), dormant(sp.gy511(pb))])
    bad = sp.gy511(pb); bad.dormant = True                       # enabled AND dormant
    files["cfg_both.bin"] = cfg(2, [sp.dht22(pb), bad])
    clash = dormant(sp.gy511(pb)); clash.i2c.sda = dht.gpio.data_pin   # dormant sensor shares a pin with the running DHT22
    files["cfg_pinclash.bin"] = cfg(2, [sp.dht22(pb), clash])
    files["cfg_enable_gy.bin"] = cfg(10, [sp.gy511(pb)])
    off = pb.SensorProfile(sensor_id=1, bus=pb.BUS_I2C, enabled=False)
    files["cfg_off_i2c.bin"] = cfg(11, [off])
    files["cfg_dormant_again.bin"] = cfg(12, [dormant(sp.gy511(pb))])

    S, X = pb.RACT_SENSOR_START, pb.RACT_SENSOR_STOP
    files["rules_ok.bin"] = rules(1, [
        rule(1, 4, S, 1, 5000),      # a tag tap (sensor 4 value > 0.5): wake the GY-511 for 5 s
        rule(2, 5, X, 1),            # "stop button"
        rule(3, 6, S, 3, 1000),      # unknown sensor: refused at fire time
        rule(4, 7, X, 2),            # stop a configured (never woken) sensor: refused
        rule(5, 8, S, 2),            # start a sensor that is already running by configuration: refused
    ])
    files["rules_bad_zero.bin"] = rules(2, [rule(1, 4, S, 0, 1000)])
    files["rules_bad_virtual.bin"] = rules(2, [rule(1, 4, S, 200, 1000)])
    files["rules_bad_long.bin"] = rules(2, [rule(1, 4, S, 1, 3600001)])
    files["rules_bad_stopdur.bin"] = rules(2, [rule(1, 4, X, 1, 500)])
    files["rules_edge_ok.bin"] = rules(2, [rule(1, 4, S, 1, 3600000), rule(2, 5, X, 127)])
    for name, data in files.items():
        (OUT / name).write_bytes(data)

    exe = OUT / "test_dormant.exe"
    comps = ROOT / "components"
    srcs = [HERE / "test_dormant.c", comps / "node_core/node_core.c", comps / "sensor_runtime/sensor_runtime.c",
            comps / "sensor_decode/sensor_decode.c", comps / "config_manager/config_manager.c",
            *[comps / "node_services" / n for n in ("rule_engine.c", "bridge.c", "diag.c", "repl.c")],
            *[comps / "ml_core" / n for n in ("sha256.c", "ml_features.c", "ml_package.c", "ml_selector.c", "model_store.c")],
            comps / "aai16_protocol/generated/aai16.pb.c", *[comps / "aai16_protocol/nanopb" / n for n in ("pb_common.c", "pb_decode.c", "pb_encode.c")]]
    incs = [comps / n / "include" for n in ("node_core", "node_services", "sensor_runtime", "sensor_decode", "ml_core", "config_manager")]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g", *[f"-I{i}" for i in incs],
           f"-I{comps / 'aai16_protocol/generated'}", f"-I{comps / 'aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs), "-lm"]
    subprocess.run(cmd, check=True)
    sys.exit(subprocess.run([str(exe), str(OUT)]).returncode)


if __name__ == "__main__":
    main()
