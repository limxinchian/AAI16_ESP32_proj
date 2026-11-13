"""Host tests for node_services: rule engine, bridge, heartbeat, REPL.
Fixtures are encoded by Python's independent protobuf; the heartbeat produced by C is decoded in Python.

Run:  .venv\\Scripts\\python.exe tests\\services_host\\run.py
"""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
sys.path.insert(0, str(ROOT / "tools"))
from profile_lib import load_pb  # noqa: E402


def main():
    subprocess.run([sys.executable, str(ROOT / "tools" / "generate_proto.py")], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=True)
    pb = load_pb()
    OUT.mkdir(exist_ok=True)

    # fixtures come from tools/make_example_rules.py, so the firmware's own validators check what that tool produces
    import make_example_rules as mer
    (OUT / "rules_ok.bin").write_bytes(mer.build_rules(gen=1, sensor=2, index=1, above=30.0, text="hot", cooldown_ms=1000).SerializeToString())
    (OUT / "bridge_ok.bin").write_bytes(mer.build_bridge(gen=1, remote_node="nodeA", remote_sensor=2, local=130,
                                                          kinds=("humidity_pct", "temperature_c"), max_age_ms=5000, period_ms=2000).SerializeToString())
    def reading(values, valid=True, sensor=2):
        m = pb.Reading(sensor_id=sensor, generation=7, timestamp_us=123456789, valid=valid)
        m.values.extend(values)
        return m.SerializeToString()

    (OUT / "remote_ok.bin").write_bytes(reading([65.2, 25.3]))
    (OUT / "remote_invalid.bin").write_bytes(reading([], valid=False))
    (OUT / "remote_shape.bin").write_bytes(reading([1.0, 2.0, 3.0]))
    (OUT / "remote_other_sensor.bin").write_bytes(reading([65.2, 25.3], sensor=9))
    (OUT / "remote_garbage.bin").write_bytes(bytes([0xFF] * 12))

    exe = OUT / "test_services.exe"
    comps = ROOT / "components"
    srcs = [HERE / "test_services.c", comps / "node_services/rule_engine.c", comps / "node_services/bridge.c",
            comps / "node_services/diag.c", comps / "node_services/repl.c", comps / "config_manager/config_manager.c",
            comps / "ml_core/ml_selector.c", comps / "ml_core/ml_features.c",
            comps / "aai16_protocol/generated/aai16.pb.c", comps / "aai16_protocol/nanopb/pb_common.c",
            comps / "aai16_protocol/nanopb/pb_decode.c", comps / "aai16_protocol/nanopb/pb_encode.c"]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{comps / 'node_services/include'}", f"-I{comps / 'config_manager/include'}",
           f"-I{comps / 'ml_core/include'}", f"-I{comps / 'aai16_protocol/generated'}",
           f"-I{comps / 'aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs)]
    subprocess.run(cmd, check=True)
    rc = subprocess.run([str(exe), str(OUT)]).returncode

    hb = pb.Heartbeat()
    hb.ParseFromString((OUT / "heartbeat.bin").read_bytes())
    assert hb.node_id == "nodeB" and hb.uptime_ms == 123456 and hb.config_generation == 4
    assert list(hb.active_sensor_ids) == [1, 2, 3, 4] and hb.model_state == pb.MODEL_STATE_ACTIVE
    assert hb.model_kind == pb.MODEL_MOTION and hb.model_version == 1 and hb.readings_ok == 1000
    assert hb.rules_fired == 3 and hb.free_heap == 123456 and hb.bridge_generation == 2
    print("python decoder agrees with the C-encoded heartbeat")
    sys.exit(rc)


if __name__ == "__main__":
    main()
