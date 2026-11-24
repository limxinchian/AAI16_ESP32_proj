"""FlexiCargo live-demo driver for the two AAI16 nodes (laptop side, binary Protobuf over MQTT).

Run from the project folder with the project venv, one step at a time:
    .\\.venv\\Scripts\\python.exe demo\\demo.py status
    .\\.venv\\Scripts\\python.exe demo\\demo.py watch            (keep this open in its own window during the demo)

Steps (in demo order):  status  check-wiring  reset  prepare  modeA  bridge  motion  env  swap-bme280  badmap  modeB  upload  timing
'models' = upload + activate the Motion model and stage the Environment model (transfer id saved in demo\\.state.json).
'setup'  = prepare + modeA + bridge + motion in one go (the state the demo starts from).
'watch'  = live view of both nodes (readings, notifications, model status).
Add --dry-run to any step to print what would be sent, without touching the nodes.
Edit demo\\rig.json for the broker address, node roles, pins and rule thresholds.
"""
import argparse
import json
import pathlib
import subprocess
import sys
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "tools"))
from google.protobuf import json_format  # noqa: E402

import config_shorthand as short  # noqa: E402
import node_client as nc  # noqa: E402

pb = nc.pb
RIG = json.loads((HERE / "rig.json").read_text(encoding="utf-8"))
STATE_FILE = HERE / ".state.json"
NAMES = {1: "GY-511", 2: "DHT22", 3: "LD2450", 4: "RC522", 5: "BME280", 6: "HC-SR04", 7: "MH-Z19B", 8: "MAX6675"}
VIRTUAL = {130: "node2 LD2450 (virtual)", 131: "node2 RC522 (virtual)"}
LABELS = {pb.MODEL_MOTION: ["STATIONARY", "MOVING", "VIBRATING"], pb.MODEL_ENVIRONMENT: ["STABLE", "CHANGING"],
          pb.MODEL_ANOMALY: ["NORMAL", "ANOMALY"]}
ML, ST = RIG["ml_node"], RIG["station_node"]
DRY = False


def sname(sid):
    return NAMES.get(sid) or VIRTUAL.get(sid) or str(sid)


def say(text=""):
    print(text, flush=True)


# ---------------------------------------------------------------------------------------------- MQTT link
class Link:
    """One MQTT connection; keeps every message from the nodes with its arrival time."""

    def __init__(self, nodes):
        import paho.mqtt.client as mqtt
        self.t0 = time.time()
        self.msgs = []          # (t, node, suffix, decoded)
        self.cv = threading.Condition()
        self.gen = {}           # (node, kind) -> last known generation
        self.found = 0          # index just after the message the last wait() returned
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-demo-{time.time_ns()}")
        self.c.on_message = self._on
        self.c.connect(RIG["broker"], RIG["port"], 30)
        for n in nodes:
            self.c.subscribe(f"aai16/{n}/out/#", qos=1)
        self.c.loop_start()

    def _on(self, _c, _u, m):
        parts = m.topic.split("/")
        node, suffix = parts[1], "/".join(parts[3:])
        obj = nc.decode(suffix, bytes(m.payload))
        with self.cv:
            self.msgs.append((time.time() - self.t0, node, suffix, obj))
            self.cv.notify_all()

    def mark(self):
        with self.cv:
            return len(self.msgs)

    def send(self, node, suffix, payload):
        self.c.publish(f"aai16/{node}/in/{suffix}", payload, qos=1).wait_for_publish()

    def wait(self, node, suffix, pred, timeout=10.0, since=0):
        end, i = time.time() + timeout, since
        with self.cv:
            while True:
                while i < len(self.msgs):
                    _t, n, s, o = self.msgs[i]
                    i += 1
                    if n == node and s == suffix and o is not None and pred(o):
                        self.found = i
                        return o
                left = end - time.time()
                if left <= 0:
                    return None
                self.cv.wait(min(left, 0.3))

    def close(self):
        self.c.loop_stop()
        self.c.disconnect()

    # ---- generations: read the nodes' current numbers once (heartbeat is every 10 s), then count up locally
    def learn(self, nodes, timeout=14.0):
        need = [n for n in nodes if (n, "config") not in self.gen]
        if not need or DRY:
            return
        say(f"  reading the nodes' current generation numbers (heartbeat, up to 10 s) ...")
        for n in need:
            hb = self.wait(n, "heartbeat", lambda m: True, timeout, since=0)
            if hb is None:
                raise SystemExit(f"no heartbeat from {n}: is it powered, on Wi-Fi and pointing at this broker?")
            self.gen[(n, "config")] = hb.config_generation
            self.gen[(n, "rules")] = hb.rule_generation
            self.gen[(n, "bridge")] = hb.bridge_generation

    def next_gen(self, node, kind):
        return self.gen[(node, kind)] + 1

    def model_gen(self, node):
        since = self.mark()
        self.send(node, "model/control", nc.control(1, pb.ACTION_QUERY))
        st = self.wait(node, "model_status", lambda m: True, 8, since)
        return st.generation if st else 0


# ---------------------------------------------------------------------------------------------- helpers
def load_state():
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_state(**kw):
    s = load_state()
    s.update(kw)
    STATE_FILE.write_text(json.dumps(s, indent=1), encoding="utf-8")


def config_doc(items):
    """items: list of short-form sensor dicts; the pins from rig.json are merged in (item values win)."""
    out = []
    for it in items:
        it = dict(it)
        name = it["sensor"]
        out.append({**RIG["wiring"].get(name, {}), **it})
    return {"sensors": out}


def build_config(items):
    doc = short.expand(config_doc(items))
    doc.pop("generation", None)
    return json_format.ParseDict(doc, pb.ConfigApply())


def ack_text(a):
    s = f"{pb.ApplyPhase.Name(a.phase).replace('PHASE_', '')}"
    if a.phase == pb.PHASE_REJECTED:
        s += f" reason={pb.Reason.Name(a.reason).replace('REASON_', '')} sensor={a.sensor_id} profile#{a.profile_index} detail={a.detail}"
    return s


def apply_config(link, node, items, label):
    msg = build_config(items)
    names = ", ".join(sname(p.sensor_id) + ("(dormant)" if p.dormant else "" if p.enabled else "(off)") for p in msg.profiles)
    say(f"  {node}: {label}: {names}")
    if DRY:
        say(f"    [dry-run] ConfigApply with {len(msg.profiles)} profile(s), {len(msg.SerializeToString())} bytes")
        return True
    msg.generation = link.next_gen(node, "config")
    since = link.mark()
    link.send(node, "config", msg.SerializeToString())
    seen = []
    end = time.time() + 20
    while time.time() < end:
        a = link.wait(node, "config_ack", lambda m: m.generation == msg.generation, max(0.2, end - time.time()), since)
        if a is None:
            break
        since = link.found
        seen.append(ack_text(a))
        if a.phase in (pb.PHASE_APPLIED, pb.PHASE_REJECTED):
            break
    say(f"    generation {msg.generation}: " + " -> ".join(seen) if seen else "    no ACK (node offline?)")
    ok = bool(seen) and seen[-1].startswith("APPLIED")
    if ok:
        link.gen[(node, "config")] = msg.generation
    return ok


# ---------------------------------------------------------------------------------------------- rule sets
def R(i):
    return pb.CalcToken(op=pb.CALC_PUSH_REG, reg=i)


def C(x):
    return pb.CalcToken(op=pb.CALC_PUSH_CONST, value=float(x))


def OP(name):
    return pb.CalcToken(op=getattr(pb, "CALC_" + name))


def add_rule(rs, rid, vars_, when, action, cooldown, edge=False):
    r = rs.rules.add(rule_id=rid, enabled=True, cooldown_ms=cooldown, edge_triggered=edge)
    for sid, idx, stat, win in vars_:
        r.vars.add(sensor_id=sid, value_index=idx, stat=getattr(pb, "STAT_" + stat), window=win)
    r.when.extend(when)
    r.action.CopyFrom(action)


def notify(text):
    assert len(text) <= 30, text
    return pb.RuleAction(type=pb.RACT_NOTIFY, text=text)


def start_sensor(sid):
    return pb.RuleAction(type=pb.RACT_SENSOR_START, sensor_id=sid, duration_ms=0)


def rules_for(mode, role):
    t = RIG["thresholds"]
    rs = pb.RuleSetApply()
    if mode == "A" and role == "station":
        tag = ([(4, 0, "LATEST", 1)], [R(0), C(0.5), OP("GT")])
        add_rule(rs, 1, *tag, notify("PACKAGE_CHECK_IN_DETECTED"), 10000, True)
        add_rule(rs, 2, *tag, start_sensor(3), 10000, True)
        zone = [R(0), OP("ABS"), C(t["zone_half_width_mm"]), OP("LT"), R(1), C(0), OP("GT"), OP("AND"),
                R(1), C(t["zone_depth_mm"]), OP("LT"), OP("AND")]
        add_rule(rs, 3, [(3, 0, "LATEST", 1), (3, 1, "LATEST", 1)], zone, notify("RECEIVING_ZONE_ACTIVITY"), 5000)
    elif mode == "A" and role == "ml":
        mag = [R(0), R(0), OP("MUL"), R(1), R(1), OP("MUL"), OP("ADD"), R(2), R(2), OP("MUL"), OP("ADD"), OP("SQRT"),
               C(t["accel_g"]), OP("GT")]
        add_rule(rs, 1, [(1, 0, "LATEST", 1), (1, 1, "LATEST", 1), (1, 2, "LATEST", 1)], mag,
                 notify("PACKAGE_HANDLING_ADVISORY"), 5000)
        add_rule(rs, 2, [(2, 1, "MIN", 3)], [R(0), C(t["temperature_c"]), OP("GT")], notify("TEMPERATURE_ADVISORY"), 30000)
        add_rule(rs, 3, [(2, 0, "MIN", 3)], [R(0), C(t["humidity_pct"]), OP("GT")], notify("HUMIDITY_ADVISORY"), 30000)
        add_rule(rs, 4, [(131, 0, "LATEST", 1)], [R(0), C(0.5), OP("GT")], notify("REMOTE_CHECK_IN_SEEN"), 5000, True)
        add_rule(rs, 5, [(130, 1, "LATEST", 1)], [R(0), C(0), OP("GT")], notify("REMOTE_RADAR_TARGET"), 5000)
    elif mode == "B" and role == "ml":
        add_rule(rs, 1, [(6, 0, "MAX", 3)], [R(0), C(t["obstruction_cm"]), OP("LT")], notify("LOADING_LANE_OBSTRUCTION"), 5000)
        add_rule(rs, 2, [(8, 0, "MEAN", 4)], [R(0), C(t["equipment_c"]), OP("GT")], notify("EQUIPMENT_TEMPERATURE_ADVISORY"), 10000)
    elif mode == "B" and role == "station":
        add_rule(rs, 1, [(7, 0, "MEAN", 15)], [R(0), C(t["co2_ppm"]), OP("GT")], notify("VENTILATION_REVIEW_ADVISORY"), 60000)
    else:
        raise ValueError((mode, role))
    return rs


def apply_rules(link, node, rs, label):
    say(f"  {node}: {label}: {len(rs.rules)} rule(s): " + ", ".join(r.action.text or "start " + sname(r.action.sensor_id) for r in rs.rules))
    if DRY:
        say(f"    [dry-run] RuleSetApply, {len(rs.SerializeToString())} bytes")
        return True
    rs.generation = link.next_gen(node, "rules")
    since = link.mark()
    link.send(node, "rules", rs.SerializeToString())
    a = link.wait(node, "rules_ack", lambda m: True, 10, since)
    say(f"    generation {rs.generation}: {ack_text(a) if a else 'no ACK'}")
    if a and a.phase == pb.PHASE_APPLIED:
        link.gen[(node, "rules")] = rs.generation
        return True
    return False


# ---------------------------------------------------------------------------------------------- model helpers
def model_status_text(s):
    return (f"{pb.ModelKind.Name(s.kind).replace('MODEL_', '')} v{s.model_version} "
            f"{pb.ModelState.Name(s.state).replace('MODEL_STATE_', '')}"
            + (f" ({pb.ModelReason.Name(s.reason).replace('MODEL_REASON_', '')})" if s.reason else ""))


def upload_model(link, node, kind, path, transfer, chunk=512):
    pkg = pathlib.Path(path).read_bytes()
    chunks = nc.make_chunks(pkg, nc.KINDS[kind], transfer, chunk, nc.package_version(pkg))
    say(f"  {node}: uploading {pathlib.Path(path).name} ({len(pkg)} bytes) as {len(chunks)} chunks of <= {chunk} bytes")
    if DRY:
        say("    [dry-run] not sent")
        return True
    t0, i, last = time.time(), 0, 0
    while i < len(chunks):
        ack = None
        for _ in range(3):
            since = link.mark()
            link.send(node, "model/chunk", chunks[i])
            ack = link.wait(node, "model_chunk_ack", lambda a: a.transfer_id == transfer, 5.0, since)
            if ack:
                break
        if ack is None:
            say(f"    no ChunkAck for chunk {i}")
            return False
        if ack.status in (pb.CHUNK_OK, pb.CHUNK_DUPLICATE):
            i = max(i + 1, ack.next_expected) if ack.status == pb.CHUNK_DUPLICATE else i + 1
        elif ack.status == pb.CHUNK_OUT_OF_ORDER:
            i = ack.next_expected
        else:
            say(f"    chunk {i} refused: {pb.ChunkStatus.Name(ack.status)}")
            return False
        if i - last >= 4 or i == len(chunks):
            say(f"    chunk {i}/{len(chunks)} acknowledged")
            last = i
    say(f"    staged on the node in {time.time() - t0:.1f} s (SHA-256 and contract checked)")
    return True


def model_control(link, node, action, kind, transfer=0, inputs=(), label=""):
    g = link.model_gen(node) + 1
    since = link.mark()
    link.send(node, "model/control", nc.control(g, action, nc.KINDS[kind], transfer, inputs))
    st = link.wait(node, "model_status", lambda m: True, 15, since)
    say(f"  {node}: {label} (model generation {g}) -> {model_status_text(st) if st else 'no status'}")
    return st


# ---------------------------------------------------------------------------------------------- steps
def step_status(a):
    link = Link([ML, ST])
    for n in (ML, ST):
        hb = link.wait(n, "heartbeat", lambda m: True, 14)
        if not hb:
            say(f"{n}: NO HEARTBEAT (not powered / not on Wi-Fi / wrong broker)")
            continue
        sensors = ", ".join(sname(i) for i in hb.active_sensor_ids) or "none running"
        say(f"{n}: up {hb.uptime_ms // 1000} s | config gen {hb.config_generation}, rules gen {hb.rule_generation}, "
            f"bridge gen {hb.bridge_generation} | running: {sensors} | model {pb.ModelKind.Name(hb.model_kind).replace('MODEL_', '')} "
            f"{pb.ModelState.Name(hb.model_state).replace('MODEL_STATE_', '')} | free heap {hb.free_heap // 1024} KiB | "
            f"reads ok {hb.readings_ok}, errors {hb.read_errors}, overruns {hb.overruns}")
    link.close()


SENSOR_ID = {"gy511": 1, "dht22": 2, "ld2450": 3, "rc522": 4, "bme280": 5, "hcsr04": 6, "mhz19b": 7, "max6675": 8}
PROBES = {
    ML: [("gy511", {}), ("bme280", {}), ("dht22", {}), ("hcsr04", {}), ("max6675", {})],
    ST: [("rc522", {}), ("ld2450", {}), ("mhz19b", {})],
}
PROBE_SECONDS = {"dht22": 7, "bme280": 6, "mhz19b": 7, "hcsr04": 4, "gy511": 3, "max6675": 3, "rc522": 3, "ld2450": 4}


def step_check_wiring(a):
    link = Link([ML, ST])
    link.learn([ML, ST])
    say("Probing each sensor on its own (one at a time per bus). A sensor passes when it returns valid readings.\n")
    results = []
    for node, sensors in PROBES.items():
        for name, extra in sensors:
            sid = SENSOR_ID[name]
            if DRY:
                apply_config(link, node, [{"sensor": name, **extra}], "probe")
                continue
            ok = apply_config(link, node, [{"sensor": name, **extra}], "probe")
            since = link.mark()
            time.sleep(PROBE_SECONDS[name])
            with link.cv:
                rd = [(o) for (_t, n, s, o) in link.msgs[since:] if n == node and s == f"reading/{sid}"]
            good = [r for r in rd if r.valid]
            errs = sorted({pb.ReadError.Name(r.error).replace("READ_", "") for r in rd if not r.valid})
            last = [round(v, 2) for v in good[-1].values] if good else None
            verdict = "PASS" if good and ok else "FAIL"
            say(f"    {verdict}: {len(good)}/{len(rd)} valid readings" + (f", last {last}" if last else "")
                + (f", errors {errs}" if errs else ""))
            results.append((node, name, verdict))
    if not DRY:
        say("\nSummary:")
        for node, name, verdict in results:
            say(f"  {node:6s} {name:8s} {verdict}")
        say("\nLeaving the nodes with every bus switched off: run 'reset' next.")
        for node in (ML, ST):
            nc_node = pb.NodeCommand(cmd=pb.RCMD_CLEAR).SerializeToString()
            link.send(node, "command", nc_node)
    link.close()


def step_reset(a):
    link = Link([ML, ST])
    for n in (ML, ST):
        say(f"  {n}: CLEAR (all buses off, rules and bridge removed, running)")
        if not DRY:
            link.send(n, "command", pb.NodeCommand(cmd=pb.RCMD_CLEAR).SerializeToString())
    if not DRY:
        time.sleep(2)
    say("Done. Run 'status' to confirm nothing is running.")
    link.close()


def step_prepare(a):
    link = Link([ML, ST])
    for n in (ML, ST):
        say(f"  {n}: CLEAR")
        if not DRY:
            link.send(n, "command", pb.NodeCommand(cmd=pb.RCMD_CLEAR).SerializeToString())
    if not DRY:
        time.sleep(2)
    base = int(time.time()) % 1000000
    say("\nMotion model -> active on the ML node:")
    ok = upload_model(link, ML, "motion", ROOT / "ml/artifacts/motion/motion.amdl", base + 1)
    if ok and not DRY:
        model_control(link, ML, pb.ACTION_ACTIVATE, "motion", base + 1, label="ACTIVATE motion")
    say("\nEnvironment model -> staged (not active) in the spare bank:")
    ok2 = upload_model(link, ML, "environment", ROOT / "ml/artifacts/environment/environment.amdl", base + 2)
    if ok2 and not DRY:
        save_state(env_transfer=base + 2)
        say("  environment model is STAGED; do not reboot the node before the 'env' step")
    link.close()


def step_modeA(a):
    link = Link([ML, ST])
    link.learn([ML, ST])
    say("Mode A: receiving and handling\n")
    apply_config(link, ST, [{"sensor": "rc522"}, {"sensor": "ld2450", "dormant": True}], "profiles (RC522 running, radar stored asleep)")
    apply_rules(link, ST, rules_for("A", "station"), "rules")
    apply_config(link, ML, [{"sensor": "gy511"}, {"sensor": "dht22"}], "profiles")
    apply_rules(link, ML, rules_for("A", "ml"), "rules")
    link.close()


def step_bridge(a):
    link = Link([ML, ST])
    link.learn([ML, ST])
    doc = json.loads((ROOT / "configs/bridge/bridge_node1_listens_to_node2_ld2450_rc522.json").read_text(encoding="utf-8"))
    doc.pop("generation", None)
    for s in doc["sources"]:
        s["remote_node"] = ST
    msg = json_format.ParseDict(doc, pb.BridgeApply())
    say(f"  {ML}: listens to {ST}: LD2450 -> virtual 130, RC522 -> virtual 131")
    if DRY:
        say(f"    [dry-run] BridgeApply, {len(msg.SerializeToString())} bytes")
    else:
        msg.generation = link.next_gen(ML, "bridge")
        since = link.mark()
        link.send(ML, "bridge", msg.SerializeToString())
        ack = link.wait(ML, "bridge_ack", lambda m: True, 10, since)
        say(f"    generation {msg.generation}: {ack_text(ack) if ack else 'no ACK'}")
    link.close()


def step_motion(a):
    link = Link([ML, ST])
    say("Motion model: map GY-511 x, y, z (sensor 1, values 0..2) to the model's three channels")
    if DRY:
        say("  [dry-run] SET_INPUTS 1:0:0 1:1:1 1:2:2")
    else:
        model_control(link, ML, pb.ACTION_SET_INPUTS, "motion", inputs=[(1, 0, 0), (1, 1, 1), (1, 2, 2)], label="SET_INPUTS GY-511")
        say("  first classification after the 64-sample window fills (about 1.3 s)")
    link.close()


def step_env(a):
    link = Link([ML, ST])
    tr = a.transfer or load_state().get("env_transfer")
    if not tr:
        raise SystemExit("no staged environment model recorded: run 'prepare' first (or pass --transfer N)")
    say("Activate the staged Environment model, then map the DHT22 (temperature = value 1, humidity = value 0)")
    if DRY:
        say(f"  [dry-run] ACTIVATE transfer {tr}; SET_INPUTS 2:1:0 2:0:1")
    else:
        st = model_control(link, ML, pb.ACTION_ACTIVATE, "environment", int(tr), label="ACTIVATE environment")
        if st and st.state != pb.MODEL_STATE_REJECTED:
            model_control(link, ML, pb.ACTION_SET_INPUTS, "environment", inputs=[(2, 1, 0), (2, 0, 1)], label="SET_INPUTS DHT22")
            say("  the first classification needs 20 readings 2 s apart: about 38-40 s")
    link.close()


def step_swap(a):
    link = Link([ML, ST])
    link.learn([ML, ST])
    say("Sensor replacement on the I2C bus: GY-511 -> BME280 (same firmware, different address, registers, calibration)")
    ok = apply_config(link, ML, [{"sensor": "bme280"}], "I2C profile")
    if ok or DRY:
        time.sleep(0 if DRY else 3)
        say("Re-point the Environment model at the BME280 (temperature = value 0, humidity = value 2):")
        if DRY:
            say("  [dry-run] SET_INPUTS 5:0:0 5:2:1")
        else:
            model_control(link, ML, pb.ACTION_SET_INPUTS, "environment", inputs=[(5, 0, 0), (5, 2, 1)], label="SET_INPUTS BME280")
            say("  a fresh 20-sample window is needed: about 38-40 s")
    link.close()


def step_modeB(a):
    link = Link([ML, ST])
    link.learn([ML, ST])
    say("Mode B: storage and equipment monitoring\n")
    apply_config(link, ML, [{"sensor": "hcsr04"}, {"sensor": "max6675"}, {"sensor": "gy511", "enabled": False}],
                 "profiles (GPIO: DHT22 -> HC-SR04, SPI: MAX6675, I2C switched off)")
    apply_rules(link, ML, rules_for("B", "ml"), "rules")
    apply_config(link, ST, [{"sensor": "mhz19b"}], "UART profile (replaces the dormant radar profile, so the radar session ends)")
    apply_rules(link, ST, rules_for("B", "station"), "rules")
    link.close()


def step_badmap(a):
    link = Link([ML, ST])
    say("Invalid mapping: BME280 PRESSURE (value 1) onto the Environment model's TEMPERATURE channel")
    if DRY:
        say("  [dry-run] SET_INPUTS 5:1:0 5:2:1")
    else:
        st = model_control(link, ML, pb.ACTION_SET_INPUTS, "environment", inputs=[(5, 1, 0), (5, 2, 1)], label="SET_INPUTS (wrong kind)")
        if st is not None:
            say("  expected: REJECTED with VALUE_KIND_MISMATCH; the previous BME280 mapping is kept (ask the node):")
            since = link.mark()
            link.send(ML, "model/control", nc.control(1, pb.ACTION_QUERY))
            q = link.wait(ML, "model_status", lambda m: True, 8, since)
            say(f"  after the refusal the node reports: {model_status_text(q) if q else 'no status'}")
    link.close()


RECORD = ROOT / "demo" / "upload_run.csv"


def step_upload(a):
    kind_path = ROOT / "ml/artifacts/anomaly_real/anomaly.amdl"
    link = Link([ML, ST])
    secs = 75
    say(f"Recording the sensor streams for {secs} s while the model is uploaded (baseline, upload, activation, after).")
    rec = None
    if not DRY:
        cmd = [sys.executable, str(ROOT / "tools/record_slot_data.py"), "--broker", RIG["broker"], "--seconds", str(secs),
               "--out", str(RECORD), "--source", f"{ML}:5", "--source", f"{ML}:6", "--source", f"{ML}:8",
               "--source", f"{ST}:4", "--source", f"{ST}:7"]
        rec = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        say("  baseline: 15 s ...")
        time.sleep(15)
    tr = int(time.time()) % 1000000
    t0 = time.time()
    ok = upload_model(link, ML, "anomaly", kind_path, tr, chunk=1024)
    if ok and not DRY:
        say(f"  (upload took {time.time() - t0:.1f} s)")
        model_control(link, ML, pb.ACTION_ACTIVATE, "anomaly", tr, label="ACTIVATE anomaly model")
        model_control(link, ML, pb.ACTION_SET_INPUTS, "anomaly", inputs=[(5, 0, 3), (5, 2, 4), (5, 1, 5), (8, 0, 9)],
                      label="SET_INPUTS BME280 + MAX6675")
    if rec is not None:
        left = secs - (time.time() - t0) - 15
        if left > 0:
            say(f"  recording continues for {left:.0f} s ...")
            time.sleep(left)
        rec.wait(timeout=30)
        say("\nSampling analysis of the whole run (node timestamps):")
        step_timing(a)
    link.close()


def step_timing(a):
    if not RECORD.exists():
        raise SystemExit("no recording yet: run 'upload' first")
    subprocess.run([sys.executable, str(ROOT / "tools/analyze_sampling_timing.py"), str(RECORD), "--infer-period"])


# ---------------------------------------------------------------------------------------------- live monitor
def step_watch(a):
    link = Link([ML, ST])
    seen = 0
    last = {}
    stats = {}
    tag_on = {}
    t_line = time.time()
    end = time.time() + a.seconds if a.seconds else None
    say(f"Watching {ML} and {ST} (Ctrl+C to stop). Events print immediately; sensor summary every 5 s.\n")
    try:
        while end is None or time.time() < end:
            with link.cv:
                link.cv.wait(0.4)
                batch, seen = link.msgs[seen:], len(link.msgs)
            for t, node, suffix, o in batch:
                if o is None:
                    continue
                ts = f"{t:6.1f}s"
                if suffix == "notification":
                    say(f"{ts}  {node}  NOTIFICATION  {o.text}   (value {o.value:.2f})")
                elif suffix == "inference":
                    lab = LABELS.get(o.kind, [])
                    if o.kind == pb.MODEL_ANOMALY:
                        say(f"{ts}  {node}  INFERENCE  anomaly model: score {o.anomaly_score:.4f} / threshold {o.anomaly_threshold:.4f} "
                            f"-> {'ANOMALY' if o.anomaly else 'normal'}  ({o.inference_us} us)")
                    else:
                        c = lab[o.predicted_class] if o.predicted_class < len(lab) else o.predicted_class
                        say(f"{ts}  {node}  INFERENCE  {pb.ModelKind.Name(o.kind).replace('MODEL_', '').lower()} model: {c}  "
                            f"scores {[round(x, 2) for x in o.scores]}  ({o.inference_us} us)")
                elif suffix == "model_status":
                    say(f"{ts}  {node}  MODEL  {model_status_text(o)}")
                elif suffix in ("config_ack", "rules_ack", "bridge_ack"):
                    say(f"{ts}  {node}  {suffix.split('_')[0].upper()} ACK  generation {o.generation}: {ack_text(o)}")
                elif suffix == "heartbeat":
                    ids = tuple(o.active_sensor_ids)
                    key = (ids, o.model_state)
                    if last.get(node) != key:
                        last[node] = key
                        say(f"{ts}  {node}  RUNNING  {', '.join(sname(i) for i in ids) or 'no sensors'}  | model "
                            f"{pb.ModelKind.Name(o.model_kind).replace('MODEL_', '')} {pb.ModelState.Name(o.model_state).replace('MODEL_STATE_', '')}")
                elif suffix.startswith("reading/"):
                    sid = int(suffix.split("/")[1])
                    s = stats.setdefault((node, sid), {"n": 0, "ok": 0, "err": set(), "last": None})
                    s["n"] += 1
                    if o.valid:
                        s["ok"] += 1
                        s["last"] = [round(v, 2) for v in o.values]
                    else:
                        s["err"].add(pb.ReadError.Name(o.error).replace("READ_", ""))
                    if sid == 4 and o.valid and o.values:
                        on = o.values[0] > 0.5
                        if on and not tag_on.get(node):
                            say(f"{ts}  {node}  RC522  CARD DETECTED")
                        tag_on[node] = on
            if time.time() - t_line >= 5 and stats:
                t_line = time.time()
                say("   -- last 5 s --")
                for (node, sid), s in sorted(stats.items()):
                    say(f"      {node} {sname(sid):8s} {s['ok']:3d}/{s['n']:3d} valid"
                        + (f"  last {s['last']}" if s["last"] else "") + (f"  errors {sorted(s['err'])}" if s["err"] else ""))
                stats = {}
    except KeyboardInterrupt:
        pass
    link.close()


def step_models(a):
    """Motion model active on the ML node and Environment model staged, WITHOUT clearing anything else
    (use this instead of 'prepare' when Mode A is already loaded)."""
    link = Link([ML, ST])
    base = int(time.time()) % 1000000
    say("Motion model -> active on the ML node:")
    ok = upload_model(link, ML, "motion", ROOT / "ml/artifacts/motion/motion.amdl", base + 1)
    if ok and not DRY:
        model_control(link, ML, pb.ACTION_ACTIVATE, "motion", base + 1, label="ACTIVATE motion")
    say("\nEnvironment model -> staged (not active) in the spare bank:")
    ok2 = upload_model(link, ML, "environment", ROOT / "ml/artifacts/environment/environment.amdl", base + 2)
    if ok2 and not DRY:
        save_state(env_transfer=base + 2)
        say("  environment model is STAGED; do not reboot the node before the 'env' step")
    link.close()


def step_setup(a):
    """Scene 0: everything that must already be running before the professor arrives, in the one order that works
    (prepare clears both nodes, so it goes first; the Motion inputs need GY-511 running, so they go last)."""
    for name, fn in (("prepare (clear, Motion model active, Environment model staged)", step_prepare),
                     ("modeA (Mode A profiles and rules on both nodes)", step_modeA),
                     ("bridge (node1 listens to node2)", step_bridge),
                     ("motion (map GY-511 to the Motion model)", step_motion)):
        say(f"\n===== setup: {name} =====")
        fn(a)
    say("\nSetup finished. Scene 1 can start: tap the card on the RC522.")


STEPS = {"status": step_status, "check-wiring": step_check_wiring, "reset": step_reset, "prepare": step_prepare,
         "setup": step_setup, "models": step_models,
         "modeA": step_modeA, "bridge": step_bridge, "motion": step_motion, "env": step_env, "swap-bme280": step_swap,
         "modeB": step_modeB, "badmap": step_badmap, "upload": step_upload, "timing": step_timing, "watch": step_watch}


def main():
    global DRY
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=sorted(STEPS))
    ap.add_argument("--dry-run", action="store_true", help="show what would be sent; do not touch the nodes")
    ap.add_argument("--seconds", type=float, default=0, help="watch: stop after N seconds (default: until Ctrl+C)")
    ap.add_argument("--transfer", type=int, default=0, help="env: transfer id of the staged model (default: remembered)")
    a = ap.parse_args()
    DRY = a.dry_run
    STEPS[a.step](a)


if __name__ == "__main__":
    main()
