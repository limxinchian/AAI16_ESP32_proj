"""Live JSON view of everything the node publishes (the wire format stays binary Protobuf).

  python tools/json_monitor.py --broker 172.20.10.2 --node node1 [--only reading heartbeat config_ack] [--hide reading]

Prints one JSON line per message: time, topic, decoded message. Enums are shown by name, bytes as hex.
Ctrl+C to stop. Example output:
  15:54:07 config_ack  {"generation": 9, "phase": "PHASE_ADMITTED"}
"""
import argparse
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from google.protobuf import json_format  # noqa: E402

import node_client as nc  # noqa: E402

pb = nc.pb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--node", default="node1")
    ap.add_argument("--only", nargs="*", help="show only these topic names (e.g. reading heartbeat config_ack)")
    ap.add_argument("--hide", nargs="*", default=[], help="hide these topic names (e.g. reading)")
    ap.add_argument("--seconds", type=int, default=0, help="stop after N seconds (default: run until Ctrl+C)")
    a = ap.parse_args()

    import paho.mqtt.client as mqtt

    def on_message(_c, _u, msg):
        suffix = msg.topic.split("/out/", 1)[-1]
        name = suffix.split("/")[0]
        if (a.only and name not in a.only) or name in a.hide:
            return
        try:
            m = nc.decode(suffix, bytes(msg.payload))
            if m is None:
                text = {"raw_hex": msg.payload.hex()}
            else:
                text = json_format.MessageToDict(m, preserving_proto_field_name=True, always_print_fields_with_no_presence=False)
        except Exception as e:  # keep the monitor alive on any bad message
            text = {"decode_error": str(e), "raw_hex": msg.payload.hex()}
        label = suffix if suffix.startswith("reading") else name
        print(f"{time.strftime('%H:%M:%S')} {label:<12} {json.dumps(text)}", flush=True)

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-json-monitor-{int(time.time())}")
    c.on_message = on_message
    c.connect(a.broker, a.port, 30)
    c.subscribe(f"aai16/{a.node}/out/#", qos=1)
    print(f"watching aai16/{a.node}/out/#  (Ctrl+C to stop)", flush=True)
    c.loop_start()
    try:
        end = time.time() + a.seconds if a.seconds else None
        while end is None or time.time() < end:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    c.loop_stop()


if __name__ == "__main__":
    main()
