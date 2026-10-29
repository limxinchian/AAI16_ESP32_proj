"""Send bounded Stage 2 Protobuf configurations and wait for owner APPLY/ACK.

Without --host, only print/write the encoded frame. QoS 1, never retained.
"""
import argparse
import os
from pathlib import Path
import json
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parent / "generated"))
import aai16_pb2 as pb
from request_ids import DEFAULT_STATE, allocate_request_id, atomic_write, unique_object, state_lock

SENSORS = {
    "dht22": (pb.SENSOR_DHT22, 2000, 10000),
    "gy511": (pb.SENSOR_GY511, 100, 10000),
    "rc522": (pb.SENSOR_RC522, 200, 2000),
    "ld2450": (pb.SENSOR_LD2450, 500, 10000),
    "hcsr04p": (pb.SENSOR_HCSR04P, 100, 10000),
    "mhz1911a": (pb.SENSOR_MHZ1911A, 5000, 60000),
    "bme280": (pb.SENSOR_BME280, 500, 10000),
    "max6675": (pb.SENSOR_MAX6675, 250, 10000),
}


def uint(value, bits):
    number = int(value, 0)
    if not 0 <= number < 1 << bits:
        raise argparse.ArgumentTypeError(f"Expected unsigned {bits}-bit integer")
    return number


def make_config(sensor, enabled=None, interval_ms=None, request_id=None,
                expected_generation=None, boot_id=None, state_path=DEFAULT_STATE, id_floor=0):
    if sensor not in SENSORS:
        raise ValueError("Unsupported sensor")
    if (enabled is None) == (interval_ms is None):
        raise ValueError("Specify exactly one of enabled or interval_ms")
    identity, low, high = SENSORS[sensor]
    if enabled is not None and not isinstance(enabled, bool):
        raise ValueError("enabled must be a bool")
    if interval_ms is not None and (type(interval_ms) is not int or not low <= interval_ms <= high):
        raise ValueError(f"{sensor}: interval_ms must be an integer in {low}..{high}")
    for name, value in (("expected_generation", expected_generation), ("boot_id", boot_id)):
        if value is not None and (type(value) is not int or not 0 <= value < 1 << 32):
            raise ValueError(f"{name} must be uint32")
    request_id = allocate_request_id(state_path, floor=id_floor) if request_id is None else request_id
    if type(request_id) is not int or not 1 <= request_id < 1 << 64:
        raise ValueError("request_id must be a nonzero uint64")
    envelope = pb.Envelope(protocol_version=1, request_id=request_id, phase=pb.PHASE_VALIDATE)
    command = envelope.sensor_config
    command.sensor = identity
    if enabled is not None:
        command.enabled = enabled  # Explicit false remains present on the wire.
    else:
        command.interval_ms = interval_ms
    if expected_generation is not None:
        command.expected_generation = expected_generation
    if boot_id is not None:
        envelope.boot_id = boot_id
    if len(envelope.SerializeToString()) > 128:
        raise ValueError("Configuration exceeds 128-byte frame budget")
    return envelope


def make_from_input(text, state_path=DEFAULT_STATE, id_floor=0):
    if len(text) > 4096:
        raise ValueError("Configuration JSON exceeds 4096 characters")
    value = json.loads(text, object_pairs_hook=unique_object)
    if not isinstance(value, dict) or set(value) - {"sensor_config", "boot_id"}:
        raise ValueError("Input must contain sensor_config and optional boot_id only")
    config = value.get("sensor_config")
    if (not isinstance(config, dict) or
            set(config) - {"sensor", "enabled", "interval_ms", "expected_generation"}):
        raise ValueError("Unsupported sensor_config field")
    sensor = config.get("sensor")
    names = {identity: name for name, (identity, _, _) in SENSORS.items()}
    if type(sensor) is not int or sensor not in names:
        raise ValueError("sensor must be a supported numeric wire ID")
    for field in ("enabled", "interval_ms", "expected_generation"):
        if field in config and config[field] is None:
            raise ValueError(f"{field} cannot be null")
    if "boot_id" in value and value["boot_id"] is None:
        raise ValueError("boot_id cannot be null")
    return make_config(names[sensor], config.get("enabled"), config.get("interval_ms"),
                       expected_generation=config.get("expected_generation"), boot_id=value.get("boot_id"),
                       state_path=state_path, id_floor=id_floor)


def load_retry(path):
    # Only accept the canonical request files produced by this client. Unknown,
    # duplicate or edited wire fields cannot silently enter a different request.
    with Path(path).open("rb") as file:
        payload = file.read(129)
    if not 0 < len(payload) <= 128:
        raise ValueError("Retry payload must be 1..128 bytes")
    from google.protobuf.message import DecodeError
    try:
        e = pb.Envelope.FromString(payload)
    except DecodeError as error:
        raise ValueError("Malformed retry payload") from error
    if (e.protocol_version != 1 or not e.HasField("protocol_version") or not e.HasField("request_id") or
            e.phase != pb.PHASE_VALIDATE or not e.HasField("phase") or
            e.WhichOneof("body") != "sensor_config" or e.HasField("generation") or e.HasField("sequence")):
        raise ValueError("Retry file is not a v1 configuration request")
    c = e.sensor_config
    names = {identity: name for name, (identity, _, _) in SENSORS.items()}
    if not c.HasField("sensor") or c.sensor not in names:
        raise ValueError("Invalid retry sensor")
    verified = make_config(names[c.sensor], c.enabled if c.HasField("enabled") else None,
        c.interval_ms if c.HasField("interval_ms") else None, request_id=e.request_id,
        expected_generation=c.expected_generation if c.HasField("expected_generation") else None,
        boot_id=e.boot_id if e.HasField("boot_id") else None)
    if verified.SerializeToString(deterministic=True) != payload:
        raise ValueError("Retry file has unknown, duplicate or noncanonical wire fields")
    return e, payload


def publish(args, message, payload):
    import paho.mqtt.client as mqtt
    connected, subscribed, finished = threading.Event(), threading.Event(), threading.Event()
    outcome = {"error": None, "ack": None}
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    username = os.getenv("AAI16_MQTT_USERNAME")
    if username:
        client.username_pw_set(username, os.getenv("AAI16_MQTT_PASSWORD"))
    if args.tls:
        client.tls_set(ca_certs=args.ca_file)

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            outcome["error"] = "Broker connection rejected"
        elif args.ack_topic:
            client.subscribe(args.ack_topic, qos=1)
        else:
            subscribed.set()
        connected.set()

    def on_subscribe(client, userdata, mid, reason_codes, properties):
        if any(code.is_failure for code in reason_codes):
            outcome["error"] = "ACK subscription rejected"
        subscribed.set()

    def on_message(client, userdata, incoming):
        if incoming.retain or len(incoming.payload) > 768:
            return
        try:
            ack = pb.Envelope.FromString(incoming.payload)
        except Exception:
            return
        if (ack.protocol_version != 1 or not ack.HasField("request_id") or
                ack.request_id != message.request_id or ack.WhichOneof("body") != "config_ack" or
                not ack.HasField("boot_id") or not ack.HasField("generation") or
                not ack.HasField("phase") or not ack.config_ack.HasField("result")):
            return
        if message.HasField("boot_id") and ack.boot_id != message.boot_id:
            # A boot-target rejection must expose the current device boot ID.
            if ack.phase != pb.PHASE_VALIDATE or ack.config_ack.result != pb.RESULT_STALE_GENERATION:
                return
        if ack.config_ack.HasField("sensor") and ack.config_ack.sensor != message.sensor_config.sensor:
            return
        if ack.config_ack.result == pb.RESULT_OK:
            if (not ack.config_ack.HasField("sensor") or not ack.config_ack.HasField("requested") or
                    ack.config_ack.requested != message.sensor_config):
                return
            if (message.sensor_config.HasField("expected_generation") and
                    ack.generation != message.sensor_config.expected_generation + 1):
                return
        if ack.config_ack.result != pb.RESULT_OK or ack.phase == pb.PHASE_APPLY:
            outcome["ack"] = ack
            finished.set()

    client.on_connect, client.on_subscribe, client.on_message = on_connect, on_subscribe, on_message
    client.connect(args.host, args.port, keepalive=30)
    client.loop_start()
    try:
        if not connected.wait(args.timeout) or not subscribed.wait(args.timeout):
            raise RuntimeError("Connection/subscription timeout")
        if outcome["error"]:
            raise RuntimeError(outcome["error"])
        delivery = client.publish(args.topic, payload, qos=1, retain=False)
        delivery.wait_for_publish(timeout=args.timeout)
        if delivery.rc != mqtt.MQTT_ERR_SUCCESS or not delivery.is_published():
            raise RuntimeError("MQTT publication failed/timed out")
        print("Broker PUBACK received; this does not prove configuration APPLY")
        if args.ack_topic:
            if not finished.wait(args.timeout):
                raise RuntimeError("No matching application ACK; retry the identical request ID and values")
            ack = outcome["ack"]
            print(f"request_id={ack.request_id} boot_id=0x{ack.boot_id:08x} "
                  f"phase={pb.Phase.Name(ack.phase)} generation={ack.generation} "
                  f"result={pb.ResultCode.Name(ack.config_ack.result)}")
            if ack.config_ack.result != pb.RESULT_OK:
                raise RuntimeError("Device rejected configuration")
    finally:
        client.disconnect()
        client.loop_stop()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "model":
        from model_cli import main as model_main
        return model_main(sys.argv[2:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sensor", choices=SENSORS, nargs="?")
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--enabled", choices=("true", "false"))
    operation.add_argument("--interval-ms", type=int)
    parser.add_argument("--request-id", type=lambda value: uint(value, 64))
    parser.add_argument("--expected-generation", type=lambda value: uint(value, 32))
    parser.add_argument("--boot-id", type=lambda value: uint(value, 32))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--input", help="Simplified JSON file, or - for stdin; mutually exclusive with sensor arguments")
    parser.add_argument("--retry-file", type=Path, help="Resend saved request bytes without allocating a new ID")
    parser.add_argument("--id-state", type=Path, default=DEFAULT_STATE)
    parser.add_argument("--id-floor", type=lambda value: uint(value, 64), default=0,
                        help="Optional known device high-water floor when adopting a new local store")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--topic", default="aai16/config/set")
    parser.add_argument("--ack-topic", default="aai16/config/ack")
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--tls", action="store_true")
    parser.add_argument("--ca-file")
    args = parser.parse_args()
    if args.timeout <= 0 or not 1 <= args.port <= 65535 or (args.ca_file and not args.tls):
        parser.error("Invalid timeout/port or --ca-file without --tls")
    direct = args.sensor is not None
    if sum((direct, args.input is not None, args.retry_file is not None)) != 1:
        parser.error("Choose sensor arguments, --input, or --retry-file")
    if not direct and any(value is not None for value in
            (args.enabled, args.interval_ms, args.request_id, args.expected_generation, args.boot_id)):
        parser.error("--input/--retry-file cannot be combined with sensor operations or metadata overrides")
    if args.retry_file and (args.id_floor or args.output):
        parser.error("Retries preserve the saved transaction; no --id-floor or --output")
    try:
        if args.retry_file:
            message, payload = load_retry(args.retry_file)
            saved = args.retry_file
        else:
            if args.input:
                if args.input == "-":
                    source = sys.stdin.read(4097)
                else:
                    with Path(args.input).open(encoding="utf-8-sig") as file:
                        source = file.read(4097)
                message = make_from_input(source, args.id_state, args.id_floor)
            else:
                message = make_config(args.sensor, None if args.enabled is None else args.enabled == "true",
                    args.interval_ms, args.request_id, args.expected_generation, args.boot_id,
                    state_path=args.id_state, id_floor=args.id_floor)
            payload = message.SerializeToString(deterministic=True)
            saved = args.id_state.parent / "requests" / f"{message.request_id}.pb"
            # Save before connecting: even a timeout leaves the exact bytes for retry.
            # Explicit manual IDs must not overwrite a different prior request.
            with state_lock(args.id_state):
                if saved.exists() and saved.read_bytes() != payload:
                    raise ValueError("This ID already has a different saved request; use automatic IDs")
                if not saved.exists():
                    atomic_write(saved, payload)
            if args.output:
                atomic_write(args.output, payload)
        print(f"request_id={message.request_id} bytes={len(payload)} hex={payload.hex()}")
        print(f"Exact retry: --retry-file {saved}")
        if args.host:
            publish(args, message, payload)
        else:
            print("Offline output only; no MQTT message sent")
    except (ValueError, RuntimeError, OSError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
