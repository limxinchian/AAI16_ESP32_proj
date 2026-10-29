"""Store, activate and roll back models using correlated Protobuf MQTT commands."""
import argparse
import json
import os
from pathlib import Path
import queue
import threading
import time
import uuid

from generated import aai16_pb2 as pb
from request_ids import DEFAULT_STATE, allocate_request_id, atomic_write, unique_object
from model_package import package_model, validate_package

CONTROL, CHUNKS = "aai16/model/control", "aai16/model/upload"
ACK, STATUS = "aai16/model/upload/ack", "aai16/model/status"


class DeviceRestart(RuntimeError):
    pass


class MqttTransport:
    def __init__(self, args):
        import paho.mqtt.client as mqtt
        self.args, self.mqtt = args, mqtt
        self.ready = threading.Event()
        self.responses = queue.Queue(maxsize=8)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-model-{uuid.uuid4().hex[:12]}")
        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = lambda *unused: self.ready.clear()
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        username, password = os.getenv("AAI16_MQTT_USERNAME"), os.getenv("AAI16_MQTT_PASSWORD")
        if username:
            self.client.username_pw_set(username, password)
        if args.tls:
            self.client.tls_set(ca_certs=args.ca_file)
        self.subscription = None

    def on_connect(self, client, userdata, flags, reason, properties):
        self.ready.clear()
        if not reason.is_failure:
            rc, self.subscription = client.subscribe([(ACK, 1), (STATUS, 1)])
            if rc != self.mqtt.MQTT_ERR_SUCCESS:
                self.subscription = None

    def on_subscribe(self, client, userdata, mid, reasons, properties):
        if mid == self.subscription and len(reasons) == 2 and not any(r.is_failure for r in reasons):
            self.ready.set()

    def on_message(self, client, userdata, msg):
        if msg.retain or len(msg.payload) > 768 or msg.topic not in (ACK, STATUS):
            return
        try:
            reply = pb.Envelope.FromString(msg.payload)
            if reply.protocol_version != 1 or not reply.request_id:
                return
            if reply.WhichOneof("body") != ("model_status" if msg.topic == STATUS else "model_upload_ack"):
                return
            self.responses.put_nowait(reply)
        except (ValueError, queue.Full):
            pass
        except Exception as error:
            # Protobuf DecodeError is optional until generated module is loaded.
            from google.protobuf.message import DecodeError
            if not isinstance(error, DecodeError):
                raise

    def __enter__(self):
        self.client.connect_async(self.args.host, self.args.port, 30)
        self.client.loop_start()
        return self

    def __exit__(self, *unused):
        self.client.disconnect()
        self.client.loop_stop()

    def exchange(self, request, topic, body):
        wire = request.SerializeToString(deterministic=True)
        if len(wire) > 768:
            raise ValueError("Frame exceeds device limit")
        for attempt in range(self.args.retries + 1):
            if attempt:
                print(f"Retry {attempt}/{self.args.retries}: request_id={request.request_id} (identical bytes)")
            if not self.ready.wait(self.args.timeout):
                continue
            delivery = self.client.publish(topic, wire, qos=1, retain=False)
            if delivery.rc != self.mqtt.MQTT_ERR_SUCCESS:
                continue
            deadline = time.monotonic() + self.args.timeout
            while (remaining := deadline - time.monotonic()) > 0:
                try:
                    reply = self.responses.get(timeout=remaining)
                except queue.Empty:
                    break
                if reply.request_id == request.request_id and reply.WhichOneof("body") == body:
                    return reply
        if request.WhichOneof("body") == "model_control" and request.model_control.operation in (pb.MODEL_OP_ACTIVATE, pb.MODEL_OP_ROLLBACK):
            raise RuntimeError(f"No matching activation receipt for {request.request_id}; outcome is unknown. "
                               "Use model status before another action; a timeout does not mean the activation failed.")
        raise RuntimeError(f"No matching application reply for {request.request_id}; upload may remain pending for 60 s. "
                           "Use model status; retry with a new transfer after timeout, or model abort with the printed transfer ID.")


class Uploader:
    def __init__(self, transport, next_id, progress=print):
        self.transport, self.next_id, self.progress = transport, next_id, progress

    def request(self):
        return pb.Envelope(protocol_version=1, request_id=self.next_id())

    def status(self):
        r = self.request()
        r.model_control.operation = pb.MODEL_OP_GET_STATUS
        reply = self.transport.exchange(r, CONTROL, "model_status")
        if (not reply.HasField("boot_id") or not reply.HasField("generation") or
                not reply.model_status.HasField("result") or reply.model_status.result != pb.RESULT_OK):
            raise RuntimeError("Invalid/rejected model status")
        return reply

    def control(self, op, boot, generation, transfer, sequence=0, manifest=None):
        r = self.request()
        r.boot_id, r.generation, r.sequence = boot, generation, sequence
        r.model_control.operation, r.model_control.transfer_id = op, transfer
        if manifest:
            r.model_control.upload.CopyFrom(pb.UploadManifest(
                family=pb.MODEL_A if manifest["family"] == "A" else pb.MODEL_B, version=manifest["version"],
                package_size=manifest["package_size"], package_sha256=bytes.fromhex(manifest["package_sha256"]),
                model_size=manifest["model_size"], model_sha256=bytes.fromhex(manifest["model_sha256"]), compatibility_profile=1))
        return r

    def prepare_change(self, family=None, bank=None):
        status = self.status()
        s = status.model_status
        if not s.HasField("model_epoch"):
            raise RuntimeError("Status is missing model_epoch")
        r = self.request()
        r.boot_id, r.generation = status.boot_id, status.generation
        r.model_control.expected_model_epoch = s.model_epoch
        if family is None:
            r.model_control.operation = pb.MODEL_OP_ROLLBACK
        else:
            r.model_control.operation = pb.MODEL_OP_ACTIVATE
            if family == "a" and bank is None:
                root = Path(__file__).resolve().parents[2]
                manifest = json.loads((root / "models/motion_model_a/manifest.json").read_text())
                r.model_control.target.CopyFrom(pb.ModelIdentity(family=pb.MODEL_A,
                    version=manifest["model_version"], source=pb.SOURCE_BUNDLED,
                    sha256=bytes.fromhex(manifest["artifacts"]["model_int8.tflite"]["sha256"])))
            else:
                wanted = pb.MODEL_A if family == "a" else pb.MODEL_B
                matches = [m for m in s.stored if m.family == wanted and (bank is None or m.bank == bank)]
                if len(matches) != 1:
                    raise ValueError(f"Expected one stored Model {family.upper()}, found {len(matches)}; use model status and --bank 0/1")
                r.model_control.target.CopyFrom(matches[0])
        return r

    def change(self, family=None, bank=None, mqttx_dir=None):
        r = self.prepare_change(family, bank)
        wire = r.SerializeToString(deterministic=True)
        if mqttx_dir is not None:
            destination = Path(mqttx_dir) / f"{r.request_id}.pb"
            atomic_write(destination, wire)
            atomic_write(destination.with_suffix(".hex"), (wire.hex() + "\n").encode())
            self.progress(f"Prepared only; activation NOT sent. MQTTX: disable send Function/encoding, choose Hex, "
                          f"topic={CONTROL}, QoS=1, Retain=false\nrequest_id={r.request_id}\nhex={wire.hex()}\nsaved={destination}")
            return r
        reply = self.transport.exchange(r, CONTROL, "model_status")
        if not reply.HasField("boot_id") or reply.boot_id != r.boot_id:
            raise DeviceRestart("Device rebooted; activation was not confirmed. Get fresh status before another command.")
        s = reply.model_status
        if (not s.HasField("result") or not s.HasField("model_epoch") or not s.HasField("active") or
                not reply.HasField("generation") or reply.generation != r.generation):
            raise RuntimeError("Incomplete/uncorrelated activation receipt")
        self.progress(f"request_id={reply.request_id} boot_id=0x{reply.boot_id:08x} "
                      f"{pb.ResultCode.Name(s.result)} {pb.ModelState.Name(s.state)} epoch={s.model_epoch} "
                      f"active={s.active.version} source={pb.ModelSource.Name(s.active.source)} detail={s.detail}")
        if s.result != pb.RESULT_OK:
            raise RuntimeError(f"Activation rejected; current active={s.active.version}: {s.detail}")
        expected = r.model_control.expected_model_epoch
        if s.model_epoch not in (expected, expected + 1):
            raise RuntimeError("Unexpected model epoch in activation receipt")
        if family and s.active != r.model_control.target:
            raise RuntimeError("Activation receipt has the wrong active identity")
        current = self.status()
        if current.boot_id != reply.boot_id or current.model_status.model_epoch != s.model_epoch or current.model_status.active != s.active:
            raise RuntimeError("Active model changed after this receipt (rollback, reboot or another controller); inspect model status")
        return reply

    def send(self, r, expected_generation, transfer, allowed):
        body = r.WhichOneof("body")
        reply = self.transport.exchange(r, CHUNKS if body == "model_upload_chunk" else CONTROL, "model_upload_ack")
        a = reply.model_upload_ack
        if not reply.HasField("boot_id") or reply.boot_id != r.boot_id:
            raise DeviceRestart("Device rebooted during upload")
        if (a.transfer_id != transfer or not reply.HasField("sequence") or reply.sequence != r.sequence or
                not a.HasField("result") or not a.HasField("state") or not a.HasField("next_offset")):
            raise RuntimeError("Uncorrelated/incomplete upload ACK")
        self.progress(f"request_id={reply.request_id} generation={reply.generation} "
                      f"{pb.UploadState.Name(a.state)} {pb.ResultCode.Name(a.result)} next_offset={a.next_offset} {a.detail}")
        if a.result != pb.RESULT_OK or a.state not in allowed:
            raise RuntimeError(f"Device rejected upload: {pb.ResultCode.Name(a.result)} {a.detail}")
        if not reply.HasField("generation") or reply.generation != expected_generation or a.max_chunk_bytes != 512:
            raise RuntimeError("Unexpected upload generation/chunk contract")
        return a

    def confirm(self, status, manifest):
        family = pb.MODEL_A if manifest["family"] == "A" else pb.MODEL_B
        return [s for s in status.model_status.stored if s.family == family and s.version == manifest["version"] and
                s.sha256.hex() == manifest["model_sha256"] and s.source == pb.SOURCE_STORED and s.HasField("bank")]

    def upload(self, package, manifest):
        validate_package(package, manifest)
        for reboot_attempt in range(2):
            status = self.status()
            if reboot_attempt and (stored := self.confirm(status, manifest)):
                self.progress(f"STORED confirmed after reboot, bank={stored[0].bank}; no activation requested")
                return status
            boot, generation = status.boot_id, status.generation
            active = status.model_status.active.SerializeToString(deterministic=True)
            transfer = uuid.uuid4().bytes
            self.progress(f"transfer_id={transfer.hex()} boot_id=0x{boot:08x} package_bytes={len(package)}")
            try:
                begin = self.control(pb.MODEL_OP_BEGIN_UPLOAD, boot, generation, transfer, manifest=manifest)
                generation += 1
                a = self.send(begin, generation, transfer, (pb.UPLOAD_ACCEPTED, pb.UPLOAD_RECEIVING))
                offset = a.next_offset
                if offset > len(package) or (offset != len(package) and offset % 512):
                    raise RuntimeError("Invalid BEGIN next_offset")
                while offset < len(package):
                    r = self.request()
                    r.boot_id, r.generation, r.sequence = boot, generation, offset // 512
                    r.model_upload_chunk.transfer_id = transfer
                    r.model_upload_chunk.offset = offset
                    r.model_upload_chunk.data = package[offset:offset + 512]
                    a = self.send(r, generation, transfer, (pb.UPLOAD_RECEIVING,))
                    expected = min(offset + 512, len(package))
                    if a.next_offset != expected:
                        raise RuntimeError("Unexpected chunk acknowledgement offset")
                    offset = expected
                    self.progress(f"Upload {offset}/{len(package)} bytes ({offset * 100 // len(package)}%)")
                finish = self.control(pb.MODEL_OP_FINISH_UPLOAD, boot, generation, transfer, (len(package) + 511) // 512)
                a = self.send(finish, generation, transfer, (pb.UPLOAD_STORED,))
                if a.next_offset != len(package):
                    raise RuntimeError("Stored ACK has incorrect size")
                final = self.status()
                if final.boot_id != boot:
                    raise DeviceRestart("Reboot before final status")
                stored = self.confirm(final, manifest)
                if not stored or final.model_status.active.SerializeToString(deterministic=True) != active:
                    raise RuntimeError("Final stored identity/unchanged active identity not confirmed")
                self.progress(f"STORED confirmed: {manifest['version']} bank={stored[0].bank} sha256={manifest['model_sha256']}; active unchanged; NOT activated")
                return final
            except DeviceRestart:
                if reboot_attempt:
                    raise
                self.progress("Device reboot: checking committed storage, then restarting interrupted upload once")
        raise RuntimeError("Upload could not complete")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("operation", choices=("package", "upload", "status", "abort", "activate", "rollback"))
    p.add_argument("target_family", nargs="?", choices=("a", "b"), help="activate a (bundled) or b (stored)")
    p.add_argument("--bank", type=int, choices=(0, 1), help="Select a specific stored bank")
    p.add_argument("--mqttx-dir", type=Path, help="Prepare activation/rollback .pb and .hex for MQTTX without sending it")
    p.add_argument("--model-dir", type=Path)
    p.add_argument("--family", choices=("a", "b"), default="b")
    p.add_argument("--package", type=Path)
    p.add_argument("--manifest", type=Path)
    p.add_argument("--output-dir", type=Path, default=Path("build/model_b_package"))
    p.add_argument("--transfer-id", help="32 hex characters, printed at BEGIN")
    p.add_argument("--id-state", type=Path, default=DEFAULT_STATE)
    p.add_argument("--host")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--tls", action="store_true")
    p.add_argument("--ca-file")
    p.add_argument("--timeout", type=float, default=15)
    p.add_argument("--retries", type=int, default=4)
    args = p.parse_args(argv)
    if args.timeout <= 0 or args.retries < 0 or args.retries > 20 or not 1 <= args.port <= 65535 or (args.ca_file and not args.tls):
        p.error("Invalid timeout/retries/port/TLS options")
    if args.operation != "package" and not args.host:
        p.error("--host required for device operations")
    if args.operation == "activate" and not args.target_family:
        p.error("Use model activate a or model activate b")
    if args.operation != "activate" and (args.target_family or args.bank is not None):
        p.error("Family and --bank are only for activate")
    if args.mqttx_dir and args.operation not in ("activate", "rollback"):
        p.error("--mqttx-dir is only for activate/rollback")
    try:
        if args.operation in ("package", "upload"):
            if args.model_dir and not args.package and not args.manifest:
                package, manifest = package_model(args.model_dir, args.family)
                atomic_write(args.output_dir / "model.package", package)
                atomic_write(args.output_dir / "upload_manifest.json", (json.dumps(manifest, indent=2) + "\n").encode())
                print(f"Package: {args.output_dir / 'model.package'}; manifest: {args.output_dir / 'upload_manifest.json'}")
            elif args.package and args.manifest and not args.model_dir:
                if args.package.stat().st_size > 73728 or args.manifest.stat().st_size > 4096:
                    raise ValueError("Package/manifest too large")
                package = args.package.read_bytes()
                manifest = json.loads(args.manifest.read_text(), object_pairs_hook=unique_object)
            else:
                p.error("Choose --model-dir or both --package and --manifest")
            validate_package(package, manifest)
            if args.operation == "package":
                print(f"Verified package {len(package)} bytes SHA256={manifest['package_sha256']}; no MQTT sent")
                return
        with MqttTransport(args) as transport:
            uploader = Uploader(transport, lambda: allocate_request_id(args.id_state))
            if args.operation == "upload":
                uploader.upload(package, manifest)
            elif args.operation == "status":
                print(uploader.status())
            elif args.operation in ("activate", "rollback"):
                uploader.change(args.target_family, args.bank, args.mqttx_dir)
            else:
                transfer = bytes.fromhex(args.transfer_id or "")
                if len(transfer) != 16:
                    raise ValueError("--transfer-id must contain 16 bytes")
                status = uploader.status()
                request = uploader.control(pb.MODEL_OP_ABORT_UPLOAD, status.boot_id, status.generation, transfer)
                uploader.send(request, status.generation, transfer, (pb.UPLOAD_ABORTED,))
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        p.exit(1, f"{error}\n")
