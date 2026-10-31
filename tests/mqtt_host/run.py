"""Run actual MQTT manager logic on the host with deterministic platform doubles.

Requires host gcc and an ESP-IDF checkout (uses its real cJSON implementation).
Usage: python tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
No network/serial access. Credential headers are never read by this runner.
"""
import argparse
import os
import re
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--idf-path", default=os.environ.get("IDF_PATH"), required=not os.environ.get("IDF_PATH"))
parser.add_argument("--cc", default="gcc")
parser.add_argument("--backend", choices=('optimized', 'ansi', 'reference'), default='optimized',
                    help='Compile/reporting configuration for SDK doubles; no S3 kernels execute here')
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
test = Path(__file__).resolve().parent
output = repo / "build" / ("mqtt_host" if args.backend == 'optimized' else f"mqtt_host_{args.backend}")
output.mkdir(parents=True, exist_ok=True)
headers = ["esp_err.h", "esp_event.h", "esp_log.h", "esp_netif.h", "esp_timer.h",
           "esp_random.h", "esp_system.h", "mqtt_client.h", "freertos/FreeRTOS.h",
           "freertos/event_groups.h", "freertos/queue.h", "freertos/task.h"]
for name in headers:
    path = output / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('#include "platform.h"\n')
source = (repo / "components/mqtt_manager/mqtt_manager.c").read_text()
source = source.replace('#include "mqtt_credentials.h"', '''// Host test constants, never real credentials.
#define MQTT_BROKER_URI "mqtt://test.invalid"
#define MQTT_USERNAME "test"
#define MQTT_PASSWORD "test"
''')
(output / "mqtt_manager_under_test.c").write_text(source)
cjson = repo / "managed_components/espressif__cjson/cJSON"
if not (cjson / "cJSON.c").is_file():
    cjson = Path(args.idf_path) / "components/json/cJSON"
binary = output / ("test_mqtt.exe" if os.name == "nt" else "test_mqtt")
compiler = shutil.which(args.cc)
if not compiler:
    raise SystemExit(f"Host compiler not found: {args.cc}")
backend_id = {'reference': 1, 'ansi': 2, 'optimized': 3}[args.backend]
backend_flags = [f'-DTEST_EXPECTED_BACKEND={backend_id}']
if args.backend != 'reference':
    backend_flags += ['-DESP_NN=1', '-DCONFIG_NN_OPTIMIZED=1' if args.backend == 'optimized' else '-DCONFIG_NN_ANSI_C=1']
subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-pthread",
                *backend_flags,
                "-DPB_VALIDATE_UTF8=1", "-DPB_MESSAGE_NESTING_MAX=8", "-fstack-usage",
                "-I", str(test), "-I", str(output),
                "-I", str(repo / "components/mqtt_manager"),
                "-I", str(repo / "components/aai16_protocol/include"),
                "-I", str(repo / "components/aai16_protocol/generated"),
                "-I", str(repo / "components/aai16_protocol/vendor/nanopb"),
                "-I", str(repo / "components/mqtt_manager/include"), "-I", str(cjson),
                "-I", str(repo / "components/motion_ml/include"),
                "-I", str(repo / "components/model_upload/include"),
                "-I", str(repo / "components/sensor_status/include"),
                "-I", str(repo / "components/ld2450_sensor/include"),
                "-I", str(repo / "components/hcsr04p_sensor/include"),
                "-I", str(repo / "components/mhz1911a_sensor/include"),
                str(test / "test_mqtt.c"), str(cjson / "cJSON.c"),
                str(repo / "components/mqtt_manager/config_transactions.c"),
                str(repo / "components/aai16_protocol/aai16_protocol.c"),
                str(repo / "components/aai16_protocol/generated/aai16.pb.c"),
                *[str(repo / "components/aai16_protocol/vendor/nanopb" / name) for name in
                  ("pb_common.c", "pb_encode.c", "pb_decode.c")],
                str(repo / "components/motion_ml/motion_core.c"),
                str(repo / "components/motion_ml/motion_backend.c"),
                str(repo / "components/sensor_status/sensor_status.c"), "-lm", "-o", str(binary)], check=True)
subprocess.run([str(binary)], check=True)
# Python -> real C MQTT ingress/FIFO/sidecar/ACK -> Python, without a broker.
import sys
sys.path.insert(0, str(repo / "tools/protocol"))
from config_cli import make_config, SENSORS, pb
# Capture the bytes actually handed to the MQTT SDK by the C/nanopb publisher.
# Keep fixtures for the MQTTX decoding test; these are synthetic host results.
for name in ('a', 'b', 'status'):
    wire = subprocess.check_output([str(binary), '--motion-bridge', name])
    message = pb.Envelope.FromString(wire)
    p = message.ml_prediction
    assert p.HasField('backend') and p.backend == backend_id
    assert wire[:2] == b'\x08\x01' and b'\x00' in wire and len(wire) < 320
    assert message.WhichOneof('body') == 'ml_prediction'
    assert message.boot_id == 0x12345678 and message.generation == 2
    assert not message.HasField('request_id') and not message.HasField('phase')
    assert p.model.family == (pb.MODEL_B if name == 'b' else pb.MODEL_A)
    assert p.model.source == (pb.SOURCE_STORED if name == 'b' else pb.SOURCE_BUNDLED)
    assert p.model.sha256 == bytes(range(32)) and p.model_epoch == (7 if name == 'b' else 0)
    assert p.uptime_us == (1 << 54) + 5000123 and p.interval_ms == 100
    assert p.HasField('model_ready') and p.model_ready == (name != 'status')
    if name != 'status':
        assert p.classification == (pb.CLASS_REST if name == 'b' else pb.CLASS_MOVING)
        assert p.first_sequence == 0xfffffff8 and p.last_sequence == 11
        assert p.window_start_us == p.uptime_us - 1900000 and p.window_end_us == p.uptime_us
        assert p.output_int8 == b'\x80\x00' and list(p.logits) == [-1.25, 2.5]
        assert p.inference_us == 123 and p.clipped == 3
    else:
        assert p.status == pb.MOTION_INIT_FAILED and p.classification == pb.CLASS_UNKNOWN
        assert not p.HasField('output_int8') and not p.HasField('first_sequence') and not p.logits
    (output / f'ml_{name}.pb').write_bytes(wire)
print('PASS: 3 live C/nanopb MQTT payloads decoded by Python; embedded NUL, A/B/status, exact uint64, identity/epoch/fields', flush=True)
# RC522 card events: bytes from the real C/nanopb publisher decoded by Python.
# Zero bytes inside the UID and the exact uint64 uptime exercise binary handling.
for uid_hex in ('5a2dfd03', '5a2dfd030080ff', '5a2dfd030080ff0110ab'):
    wire = subprocess.check_output([str(binary), '--rc522-bridge', uid_hex])
    message = pb.Envelope.FromString(wire)
    t = message.sensor_telemetry
    assert message.WhichOneof('body') == 'sensor_telemetry' and wire[:2] == b''
    assert message.protocol_version == 1 and message.boot_id == 0x12345678 and message.sequence == 1
    assert message.HasField('generation') and message.generation == 0
    assert not message.HasField('request_id') and not message.HasField('phase')
    assert t.sensor == pb.SENSOR_RC522 and t.WhichOneof('reading') == 'rc522'
    assert t.rc522.uid == bytes.fromhex(uid_hex)
    assert t.acquired_us == t.uptime_us == (1 << 54) + 5000123
    assert not t.HasField('health') and not t.HasField('interval_ms') and len(wire) < 64
    (output / f'rc522_{len(uid_hex) // 2}.pb').write_bytes(wire)
print('PASS: 3 live C/nanopb RC522 payloads decoded by Python; UID 4/7/10 incl. embedded NUL, exact uint64, sequence/generation', flush=True)
# HC-SR04P distance: bytes from the real C/nanopb mailbox publisher decoded by Python.
import struct
def f32(x): return struct.unpack('<f', struct.pack('<f', x))[0]  # The wire carries float32.
wire = subprocess.check_output([str(binary), '--hcsr04p-bridge'])
message = pb.Envelope.FromString(wire)
t = message.sensor_telemetry
assert message.WhichOneof('body') == 'sensor_telemetry' and wire[:2] == b''
assert message.protocol_version == 1 and message.boot_id == 0x12345678 and message.sequence == 1
assert message.HasField('generation') and message.generation == 7
assert not message.HasField('request_id') and not message.HasField('phase')
assert t.sensor == pb.SENSOR_HCSR04P and t.WhichOneof('reading') == 'hcsr04p'
assert t.acquired_us == 99385041 and t.uptime_us == 99885041  # Acquired 0.5 s before building.
assert t.hcsr04p.distance_cm == f32(42.1) and t.hcsr04p.echo_us == f32(2456.3)
assert not t.HasField('health') and not t.HasField('interval_ms') and len(wire) < 64
(output / 'hcsr04p.pb').write_bytes(wire)
print(f'PASS: live C/nanopb HC-SR04P payload ({len(wire)} bytes) decoded by Python; exact float32, microsecond times, sequence/generation', flush=True)
roundtrips = 0
maximum_request = maximum_ack = 0
for sensor, (_, minimum, maximum) in SENSORS.items():
    for operation in ({"enabled": False}, {"enabled": True}, {"interval_ms": minimum}, {"interval_ms": maximum}):
        request = make_config(sensor, request_id=0xf123456789abcdef, expected_generation=0, boot_id=0x12345678, **operation)
        result = subprocess.run([str(binary), "--config-bridge"], input=request.SerializeToString(), capture_output=True, check=True)
        ack = pb.Envelope.FromString(result.stdout)
        assert ack.request_id == request.request_id and ack.protocol_version == 1
        assert ack.boot_id == 0x12345678 and ack.generation == 1 and ack.phase == pb.PHASE_APPLY
        assert ack.config_ack.result == pb.RESULT_OK and ack.config_ack.requested == request.sensor_config
        maximum_request = max(maximum_request, len(request.SerializeToString()))
        maximum_ack = max(maximum_ack, len(result.stdout))
        roundtrips += 1
assert maximum_request == 31 and maximum_ack == 42
print(f"PASS: {roundtrips} Python/live-C-configuration/ACK round trips; live maxima request={maximum_request} ACK={maximum_ack}", flush=True)


# Exercise real installed ESP-MQTT code as well as manager API doubles. This
# avoids a mock accidentally promising behaviour the SDK does not implement.
sdk = repo / "managed_components/espressif__mqtt"
if not (sdk / "mqtt_client.c").is_file():
    sdk = Path(args.idf_path) / "components/mqtt/esp-mqtt"
print(f"Testing MQTT SDK source: {sdk}", flush=True)
sdk_output = output / "sdk"
sdk_output.mkdir(exist_ok=True)
for name in ["platform.h", "mqtt_client.h", "mqtt_config.h", "esp_err.h", "esp_log.h", "esp_heap_caps.h"]:
    (sdk_output / name).write_text('#include "sdk_platform.h"\n')
for name in ["mqtt_msg.h", "mqtt_outbox.h"]:
    shutil.copyfile(sdk / "lib/include" / name, sdk_output / name)
(sdk_output / "sys").mkdir(exist_ok=True)
# Portable BSD list macros already vendored in the same IDF checkout.
shutil.copyfile(Path(args.idf_path) / "components/openthread/openthread/third_party/tcplp/bsdtcp/sys/queue.h",
                sdk_output / "sys/queue.h")
sdk_source = (sdk / "mqtt_client.c").read_text()

def sdk_function(name):
    match = re.search(r"(?m)^(?:static (?:inline )?)?\w+ " + name + r"\(", sdk_source)
    if not match:
        raise RuntimeError(f"SDK source changed: cannot extract {name}")
    end = sdk_source.index("\n}\n", match.start()) + 3
    return sdk_source[match.start():end]

functions = [sdk_function(name) for name in ["mqtt_enqueue", "make_publish", "mqtt_client_enqueue_publish",
             "esp_mqtt_client_publish", "esp_mqtt_client_enqueue", "mqtt_resend_queued"]]
start = sdk_source.index("            outbox_item_handle_t item = outbox_dequeue(client->outbox, QUEUED, NULL);")
end = sdk_source.index("            } else if (has_timed_out(last_retransmit", start)
functions.append("static void sdk_drain_one_queued(esp_mqtt_client_handle_t client)\n{\n" +
                 sdk_source[start:end] + "            }\n}\n")
config = (sdk / "lib/include/mqtt_config.h").read_text()
assert re.search(r"#define MQTT_POLL_READ_TIMEOUT_MS\s+\(1000\)", config)
assert "esp_transport_poll_read(client->transport, max_poll_timeout(client, MQTT_POLL_READ_TIMEOUT_MS))" in sdk_source
(sdk_output / "sdk_functions.inc").write_text("#define SDK_IDLE_POLL_MS 1000\n" + "\n".join(functions))
sdk_binary = sdk_output / ("test_sdk.exe" if os.name == "nt" else "test_sdk")
subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                "-Wno-sign-compare", "-I", str(sdk_output), "-I", str(test),
                str(test / "test_sdk.c"), str(sdk / "lib/mqtt_msg.c"), str(sdk / "lib/mqtt_outbox.c"),
                "-o", str(sdk_binary)], check=True)
subprocess.run([str(sdk_binary)], check=True)
