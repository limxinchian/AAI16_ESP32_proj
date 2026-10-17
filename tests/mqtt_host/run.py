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
args = parser.parse_args()
repo = Path(__file__).resolve().parents[2]
test = Path(__file__).resolve().parent
output = repo / "build" / "mqtt_host"
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
cjson = Path(args.idf_path) / "components/json/cJSON"
binary = output / ("test_mqtt.exe" if os.name == "nt" else "test_mqtt")
compiler = shutil.which(args.cc)
if not compiler:
    raise SystemExit(f"Host compiler not found: {args.cc}")
subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
                "-I", str(test), "-I", str(output),
                "-I", str(repo / "components/mqtt_manager/include"), "-I", str(cjson),
                "-I", str(repo / "components/sensor_status/include"),
                str(test / "test_mqtt.c"), str(cjson / "cJSON.c"), "-lm", "-o", str(binary)], check=True)
subprocess.run([str(binary)], check=True)

# Exercise real installed ESP-MQTT code as well as manager API doubles. This
# avoids a mock accidentally promising behaviour the SDK does not implement.
sdk = Path(args.idf_path) / "components/mqtt/esp-mqtt"
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
