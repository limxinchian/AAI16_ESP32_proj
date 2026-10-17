"""Run sensor status and real app/cleanup paths with deterministic host doubles.

Requires host gcc. No ESP-IDF checkout, credentials, serial, or network access.
These tests do not emulate concurrent FreeRTOS tasks or physical sensor timing.
Usage: python tests/sensor_host/run.py
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cc", default="gcc")
args = parser.parse_args()
compiler = shutil.which(args.cc)
if not compiler:
    raise SystemExit(f"Host compiler not found: {args.cc}")
test = Path(__file__).resolve().parent
repo = test.parents[1]
output = repo / "build" / "sensor_host"
output.mkdir(parents=True, exist_ok=True)
for name in ["esp_err.h", "esp_log.h", "esp_timer.h", "freertos/FreeRTOS.h",
             "freertos/task.h", "driver/gpio.h", "driver/i2c_master.h"]:
    header = output / name
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text('#include "platform.h"\n')
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-I", str(test), "-I", str(output)]
for component in ["sensor_status", "dht22_moisture_sensor", "gy511_sensor",
                  "i2c_bus_manager", "wifi_manager", "mqtt_manager"]:
    flags += ["-I", str(repo / "components" / component / "include")]
for name in ["test_status", "test_app", "test_gy511"]:
    binary = output / (name + (".exe" if os.name == "nt" else ""))
    sources = [str(test / (name + ".c")), str(test / "platform.c")]
    if name != "test_gy511":
        sources += [str(repo / "components/sensor_status/sensor_status.c")]
    subprocess.run(flags + sources + ["-o", str(binary)], check=True)
    scenarios = (["healthy", "dht_init", "gy_init", "both_init", "bus_init", "gy_task",
                  "dht_reads", "gy_reads", "gy_overrun", "healthy_config", "healthy_flood",
                  "unavailable_flood"] if name == "test_app" else [None])
    for scenario in scenarios:
        subprocess.run([str(binary)] + ([scenario] if scenario else []), check=True)
