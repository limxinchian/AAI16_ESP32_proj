"""Run sensor status and real app/cleanup paths with deterministic host doubles.

Requires host gcc with pthread support. No ESP-IDF checkout, credentials, serial,
or network access. Cooperative task doubles do not emulate ESP32 SMP execution
or physical sensor timing.
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
             "freertos/task.h", "driver/gpio.h", "driver/i2c_master.h", "driver/spi_master.h"]:
    header = output / name
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text('#include "platform.h"\n')
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-I", str(test), "-I", str(output)]
for component in ["sensor_status", "dht22_moisture_sensor", "gy511_sensor",
                  "i2c_bus_manager", "wifi_manager", "mqtt_manager", "rc522_sensor", "spi_bus_manager"]:
    flags += ["-I", str(repo / "components" / component / "include")]
for name in ["test_status", "test_app", "test_gy511", "test_runtime"]:
    binary = output / (name + (".exe" if os.name == "nt" else ""))
    sources = [str(test / (name + ".c")), str(test / "platform.c")]
    if name != "test_gy511":
        sources += [str(repo / "components/sensor_status/sensor_status.c")]
    threading_flags = ["-pthread"] if name == "test_runtime" else []
    subprocess.run(flags + threading_flags + sources + ["-o", str(binary)], check=True)
    scenarios = (["healthy", "dht_init", "gy_init", "both_init", "bus_init", "gy_task",
                  "dht_reads", "gy_reads", "gy_overrun", "healthy_config", "healthy_flood",
                  "unavailable_flood"] if name == "test_app" else [None])
    if name == "test_runtime":
        scenarios = ["default", "changes", "mixed", "burst", "shorten", "rounding",
                     "during_read", "gy_unavailable", "gy_task_failure", "dht_unavailable", "tick_wrap",
                     "gate_gy", "gate_dht", "gate_both", "gate_burst", "gate_preempt", "gate_read",
                     "gate_dht_read", "gate_gy_unavailable", "gate_task_unavailable", "gate_dht_unavailable",
                     "gate_degraded", "gate_wrap",
                     "rc_default", "rc_idle", "rc_interval", "rc_rounding", "rc_shorten",
                     "rc_gates", "rc_remove", "rc_different", "rc_partial_idle", "rc_paused",
                     "rc_burst", "rc_preempt", "rc_flood", "rc_during_read", "rc_degraded",
                     "rc_unavailable", "rc_task_failure", "rc_bus_failure", "rc_uninitialized",
                     "rc_all_config", "rc_wrap"]
    for scenario in scenarios:
        subprocess.run([str(binary)] + ([scenario] if scenario else []), check=True, timeout=15)
