"""Test the real HC-SR04P driver/reader with deterministic SDK/RTOS doubles.

No serial, network or credentials; requires host gcc. Hardware capture and SMP
interrupt latency still require on-board verification. Run from any directory.
"""
import os
from pathlib import Path
import shutil
import subprocess

test = Path(__file__).resolve().parent
repo = test.parents[1]
output = repo / "build" / "hcsr04p_host"
output.mkdir(parents=True, exist_ok=True)
compiler = shutil.which("gcc")
if not compiler:
    raise SystemExit("Host gcc not found")
for name in ["esp_err.h", "esp_log.h", "esp_timer.h", "esp_rom_sys.h",
             "freertos/FreeRTOS.h", "freertos/task.h", "freertos/queue.h",
             "driver/gpio.h", "driver/mcpwm_cap.h"]:
    header = output / name
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text('#include "platform.h"\n')
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-I", str(test), "-I", str(output), "-I", str(repo / "main")]
for component in ["hcsr04p_sensor", "sensor_status", "mqtt_manager", "ld2450_sensor"]:
    flags += ["-I", str(repo / "components" / component / "include")]
for name in ["test_driver", "test_reader", "test_control"]:
    binary = output / (name + (".exe" if os.name == "nt" else ""))
    sources = [str(test / (name + ".c")), str(repo / "tests/sensor_host/platform.c")]
    if name != "test_driver":
        sources += [str(repo / "components/sensor_status/sensor_status.c")]
    subprocess.run(flags + sources + ["-lm", "-o", str(binary)], check=True)
    scenarios = (["healthy", "missing", "invalid", "resume", "init", "task",
                  "overrun", "jitter", "wrap", "alternating", "mqtt_failure"] if name == "test_reader" else [None])
    if name == "test_control":
        scenarios = ["valid", "timeout"]
    for scenario in scenarios:
        subprocess.run([str(binary)] + ([scenario] if scenario else []), check=True, timeout=15)
