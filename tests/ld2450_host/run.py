"""LD2450 production parser/driver/reader with explicit UART and RTOS doubles.

No serial, network, credentials or production fault injection. Requires gcc.
"""
import os
from pathlib import Path
import shutil
import subprocess

test = Path(__file__).resolve().parent
repo = test.parents[1]
output = repo / "build" / "ld2450_host"
output.mkdir(parents=True, exist_ok=True)
compiler = shutil.which("gcc")
if not compiler:
    raise SystemExit("Host gcc not found")
for name in ["esp_err.h", "esp_log.h", "esp_timer.h", "freertos/FreeRTOS.h",
             "freertos/task.h", "freertos/queue.h", "driver/uart.h"]:
    header = output / name
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text('#include "platform.h"\n')
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-I", str(test), "-I", str(output), "-I", str(repo / "main")]
for component in ["ld2450_sensor", "sensor_status", "mqtt_manager"]:
    flags += ["-I", str(repo / "components" / component / "include")]
for name in ["test_parser", "test_driver", "test_reader"]:
    binary = output / (name + (".exe" if os.name == "nt" else ""))
    sources = [str(test / (name + ".c")), str(repo / "tests/sensor_host/platform.c")]
    sources += [str(repo / "components/ld2450_sensor/ld2450_parser.c")]
    if name == "test_reader":
        sources += [str(repo / "components/sensor_status/sensor_status.c")]
    subprocess.run(flags + sources + ["-lm", "-o", str(binary)], check=True)
    scenarios = (["healthy", "idle", "missing", "noise", "silence", "resume", "changes",
                  "init", "task", "cleanup", "uart_errors", "resync"] if name == "test_reader" else [None])
    for scenario in scenarios:
        subprocess.run([str(binary)] + ([scenario] if scenario else []), check=True, timeout=15)
