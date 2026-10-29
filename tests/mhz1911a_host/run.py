"""Permanent MH-Z1911A tests with explicit UART/RTOS doubles.

The user-supplied hardware capture is a decoding fixture. Timing/fault injection
is synthetic, not proof of electrical safety, accuracy or SMP behavior.
No serial, network or credentials are accessed. Requires host gcc.
"""
import os
from pathlib import Path
import shutil
import subprocess

test = Path(__file__).resolve().parent
repo = test.parents[1]
output = repo / "build" / "mhz1911a_host"
output.mkdir(parents=True, exist_ok=True)
compiler = shutil.which("gcc")
if not compiler:
    raise SystemExit("Host gcc not found")
for name in ["esp_err.h", "esp_log.h", "esp_timer.h", "freertos/FreeRTOS.h",
             "freertos/task.h", "freertos/queue.h", "driver/uart.h"]:
    header = output / name
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text('#include "mhz_platform.h"\n')
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-I", str(test), "-I", str(output)]
for component in ["sensor_status", "mhz1911a_sensor", "ld2450_sensor", "mqtt_manager", "hcsr04p_sensor"]:
    flags += ["-I", str(repo / "components" / component / "include")]
suffix = ".exe" if os.name == "nt" else ""
driver_cases = ["valid", "fragmented", "min", "max", "opaque", "header", "command", "checksum",
                "range_low", "range_high", "partial", "timeout", "stale", "stale_fresh", "echo",
                "extra", "trailing", "duplicate", "noise", "flood", "uart_error", "parity", "break",
                "overflow", "buffer_full", "event_full", "post_read_error", "read_error", "zero_read",
                "length_error", "short_tx", "failed_tx", "tx_timeout", "late", "owned", "install",
                "config", "pins", "flush", "preflight_tx", "preflight_error", "boundary_noise",
                "cleanup", "recovery", "repeated"]
reader_cases = ["healthy", "warmup", "missing", "startup_recovery", "degraded", "unavailable",
                "recovery", "checksum", "overrun", "late_wake", "wrap", "init", "task", "mqtt_failed"]
for name, scenarios in [("test_driver", driver_cases), ("test_reader", reader_cases)]:
    binary = output / (name + suffix)
    sources = [str(test / (name + ".c")), str(repo / "tests/sensor_host/platform.c")]
    if name in ("test_reader", "test_control"):
        sources += [str(repo / "tests/sensor_host/config_stub.c")]
    if name == "test_driver":
        sources += [str(repo / "components/mhz1911a_sensor/mhz1911a_sensor.c"),
                    str(repo / "components/ld2450_sensor/ld2450_sensor.c"),
                    str(repo / "components/ld2450_sensor/ld2450_parser.c")]
    else:
        sources += [str(repo / "components/sensor_status/sensor_status.c")]
    subprocess.run(flags + sources + ["-o", str(binary)], check=True)
    for scenario in scenarios:
        subprocess.run([str(binary), scenario], check=True, timeout=15)
