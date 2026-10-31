"""Compile the production bridge against real IDF BLE headers and host I/O mocks."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
idf = os.environ.get("IDF_PATH")
if not idf:
    description = root / "build" / "project_description.json"
    if description.exists():
        idf = json.loads(description.read_text(encoding="utf-8"))["idf_path"]
if not idf:
    raise SystemExit("Set IDF_PATH to the ESP-IDF 6.0 directory first.")
ble_headers = Path(idf) / "components/bt/host/bluedroid/api/include/api"
if not (ble_headers / "esp_gatts_api.h").is_file():
    raise SystemExit(f"ESP-IDF BLE headers not found: {ble_headers}")
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not compiler:
    raise SystemExit("A host gcc or clang compiler is required.")

with tempfile.TemporaryDirectory(prefix="bletransp-test-") as directory:
    temporary = Path(directory)
    for header in (
        "freertos/FreeRTOS.h", "freertos/queue.h", "freertos/semphr.h",
        "freertos/task.h", "driver/uart.h", "esp_bt.h", "esp_bt_main.h",
        "esp_log.h", "esp_err.h", "nvs_flash.h",
    ):
        path = temporary / header
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("/* Platform dependency supplied by test_bridge.c. */\n")
    executable = temporary / ("test_bridge.exe" if os.name == "nt" else "test_bridge")
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-parameter", "-Wno-unused-function",
        "-I", str(temporary), "-I", str(ble_headers),
        str(root / "tests/test_bridge.c"), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
