"""Build the real reader against a scripted NCI engine and simulated buses."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

test = Path(__file__).resolve().parent
root = test.parents[1]
with tempfile.TemporaryDirectory(prefix="pn7160-reader-test-") as temp:
    include = Path(temp)
    # Shadow only the engine/SDK APIs. Copy the unchanged public reader header
    # so its quoted includes resolve to the scripted engine in this directory.
    shutil.copy(root / "include/reader.hpp", include / "reader.hpp")
    for file in (test / "stubs").iterdir():
        shutil.copy(file, include / file.name)
    for file in ("esp32-hal.h", "esp_log_buffer.h", "esp_log_level.h", "portmacro.h"):
        (include / file).write_text("#pragma once\n")
    (include / "esp_log_buffer.h").write_text(
        "#pragma once\n#define ESP_LOG_BUFFER_HEXDUMP(...) ((void)0)\n"
        "#define ESP_LOG_BUFFER_HEX_LEVEL(...) ((void)0)\n")
    (include / "esp_log.h").write_text(
        "#pragma once\n#define ESP_LOGE(...) ((void)0)\n#define ESP_LOGD(...) ((void)0)\n"
        "#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n")
    (include / "freertos").mkdir()
    (include / "freertos/projdefs.h").write_text("#pragma once\n#define portTICK_PERIOD_MS 1\n")
    binary = str(include / "reader-test")
    subprocess.run([
        os.environ.get("CXX", "c++"), "-std=c++20", "-O1", "-UNDEBUG",
        "-fsanitize=address,undefined", "-DARDUINO_ARCH_ESP32", "-I" + str(include),
        "-I" + str(root / "tests/arduino_i2c/stubs"), "-I" + str(root / "include"),
        str(test / "reader_test.cpp"), str(root / "src/reader.cpp"),
        str(root / "src/arduino_i2c.cpp"), "-o", binary,
    ], check=True)
    subprocess.run([binary], check=True)
