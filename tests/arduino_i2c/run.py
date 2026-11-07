"""Exercise the real transport against simulated Arduino/IDF I2C APIs."""
from pathlib import Path
import os
import subprocess
import tempfile

test = Path(__file__).resolve().parent
root = test.parents[1]
with tempfile.TemporaryDirectory(prefix="pn7160-i2c-tests-") as temp:
    for version in (50300, 50400, 50504, 60000):
        binary = str(Path(temp) / str(version))
        subprocess.run([
            os.environ.get("CXX", "c++"), "-std=c++17", "-O1", "-UNDEBUG",
            "-fsanitize=address,undefined", "-DARDUINO_ARCH_ESP32",
            f"-DESP_IDF_VERSION={version}", "-I" + str(test / "stubs"),
            "-I" + str(root / "include"), str(test / "transport_test.cpp"),
            str(root / "src/arduino_i2c.cpp"), "-o", binary,
        ], check=True)
        subprocess.run([binary], check=True)
        print(f"Simulated IDF API branch {version}: passed", flush=True)
