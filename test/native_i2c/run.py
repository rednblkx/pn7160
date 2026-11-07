"""Run native transport tests against simulated ESP-IDF API error branches."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get('CXX', 'c++')
with tempfile.TemporaryDirectory(prefix='pn7160-native-i2c-') as temporary:
    for version in [50400, 50504, 60000]:
        binary = str(Path(temporary) / f'transport-{version}')
        subprocess.run([
            compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            f'-DESP_IDF_VERSION={version}',
            '-I' + str(root / 'test/native_i2c/stubs'), '-I' + str(root / 'include'),
            str(root / 'src/i2c.cpp'), str(root / 'test/native_i2c/transport_test.cpp'),
            '-o', binary,
        ], check=True)
        subprocess.run([binary], check=True)
        print(f'Simulated API branch {version}: passed', flush=True)
