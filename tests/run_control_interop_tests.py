"""Compile the read-only control-2026 protocol in a temporary host harness.

Usage: python3 tests/run_control_interop_tests.py /path/to/control-2026
Pins the three upstream files to a6e1638; only hardware includes are stubbed.
No controller files or physical devices are modified/opened.
"""
from pathlib import Path
import argparse
import hashlib
import os
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("control_root", type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
expected = {
    "srm_protocol.c": "0af1cb0a9093ffd5b062dcc064ac58c93e2f630d",
    "srm_protocol.h": "48487d08a7f4406778dc19c48691048a6414dcc0",
    "master_process.h": "f7f19e7612a6114220b85a7ad69e1366281aca42",
}
with tempfile.TemporaryDirectory(prefix="awakening-interop-") as folder:
    temp = Path(folder)
    module = temp / "Modules/master_machine"
    module.mkdir(parents=True)
    for name, sha in expected.items():
        data = (args.control_root / "Modules/master_machine" / name).read_bytes()
        data = data.replace(b"\r\n", b"\n")
        actual = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        if actual != sha:
            raise SystemExit(f"Upstream version mismatch: {name}: {actual} != {sha}")
        (module / name).write_bytes(data)
    usb = temp / "Hardware/stm32-h7/USB_DEVICE/App/usbd_cdc_if.h"
    usb.parent.mkdir(parents=True)
    usb.write_text("#define USBD_OK 0\n")
    (module / "bsp_usart.h").write_text("#include <stdint.h>\n")
    (module / "ins_task.h").write_text("typedef struct IMU_Init_Config_s IMU_Init_Config_s;\n")
    (module / "seasky_protocol.h").write_text("")
    flags = ["-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie"]
    obj = temp / "protocol.o"
    subprocess.run(["gcc", "-std=c11", *flags, "-c", str(module / "srm_protocol.c"),
                    "-o", str(obj)], check=True)
    binary = temp / "interop"
    subprocess.run(["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror", *flags,
                    "-no-pie", "-I" + str(module), "-I" + str(root / "src"),
                    str(root / "tests/control_interop_test.cpp"), str(obj), "-o", str(binary)],
                   check=True)
    subprocess.run([str(binary)], check=True, timeout=30,
                   env={**os.environ, "ASAN_OPTIONS": "detect_leaks=1"})
    print("PASS: pinned upstream C protocol + production vision codec, ASan/UBSan")
