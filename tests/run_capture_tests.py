#!/usr/bin/env python3
"""Linux headless smoke test: real capture binary, generated video and PTY telemetry."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import pty
import struct
import subprocess
import tempfile
import threading
import tty

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    args = parser.parse_args()
    binary = args.binary.resolve()
    with tempfile.TemporaryDirectory(prefix="awakening-calibration-") as directory:
        directory = Path(directory)
        video = directory / "board.avi"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 10, (800, 600))
        assert writer.isOpened(), "Video fixture writer failed"
        image = np.full((600, 800, 3), 255, np.uint8)
        for row in range(7):
            for col in range(10):
                cv2.circle(image, (200+col*40, 160+row*40), 10, (0, 0, 0), -1)
        for _ in range(30):
            writer.write(image)
        writer.release()
        master, slave = pty.openpty()
        tty.setraw(slave)
        config = {"camera": {"type": "video", "video": {"path": str(video), "fps": 10,
                    "loop": False, "start_frame": 0}},
                  "serial": {"enable": True, "device_name": os.ttyname(slave),
                    "telemetry_gimbal_id": 1, "telemetry_shoot_id": 2}}
        config_path = directory / "robot.yaml"
        config_path.write_text(yaml.safe_dump(config))
        common = [str(binary), str(config_path), f"--settings={ROOT / 'config/calibration.yaml'}",
                  "--headless=true", "--interval=0.2", "--count=2"]

        def run(name, extra=(), success=True):
            result = subprocess.run(common + [f"--output={directory/name}"] + list(extra),
                                    text=True, capture_output=True, timeout=20)
            assert (result.returncode == 0) == success, result.stdout + result.stderr
            return directory/name

        stop = threading.Event()
        thread = None
        try:
            images_only = run("images", ["--intrinsics-only=true"])
            assert len(list(images_only.glob("*.png"))) == 2
            assert not list(images_only.glob("*.txt"))
            missing = run("missing-pose", success=False)
            assert not list(missing.glob("*.png")), "Saved fabricated pose"

            # degrees on wire; q expected = Rz(20 deg) Ry(-10 deg) Rx(5 deg).
            frame = struct.pack("<HHfffiiHf", 28, 1, 20., -10., 5., 0, 1, 2, 20.)
            def send():
                while not stop.wait(.02):
                    os.write(master, frame)
            thread = threading.Thread(target=send)
            thread.start()
            paired = run("paired")
            assert len(list(paired.glob("*.png"))) == 2
            assert len(list(paired.glob("*.txt"))) == 2
            y, p, r = np.radians([20., -10., 5.])/2
            cy, sy, cp, sp, cr, sr = math.cos(y), math.sin(y), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
            expected = [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                        cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]
            for path in paired.glob("*.txt"):
                np.testing.assert_allclose(np.loadtxt(path), expected, atol=1e-6)
            original = (paired / "1.png").read_bytes()
            run("paired", success=False)
            assert (paired / "1.png").read_bytes() == original
            assert cv2.imread(str(paired / "1.png")).shape == (600, 800, 3)
        finally:
            stop.set()
            if thread:
                thread.join()
            os.close(master)
            os.close(slave)

        spec = importlib.util.spec_from_file_location("split_video", ROOT / "calibration/split_video.py")
        split = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(split)
        source = directory / "board"
        lines = [f"{i/10} 1 0 0 0" for i in range(30)]
        source.with_suffix(".txt").write_text("\n".join(lines)+"\n")
        output = directory / "clip"
        assert split.split_video(source, output, 4, 8) == 5
        assert output.with_suffix(".txt").read_text().splitlines() == lines[4:9]
        clip = cv2.VideoCapture(str(output.with_suffix(".avi")))
        frames = 0
        while clip.read()[0]:
            frames += 1
        clip.release()
        assert frames == 5
    print("PASS: images, paired poses, missing-pose rejection, non-overwrite, resolution, paired video trim")


if __name__ == "__main__":
    main()
