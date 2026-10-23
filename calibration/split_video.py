#!/usr/bin/env python3
"""Trim Tongji paired .avi/.txt recordings, preserving t w x y z frame alignment.

Adapted from sp_vision_25/calibration/split_video.cpp; see LICENSE-TongjiSuperPower.
"""
import argparse
import math
from pathlib import Path
import sys

import cv2


def split_video(source, output, start, end=None):
    if start < 0 or (end is not None and end < start):
        raise ValueError("Require 0 <= start <= end (inclusive)")
    video_path, text_path = Path(str(source)+".avi"), Path(str(source)+".txt")
    out_video, out_text = Path(str(output)+".avi"), Path(str(output)+".txt")
    if out_video.exists() or out_text.exists():
        raise ValueError("Output already exists")
    lines = text_path.read_text(encoding="utf-8-sig").splitlines()
    for line in lines:
        values = [float(v) for v in line.split()]
        if len(values) != 5 or not all(math.isfinite(v) for v in values):
            raise ValueError("Pose recording must have one finite t w x y z row per frame")
        if not .99 <= math.sqrt(sum(v*v for v in values[1:])) <= 1.01:
            raise ValueError("Invalid quaternion in recording")
    if end is None:
        end = len(lines)-1
    if end >= len(lines) or start >= len(lines):
        raise ValueError("Requested frame range exceeds pose recording")
    capture, writer = cv2.VideoCapture(str(video_path)), None
    out_video.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    try:
        if not capture.isOpened():
            raise ValueError(f"Cannot open video: {video_path}")
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Invalid video FPS")
        # Sequential reading avoids approximate seek positions in compressed video.
        with out_text.open("x", encoding="utf-8") as stream:
            for index in range(end+1):
                ok, frame = capture.read()
                if not ok:
                    raise ValueError("Video ended before requested pose range; outputs are incomplete")
                if index < start:
                    continue
                if writer is None:
                    writer = cv2.VideoWriter(str(out_video), cv2.VideoWriter_fourcc(*"MJPG"), fps,
                                            (frame.shape[1], frame.shape[0]))
                    if not writer.isOpened():
                        raise ValueError("Cannot create output video")
                writer.write(frame)
                stream.write(lines[index]+"\n")
                count += 1
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="prefix without .avi/.txt")
    parser.add_argument("output", type=Path, help="new prefix without .avi/.txt")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int, help="inclusive, zero based; default through last pose")
    args = parser.parse_args()
    try:
        print(f"Saved {split_video(args.input, args.output, args.start, args.end)} paired frames")
        return 0
    except (OSError, ValueError, cv2.error) as error:
        print(f"Split failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
