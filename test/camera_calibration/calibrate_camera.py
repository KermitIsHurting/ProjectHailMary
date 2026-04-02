#!/usr/bin/env python3
"""
Camera color calibration test script.
Captures raw BA10 frames via v4l2-ctl, applies black level subtraction,
BayerGB demosaic, white balance, and tone mapping. Mirrors the C++ pipeline
in camera_driver.cpp so results can be compared visually.

Usage:
    python3 calibrate_camera.py [--device /dev/video0] [--frames 5]
"""

import argparse
import subprocess
import tempfile
import cv2
import numpy as np

# Must match constants.hpp
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1200
BLACK_LEVEL = 2752
WB_B, WB_G, WB_R = 2.987, 1.000, 0.861
TONE_SCALE = 0.026


def capture_raw(device: str, num_frames: int) -> bytes:
    with tempfile.NamedTemporaryFile(suffix=".bin") as f:
        subprocess.run([
            "v4l2-ctl", "-d", device,
            f"--set-fmt-video=width={CAMERA_WIDTH},height={CAMERA_HEIGHT},pixelformat=BA10",
            f"--stream-mmap=1", f"--stream-count={num_frames}",
            f"--stream-to={f.name}",
        ], check=True, capture_output=True)
        return open(f.name, "rb").read()


def process_frame(raw16: np.ndarray) -> np.ndarray:
    """Replicate the C++ camera_driver.cpp pipeline exactly."""
    raw_sub = np.clip(raw16.astype(np.int32) - BLACK_LEVEL, 0, 65535).astype(np.uint16)
    bgr16 = cv2.cvtColor(raw_sub, cv2.COLOR_BayerGB2BGR)

    ch = list(cv2.split(bgr16))
    ch[0] = np.clip(ch[0].astype(np.float32) * WB_B * TONE_SCALE, 0, 255).astype(np.uint8)
    ch[1] = np.clip(ch[1].astype(np.float32) * WB_G * TONE_SCALE, 0, 255).astype(np.uint8)
    ch[2] = np.clip(ch[2].astype(np.float32) * WB_R * TONE_SCALE, 0, 255).astype(np.uint8)
    return cv2.merge(ch)


def main():
    parser = argparse.ArgumentParser(description="Camera calibration test")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--output", default="calibration_result.png")
    args = parser.parse_args()

    print(f"Capturing {args.frames} frames from {args.device}...")
    data = capture_raw(args.device, args.frames)

    frame_size = CAMERA_WIDTH * CAMERA_HEIGHT * 2
    raw = np.frombuffer(data[-frame_size:], dtype=np.uint16).reshape(CAMERA_HEIGHT, CAMERA_WIDTH)

    print("Processing...")
    result = process_frame(raw)

    b, g, r = cv2.split(result)
    print(f"Output channels: B={b.mean():.1f} G={g.mean():.1f} R={r.mean():.1f}")

    cv2.imwrite(args.output, result)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
