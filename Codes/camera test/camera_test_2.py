#!/usr/bin/env python3
"""Camera bench test for the Raspberry Pi 5.

Checks that the camera is detected, records a short clip, saves sample stills
and reports the frame rate, sharpness and colour balance of each frame. Works
over SSH with no display; everything is written to an output folder.

Install once on the Pi (system packages, not pip):
    sudo apt install -y python3-picamera2 python3-opencv

Run:
    python3 camera_test.py                  # 10 s at 1280x720
    python3 camera_test.py --seconds 20 --width 1920 --height 1080
    python3 camera_test.py --exposure-us 4000   # fixed short exposure (less motion blur)

Output (in camera_test_<date-time>/):
    clip.mp4      the recorded clip
    still_*.jpg   one still every 2 s
    metrics.csv   per-frame time, sharpness and mean B/G/R
"""

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    sys.exit("picamera2 not found. Install it with: sudo apt install -y python3-picamera2")


def sharpness(gray):
    """Variance of the Laplacian (on a half-size frame): higher = sharper.

    Only meaningful on a textured scene such as printed text or a checkerboard:
    a plain wall or a light gives low values even when in focus.
    """
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main():
    ap = argparse.ArgumentParser(description="Pi 5 camera bench test")
    ap.add_argument("--seconds", type=float, default=10.0, help="recording length")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=30.0, help="requested frame rate")
    ap.add_argument("--exposure-us", type=int, default=0,
                    help="fixed exposure time in microseconds (0 = automatic)")
    ap.add_argument("--still-every", type=float, default=2.0, help="seconds between stills")
    args = ap.parse_args()

    cams = Picamera2.global_camera_info()
    if not cams:
        sys.exit(
            "No camera detected.\n"
            "  - Pi 5 uses the small 22-pin camera connector: you need a 22-pin to 15-pin cable.\n"
            "  - Check the cable direction and that both latches are closed.\n"
            "  - Then run: rpicam-hello --list-cameras"
        )
    for i, c in enumerate(cams):
        print(f"Camera {i}: {c.get('Model', '?')}  ({c.get('Location', '?')})")

    out_dir = time.strftime("camera_test_%Y%m%d_%H%M%S")
    os.makedirs(out_dir, exist_ok=True)

    picam = Picamera2(0)
    frame_us = int(1_000_000 / args.fps)
    config = picam.create_video_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},  # BGR order, as OpenCV expects
        controls={"FrameDurationLimits": (frame_us, frame_us)},
    )
    picam.configure(config)
    if args.exposure_us > 0:
        picam.set_controls({"AeEnable": False, "ExposureTime": args.exposure_us, "AnalogueGain": 4.0})
    picam.start()
    print("Warming up (2 s) ...")
    time.sleep(2.0)

    w, h = picam.camera_configuration()["main"]["size"]
    writer = cv2.VideoWriter(os.path.join(out_dir, "clip.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))

    rows = []
    t0 = time.monotonic()
    next_still = 0.0
    next_print = 1.0
    n = 0
    print(f"Recording {args.seconds:.0f} s at {w}x{h} ...")
    while True:
        t = time.monotonic() - t0
        if t >= args.seconds:
            break
        frame = picam.capture_array("main")
        if frame.shape[2] == 4:          # some sensors deliver XBGR; drop the padding byte
            frame = frame[:, :, :3]
        frame = np.ascontiguousarray(frame)
        writer.write(frame)
        n += 1

        small = cv2.resize(frame, (w // 2, h // 2), interpolation=cv2.INTER_AREA)  # cheaper metrics
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        s = sharpness(gray)
        b, g, r = (float(v) for v in cv2.mean(small)[:3])
        rows.append((round(t, 3), round(s, 1), round(b, 1), round(g, 1), round(r, 1)))

        if t >= next_still:
            cv2.imwrite(os.path.join(out_dir, f"still_{int(t):03d}s.jpg"), frame)
            next_still += args.still_every
        if t >= next_print:
            print(f"  t={t:4.1f} s  fps={n / t:5.1f}  sharpness={s:7.1f}  mean B/G/R={b:5.1f}/{g:5.1f}/{r:5.1f}")
            next_print += 1.0

    elapsed = time.monotonic() - t0
    writer.release()

    # The clip was written assuming the requested rate. If the Pi delivered fewer
    # frames, rewrite it at the measured rate so it plays back at real speed.
    real_fps = n / elapsed if elapsed > 0 else args.fps
    if abs(real_fps - args.fps) / args.fps > 0.05:
        src = os.path.join(out_dir, "clip.mp4")
        tmp = os.path.join(out_dir, "clip_tmp.mp4")
        os.replace(src, tmp)
        cap = cv2.VideoCapture(tmp)
        fixed = cv2.VideoWriter(src, cv2.VideoWriter_fourcc(*"mp4v"), real_fps, (w, h))
        ok, f = cap.read()
        while ok:
            fixed.write(f)
            ok, f = cap.read()
        cap.release()
        fixed.release()
        os.remove(tmp)
    md = picam.capture_metadata()
    picam.stop()
    picam.close()

    with open(os.path.join(out_dir, "metrics.csv"), "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["t_s", "sharpness", "mean_b", "mean_g", "mean_r"])
        wcsv.writerows(rows)

    sharp = np.array([r[1] for r in rows]) if rows else np.array([0.0])
    blurred = int((sharp < 50).sum())
    print("\n=== Summary ===")
    print(f"Frames: {n} in {elapsed:.1f} s  ->  {n / elapsed:.1f} fps (requested {args.fps:.0f})")
    print(f"Sharpness: median {np.median(sharp):.0f}, min {sharp.min():.0f}; "
          f"{blurred} frames below 50 (likely blurred)")
    print(f"Last exposure {md.get('ExposureTime', '?')} us, gain {md.get('AnalogueGain', 0):.2f}, "
          f"colour gains {md.get('ColourGains', '?')}")
    print(f"Saved to {os.path.abspath(out_dir)}/")


if __name__ == "__main__":
    main()
