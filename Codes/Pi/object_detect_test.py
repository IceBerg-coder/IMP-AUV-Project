#!/usr/bin/env python3
"""Object detection test on the Raspberry Pi 5 (Picamera2 + Ultralytics YOLO).

Runs a pretrained YOLO model on the live camera, saves an annotated clip and
stills, and logs every detection to a CSV. Works over SSH with no display.

The default model (yolo11n) knows the 80 COCO classes: person, bottle, cup,
cell phone, sports ball, chair, etc. Use everyday objects for this first test.

Install once on the Pi (camera packages from apt, YOLO in a venv that can see them):
    sudo apt install -y python3-picamera2 python3-opencv
    python3 -m venv --system-site-packages ~/auv-venv
    source ~/auv-venv/bin/activate
    pip install ultralytics

Run (inside the venv):
    python3 object_detect_test.py                       # 20 s, model downloads on first run
    python3 object_detect_test.py --imgsz 640           # more accurate, slower
    python3 object_detect_test.py --classes bottle cup  # only these classes

Faster on the Pi: export once to NCNN, then use the exported folder as the model:
    yolo export model=yolo11n.pt format=ncnn
    python3 object_detect_test.py --model yolo11n_ncnn_model

Output (in detect_test_<date-time>/):
    detections.mp4    annotated clip
    still_*.jpg       one annotated still every 2 s
    detections.csv    time, class, confidence and box of every detection
"""

import argparse
import csv
import os
import sys
import time
from collections import Counter

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    sys.exit("picamera2 not found: sudo apt install -y python3-picamera2 "
             "(and create the venv with --system-site-packages)")

try:
    from ultralytics import YOLO
except ImportError:
    sys.exit("ultralytics not found: activate the venv and run: pip install ultralytics")


def rewrite_at_fps(path, fps, size):
    """Re-save a clip at the real frame rate so it plays back at real speed."""
    tmp = path + ".tmp.mp4"
    os.replace(path, tmp)
    cap = cv2.VideoCapture(tmp)
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    ok, f = cap.read()
    while ok:
        out.write(f)
        ok, f = cap.read()
    cap.release()
    out.release()
    os.remove(tmp)


def main():
    ap = argparse.ArgumentParser(description="YOLO object detection test on the Pi 5 camera")
    ap.add_argument("--model", default="yolo11n.pt", help="YOLO model file or NCNN folder")
    ap.add_argument("--imgsz", type=int, default=320, help="inference size (320 fast, 640 accurate)")
    ap.add_argument("--conf", type=float, default=0.4, help="minimum confidence")
    ap.add_argument("--classes", nargs="*", default=None, help="only detect these class names")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--still-every", type=float, default=2.0)
    args = ap.parse_args()

    if not Picamera2.global_camera_info():
        sys.exit("No camera detected. Check: rpicam-hello --list-cameras")

    print(f"Loading model {args.model} ...")
    model = YOLO(args.model)
    names = model.names
    class_ids = None
    if args.classes:
        lookup = {v: k for k, v in names.items()}
        unknown = [c for c in args.classes if c not in lookup]
        if unknown:
            sys.exit(f"Unknown class names: {unknown}. Available: {sorted(lookup)}")
        class_ids = [lookup[c] for c in args.classes]

    out_dir = time.strftime("detect_test_%Y%m%d_%H%M%S")
    os.makedirs(out_dir, exist_ok=True)

    picam = Picamera2(0)
    picam.configure(picam.create_video_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}))  # BGR order for OpenCV/YOLO
    picam.start()
    time.sleep(2.0)
    w, h = picam.camera_configuration()["main"]["size"]

    clip_path = os.path.join(out_dir, "detections.mp4")
    nominal_fps = 10.0
    writer = cv2.VideoWriter(clip_path, cv2.VideoWriter_fourcc(*"mp4v"), nominal_fps, (w, h))

    counts = Counter()
    infer_ms = []
    rows = []
    n = 0
    t0 = time.monotonic()
    next_still = 0.0
    next_print = 1.0
    print(f"Detecting for {args.seconds:.0f} s at {w}x{h}, imgsz {args.imgsz}, conf >= {args.conf} ...")

    while True:
        t = time.monotonic() - t0
        if t >= args.seconds:
            break
        frame = picam.capture_array("main")
        if frame.shape[2] == 4:
            frame = frame[:, :, :3]
        frame = np.ascontiguousarray(frame)

        res = model.predict(frame, imgsz=args.imgsz, conf=args.conf,
                            classes=class_ids, verbose=False)[0]
        infer_ms.append(res.speed.get("inference", 0.0))
        annotated = res.plot()
        writer.write(annotated)
        n += 1

        seen = []
        for box in res.boxes:
            cid = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            name = names[cid]
            counts[name] += 1
            seen.append(f"{name} {conf:.2f}")
            rows.append((round(t, 3), name, round(conf, 3), x1, y1, x2, y2))

        if t >= next_still:
            cv2.imwrite(os.path.join(out_dir, f"still_{int(t):03d}s.jpg"), annotated)
            next_still += args.still_every
        if t >= next_print:
            print(f"  t={t:4.1f} s  fps={n / t:4.1f}  infer={infer_ms[-1]:5.0f} ms  "
                  f"{', '.join(seen) if seen else '(nothing)'}")
            next_print += 1.0

    elapsed = time.monotonic() - t0
    writer.release()
    picam.stop()
    picam.close()

    real_fps = n / elapsed if elapsed > 0 else nominal_fps
    if real_fps > 0 and abs(real_fps - nominal_fps) / nominal_fps > 0.05:
        rewrite_at_fps(clip_path, real_fps, (w, h))

    with open(os.path.join(out_dir, "detections.csv"), "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["t_s", "class", "confidence", "x1", "y1", "x2", "y2"])
        wcsv.writerows(rows)

    print("\n=== Summary ===")
    print(f"Frames: {n} in {elapsed:.1f} s  ->  {real_fps:.1f} fps")
    if infer_ms:
        print(f"Inference: median {np.median(infer_ms):.0f} ms per frame")
    if counts:
        print("Detections (frames containing each class):")
        for name, c in counts.most_common(10):
            print(f"  {name:15s} {c}")
    else:
        print("No detections. Try objects like a bottle, cup, phone or person, or lower --conf.")
    print(f"Saved to {os.path.abspath(out_dir)}/")


if __name__ == "__main__":
    main()
