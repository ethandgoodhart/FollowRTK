#
# eval_run.py
#
# Created on July 10, 2026
#
# Created by Georg von Manstein
#

"""
Offline sweep of the eval video through the same Detector used by the live
service: process every Nth frame, print each brake window, and save a few
annotated frames around triggers for eyeballing.

    obstacle/.venv/bin/python obstacle/eval_run.py [--stride 15] [--out DIR]
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from detector import Detector


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--video", default="/Users/georgv.manstein/Downloads/pi-stanford-test-data/front.mp4")
    p.add_argument("--stride", type=int, default=15)
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--out", default=None, help="dir to save annotated trigger frames")
    p.add_argument("--max-saves", type=int, default=12)
    args = p.parse_args()

    det = Detector(args.model)
    # Offline sweep: the live debounce (2 consecutive frames + 1s hold) is
    # wall-clock based, so judge raw per-frame zone hits here instead.
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    n = 0
    processed = 0
    saves = 0
    windows: list[list[float]] = []  # [start_t, end_t]
    in_window = False
    t0 = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        n += 1
        if n % args.stride:
            continue
        processed += 1
        video_t = n / fps
        det.process(frame, video_t)
        dets = det.status["detections"]
        hit = any(d["in_zone"] for d in dets)

        if hit and not in_window:
            in_window = True
            windows.append([video_t, video_t])
            zone = [d for d in dets if d["in_zone"]]
            print(f"[{video_t:7.1f}s] BRAKE  " + ", ".join(f'{d["cls"]}({d["conf"]:.2f},ov={d["overlap"]:.2f})' for d in zone))
            if args.out and saves < args.max_saves:
                annotated = cv2.imdecode(np.frombuffer(det.latest_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                cv2.imwrite(f"{args.out}/brake_{video_t:07.1f}s.jpg", annotated)
                saves += 1
        elif hit:
            windows[-1][1] = video_t
        elif in_window:
            in_window = False

    cap.release()
    dur = n / fps
    brake_s = sum(b - a + args.stride / fps for a, b in windows)
    print(f"\n{processed} frames checked over {dur:.0f}s of video in {time.time() - t0:.0f}s wall")
    print(f"{len(windows)} brake window(s), ~{brake_s:.0f}s total ({100 * brake_s / dur:.1f}% of drive)")
    for a, b in windows:
        print(f"  {a:7.1f}s - {b:7.1f}s")


if __name__ == "__main__":
    main()
