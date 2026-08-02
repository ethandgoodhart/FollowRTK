#!/usr/bin/env python3
"""
bench_models.py — measure what inference actually costs on this Thor.

The speed-control plan assumes the detector can run comfortably faster than the
15 Hz control loop. That assumption should be measured on this box with this
camera, not inherited from a blog post about an Orin Nano.

What it measures
----------------
  * capture    — real MJPG grab + decode from the front camera, which is a
                 genuine cost at 1920x1200 and is easy to forget.
  * detect     — end-to-end predict(): preprocess + forward + NMS, which is the
                 number the control loop actually experiences. Reporting only
                 the forward pass would flatter the result.
  * backends   — PyTorch FP16 vs exported TensorRT FP16.

Timings are reported as median and p95. p95 is the one that matters: a loop
that usually makes 30 Hz but stalls to 8 Hz every twentieth frame is a loop
that will occasionally react late.

Usage
-----
    python3 tools/bench_models.py                    # torch backend sweep
    python3 tools/bench_models.py --trt              # + TensorRT exports (slow)
    python3 tools/bench_models.py --camera-only
    python3 tools/bench_models.py --models yolo11n,yolo11s --sizes 640,960
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from typing import List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Classes the speed controller actually cares about (COCO ids).
VRU_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
               5: "bus", 7: "truck", 16: "dog"}

FRONT_CAM = 0


def pct(vals: List[float], p: float) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


def fmt_row(name: str, ms: List[float], extra: str = "") -> str:
    med = statistics.median(ms)
    return (f"  {name:<26} {med:7.1f} ms  ({1000/med:5.1f} Hz)   "
            f"p95 {pct(ms, 95):6.1f} ms  ({1000/pct(ms, 95):5.1f} Hz)  {extra}")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
def bench_camera(device: int, width: int, height: int, n: int = 120,
                 fps: int = 90) -> dict:
    import cv2

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Without an explicit FPS request the driver serves its slowest mode (15 fps
    # here, i.e. 67 ms/frame) even though the sensor advertises 90. Asking for
    # 90 nearly doubles real throughput; the limit after that is CPU-side MJPG
    # decode, since this OpenCV build has no GStreamer and so cannot reach
    # Thor's hardware JPEG decoder.
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open /dev/video{device}")

    # Fail fast on a genuinely wedged device. V4L2 gives each blocked read a
    # 10 s select() timeout, so a camera held by another process turns a
    # 5-second benchmark into a 20-minute hang that looks exactly like a slow
    # model download. Only one process may hold /dev/video0 at a time.
    #
    # But the FIRST read or two after opening can legitimately time out while
    # the stream spins up, so the check is on CONSECUTIVE failures rather than
    # any failure -- otherwise the guard itself becomes the flaky part.
    fails = 0

    def read_guarded(what: str):
        nonlocal fails
        t0 = time.perf_counter()
        ok, f = cap.read()
        dt = (time.perf_counter() - t0) * 1000
        if not ok:
            fails += 1
            if fails >= 3:
                cap.release()
                raise RuntimeError(
                    f"/dev/video{device} is not delivering frames ({what}): "
                    f"{fails} consecutive failures. Another process is "
                    f"probably holding it -- check: fuser -v /dev/video{device}")
            return None, dt
        fails = 0
        return f, dt

    for i in range(20):                     # settle auto-exposure + stream start
        read_guarded(f"settle {i}")

    grab_ms, frame = [], None
    for i in range(n):
        f, dt = read_guarded(f"sample {i}")
        if f is None:
            continue                        # transient miss: don't time it
        grab_ms.append(dt)
        frame = f
    cap.release()
    if frame is None or not grab_ms:
        raise RuntimeError("no frames captured")
    return {"ms": grab_ms, "frame": frame,
            "shape": [int(frame.shape[1]), int(frame.shape[0])]}


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------
def bench_detector(model_name: str, imgsz: int, frame, backend: str,
                   warmup: int, iters: int, half: bool = True) -> Optional[dict]:
    from ultralytics import YOLO

    weights = f"{model_name}.pt"
    try:
        model = YOLO(weights)
    except Exception as e:
        print(f"    !! cannot load {weights}: {e}")
        return None

    if backend == "trt":
        engine = f"{model_name}_{imgsz}.engine"
        if not os.path.exists(engine):
            print(f"    exporting {model_name} @ {imgsz} to TensorRT "
                  f"(this takes a few minutes)...")
            try:
                out = model.export(format="engine", imgsz=imgsz, half=half,
                                   device=0, verbose=False)
                os.replace(out, engine)
            except Exception as e:
                print(f"    !! TensorRT export failed: {e}")
                return None
        model = YOLO(engine, task="detect")

    kw = dict(imgsz=imgsz, classes=list(VRU_CLASSES), verbose=False, device=0)
    if backend == "torch":
        kw["half"] = half

    try:
        for _ in range(warmup):
            model.predict(frame, **kw)
    except Exception as e:
        print(f"    !! warmup failed: {e}")
        return None

    total_ms, pre_ms, inf_ms, post_ms, ndet = [], [], [], [], []
    for _ in range(iters):
        t0 = time.perf_counter()
        r = model.predict(frame, **kw)[0]
        total_ms.append((time.perf_counter() - t0) * 1000)
        sp = r.speed
        pre_ms.append(sp.get("preprocess", 0.0))
        inf_ms.append(sp.get("inference", 0.0))
        post_ms.append(sp.get("postprocess", 0.0))
        ndet.append(len(r.boxes))

    return {"model": model_name, "imgsz": imgsz, "backend": backend,
            "total_ms": total_ms, "pre_ms": pre_ms, "inf_ms": inf_ms,
            "post_ms": post_ms, "dets": int(statistics.median(ndet))}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", default="yolo11n,yolo11s,yolo11m")
    ap.add_argument("--sizes", default="640,960,1280")
    ap.add_argument("--trt", action="store_true", help="also benchmark TensorRT")
    ap.add_argument("--camera-only", action="store_true")
    ap.add_argument("--device", type=int, default=FRONT_CAM)
    ap.add_argument("--cap-width", type=int, default=1920)
    ap.add_argument("--cap-height", type=int, default=1200)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--iters", type=int, default=60)
    ap.add_argument("--no-camera", action="store_true",
                    help="use a synthetic frame instead of the real camera")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    print("=" * 78)
    print("inference benchmark — NVIDIA Jetson AGX Thor")
    print("=" * 78)

    results = {"camera": None, "detect": []}

    # --- camera ---------------------------------------------------------
    if args.no_camera:
        frame = (np.random.rand(args.cap_height, args.cap_width, 3) * 255).astype("uint8")
        print(f"\ncamera: SKIPPED (synthetic {args.cap_width}x{args.cap_height} frame)")
    else:
        print(f"\ncamera /dev/video{args.device} @ {args.cap_width}x{args.cap_height} MJPG")
        cam = bench_camera(args.device, args.cap_width, args.cap_height)
        frame = cam["frame"]
        print(fmt_row("grab+decode", cam["ms"], f"shape={cam['shape']}"))
        results["camera"] = {
            "median_ms": statistics.median(cam["ms"]),
            "p95_ms": pct(cam["ms"], 95),
            "shape": cam["shape"],
        }

    if args.camera_only:
        print("\ndone (camera only)")
        return 0

    # --- detectors ------------------------------------------------------
    models = [m.strip() for m in args.models.split(",") if m.strip()]
    sizes = [int(s) for s in args.sizes.split(",") if s.strip()]
    backends = ["torch"] + (["trt"] if args.trt else [])

    for backend in backends:
        print(f"\ndetector — {backend} FP16   "
              f"(end-to-end: preprocess + forward + NMS)")
        for m in models:
            for sz in sizes:
                r = bench_detector(m, sz, frame, backend, args.warmup, args.iters)
                if not r:
                    continue
                extra = (f"pre {statistics.median(r['pre_ms']):.1f} "
                         f"fwd {statistics.median(r['inf_ms']):.1f} "
                         f"nms {statistics.median(r['post_ms']):.1f}")
                print(fmt_row(f"{m} @ {sz}", r["total_ms"], extra))
                results["detect"].append({
                    "model": m, "imgsz": sz, "backend": backend,
                    "median_ms": statistics.median(r["total_ms"]),
                    "p95_ms": pct(r["total_ms"], 95),
                    "hz_median": 1000 / statistics.median(r["total_ms"]),
                    "hz_p95": 1000 / pct(r["total_ms"], 95),
                    "pre_ms": statistics.median(r["pre_ms"]),
                    "fwd_ms": statistics.median(r["inf_ms"]),
                    "nms_ms": statistics.median(r["post_ms"]),
                })

    # --- verdict --------------------------------------------------------
    # The control loop runs at 15 Hz. A detector is "comfortable" if even its
    # p95 clears 2x that, so a slow frame never starves a control cycle.
    print("\n" + "=" * 78)
    cap_ms = results["camera"]["median_ms"] if results["camera"] else 0.0
    print(f"budget: 15 Hz control loop = 66.7 ms/cycle; "
          f"camera costs {cap_ms:.1f} ms of it")
    ok = [d for d in results["detect"]
          if 1000 / (d["p95_ms"] + cap_ms) >= 30.0]
    if ok:
        best = max(ok, key=lambda d: (d["imgsz"], d["model"]))
        print(f"recommended: {best['model']} @ {best['imgsz']} ({best['backend']}) "
              f"-> {1000/(best['p95_ms']+cap_ms):.1f} Hz end-to-end at p95")
    else:
        print("NOTHING clears 30 Hz end-to-end at p95 — reduce input size or model")

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
