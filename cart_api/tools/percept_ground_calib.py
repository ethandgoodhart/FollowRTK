#!/usr/bin/env python3
"""
percept_ground_calib.py — measure the one thing a calibration board cannot.

PRODUCTION's ChArUco run gives us the lens: fx, fy, cx, cy and distortion, to
0.17 px. What it does not give us, and what no board can, is where that lens
sits relative to the GROUND: how high it is and how far down it tilts. Every
range this stack produces comes from those two numbers, so until they are
measured the minimap is fiction and the cart must stay in shadow mode.

The measurement takes about five minutes and a tape measure.

  1. Park the cart on flat ground, pointing at open space. Do not sit on it or
     load it afterwards -- weight changes the pitch, which is why
     pitch_sigma_deg exists.
  2. Put a small marker (a cone, a taped X, a water bottle) on the ground
     straight ahead at several known distances. Measure each distance from the
     point on the ground DIRECTLY BELOW THE LENS, not from the bumper. Three
     markers is the minimum, five is better, and they must span the range you
     care about: something near (3-5 m) and something far (20-25 m). Two near
     markers cannot separate height from pitch.
  3. Run this tool, click the exact point where each marker TOUCHES THE GROUND,
     and type the measured distance.

It then fits height and pitch to those points and, if you pass --fit-focal,
a correction to fy as well -- which is the check on whether the intrinsics
really do scale from the calibrated 640x480 mode to the capture mode. If the
fitted focal scale comes back near 1.0, the scaling assumption held. If it
comes back at 1.2, it did not, and you have just caught a 20% ranging error
that would otherwise have shown up as braking two metres late.

Usage
-----
    python3 tools/percept_ground_calib.py                 # click points live
    python3 tools/percept_ground_calib.py --fit-focal     # also solve fy
    python3 tools/percept_ground_calib.py --image f.jpg   # use a saved frame
    python3 tools/percept_ground_calib.py --points 1050:5,980:10,940:20
    python3 tools/percept_ground_calib.py --write         # save the result

Nothing here touches the cart. It opens the camera and nothing else.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import List, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.percept import calib as calib_mod                  # noqa: E402
from cartlib.percept.geometry import CameraModel                # noqa: E402

Point = Tuple[float, float, float]        # (u_px, v_px, measured_range_m)


# ---------------------------------------------------------------------------
# The model being fitted
# ---------------------------------------------------------------------------
def predict_range(v_px: float, cy: float, fy: float, h: float,
                  th: float) -> float:
    """Forward distance for image row ``v_px``. Same formula as geometry.py."""
    k = (v_px - cy) / fy
    den = k * math.cos(th) + math.sin(th)
    if den <= 1e-9:
        return float("inf")                # at or above the horizon
    z = h * (math.cos(th) - k * math.sin(th)) / den
    return z if z > 0 else float("inf")


def fit(points: List[Point], cam: CameraModel, fit_focal: bool = False
        ) -> dict:
    """Least squares on log range, which weights near and far points equally.

    Fitting on raw metres would let a single 25 m marker dominate and let the
    3 m one -- the one that decides whether the cart stops in time -- drift by
    half a metre unnoticed. Relative error is what the policy actually cares
    about, so relative error is what gets minimised.
    """
    import numpy as np
    from scipy.optimize import least_squares

    v = np.array([p[1] for p in points], float)
    z = np.array([p[2] for p in points], float)

    def resid(x):
        h, th = x[0], x[1]
        fy = cam.fy * (x[2] if fit_focal else 1.0)
        pred = np.array([predict_range(vi, cam.cy, fy, h, th) for vi in v])
        pred = np.where(np.isfinite(pred), pred, 1e6)
        return np.log(np.clip(pred, 1e-3, None)) - np.log(z)

    x0 = [cam.height_m, cam.pitch_rad, 1.0]
    lo = [0.3, math.radians(-20.0), 0.5]
    hi = [3.0, math.radians(40.0), 2.0]
    if not fit_focal:
        x0[2], lo[2], hi[2] = 1.0, 1.0 - 1e-9, 1.0 + 1e-9

    sol = least_squares(resid, x0, bounds=(lo, hi))
    h, th, fs = sol.x
    fy = cam.fy * fs
    pred = [predict_range(p[1], cam.cy, fy, h, th) for p in points]
    err = [pr - p[2] for pr, p in zip(pred, points)]
    rel = [abs(e) / p[2] for e, p in zip(err, points)]

    return {
        "height_m": float(h),
        "pitch_deg": float(math.degrees(th)),
        "focal_scale": float(fs),
        "predicted_m": [float(p) for p in pred],
        "error_m": [float(e) for e in err],
        "max_rel_error": float(max(rel)) if rel else 0.0,
        "rms_rel_error": float(math.sqrt(sum(r * r for r in rel) / len(rel)))
        if rel else 0.0,
    }


# ---------------------------------------------------------------------------
# Getting the points
# ---------------------------------------------------------------------------
def grab_frame(width: int, height: int, fps: int, device: int = 0):
    import cv2

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    frame = None
    for _ in range(60):                    # let auto-exposure settle
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    if frame is None:
        raise RuntimeError(f"/dev/video{device} gave no frames")
    return frame


def click_points(frame, cam: CameraModel) -> List[Point]:
    """Click each marker's ground contact point; type its measured distance."""
    import cv2

    disp_scale = min(1.0, 1400.0 / frame.shape[1])
    view = cv2.resize(frame, None, fx=disp_scale, fy=disp_scale)
    horizon = cam.horizon_y() * disp_scale
    if 0 <= horizon < view.shape[0]:
        cv2.line(view, (0, int(horizon)), (view.shape[1], int(horizon)),
                 (0, 0, 255), 1)
        cv2.putText(view, "horizon (current guess)", (10, int(horizon) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

    clicks: List[Tuple[float, float]] = []

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x / disp_scale, y / disp_scale))
            cv2.drawMarker(view, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            cv2.imshow("ground calibration", view)

    cv2.namedWindow("ground calibration", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("ground calibration", on_mouse)
    cv2.imshow("ground calibration", view)
    print("\nClick where each marker TOUCHES THE GROUND. Press 'q' when done.")
    while True:
        if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()

    points: List[Point] = []
    for i, (u, v) in enumerate(clicks):
        while True:
            s = input(f"  point {i + 1} at pixel ({u:.0f},{v:.0f}) — measured "
                      f"distance from directly below the lens, in metres: ")
            try:
                points.append((u, v, float(s)))
                break
            except ValueError:
                print("    a number, please")
    return points


def parse_points(spec: str) -> List[Point]:
    """``v:Z,v:Z`` or ``u,v:Z`` pairs, for when you already have the numbers."""
    out: List[Point] = []
    for chunk in spec.split(","):
        px, _, rng = chunk.partition(":")
        if "," in px:
            u, v = (float(t) for t in px.split(","))
        else:
            u, v = float("nan"), float(px)
        out.append((u, v, float(rng)))
    return out


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=None, help="calibration JSON to update")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--image", default=None, help="fit against a saved frame")
    ap.add_argument("--points", default=None,
                    help="non-interactive: 'v:range,v:range,...'")
    ap.add_argument("--fit-focal", action="store_true",
                    help="also solve a correction to fy (needs 3+ points)")
    ap.add_argument("--write", action="store_true",
                    help="write the fit back into the calibration JSON")
    ap.add_argument("--save-frame", default=None,
                    help="also save the captured frame here")
    args = ap.parse_args()

    path = args.calib or calib_mod.DEFAULT_CALIB
    calib = calib_mod.load(path)
    cam = calib_mod.camera_model(calib)
    print(calib_mod.describe(cam, calib))

    if args.points:
        points = parse_points(args.points)
    else:
        import cv2
        if args.image:
            frame = cv2.imread(args.image)
            if frame is None:
                print(f"cannot read {args.image}")
                return 1
        else:
            w, h = calib_mod.capture_size(calib)
            print(f"[calib] capturing {w}x{h} from /dev/video{args.device} ...")
            frame = grab_frame(w, h, calib_mod.capture_fps(calib), args.device)
            if args.save_frame:
                cv2.imwrite(args.save_frame, frame)
                print(f"[calib] frame saved to {args.save_frame}")
        if (frame.shape[1], frame.shape[0]) != calib_mod.capture_size(calib):
            print(f"[calib] WARNING: frame is {frame.shape[1]}x{frame.shape[0]}"
                  f" but the calibration expects "
                  f"{calib_mod.capture_size(calib)}; pixel rows will not mean "
                  f"what the model thinks they mean.")
        points = click_points(frame, cam)

    if len(points) < 2:
        print("need at least 2 points (3 if fitting focal length)")
        return 1
    if args.fit_focal and len(points) < 3:
        print("--fit-focal needs at least 3 points")
        return 1
    spread = max(p[2] for p in points) / max(1e-6, min(p[2] for p in points))
    if spread < 2.5:
        print(f"WARNING: your markers span only {spread:.1f}x in range. Height "
              f"and pitch trade off against each other over a short baseline, "
              f"so this fit will look good and be wrong. Add a far marker.")

    res = fit(points, cam, args.fit_focal)

    print("\n  measured   predicted   error")
    for p, pred, err in zip(points, res["predicted_m"], res["error_m"]):
        print(f"  {p[2]:7.2f} m {pred:9.2f} m {err:+7.2f} m   (row {p[1]:.0f})")
    print(f"\n  height_m   = {res['height_m']:.3f}")
    print(f"  pitch_deg  = {res['pitch_deg']:.3f}")
    if args.fit_focal:
        print(f"  focal_scale= {res['focal_scale']:.4f}  "
              f"(fy {cam.fy:.1f} -> {cam.fy * res['focal_scale']:.1f})")
        if abs(res["focal_scale"] - 1.0) > 0.05:
            print("  *** The focal length needed a correction of more than 5%. "
                  "The intrinsics do NOT scale cleanly from the calibrated "
                  "mode to this capture mode. Recalibrate with a board at the "
                  "capture resolution rather than trusting this fudge. ***")
    print(f"  rms relative error {res['rms_rel_error'] * 100:.1f}%, "
          f"worst {res['max_rel_error'] * 100:.1f}%")
    if res["max_rel_error"] > 0.10:
        print("  *** Worse than 10% at some marker. Something is off: check "
              "the ground is flat, that you clicked the contact point and not "
              "the marker's centre, and that distances were measured from "
              "below the LENS. Do not write this. ***")

    if args.write:
        mnt = calib.setdefault("mount", {})
        mnt["height_m"] = round(res["height_m"], 4)
        mnt["pitch_deg"] = round(res["pitch_deg"], 4)
        mnt["measured"] = True
        mnt["source"] = "tools/percept_ground_calib.py"
        mnt["fit"] = {
            "points": [{"u": p[0], "v": p[1], "range_m": p[2]} for p in points],
            "rms_rel_error": round(res["rms_rel_error"], 5),
            "max_rel_error": round(res["max_rel_error"], 5),
        }
        if args.fit_focal:
            ins = calib["intrinsics"]
            ins["fy"] = ins["fy"] * res["focal_scale"]
            ins["focal_scale_applied"] = round(res["focal_scale"], 5)
        with open(path, "w") as fh:
            json.dump(calib, fh, indent=2)
            fh.write("\n")
        print(f"\n[calib] written to {path}")
    else:
        print("\n(nothing written — re-run with --write when the fit looks "
              "right)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
