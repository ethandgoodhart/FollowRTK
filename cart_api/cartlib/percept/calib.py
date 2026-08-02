"""
cartlib.percept.calib — load the front camera's calibration from disk.

There is exactly one place the numbers live (``calibration/front_camera.json``)
and exactly one function that turns them into a ``CameraModel``. Nothing else
in the stack is allowed to invent intrinsics, because a camera model that
differs between the live service and the calibration tool is a camera model
nobody can check.

Two rules this module enforces rather than documents:

  * INTRINSICS SCALE ONLY WITHIN AN ASPECT RATIO. Multiplying fx and fy by the
    resolution ratio is valid when the sensor is showing the same field of view
    at more pixels. It is nonsense when the mode changes the field, which is
    what a different aspect ratio usually means -- and this camera offers both
    4:3 and 16:10 modes. Capturing 1920x1200 against a 640x480 calibration
    would silently scale fy by 2.5 while the true field changed, and a 12%
    error in fy is a 12% error in every range at every distance. So a mismatch
    raises instead of guessing.

  * AN UNMEASURED MOUNT IS ANNOUNCED. ``height_m`` and ``pitch_deg`` cannot be
    read off a calibration board; they need a tape measure. Until somebody has
    run the ground calibration, ``mount.measured`` is false and every consumer
    prints that it is driving on assumed geometry.
"""

from __future__ import annotations

import json
import os
from typing import Optional, Tuple

from .geometry import CameraModel

# cartlib/percept/calib.py -> cart_api/calibration/front_camera.json
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CALIB = os.path.normpath(
    os.path.join(_HERE, "..", "..", "calibration", "front_camera.json"))


def load(path: Optional[str] = None) -> dict:
    with open(path or DEFAULT_CALIB) as fh:
        return json.load(fh)


def capture_size(calib: dict) -> Tuple[int, int]:
    cap = calib.get("capture", {})
    return int(cap.get("width", 1600)), int(cap.get("height", 1200))


def capture_fps(calib: dict) -> int:
    return int(calib.get("capture", {}).get("fps", 90))


def mount_measured(calib: dict) -> bool:
    return bool(calib.get("mount", {}).get("measured", False))


def camera_model(calib: dict, width: Optional[int] = None,
                 height: Optional[int] = None,
                 allow_aspect_mismatch: bool = False) -> CameraModel:
    """Build the ``CameraModel`` for a given capture size.

    ``width``/``height`` default to the calibration's own capture block, which
    is the size the rest of the stack should be running at.
    """
    ins, mnt = calib["intrinsics"], calib.get("mount", {})
    cw, ch = int(ins["calibrated_width"]), int(ins["calibrated_height"])
    if width is None or height is None:
        width, height = capture_size(calib)

    sx, sy = width / cw, height / ch
    if abs(sx - sy) > 0.01 * max(sx, sy) and not allow_aspect_mismatch:
        raise ValueError(
            f"calibration is {cw}x{ch} but capture is {width}x{height}: the "
            f"aspect ratios differ, so scaling fx by {sx:.3f} and fy by "
            f"{sy:.3f} would be inventing a lens. Capture at a "
            f"{cw}:{ch}-aspect mode (e.g. {cw * 2}x{ch * 2}, {int(cw * 2.5)}x"
            f"{int(ch * 2.5)}) or recalibrate at {width}x{height}.")

    return CameraModel(
        width=int(width), height=int(height),
        fx=float(ins["fx"]) * sx, fy=float(ins["fy"]) * sy,
        cx=float(ins["cx"]) * sx, cy=float(ins["cy"]) * sy,
        dist=tuple(float(d) for d in ins.get("dist", (0, 0, 0, 0, 0))),
        height_m=float(mnt.get("height_m", 1.45)),
        pitch_deg=float(mnt.get("pitch_deg", 6.0)),
        yaw_deg=float(mnt.get("yaw_deg", 0.0)),
        offset_forward_m=float(mnt.get("offset_forward_m", 1.6)),
        offset_lateral_m=float(mnt.get("offset_lateral_m", 0.0)),
        pitch_sigma_deg=float(mnt.get("pitch_sigma_deg", 1.0)),
    )


def load_camera(path: Optional[str] = None, width: Optional[int] = None,
                height: Optional[int] = None) -> Tuple[CameraModel, dict]:
    """Convenience: ``(CameraModel, calib_dict)`` in one call."""
    calib = load(path)
    return camera_model(calib, width, height), calib


def describe(cam: CameraModel, calib: dict) -> str:
    """One block of text worth printing at startup on a safety-critical rig."""
    lines = [
        f"[calib] {calib.get('name', '?')} {cam.width}x{cam.height}  "
        f"fx={cam.fx:.1f} fy={cam.fy:.1f} cx={cam.cx:.1f} cy={cam.cy:.1f}",
        f"[calib] hfov={cam.hfov_deg():.1f}deg vfov={cam.vfov_deg():.1f}deg  "
        f"h={cam.height_m:.2f}m pitch={cam.pitch_deg:.2f}deg",
        f"[calib] intrinsics: {calib['intrinsics'].get('method', 'unknown')}",
    ]
    if not mount_measured(calib):
        lines.append(
            "[calib] *** MOUNT GEOMETRY IS NOT MEASURED. Height and pitch are "
            "placeholders, so every range is a guess. Run "
            "tools/percept_ground_calib.py before trusting a single number, "
            "and do not take this cart out of shadow mode until you have. ***")
    return "\n".join(lines)
