"""
cartlib.percept.preview — a JPEG the operator can glance at.

The detector already has the frame; this just downscales it, draws the boxes
(with range in feet) and the modelled horizon, and base64-encodes a JPEG. The
horizon line is the point of the overlay: when the operator moves the pitch
slider they should see that line sit on the real horizon, which is the
fastest way to tell a wrong pitch from a wrong height.
"""

from __future__ import annotations

import base64
from typing import Optional, Sequence

import numpy as np

from .detector import Detection
from .geometry import CameraModel, project_detection

M_TO_FT = 3.280839895

# BGR, matching the minimap's class colours so a glance transfers.
_CLS_BGR = {
    "person": (248, 189, 56),
    "bicycle": (238, 211, 34),
    "motorcycle": (238, 211, 34),
    "dog": (250, 139, 167),
    "car": (36, 191, 251),
    "truck": (60, 146, 251),
    "bus": (60, 146, 251),
}

PREVIEW_WIDTH = 480
JPEG_QUALITY = 55


def encode_preview(frame: np.ndarray,
                   dets: Sequence[Detection] = (),
                   cam: Optional[CameraModel] = None,
                   width: int = PREVIEW_WIDTH) -> str:
    """Return a base64 JPEG, or "" if encoding fails."""
    import cv2

    if frame is None or frame.size == 0:
        return ""
    h, w = frame.shape[:2]
    if w < 2 or h < 2:
        return ""
    scale = width / float(w)
    small = cv2.resize(frame, (width, max(1, int(round(h * scale)))),
                       interpolation=cv2.INTER_AREA)

    if cam is not None:
        hy = int(round(cam.horizon_y() * scale))
        if 0 <= hy < small.shape[0]:
            cv2.line(small, (0, hy), (small.shape[1] - 1, hy),
                     (160, 160, 160), 1, cv2.LINE_AA)

    for bbox, cls, conf in dets:
        color = _CLS_BGR.get(cls, (170, 170, 170))
        x1, y1, x2, y2 = bbox
        p1 = (int(round(x1 * scale)), int(round(y1 * scale)))
        p2 = (int(round(x2 * scale)), int(round(y2 * scale)))
        cv2.rectangle(small, p1, p2, color, 2)
        label = cls
        if cam is not None:
            gd = project_detection(cam, bbox, cls, conf)
            if gd is not None:
                label = f"{cls} {gd.forward_m * M_TO_FT:.0f}ft"
        ty = max(12, p1[1] - 4)
        cv2.putText(small, label, (p1[0], ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, color, 1, cv2.LINE_AA)

    ok, buf = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if not ok:
        return ""
    return base64.b64encode(buf.tobytes()).decode("ascii")
