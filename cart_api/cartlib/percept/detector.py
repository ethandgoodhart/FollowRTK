"""
cartlib.percept.detector — YOLO, wrapped down to the one thing we need.

In: a BGR frame. Out: a list of (bbox, class name, confidence) for the classes
that can be hurt by a cart or can hurt one. Nothing else. The wrapper exists so
that the rest of the stack never imports ultralytics, never sees a Results
object, and can be tested without a GPU.

Model choice is measured, not assumed. On this Thor (tools/bench_models.py,
end-to-end predict() including preprocess and NMS):

    yolo11n @ 960   118 Hz median / 91 Hz p95
    yolo11s @ 960   106 / 82
    yolo11m @ 960    68 / 55      <- default
    yolo11m @ 1280   30 / 27

The default is the largest model whose p95 still clears the 15 Hz control loop
several times over, because the thing that actually limits this system is
detecting a small, distant or partly-occluded pedestrian -- not throughput. The
camera tops out near 28 Hz anyway, so the extra model capacity is free.
"""

from __future__ import annotations

import time
from typing import List, Optional, Sequence, Tuple

# COCO ids the speed controller cares about, mapped to the names the geometry
# and tracking layers use. Everything else is filtered out inside the detector
# so no downstream code has to know COCO exists.
VRU_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
               5: "bus", 7: "truck", 16: "dog"}

Detection = Tuple[Tuple[float, float, float, float], str, float]


class Detector:
    """Thin, eager-loading wrapper around an ultralytics YOLO model."""

    def __init__(self, weights: str = "yolo11m.pt", imgsz: int = 960,
                 conf: float = 0.35, device: int = 0, half: bool = True):
        self.weights, self.imgsz, self.conf = weights, imgsz, conf
        self.device, self.half = device, half
        self._model = None
        self._half_kw = "half"
        self.last_ms = 0.0
        self.frames = 0

    def load(self) -> "Detector":
        """Load and warm up. Warm-up matters: the first inference on a cold
        CUDA context takes hundreds of milliseconds, and paying that inside the
        control loop means the very first frame of a drive reacts late."""
        from ultralytics import YOLO

        import numpy as np

        # Pick the half-precision keyword this ultralytics understands, once,
        # rather than eating a deprecation warning on every frame.
        try:
            from ultralytics.cfg import DEFAULT_CFG_DICT
            self._half_kw = "quantize" if "quantize" in DEFAULT_CFG_DICT else "half"
        except Exception:
            self._half_kw = "half"

        task = "detect"
        self._model = (YOLO(self.weights, task=task)
                       if self.weights.endswith(".engine")
                       else YOLO(self.weights))
        blank = np.zeros((self.imgsz, self.imgsz, 3), dtype="uint8")
        for _ in range(3):
            self._predict(blank)
        return self

    @property
    def loaded(self) -> bool:
        return self._model is not None

    def _predict(self, frame):
        kw = dict(imgsz=self.imgsz, conf=self.conf, classes=list(VRU_CLASSES),
                  verbose=False, device=self.device)
        if not self.weights.endswith(".engine"):
            # ultralytics 8.4 renamed half= to quantize= and warns once per
            # predict() call, which at 30 Hz buries every real log line under
            # deprecation notices. Use whichever name this version accepts.
            kw[self._half_kw] = "fp16" if self._half_kw == "quantize" else self.half
        return self._model.predict(frame, **kw)[0]

    def detect(self, frame) -> List[Detection]:
        """One frame in, boxes out. Empty list if the model is not loaded."""
        if self._model is None:
            return []
        t0 = time.perf_counter()
        res = self._predict(frame)
        self.last_ms = (time.perf_counter() - t0) * 1000.0
        self.frames += 1

        out: List[Detection] = []
        boxes = getattr(res, "boxes", None)
        if boxes is None:
            return out
        for b in boxes:
            cls_id = int(b.cls.item())
            name = VRU_CLASSES.get(cls_id)
            if name is None:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            out.append(((x1, y1, x2, y2), name, float(b.conf.item())))
        return out
