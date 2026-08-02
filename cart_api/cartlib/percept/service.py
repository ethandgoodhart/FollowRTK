"""
cartlib.percept.service — the live loop: camera in, one allowed speed out.

This is the only place the perception stack touches the running cart, and it is
built so that the touching is a single float. The service owns a thread paced by
the camera; the follower keeps its own 15 Hz loop and simply reads
``service.v_allowed_mph`` whenever it likes. Nothing is shared but that number
and the pose the follower hands back.

Why the two loops are separate
------------------------------
They run at different rates for different reasons -- perception at whatever the
camera gives (about 28 Hz), control at 15 Hz because that is what the pedals
and steering want. Coupling them would mean the slower one paces the faster, and
a dropped frame would become a skipped control tick. Keeping them apart also
means a perception crash cannot take the steering with it: the watchdog below
turns a dead thread into "no speed authority granted", not into an exception
inside the control loop.

Shadow mode
-----------
Defaults to ON. In shadow mode everything runs and publishes -- the minimap is
live, the decisions are real -- but ``v_allowed_mph`` reports ``None`` so the
follower ignores it. That is the mode to drive in for the first few outings:
you get to watch the system be right or wrong before it is allowed to be either
on your behalf.

Pose
----
Perception needs to know where the cart is and which way it points, and the
follower is the only thing that knows. So the follower pushes a pose in each
control tick (``set_context``) and the service converts it into the local metric
frame the tracker uses. If poses stop arriving, tracking is still valid in the
cart frame for a moment but world-frame velocities are not, so the service
degrades rather than guessing.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, List, Optional, Sequence, Tuple

from .. import geo
from .camera import CameraStream
from .detector import Detector
from .geometry import CameraModel, project_detection
from .policy import MPS_TO_MPH, Governor, PolicyConfig
from .telemetry import perception_payload
from .track import EgoPose, Tracker

LatLon = Tuple[float, float]


class PerceptionService:
    """Camera -> detector -> tracker -> policy, on its own thread."""

    def __init__(self, cam: Optional[CameraModel] = None,
                 cfg: Optional[PolicyConfig] = None,
                 weights: str = "yolo11m.pt", imgsz: int = 960,
                 device: int = 0, conf: float = 0.35,
                 shadow: bool = True,
                 publish: Optional[Callable[[dict], None]] = None,
                 publish_hz: float = 10.0,
                 cap_width: int = 1600, cap_height: int = 1200,
                 cap_fps: int = 90, cam_device: int = 0):
        self.cam = cam or CameraModel()
        self.cfg = cfg or PolicyConfig()
        self.shadow = shadow
        self.publish = publish
        self.publish_period = 1.0 / max(publish_hz, 1.0)

        self.stream = CameraStream(device=cam_device, width=cap_width,
                                   height=cap_height, fps=cap_fps)
        self.detector = Detector(weights=weights, imgsz=imgsz, device=device,
                                 conf=conf)
        self.tracker = Tracker()
        self.governor = Governor(self.cfg, initial_mph=0.0)

        # Context pushed in by the follower.
        self._ctx_lock = threading.Lock()
        self._origin: Optional[LatLon] = None
        self._ego: Optional[EgoPose] = None
        self._route_xy: List[Tuple[float, float]] = []
        self._ctx_ts: float = 0.0

        # Outputs.
        self._out_lock = threading.Lock()
        self._v_allowed: Optional[float] = None
        self._decision_dict: dict = {}
        self._last_tick: float = 0.0

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[str] = None
        self.hz = 0.0

    # -- lifecycle --------------------------------------------------------
    def start(self) -> "PerceptionService":
        self.stream.open()
        if not self.stream.wait_for_frame(timeout=8.0):
            raise RuntimeError(self.stream.error or "camera produced no frames")
        # A driver that quietly hands back a different size than it was asked
        # for turns every row into the wrong distance, and nothing downstream
        # can tell. Check once, loudly, rather than range wrongly all drive.
        frame, _ = self.stream.read()
        h, w = frame.shape[:2]
        if (w, h) != (self.cam.width, self.cam.height):
            self.stream.close()
            raise RuntimeError(
                f"camera delivered {w}x{h} but the calibration describes "
                f"{self.cam.width}x{self.cam.height}. Every range would be "
                f"wrong by the ratio. Fix the capture size or the calibration.")
        self.detector.load()
        self._thread = threading.Thread(target=self._run, name="perception",
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self.stream.close()

    def __enter__(self) -> "PerceptionService":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- what the follower calls -----------------------------------------
    def set_context(self, pos: LatLon, heading_deg: float, speed_mph: float,
                    path: Optional[Sequence[LatLon]] = None) -> None:
        """Tell perception where we are, where we point, and where we are going.

        Cheap enough to call every control tick, and it must be: a pose that is
        a second old puts every tracked object a couple of metres from where it
        really is.
        """
        with self._ctx_lock:
            if self._origin is None:
                # First fix becomes the local frame's origin. Fixing it once
                # keeps world coordinates stable for the whole drive, which is
                # what lets a stationary object hold a zero velocity.
                self._origin = pos
            x, y = geo.local_xy(self._origin, pos)
            self._ego = EgoPose(x=x, y=y, heading_deg=heading_deg % 360.0,
                                speed_ms=speed_mph * 0.44704, ts=time.monotonic())
            if path is not None:
                self._route_xy = [geo.local_xy(self._origin, p) for p in path]
            self._ctx_ts = time.monotonic()

    @property
    def v_allowed_mph(self) -> Optional[float]:
        """The speed cap perception grants, or None if it is not granting one.

        ``None`` means "do not let me influence the cart" and is returned in
        shadow mode, before the first frame, and whenever the watchdog decides
        the thread has stopped producing. The follower treats None as "no
        opinion" rather than as zero, because a perception fault should not be
        able to phantom-brake a cart in the middle of a road -- that is the
        operator's call, and the follower's own watchdogs already cover a truly
        dead system.
        """
        with self._out_lock:
            if self.shadow or self._v_allowed is None:
                return None
            if time.monotonic() - self._last_tick > 1.0:
                return None
            return self._v_allowed

    @property
    def decision(self) -> dict:
        with self._out_lock:
            return dict(self._decision_dict)

    @property
    def healthy(self) -> bool:
        return (self.error is None and self.stream.healthy
                and time.monotonic() - self._last_tick < 1.0)

    # -- the thread -------------------------------------------------------
    def _run(self) -> None:
        last_pub = 0.0
        last_frame_ts = -1.0
        prev = time.monotonic()
        try:
            while not self._stop.is_set():
                frame, cap_ts = self.stream.read()
                if frame is None or cap_ts == last_frame_ts:
                    time.sleep(0.004)          # no new frame yet
                    if self.stream.error:
                        self.error = self.stream.error
                        return
                    continue
                last_frame_ts = cap_ts

                with self._ctx_lock:
                    ego, route_xy, ctx_ts = self._ego, list(self._route_xy), self._ctx_ts

                dets_raw = self.detector.detect(frame)

                now = time.monotonic()
                frame_age = max(0.0, now - cap_ts)

                if ego is None or now - ctx_ts > 1.0:
                    # No usable pose: we can see, but we cannot say where any of
                    # it is in the world, so tracking would be nonsense. Publish
                    # the fact rather than inventing a pose.
                    self._set_output(None, {"layer": "degraded",
                                            "reason": "no GPS pose for perception",
                                            "v_allowed_mph": 0.0,
                                            "emergency": False, "degraded": True,
                                            "limiting_track_id": None,
                                            "conflicts": []}, now)
                    continue

                dets = []
                for bbox, cls, conf in dets_raw:
                    d = project_detection(self.cam, bbox, cls, conf)
                    if d is not None:
                        dets.append(d)

                tracks = self.tracker.update(dets, ego, now)
                dt = min(max(now - prev, 1e-3), 0.5)
                prev = now
                v = self.governor.step(tracks, route_xy, ego, now, dt,
                                       detector_ok=self.detector.loaded,
                                       frame_age_s=frame_age)
                self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt)
                self._set_output(v, self.governor.decision.to_dict(), now)

                if self.publish and (now - last_pub) >= self.publish_period:
                    last_pub = now
                    try:
                        self.publish(perception_payload(
                            tracks, self.governor.decision, ego, self.cfg,
                            detector_hz=self.hz, frame_age_s=frame_age,
                            ts=time.time(), shadow=self.shadow,
                            fov_deg=self.cam.hfov_deg()))
                    except Exception as e:               # never kill the loop
                        print(f"[percept] publish failed: {e}")
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            print(f"[percept] thread died: {self.error}")

    def _set_output(self, v: Optional[float], decision: dict,
                    now: float) -> None:
        with self._out_lock:
            self._v_allowed = v
            self._decision_dict = decision
            self._last_tick = now
