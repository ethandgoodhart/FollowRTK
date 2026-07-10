#
# predictive.py
#
# Created on July 10, 2026
#
# Created by Georg von Manstein
#

"""
obstacle.predictive — track objects, predict their paths and the cart's,
and compute HOW MUCH to brake (0..1), not just whether to stop.

Pipeline per frame:
  1. YOLO + ByteTrack gives persistent per-object IDs.
  2. Each object's footpoint (bottom-center of its box) is projected onto the
     ground plane via a flat-ground pinhole model -> cart-relative (X lateral,
     Z forward) in meters.
  3. A short least-squares fit over each track's recent footpoints gives its
     velocity *relative to the cart* (this bakes in our own motion — a static
     tree "approaches" at -v_ego, a pedestrian pacing us holds Z steady).
  4. The cart's own trajectory comes from ego speed + yaw rate (eval: the
     synchronized ego.jsonl; live: GPS). Its curvature bends the collision
     corridor; objects are checked in path-relative coordinates.
  5. If an object's predicted path enters the corridor while inside the
     standoff distance, we compute the deceleration required to arrive at its
     crossing point no faster than it can clear -> brake fraction a/A_MAX.

The output is a graduated brake signal: 0.2 = ease off, 1.0 = emergency.
"""

from __future__ import annotations

import bisect
import json
import math
import threading
import time
from collections import deque

import cv2
import numpy as np
from ultralytics import YOLO

from detector import OBSTACLE_CLASSES, CONF_THRESHOLD

# ---- camera model (approximate; tune once against a known-distance object) --
CAM_HEIGHT_M = 1.35     # lens height above ground
CAM_PITCH_DEG = 6.0     # downward tilt
HFOV_DEG = 70.0         # horizontal field of view

# ---- cart + physics ---------------------------------------------------------
CART_HALF_WIDTH_M = 0.8     # half of cart width...
LATERAL_MARGIN_M = 0.3      # ...plus clearance we refuse to shave
STANDOFF_M = 2.0            # bumper buffer: never plan to get closer than this
A_MAX = 3.0                 # max comfortable-firm decel (m/s^2) = brake 1.0
HORIZON_S = 4.0             # how far ahead we predict
EMERGENCY_TTC_S = 0.8       # hit sooner than this -> slam it regardless

TRACK_WINDOW_S = 1.2        # footpoint history used for velocity fit
TRACK_MIN_SPAN_S = 0.30     # need this much history before trusting a velocity
TRACK_STALE_S = 1.0         # drop tracks unseen this long
MAX_RANGE_M = 40.0          # ignore ground hits farther than this (noise)
BRAKE_DECAY = 0.92          # per-frame release rate (attack is instant)


class EgoLog:
    """speed/yaw-rate lookup by video time from the synchronized ego.jsonl."""

    def __init__(self, path: str):
        self.t: list[float] = []
        self.speed: list[float] = []
        self.yaw_rate: list[float] = []
        with open(path) as f:
            for line in f:
                d = json.loads(line)
                if "rel_t" not in d or "speed_mps" not in d:
                    continue
                self.t.append(d["rel_t"])
                self.speed.append(d["speed_mps"])
                self.yaw_rate.append(d.get("yaw_rate_rad_s", 0.0))

    def at(self, video_t: float) -> tuple[float, float]:
        if not self.t:
            return 0.0, 0.0
        i = min(bisect.bisect_left(self.t, video_t), len(self.t) - 1)
        return self.speed[i], self.yaw_rate[i]


class GroundCamera:
    """Flat-ground pinhole: pixel <-> cart-relative ground meters."""

    def __init__(self, w: int, h: int):
        self.w, self.h = w, h
        self.cx, self.cy = w / 2.0, h / 2.0
        self.fx = (w / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
        self.fy = self.fx  # square pixels
        self.pitch = math.radians(CAM_PITCH_DEG)

    def px_to_ground(self, u: float, v: float) -> tuple[float, float] | None:
        """(u,v) image -> (X right, Z forward) meters, None above horizon."""
        beta = self.pitch + math.atan2(v - self.cy, self.fy)
        if beta <= math.radians(0.5):
            return None
        z = CAM_HEIGHT_M / math.tan(beta)
        if z > MAX_RANGE_M:
            return None
        x = (u - self.cx) / self.fx * z
        return x, z

    def ground_to_px(self, x: float, z: float) -> tuple[int, int] | None:
        if z <= 0.3:
            return None
        beta = math.atan2(CAM_HEIGHT_M, z)
        v = self.cy + self.fy * math.tan(beta - self.pitch)
        u = self.cx + self.fx * (x / z)
        return int(u), int(v)


class Track:
    def __init__(self, tid: int, cls: str):
        self.id = tid
        self.cls = cls
        self.conf = 0.0
        self.box = (0, 0, 0, 0)
        self.hist: deque[tuple[float, float, float]] = deque()  # (t, X, Z)
        self.last_seen = 0.0
        self.vel: tuple[float, float] | None = None  # (Vx, Vz) cart-relative

    def update(self, t: float, x: float, z: float) -> None:
        self.last_seen = t
        self.hist.append((t, x, z))
        while self.hist and t - self.hist[0][0] > TRACK_WINDOW_S:
            self.hist.popleft()
        span = self.hist[-1][0] - self.hist[0][0]
        if len(self.hist) >= 3 and span >= TRACK_MIN_SPAN_S:
            ts = np.array([p[0] for p in self.hist])
            xs = np.array([p[1] for p in self.hist])
            zs = np.array([p[2] for p in self.hist])
            ts = ts - ts.mean()
            denom = float((ts * ts).sum()) or 1e-6
            vx = float((ts * (xs - xs.mean())).sum() / denom)
            vz = float((ts * (zs - zs.mean())).sum() / denom)
            self.vel = (vx, vz)

    @property
    def pos(self) -> tuple[float, float]:
        _, x, z = self.hist[-1]
        return x, z


def path_lateral_offset(z: float, curvature: float) -> float:
    """Lateral offset of the cart's future path at forward distance z.

    Small-angle arc: x(z) ~= curvature * z^2 / 2 (curvature = yaw_rate / v)."""
    return 0.5 * curvature * z * z


class PredictiveAvoidance:
    """Drop-in engine for detector.playback_loop: process() + status/jpeg."""

    def __init__(self, model_path: str, device: str | None = None,
                 ego: EgoLog | None = None):
        self.model = YOLO(model_path)
        self.device = device
        self.ego = ego
        self.lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.status: dict = {"brake": False, "brake_fraction": 0.0,
                             "detections": [], "connected": False}
        self.tracks: dict[int, Track] = {}
        self.cam: GroundCamera | None = None
        self._brake_out = 0.0

    def reset(self) -> None:
        self.tracks.clear()
        self._brake_out = 0.0

    # ---- collision & braking ------------------------------------------------
    def _assess(self, trk: Track, v_ego: float, curvature: float) -> dict:
        """Predict this track vs our arc; return collision verdict + brake."""
        x0, z0 = trk.pos
        vx, vz = trk.vel if trk.vel else (0.0, 0.0)
        half_w = CART_HALF_WIDTH_M + LATERAL_MARGIN_M

        # Already inside the standoff bubble and in our lane -> hard case.
        if z0 < STANDOFF_M and abs(x0 - path_lateral_offset(z0, curvature)) < half_w:
            return {"collide": True, "t_hit": 0.0, "a_req": A_MAX}

        if trk.vel is None:
            return {"collide": False, "t_hit": None, "a_req": 0.0}

        # Sample the relative trajectory over the horizon; find when the
        # object sits inside the corridor within stopping-relevant range.
        t_hit = None
        for t in np.arange(0.0, HORIZON_S, 0.1):
            z = z0 + vz * t
            if z < 0.2:  # passed us / reached bumper plane
                break
            x_rel = (x0 + vx * t) - path_lateral_offset(z, curvature)
            if abs(x_rel) < half_w and z < STANDOFF_M + v_ego * t:
                # It occupies our path at a range we'd cover by then.
                t_hit = float(t)
                break
        if t_hit is None:
            return {"collide": False, "t_hit": None, "a_req": 0.0}

        # Graduated braking: slow enough to cover the available gap at the
        # object's own forward speed instead of ours. v_obj_fwd is its
        # world-frame forward speed (relative + ego); a static obstacle gives
        # the classic v^2 / 2d full-stop profile.
        z_hit = max(z0 + vz * t_hit, STANDOFF_M)
        gap = max(z_hit - STANDOFF_M, 0.3)
        v_obj_fwd = max(0.0, vz + v_ego)
        a_req = (v_ego * v_ego - v_obj_fwd * v_obj_fwd) / (2.0 * gap)
        a_req = max(0.0, a_req)
        if t_hit < EMERGENCY_TTC_S:
            a_req = A_MAX
        return {"collide": True, "t_hit": t_hit, "a_req": a_req}

    # ---- per-frame entry point ----------------------------------------------
    def process(self, frame: np.ndarray, video_t: float) -> np.ndarray:
        h, w = frame.shape[:2]
        if self.cam is None:
            self.cam = GroundCamera(w, h)
        v_ego, yaw_rate = self.ego.at(video_t) if self.ego else (0.0, 0.0)
        curvature = yaw_rate / v_ego if v_ego > 0.5 else 0.0

        res = self.model.track(frame, persist=True, verbose=False,
                               device=self.device, conf=CONF_THRESHOLD)[0]
        names = res.names

        seen: set[int] = set()
        for box in res.boxes:
            if box.id is None:
                continue
            cls = names[int(box.cls[0])]
            if cls not in OBSTACLE_CLASSES:
                continue
            tid = int(box.id[0])
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            ground = self.cam.px_to_ground((x1 + x2) / 2.0, y2)
            if ground is None:
                continue
            trk = self.tracks.setdefault(tid, Track(tid, cls))
            trk.cls, trk.conf = cls, float(box.conf[0])
            trk.box = (int(x1), int(y1), int(x2), int(y2))
            trk.update(video_t, *ground)
            seen.add(tid)

        for tid in [t for t, trk in self.tracks.items()
                    if video_t - trk.last_seen > TRACK_STALE_S]:
            del self.tracks[tid]

        # Assess every live track; brake for the worst offender.
        verdicts: dict[int, dict] = {}
        a_worst = 0.0
        for tid, trk in self.tracks.items():
            if tid not in seen:
                continue
            v = self._assess(trk, v_ego, curvature)
            verdicts[tid] = v
            a_worst = max(a_worst, v["a_req"])

        target = min(1.0, a_worst / A_MAX)
        # Attack instantly, release gradually (no brake flicker mid-maneuver).
        self._brake_out = target if target > self._brake_out \
            else self._brake_out * BRAKE_DECAY
        if self._brake_out < 0.03:
            self._brake_out = 0.0
        brake_fraction = round(self._brake_out, 3)

        annotated = self._annotate(frame, verdicts, v_ego, curvature, brake_fraction)
        ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])

        tracks_out = []
        for tid, trk in self.tracks.items():
            if tid not in seen:
                continue
            x, z = trk.pos
            v = verdicts.get(tid, {})
            tracks_out.append({
                "id": tid, "cls": trk.cls, "conf": round(trk.conf, 2),
                "box": list(trk.box),
                "x_m": round(x, 1), "z_m": round(z, 1),
                "vx": round(trk.vel[0], 2) if trk.vel else None,
                "vz": round(trk.vel[1], 2) if trk.vel else None,
                "collide": v.get("collide", False),
                "t_hit": round(v["t_hit"], 1) if v.get("t_hit") is not None else None,
                "in_zone": v.get("collide", False),  # zone-mode compat for the UI
                "overlap": 0.0,
            })

        with self.lock:
            self.status = {
                "brake": brake_fraction >= 0.05,
                "brake_fraction": brake_fraction,
                "emergency": brake_fraction >= 0.999,
                "detections": tracks_out,
                "ego_speed_mps": round(v_ego, 2),
                "video_t": round(video_t, 2),
                "connected": True,
                "t": time.time(),
            }
            if ok:
                self.latest_jpeg = jpeg.tobytes()
        return annotated

    # ---- drawing --------------------------------------------------------------
    def _annotate(self, frame: np.ndarray, verdicts: dict[int, dict],
                  v_ego: float, curvature: float, brake: float) -> np.ndarray:
        out = frame.copy()
        cam = self.cam

        # Corridor: our predicted swept path (bent by current curvature),
        # colored by brake level green -> yellow -> red.
        level = min(1.0, brake / 0.999)
        color = (0, int(200 * (1 - level) + 40 * level), int(255 * level)) \
            if brake > 0 else (0, 200, 0)
        left, right = [], []
        for z in np.arange(1.0, min(18.0, MAX_RANGE_M), 0.5):
            xc = path_lateral_offset(z, curvature)
            for side, acc in ((-1, left), (1, right)):
                p = cam.ground_to_px(xc + side * CART_HALF_WIDTH_M, z)
                if p:
                    acc.append(p)
        if left and right:
            poly = np.array(left + right[::-1], dtype=np.int32)
            overlay = out.copy()
            cv2.fillPoly(overlay, [poly], color)
            cv2.addWeighted(overlay, 0.22, out, 0.78, 0, out)
            cv2.polylines(out, [poly], True, color, 2)

        # Tracks: trail, box, predicted path, verdict.
        for tid, trk in self.tracks.items():
            v = verdicts.get(tid)
            if v is None:
                continue
            danger = v["collide"]
            c = (0, 0, 255) if danger else (0, 200, 255)
            x1, y1, x2, y2 = trk.box
            cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
            speed_txt = ""
            if trk.vel:
                vx, vz = trk.vel
                speed_txt = f" {math.hypot(vx, vz + v_ego):.1f}m/s"
            label = f"#{tid} {trk.cls}{speed_txt}"
            if v.get("t_hit") is not None:
                label += f" hit {v['t_hit']:.1f}s"
            cv2.putText(out, label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
            trail = [cam.ground_to_px(x, z) for _, x, z in trk.hist]
            trail = [p for p in trail if p]
            if len(trail) > 1:
                cv2.polylines(out, [np.array(trail, dtype=np.int32)], False, c, 1)
            if trk.vel:
                x0, z0 = trk.pos
                pred = [cam.ground_to_px(x0 + trk.vel[0] * t, z0 + trk.vel[1] * t)
                        for t in np.arange(0, 2.01, 0.25)]
                pred = [p for p in pred if p]
                for p in pred:
                    cv2.circle(out, p, 2, c, -1)

        # HUD: verdict banner + brake bar + ego speed.
        if brake >= 0.999:
            txt, bg = "FULL BRAKE", (0, 0, 255)
        elif brake >= 0.05:
            txt, bg = f"BRAKE {int(brake * 100)}%", (0, 90, 230)
        else:
            txt, bg = "CLEAR", (0, 130, 0)
        cv2.rectangle(out, (0, 0), (170, 26), bg, -1)
        cv2.putText(out, txt, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
        bar_w = 120
        cv2.rectangle(out, (178, 8), (178 + bar_w, 20), (60, 60, 60), -1)
        if brake > 0:
            cv2.rectangle(out, (178, 8), (178 + int(bar_w * brake), 20),
                          (0, 0, 255), -1)
        cv2.putText(out, f"ego {v_ego * 2.237:.1f}mph", (306, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return out
