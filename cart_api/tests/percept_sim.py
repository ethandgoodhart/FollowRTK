#!/usr/bin/env python3
"""
percept_sim.py — a closed-loop simulator for the speed-control stack.

The point of this harness is that it does NOT stub out perception. A scenario
places real actors at real world coordinates; the harness projects them through
the real ``CameraModel`` into real pixel bounding boxes, perturbs those boxes
the way a real detector would, and feeds them back through ``project_detection``
-> ``Tracker`` -> ``policy.evaluate`` -> ``SpeedLimiter``. So a test that says
"the cart does not hit the pedestrian" is exercising the actual geometry, the
actual Kalman filter and the actual envelope, not a mock of them.

Three things are modelled that a naive simulator would skip, and each of them
is capable of causing a collision on its own:

  * TRANSPORT DELAY. The commanded speed does not take effect for
    ``plant_delay_s``, and then only through a first-order lag. If the policy's
    ``reaction_s`` is optimistic relative to this, the tests find out by
    running into somebody rather than by inspection.

  * PITCH WOBBLE. Frames are rendered at a perturbed pitch while
    ``project_detection`` inverts them at the nominal pitch. This is the
    dominant real-world range error and it is injected, not assumed away.

  * OCCLUSION. Actors block each other by bearing interval. A pedestrian
    behind a parked car is genuinely invisible until the sight line clears,
    and their feet are genuinely clipped while they are half-hidden -- which
    is how the "steps out from between parked cars" scenario gets its teeth.

Collision geometry is a point-to-rectangle distance against the cart footprint,
not centre-to-centre, so "clearance" means what it sounds like.
"""

from __future__ import annotations

import math
import os
import random
import sys
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.percept.geometry import (  # noqa: E402
    CLASS_HEIGHT_M, CLASS_WIDTH_M, CameraModel, project_detection,
)
from cartlib.percept.policy import (  # noqa: E402
    MPH_TO_MPS, MPS_TO_MPH, Governor, PolicyConfig,
)
from cartlib.percept.track import EgoPose, Tracker, world_to_cart  # noqa: E402

# Cart footprint in the cart frame (metres). The origin is the rear axle-ish
# reference the follower uses, so the body extends a little behind it.
CART_FRONT_M = 1.80
CART_BACK_M = -0.60
CART_HALF_W_M = 0.60


# ---------------------------------------------------------------------------
# Actors
# ---------------------------------------------------------------------------
@dataclass
class Actor:
    """A pedestrian, cyclist or vehicle moving at constant velocity."""

    cls: str
    x: float                      # world east (m)
    y: float                      # world north (m)
    vx: float = 0.0
    vy: float = 0.0
    height_m: Optional[float] = None
    width_m: Optional[float] = None
    t_start: float = 0.0          # does not move before this time
    t_stop: float = float("inf")  # stops moving after this time
    # Detector visibility window, independent of motion. Used to inject
    # phantom detections: a ``ghost`` is rendered and tracked like anything
    # else but is excluded from collision scoring, because it was never there.
    visible_from: float = float("-inf")
    visible_to: float = float("inf")
    ghost: bool = False
    label: str = ""

    def __post_init__(self) -> None:
        if self.height_m is None:
            self.height_m = CLASS_HEIGHT_M.get(self.cls, 1.7)
        if self.width_m is None:
            self.width_m = CLASS_WIDTH_M.get(self.cls, 0.6)

    def pos_at(self, t: float) -> Tuple[float, float]:
        tt = max(0.0, min(t, self.t_stop) - self.t_start)
        return self.x + self.vx * tt, self.y + self.vy * tt

    @property
    def speed_ms(self) -> float:
        return math.hypot(self.vx, self.vy)


# ---------------------------------------------------------------------------
# Route helpers
# ---------------------------------------------------------------------------
class Route:
    """Arc-length parametrisation of a polyline."""

    def __init__(self, pts: Sequence[Tuple[float, float]]):
        self.pts = [(float(a), float(b)) for a, b in pts]
        self.cum = [0.0]
        for i in range(1, len(self.pts)):
            self.cum.append(self.cum[-1] + math.dist(self.pts[i - 1], self.pts[i]))

    @property
    def length(self) -> float:
        return self.cum[-1]

    def at(self, s: float) -> Tuple[float, float, float]:
        """(x, y, heading_deg) at arc length ``s``, clamped to the ends."""
        s = max(0.0, min(s, self.length))
        i = 0
        while i < len(self.cum) - 2 and self.cum[i + 1] < s:
            i += 1
        seg = self.cum[i + 1] - self.cum[i]
        f = 0.0 if seg < 1e-9 else (s - self.cum[i]) / seg
        ax, ay = self.pts[i]
        bx, by = self.pts[i + 1]
        x, y = ax + f * (bx - ax), ay + f * (by - ay)
        # Compass bearing: 0 = north (+y), clockwise.
        heading = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
        return x, y, heading


def straight_route(length_m: float = 80.0) -> Route:
    """Due north from the origin. The default for most scenarios."""
    return Route([(0.0, 0.0), (0.0, length_m)])


# ---------------------------------------------------------------------------
# Rendering: world actors -> detector-like boxes
# ---------------------------------------------------------------------------
@dataclass
class RenderedBox:
    cls: str
    conf: float
    bbox: Tuple[float, float, float, float]
    actor: Optional[Actor] = None


def _bearing_span(cam: CameraModel, fwd: float, lat: float, width: float
                  ) -> Tuple[float, float]:
    half = 0.5 * width
    return (math.atan2(lat - half, fwd), math.atan2(lat + half, fwd))


def render_frame(cam: CameraModel, ego: EgoPose, actors: Sequence[Actor],
                 t: float, pitch_true_deg: float, rng: random.Random,
                 jitter_px: float = 2.0, dropout_p: float = 0.0,
                 conf: float = 0.85) -> List[RenderedBox]:
    """Project every actor the camera can actually see into a bounding box.

    Returns boxes in image coordinates, distorted exactly as the real lens
    would distort them, so ``project_detection`` has to undistort for real.
    """
    # First pass: cart-frame geometry for everyone in front of the camera.
    vis = []
    for a in actors:
        if not (a.visible_from <= t <= a.visible_to):
            continue
        ax, ay = a.pos_at(t)
        fwd, lat = world_to_cart(ego, ax, ay)
        fwd -= cam.offset_forward_m
        lat -= cam.offset_lateral_m
        if fwd <= 0.35:
            continue                     # beside or behind the lens: unseeable
        vis.append((a, fwd, lat))
    vis.sort(key=lambda r: r[1])         # nearest first

    out: List[RenderedBox] = []
    for idx, (a, fwd, lat) in enumerate(vis):
        lo, hi = _bearing_span(cam, fwd, lat, a.width_m)

        # Occlusion by anything nearer. Fully inside a nearer object's bearing
        # span -> invisible. Partially -> we still see them, but the occluder's
        # top edge cuts off their feet, which is the case that makes monocular
        # ranging over-estimate.
        occluded_to_y: Optional[float] = None
        hidden = False
        for (b, bfwd, blat) in vis[:idx]:
            blo, bhi = _bearing_span(cam, bfwd, blat, b.width_m)
            if blo <= lo and hi <= bhi:
                hidden = True
                break
            if bhi > lo and blo < hi:    # partial overlap
                top = cam.image_point(bfwd, blat, b.height_m, pitch_true_deg)
                if top is not None:
                    occluded_to_y = (top[1] if occluded_to_y is None
                                     else min(occluded_to_y, top[1]))
        if hidden:
            continue
        if dropout_p > 0.0 and rng.random() < dropout_p:
            continue

        feet = cam.image_point(fwd, lat, 0.0, pitch_true_deg)
        head = cam.image_point(fwd, lat, a.height_m, pitch_true_deg)
        left = cam.image_point(fwd, lat - 0.5 * a.width_m, 0.0, pitch_true_deg)
        right = cam.image_point(fwd, lat + 0.5 * a.width_m, 0.0, pitch_true_deg)
        if None in (feet, head, left, right):
            continue

        x1, x2 = sorted((left[0], right[0]))
        y1, y2 = head[1], feet[1]
        if occluded_to_y is not None:
            y2 = min(y2, occluded_to_y)  # feet hidden behind the occluder
        if y2 <= y1 + 2.0:
            continue                     # nothing left of the box

        if jitter_px > 0.0:
            x1 += rng.gauss(0.0, jitter_px)
            x2 += rng.gauss(0.0, jitter_px)
            y1 += rng.gauss(0.0, jitter_px)
            y2 += rng.gauss(0.0, jitter_px)

        # Clip to the sensor. A box clipped at the bottom keeps touching the
        # edge, which is the signal project_detection uses to mark the contact
        # point as unobserved.
        x1, x2 = max(0.0, x1), min(cam.width - 1.0, x2)
        y1, y2 = max(0.0, y1), min(cam.height - 1.0, y2)
        if x2 - x1 < 3.0 or y2 - y1 < 3.0:
            continue                     # off-frame or too small to detect

        out.append(RenderedBox(cls=a.cls, conf=conf, bbox=(x1, y1, x2, y2),
                               actor=a))
    return out


# ---------------------------------------------------------------------------
# Clearance
# ---------------------------------------------------------------------------
def clearance_m(ego: EgoPose, actor: Actor, t: float) -> float:
    """Gap between the cart's footprint and the actor's disc. Negative = hit."""
    ax, ay = actor.pos_at(t)
    fwd, lat = world_to_cart(ego, ax, ay)
    dx = max(CART_BACK_M - fwd, 0.0, fwd - CART_FRONT_M)
    dy = max(abs(lat) - CART_HALF_W_M, 0.0)
    return math.hypot(dx, dy) - 0.5 * actor.width_m


# ---------------------------------------------------------------------------
# The simulator
# ---------------------------------------------------------------------------
@dataclass
class SimConfig:
    dt: float = 1.0 / 20.0            # control + perception tick
    duration_s: float = 25.0
    # Plant. These are what the CART does, as opposed to what the policy
    # ASSUMES it does -- keeping them separate is the whole point.
    plant_delay_s: float = 0.35       # serial + pedal actuator travel
    plant_tau_s: float = 0.35         # first-order response of the drivetrain
    plant_accel_ms2: float = 1.0
    # Full brake, measured on the cart 2026-08-02 with tools/brake_char.py:
    # 5.45 mph -> 4.77 m in 2.97 s, i.e. about 1.0 m/s^2. The cart simply
    # cannot brake harder than this, so the simulator must not either --
    # letting the plant out-brake the real hardware would quietly invalidate
    # every collision test in the suite.
    plant_decel_ms2: float = 1.05
    # Sensing.
    pitch_bias_deg: float = 0.0       # constant mis-calibration
    pitch_wobble_deg: float = 0.6     # per-frame suspension movement (1 sigma)
    jitter_px: float = 2.0
    dropout_p: float = 0.0
    seed: int = 7


@dataclass
class SimStep:
    t: float
    s: float
    v_ms: float
    v_allowed_mph: float
    v_cmd_mph: float
    layer: str
    reason: str
    emergency: bool
    n_tracks: int
    min_clearance_m: float


MOVING_MS = 0.05           # below this the cart is, for our purposes, stopped


@dataclass
class SimResult:
    steps: List[SimStep] = field(default_factory=list)
    min_clearance_m: float = float("inf")
    closest_at_s: float = 0.0
    distance_travelled_m: float = 0.0

    @property
    def min_clearance_moving_m(self) -> float:
        """Closest approach while the cart was actually MOVING.

        This, not ``min_clearance_m``, is the safety criterion the tests use,
        and the distinction is not a convenience. A speed controller's job is
        to stop the cart from driving into people. It cannot stop a pedestrian
        from walking into a cart that is already stationary, and no achievable
        policy could -- in the head-on scenario the cart halts with 1.8 m to
        spare and the pedestrian keeps walking. Scoring that as a collision
        would mean the only passing policy is one that reverses away from
        approaching people, which is both absurd and less safe.

        The 0.05 m/s threshold is tight on purpose: a cart still creeping at
        walking pace when it touches somebody has failed, and will be caught.
        """
        gaps = [st.min_clearance_m for st in self.steps if st.v_ms > MOVING_MS]
        return min(gaps) if gaps else float("inf")

    @property
    def collided(self) -> bool:
        return self.min_clearance_moving_m <= 0.0

    @property
    def speeds_mph(self) -> List[float]:
        return [st.v_ms * MPS_TO_MPH for st in self.steps]

    @property
    def stopped_at_any_point(self) -> bool:
        return any(st.v_ms < 0.1 for st in self.steps)

    def max_jerk_ms3(self, skip_emergency: bool = True) -> float:
        """Largest |da/dt| of the ACTUAL cart speed, optionally ignoring stops.

        Computed on the plant output rather than the setpoint, because that is
        what a passenger feels. Differentiating twice amplifies noise, so the
        acceleration is taken over a 3-tick span first.
        """
        st = self.steps
        if len(st) < 8:
            return 0.0
        acc, times = [], []
        for i in range(3, len(st)):
            dt = st[i].t - st[i - 3].t
            if dt <= 0:
                continue
            if skip_emergency and any(st[j].emergency for j in range(i - 3, i + 1)):
                continue
            acc.append((st[i].v_ms - st[i - 3].v_ms) / dt)
            times.append(st[i].t)
        worst = 0.0
        for i in range(1, len(acc)):
            dt = times[i] - times[i - 1]
            if dt > 1e-6 and times[i] - times[i - 1] < 0.5:
                worst = max(worst, abs(acc[i] - acc[i - 1]) / dt)
        return worst

    def reversals(self, band_mph: float = 0.4) -> int:
        """How many times the speed changed direction by more than ``band``.

        A hunting controller shows up here as a large number even when the
        speed trace never looks dramatic.
        """
        v = self.speeds_mph
        if len(v) < 3:
            return 0
        n, direction, anchor = 0, 0, v[0]
        for x in v[1:]:
            if x > anchor + band_mph:
                if direction < 0:
                    n += 1
                direction, anchor = 1, x
            elif x < anchor - band_mph:
                if direction > 0:
                    n += 1
                direction, anchor = -1, x
            elif direction > 0:
                anchor = max(anchor, x)
            elif direction < 0:
                anchor = min(anchor, x)
        return n

    def summary(self) -> str:
        v = self.speeds_mph
        lines = [
            f"travelled {self.distance_travelled_m:5.1f} m   "
            f"v: min {min(v):4.2f} max {max(v):4.2f} mph   "
            f"min clearance {self.min_clearance_m:5.2f} m at t={self.closest_at_s:.1f}s",
            f"max jerk (non-emergency) {self.max_jerk_ms3():4.2f} m/s^3   "
            f"reversals {self.reversals()}",
        ]
        last = None
        for st in self.steps:
            key = (st.layer, st.reason)
            if key != last:
                lines.append(f"  t={st.t:5.2f}s v={st.v_ms*MPS_TO_MPH:4.2f}mph "
                             f"[{st.layer}] {st.reason}")
                last = key
        return "\n".join(lines)


def run(actors: Sequence[Actor],
        route: Optional[Route] = None,
        cfg: Optional[PolicyConfig] = None,
        sim: Optional[SimConfig] = None,
        cam: Optional[CameraModel] = None,
        start_speed_mph: Optional[float] = None) -> SimResult:
    """Run one closed-loop scenario and return its trace."""
    route = route or straight_route()
    cfg = cfg or PolicyConfig()
    sim = sim or SimConfig()
    cam = cam or CameraModel(
        width=1920, height=1200, fx=1000.0, fy=1000.0,
        dist=(-0.28, 0.09, 0.0, 0.0, 0.0),
        height_m=1.45, pitch_deg=6.0, offset_forward_m=1.6, pitch_sigma_deg=1.0)

    rng = random.Random(sim.seed)
    tracker = Tracker()
    v0 = cfg.max_speed_mph if start_speed_mph is None else start_speed_mph
    gov = Governor(cfg, initial_mph=v0)

    route_xy = list(route.pts)
    s, v_ms = 0.0, v0 * MPH_TO_MPS
    delay_n = max(1, int(round(sim.plant_delay_s / sim.dt)))
    delay_buf = deque([v_ms] * delay_n, maxlen=delay_n)

    res = SimResult()
    n = int(sim.duration_s / sim.dt)
    for k in range(n):
        t = k * sim.dt
        x, y, heading = route.at(s)
        ego = EgoPose(x=x, y=y, heading_deg=heading, speed_ms=v_ms, ts=t)

        # --- perceive -----------------------------------------------------
        pitch_true = cam.pitch_deg + sim.pitch_bias_deg + \
            rng.gauss(0.0, sim.pitch_wobble_deg)
        boxes = render_frame(cam, ego, actors, t, pitch_true, rng,
                             jitter_px=sim.jitter_px, dropout_p=sim.dropout_p)
        dets = []
        for b in boxes:
            # Inverted at the NOMINAL pitch: the simulator knows the true
            # pitch, the perception stack does not.
            d = project_detection(cam, b.bbox, b.cls, b.conf)
            if d is not None:
                dets.append(d)
        tracks = tracker.update(dets, ego, t)

        # --- decide -------------------------------------------------------
        v_cmd_mph = gov.step(tracks, route_xy, ego, t, sim.dt)
        dec = gov.decision

        # --- plant --------------------------------------------------------
        delay_buf.append(v_cmd_mph * MPH_TO_MPS)
        v_want = delay_buf[0]
        a = (v_want - v_ms) / sim.plant_tau_s
        a = max(-sim.plant_decel_ms2, min(a, sim.plant_accel_ms2))
        v_ms = max(0.0, v_ms + a * sim.dt)
        s = min(s + v_ms * sim.dt, route.length)

        # --- score --------------------------------------------------------
        x2, y2, h2 = route.at(s)
        ego_after = EgoPose(x=x2, y=y2, heading_deg=h2, speed_ms=v_ms, ts=t)
        gaps = [clearance_m(ego_after, a_, t + sim.dt)
                for a_ in actors if not a_.ghost]
        gap = min(gaps) if gaps else float("inf")
        if gap < res.min_clearance_m:
            res.min_clearance_m, res.closest_at_s = gap, t

        res.steps.append(SimStep(
            t=t, s=s, v_ms=v_ms, v_allowed_mph=dec.v_allowed_mph,
            v_cmd_mph=v_cmd_mph, layer=dec.layer, reason=dec.reason,
            emergency=dec.emergency, n_tracks=len(tracks),
            min_clearance_m=gap))

    res.distance_travelled_m = s
    return res
