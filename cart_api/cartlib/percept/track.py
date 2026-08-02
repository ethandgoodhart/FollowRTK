"""
cartlib.percept.track — who is where, and where they are going.

Detections arrive in the CART frame, which is useless for prediction: when the
cart moves, a perfectly stationary bollard appears to fly toward it at 2 m/s.
So the first thing this module does is push every detection out into a fixed
WORLD frame using the ego pose. In that frame a stationary object has zero
velocity, and "constant velocity" finally means what it says.

Prediction is a plain constant-velocity model, on purpose. Schoeller et al.
(arXiv:1903.07933) showed a CV model matching or beating the state-of-the-art
generative pedestrian predictors it was compared against; on a cart that must
justify every stop to an operator, a two-line extrapolation that can be checked
by hand beats a network that cannot. Uncertainty grows as a cone, so a person
who might turn is handled by the cone widening rather than by pretending we
know they will.

Three deliberate choices worth knowing about:

  * MEASUREMENT NOISE COMES FROM THE GEOMETRY. ``geometry.range_uncertainty_m``
    already computes how badly pitch wobble could be fooling us, and that error
    bar is fed straight in as the range-axis variance. A detection at 20 m
    therefore moves the filter far less than one at 4 m, which is exactly the
    weighting the physics implies.

  * TRACKS COAST WHEN THEY GO MISSING. A pedestrian occluded for three frames
    keeps their predicted position and stays a hazard. Deleting them the
    instant the detector blinks is how a speed controller ends up lurching.

  * CONFIRMATION IS ASYMMETRIC. A track needs several hits before it is
    "confirmed", but the policy layer is told about TENTATIVE tracks too. It
    costs little to slow slightly for something that might be a person, and a
    great deal to ignore one for four frames while waiting for certainty.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import GroundDetection

# Association compatibility groups. Detectors routinely flip car <-> truck and
# person <-> bicycle between frames; forbidding those swaps would shred tracks
# for no safety benefit, so we associate within a group rather than by exact
# class label.
CLASS_GROUP = {
    "person": "vru", "bicycle": "vru", "motorcycle": "vru", "dog": "vru",
    "car": "vehicle", "bus": "vehicle", "truck": "vehicle",
}

# Plausible top speeds (m/s), used to size the association gate and to bound
# how fast a track is allowed to claim it is moving.
GROUP_MAX_SPEED = {"vru": 5.0, "vehicle": 20.0}

# Process noise: how much unmodelled acceleration we allow per group. A
# pedestrian can change direction far more abruptly (relative to their speed)
# than a car, so the VRU model is deliberately looser.
GROUP_ACCEL_SIGMA = {"vru": 1.5, "vehicle": 3.0}

# Ceiling on the manoeuvre cone (m). Integrating an acceleration sigma over a
# 6-second horizon gives 27 m, which would make every pedestrian on the campus
# a conflict and the cart undriveable. The cone is not a reachable set: it is
# how far somebody is PLAUSIBLY off their current course, and beyond a second
# or so that stops growing because people do not accelerate indefinitely
# sideways. Last-instant intrusions inside this cap are the L0 reflex layer's
# job, not the forecaster's -- which is precisely why that layer exists.
MANOEUVRE_CAP_M = {"vru": 1.2, "vehicle": 3.0}

# How far a forecast is allowed to be smeared by velocity uncertainty before we
# stop calling it a forecast. See Track.forecast_horizon_s.
FORECAST_SMEAR_M = 2.0


@dataclass
class EgoPose:
    """Where the cart is and which way it faces, in the world frame."""

    x: float                  # local east (m)
    y: float                  # local north (m)
    heading_deg: float        # compass bearing of the cart's forward axis
    speed_ms: float = 0.0
    ts: float = 0.0


def cart_to_world(ego: EgoPose, forward_m: float, lateral_m: float
                  ) -> Tuple[float, float]:
    """(forward, lateral) in the cart frame -> (x, y) in the world frame.

    Heading is a compass bearing (0 = north, clockwise), so forward is
    (sin, cos) and lateral (positive right) is (cos, -sin).
    """
    h = math.radians(ego.heading_deg)
    sx, cx = math.sin(h), math.cos(h)
    x = ego.x + forward_m * sx + lateral_m * cx
    y = ego.y + forward_m * cx - lateral_m * sx
    return x, y


def world_to_cart(ego: EgoPose, x: float, y: float) -> Tuple[float, float]:
    """Inverse of ``cart_to_world``: world (x, y) -> (forward, lateral)."""
    h = math.radians(ego.heading_deg)
    sx, cx = math.sin(h), math.cos(h)
    dx, dy = x - ego.x, y - ego.y
    forward = dx * sx + dy * cx
    lateral = dx * cx - dy * sx
    return forward, lateral


@dataclass
class Track:
    """One tracked object: constant-velocity state in the world frame."""

    id: int
    cls: str
    group: str
    state: np.ndarray                     # [x, y, vx, vy]
    P: np.ndarray                         # 4x4 covariance
    hits: int = 1
    misses: int = 0
    coast_s: float = 0.0                  # seconds since the last real detection
    age: int = 1
    first_ts: float = 0.0
    last_ts: float = 0.0
    conf: float = 0.0
    width_m: float = 0.6
    confirmed: bool = False
    cue_disagreement: Optional[float] = None
    # Kept purely so the UI and logs can show where a track came from.
    last_forward_m: float = 0.0
    last_lateral_m: float = 0.0
    last_range_sigma_m: float = 0.5
    last_lat_sigma_m: float = 0.25
    # Feet below the frame: range is an upper bound. Carried through so the
    # reflex layer can refuse to trust it (see policy.reflex_triggered).
    clipped_bottom: bool = False

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def y(self) -> float:
        return float(self.state[1])

    @property
    def vx(self) -> float:
        return float(self.state[2])

    @property
    def vy(self) -> float:
        return float(self.state[3])

    @property
    def speed_ms(self) -> float:
        return float(math.hypot(self.state[2], self.state[3]))

    @property
    def pos_sigma_m(self) -> float:
        """One-sigma positional uncertainty right now."""
        return float(math.sqrt(max(0.0, 0.5 * (self.P[0, 0] + self.P[1, 1]))))

    def predict_at(self, dt: float) -> Tuple[float, float]:
        """Constant-velocity position ``dt`` seconds from the last update."""
        return self.x + self.vx * dt, self.y + self.vy * dt

    @property
    def is_moving(self) -> bool:
        """Is this thing actually going somewhere, or is that just noise?

        Judged on SIGNIFICANCE, not magnitude. Monocular range noise routinely
        gives a parked car an apparent 3 m/s -- but with a 3 m/s uncertainty
        attached, which is the filter correctly saying it has no idea. Calling
        that "moving" hands it to the arrival-time reasoning, which then argues
        the car will have driven off before we get there. Requiring the speed
        to stand clear of its own error bar keeps unconvincing velocities in
        the stationary case, where they are treated as obstacles wherever they
        are. The failure direction is over-caution, which is the right one.
        """
        return self.speed_ms > max(0.5, 2.0 * self.vel_sigma_ms)

    @property
    def observed_s(self) -> float:
        """How long we have actually been watching this object."""
        return max(0.0, self.last_ts - self.first_ts)

    @property
    def vel_sigma_ms(self) -> float:
        """One-sigma speed uncertainty. Large for a young or noisy track."""
        return float(math.sqrt(max(0.0, 0.5 * (self.P[2, 2] + self.P[3, 3]))))

    def forecast_horizon_s(self, cap: float) -> float:
        """How far ahead this particular track may be extrapolated.

        Constant-velocity prediction is only as good as the velocity estimate,
        and a velocity estimate differentiated from monocular range is bad
        while a track is young: range noise of a few metres over a tenth of a
        second reads as tens of metres per second. Extrapolating that for six
        seconds teleports a stationary pedestrian across the campus, and the
        cart brakes hard for a ghost. So a track is never forecast further
        ahead than roughly half as long as we have been watching it -- the
        forecast has to be paid for with evidence.

        Age alone is not enough evidence, though. A parked car at 40 m can be
        watched for ten seconds and still have a velocity estimate that is
        mostly range noise, and extrapolating that for three seconds walks it
        straight into the cart -- a full stop for a stationary object three
        metres off the path. So the horizon is also bounded by the point at
        which velocity uncertainty alone would smear the prediction over a
        couple of metres. Past there the forecast is not a prediction, it is a
        shrug, and the tt = 0 test in the policy is strictly better evidence.
        """
        by_age = 0.5 * self.observed_s + 0.75
        by_quality = (FORECAST_SMEAR_M / self.vel_sigma_ms
                      if self.vel_sigma_ms > 1e-6 else cap)
        return max(0.0, min(cap, by_age, by_quality))

    @property
    def lat_sigma_m(self) -> float:
        """Cross-track positional sigma, grown while the track is coasting.

        Deliberately NOT ``pos_sigma_m``: that is dominated by RANGE error,
        which for a monocular ground-plane estimate is large and points along
        the line of sight. The corridor test cares about the perpendicular
        direction, where a calibrated camera's bearing measurement is good.
        Using the isotropic number would throw away exactly the anisotropy the
        geometry module went to the trouble of computing, and would make the
        cart stop for people standing well clear of the path.
        """
        # Growth while coasting is capped. An unbounded cone is formally
        # honest and practically useless: after a couple of seconds it
        # swallows the cart itself, and everything in sight becomes a reason
        # to stop. Past the cap the right response is to stop believing the
        # track at all, which is what max_coast_s does.
        return self.last_lat_sigma_m + min(0.5 * self.coast_s, 0.6)

    def radius_at(self, dt: float) -> float:
        """Cross-track reach ``dt`` ahead: sigma + manoeuvre cone + half-width.

        The cone is (1/2)*a*dt^2, capped (see MANOEUVRE_CAP_M) and scaled by how
        much the object is actually moving. Someone standing still is far less
        likely to translate a metre in the next second than someone already
        walking, and treating the two identically is what makes a cautious cart
        useless. The longitudinal dimension needs no term here: the route is
        sampled every ``station_step_m``, so an object simply conflicts with
        whichever station it is nearest.
        """
        a = GROUP_ACCEL_SIGMA.get(self.group, 1.5)
        cap = MANOEUVRE_CAP_M.get(self.group, 1.2)
        v_max = GROUP_MAX_SPEED.get(self.group, 5.0)
        cone = min(0.5 * a * dt * dt, cap)
        motion = min(1.0, 0.25 + self.speed_ms / v_max)
        # Propagating the velocity covariance is what stops a young track's
        # made-up velocity from being treated as fact: the same noise that
        # invents the velocity also widens the cone around it.
        drift = min(self.vel_sigma_ms * dt, v_max * dt)
        return self.lat_sigma_m + drift + motion * cone + 0.5 * self.width_m


class Tracker:
    """Greedy-optimal association + constant-velocity Kalman filtering."""

    def __init__(self, max_coast_s: float = 2.0, confirm_hits: int = 3,
                 max_tracks: int = 64, unconfirmed_coast_s: float = 0.25):
        # Coasting is bounded in SECONDS, not frames. A frame count silently
        # means something different at 30 Hz than at 8 Hz, and the quantity
        # that actually matters is how long a constant-velocity extrapolation
        # stays believable. Two seconds covers a pedestrian passing behind a
        # parked car, and covers the final approach to a stop, where the
        # obstacle drops into the camera's near blind zone and simply cannot
        # be re-detected however healthy the detector is.
        self.max_coast_s = max_coast_s
        # Coasting is an act of belief, and an unconfirmed track has not earned
        # any. Two frames of detector noise must not be extrapolated for two
        # seconds into the cart's path -- that turns a harmless false positive
        # into an emergency stop, and a system that emergency-stops for noise
        # gets switched off, which is its own kind of unsafe.
        self.unconfirmed_coast_s = unconfirmed_coast_s
        self.confirm_hits = confirm_hits
        self.max_tracks = max_tracks
        self.tracks: List[Track] = []
        self._ids = itertools.count(1)
        self._last_ts: Optional[float] = None

    # -- filter internals -------------------------------------------------
    @staticmethod
    def _F(dt: float) -> np.ndarray:
        return np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]], dtype=float)

    @staticmethod
    def _Q(dt: float, accel_sigma: float) -> np.ndarray:
        """Discrete white-noise acceleration process noise."""
        q = accel_sigma ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        return q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                             [0, dt4 / 4, 0, dt3 / 2],
                             [dt3 / 2, 0, dt2, 0],
                             [0, dt3 / 2, 0, dt2]], dtype=float)

    @staticmethod
    def _sigmas(det: GroundDetection) -> Tuple[float, float]:
        """(along-sight, cross-sight) one-sigma measurement error, in metres."""
        far = det.forward_far_m if math.isfinite(det.forward_far_m) else det.forward_m * 3.0
        sigma_range = max(0.15, 0.5 * (far - det.forward_near_m))
        sigma_lat = max(0.10, 0.02 * det.forward_m + 0.10)
        return sigma_range, sigma_lat

    @staticmethod
    def _R(ego: EgoPose, det: GroundDetection) -> np.ndarray:
        """Measurement covariance in WORLD axes, from the geometry's error bars.

        The range error is large and grows with distance; the lateral error is
        small and roughly constant (it is a bearing measurement, and bearings
        from a calibrated camera are good). So the error ellipse is long and
        thin, pointed along the line of sight -- and it has to be rotated into
        world axes to be used, which is what the similarity transform below is.
        """
        sigma_range, sigma_lat = Tracker._sigmas(det)

        # Bearing of the line of sight in the world frame.
        bearing = math.radians(ego.heading_deg) + math.atan2(det.lateral_m,
                                                             max(det.forward_m, 1e-3))
        s, c = math.sin(bearing), math.cos(bearing)
        # Columns: along-sight (world x = sin, y = cos) and cross-sight.
        Rot = np.array([[s, c], [c, -s]], dtype=float)
        D = np.diag([sigma_range ** 2, sigma_lat ** 2])
        return Rot @ D @ Rot.T

    # -- public API -------------------------------------------------------
    def update(self, dets: Sequence[GroundDetection], ego: EgoPose,
               ts: float) -> List[Track]:
        """Advance every track to ``ts`` and fold in this frame's detections."""
        dt = 0.0 if self._last_ts is None else max(0.0, min(ts - self._last_ts, 1.0))
        self._last_ts = ts

        # --- predict ----------------------------------------------------
        if dt > 0:
            for t in self.tracks:
                F = self._F(dt)
                t.state = F @ t.state
                t.P = F @ t.P @ F.T + self._Q(dt, GROUP_ACCEL_SIGMA.get(t.group, 1.5))
                # Never let the filter claim a pedestrian is doing 40 mph.
                cap = GROUP_MAX_SPEED.get(t.group, 5.0)
                sp = math.hypot(t.state[2], t.state[3])
                if sp > cap:
                    t.state[2] *= cap / sp
                    t.state[3] *= cap / sp

        # --- measurements in world coords -------------------------------
        meas = []
        for d in dets:
            wx, wy = cart_to_world(ego, d.forward_m, d.lateral_m)
            meas.append((d, wx, wy))

        # --- association -------------------------------------------------
        matches, un_trk, un_det = self._associate(meas, ego, dt)

        for ti, di in matches:
            self._correct(self.tracks[ti], meas[di], ego)

        for ti in un_trk:
            t = self.tracks[ti]
            t.misses += 1
            t.age += 1
            t.coast_s += dt

        for di in un_det:
            self._spawn(meas[di], ego, ts)

        # --- retire ------------------------------------------------------
        self.tracks = [
            t for t in self.tracks
            if t.coast_s <= (self.max_coast_s if t.confirmed
                             else self.unconfirmed_coast_s)]
        self._merge_duplicates()
        if len(self.tracks) > self.max_tracks:
            # Keep the closest ones: they are the ones that can hurt anybody.
            self.tracks.sort(key=lambda t: t.last_forward_m)
            self.tracks = self.tracks[:self.max_tracks]

        for t in self.tracks:
            t.last_ts = ts
            # Refresh cart-relative position each frame so consumers (UI, logs)
            # see where a coasting track is NOW, not where it was last seen.
            t.last_forward_m, t.last_lateral_m = world_to_cart(ego, t.x, t.y)
        return self.tracks

    def _merge_duplicates(self) -> None:
        """Drop tracks that are plainly the same object seen twice.

        A widened gate makes duplicates rare, not impossible: a detection
        dropout at the wrong moment still splits one object in two. Two tracks
        on one obstacle is worse than it sounds -- they alternate as the
        limiting conflict and the speed decision flaps between their two
        estimates. Keeping the better-supported one is both safer and much
        easier for an operator to read.
        """
        keep: List[Track] = []
        for t in sorted(self.tracks, key=lambda k: (-k.hits, k.coast_s)):
            dup = False
            for k in keep:
                if k.group != t.group:
                    continue
                span = 2.0 * max(k.pos_sigma_m, t.pos_sigma_m) + \
                    0.5 * (k.width_m + t.width_m)
                if math.hypot(k.x - t.x, k.y - t.y) <= span:
                    dup = True
                    break
            if not dup:
                keep.append(t)
        self.tracks = keep

    def _associate(self, meas, ego: EgoPose, dt: float):
        """Optimal assignment within a distance gate, per compatibility group."""
        if not self.tracks or not meas:
            return [], list(range(len(self.tracks))), list(range(len(meas)))

        n_t, n_m = len(self.tracks), len(meas)
        BIG = 1e6
        cost = np.full((n_t, n_m), BIG, dtype=float)
        for i, t in enumerate(self.tracks):
            travel = GROUP_MAX_SPEED.get(t.group, 5.0) * max(dt, 0.1)
            for j, (d, wx, wy) in enumerate(meas):
                if CLASS_GROUP.get(d.cls, "vru") != t.group:
                    continue
                # The gate has to cover how far the object could have moved
                # AND how wrong either end of the comparison could be. Sizing
                # it on the track's posterior sigma alone is a trap: that
                # number shrinks as the filter converges, while the range
                # measurement stays as noisy as the geometry says it is. The
                # gate then closes below the measurement spread and a single
                # bad frame spawns a duplicate track, after which two
                # disagreeing estimates of one object fight over the speed
                # decision.
                sigma_meas = Tracker._sigmas(d)[0]
                gate = travel + 2.0 * math.hypot(t.pos_sigma_m, sigma_meas) + 1.0
                dist = math.hypot(wx - t.x, wy - t.y)
                if dist <= gate:
                    cost[i, j] = dist

        try:
            from scipy.optimize import linear_sum_assignment
            rows, cols = linear_sum_assignment(cost)
            pairs = [(int(r), int(c)) for r, c in zip(rows, cols)
                     if cost[r, c] < BIG]
        except Exception:
            # Greedy fallback keeps the tracker working if scipy is missing.
            pairs, used_t, used_m = [], set(), set()
            order = np.dstack(np.unravel_index(np.argsort(cost, axis=None),
                                               cost.shape))[0]
            for r, c in order:
                if cost[r, c] >= BIG:
                    break
                if r in used_t or c in used_m:
                    continue
                used_t.add(int(r)); used_m.add(int(c))
                pairs.append((int(r), int(c)))

        matched_t = {p[0] for p in pairs}
        matched_m = {p[1] for p in pairs}
        return (pairs,
                [i for i in range(n_t) if i not in matched_t],
                [j for j in range(n_m) if j not in matched_m])

    def _correct(self, t: Track, m, ego: EgoPose) -> None:
        d, wx, wy = m
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = self._R(ego, d)
        z = np.array([wx, wy], dtype=float)
        y = z - H @ t.state
        S = H @ t.P @ H.T + R
        try:
            K = t.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        t.state = t.state + K @ y
        t.P = (np.eye(4) - K @ H) @ t.P
        t.hits += 1
        t.age += 1
        t.misses = 0
        t.coast_s = 0.0
        t.conf = d.conf
        t.cls = d.cls
        t.width_m = max(0.3, min(d.width_m, 3.0))
        # Smoothed, not replaced. Cue disagreement is the health signal that
        # drops the cart into degraded mode, and a single frame of it is
        # nothing -- a jittered box edge, a foot mid-stride. What it is meant
        # to catch, a broken ground-plane assumption, is a sustained
        # condition. Taking the raw per-frame value let one noisy frame cap
        # the speed at 2 mph, which then costs several seconds to recover
        # from: an instantaneous input driving a slow-to-release output.
        if d.cue_disagreement is not None:
            prior = t.cue_disagreement
            t.cue_disagreement = (d.cue_disagreement if prior is None
                                  else 0.75 * prior + 0.25 * d.cue_disagreement)
        t.clipped_bottom = d.clipped_bottom
        t.last_range_sigma_m, t.last_lat_sigma_m = self._sigmas(d)
        if t.hits >= self.confirm_hits:
            t.confirmed = True

    def _spawn(self, m, ego: EgoPose, ts: float) -> None:
        d, wx, wy = m
        group = CLASS_GROUP.get(d.cls, "vru")
        R = self._R(ego, d)
        P = np.zeros((4, 4), dtype=float)
        P[:2, :2] = R
        # A brand-new track has no velocity information at all. Saying so --
        # with a variance covering the group's full speed range -- is what lets
        # the very next frame move the estimate sharply instead of dragging it.
        v0 = GROUP_MAX_SPEED.get(group, 5.0)
        P[2, 2] = P[3, 3] = v0 ** 2
        self.tracks.append(Track(
            id=next(self._ids), cls=d.cls, group=group,
            state=np.array([wx, wy, 0.0, 0.0], dtype=float), P=P,
            first_ts=ts, last_ts=ts,
            conf=d.conf, width_m=max(0.3, min(d.width_m, 3.0)),
            cue_disagreement=d.cue_disagreement,
            clipped_bottom=d.clipped_bottom,
            last_forward_m=d.forward_m, last_lateral_m=d.lateral_m,
            last_range_sigma_m=self._sigmas(d)[0],
            last_lat_sigma_m=self._sigmas(d)[1],
        ))
