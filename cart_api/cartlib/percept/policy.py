"""
cartlib.percept.policy — turn tracked objects into one number: how fast may we go.

This is the safety core, and it is written to be read. The entire output of the
perception stack is a single scalar, ``v_allowed_mph``, which the follower takes
a ``min`` against its route target. Nothing here touches gas, brake or steering.
If this module is wrong, the failure is a cart that drives too fast or too slow
-- never a cart that steers somewhere unexpected.

The rule, in one sentence
------------------------
    Never go faster than a speed from which we could still stop short of the
    nearest place our route is predicted to be occupied.

Formally, for a conflict at arc-length ``s`` along our own route, we require
``v*rho + v^2/(2a) <= s - margin``, whose positive root is

    v_safe = -a*rho + sqrt((a*rho)^2 + 2a*(s - margin))

with ``rho`` the reaction time (dominated by how long the brake actuator takes
to move -- measure it with tools/brake_char.py, do not guess it) and ``a`` the
deceleration we are willing to use. This is the Responsibility-Sensitive Safety
idea reduced to the one case a 5 mph cart on a footpath actually faces.

Three layers, deliberately unequal
----------------------------------
  L0 REFLEX      Something big and close in the corridor -> brake now. Depends
                 only on detection and range: no tracking, no forecasting, no
                 route matching. It is a handful of lines so that a bug in the
                 clever parts still cannot drive into somebody.
  L1 NOMINAL     The envelope above. Does all the everyday work, smoothly.
  L2 DEGRADATION Perception health is poor -> cap the speed. Never raises it.

The three combine by ``min``, so no layer can ever authorise more speed than
another allows.

What "conflict" means
---------------------
We do not stop for everyone we can see; that would make the cart useless on a
campus footpath. A track is a conflict only if its uncertainty region is
predicted to overlap our route corridor AROUND THE TIME WE WOULD BE THERE --
within ``arrival_window_s`` either side. Someone who will have crossed and gone
long before we arrive is not a conflict, and someone who has not arrived yet is
not either. That window is the single knob trading over-caution against
permissiveness, and it is checked over a spread of arrival times precisely
because slowing down changes when we arrive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .track import GROUP_MAX_SPEED, EgoPose, Track

MPS_TO_MPH = 2.2369363
MPH_TO_MPS = 1.0 / MPS_TO_MPH
M_TO_FT = 3.280839895


@dataclass
class PolicyConfig:
    # -- vehicle capability ------------------------------------------------
    # rho is the whole reaction chain: perception latency + control period +
    # serial + brake actuator travel. brake_char.py reports it as "motion lag".
    reaction_s: float = 0.8
    # Full-brake field calibration, 2026-08-02 (brake command 0.45):
    #   5.446 mph -> 4.768 m in 2.968 s
    #   5.031 mph -> 4.312 m in 2.973 s
    # With the 0.8 s reaction allowance below, both runs fit about 1.0 m/s^2.
    # Do not let planned braking assume more authority than full brake has
    # demonstrated. Revisit comfort separately after lower-brake trials.
    decel_comfort_ms2: float = 1.0
    decel_emergency_ms2: float = 1.0

    # -- geometry ---------------------------------------------------------
    cart_half_width_m: float = 0.65      # cart half-width incl. mirrors
    corridor_margin_m: float = 0.35      # clearance we insist on beyond that
    # How close, measured from the CART ORIGIN, we will plan to stop. This is
    # not a comfort number: a forward-facing camera cannot see the ground
    # nearer than ``blind_zone_m``, so planning to stop inside that means
    # finishing the manoeuvre blind, with the obstacle's last known position
    # several seconds stale. Stopping short of the blind zone keeps whatever
    # we stopped for in frame the whole way in -- which is also what makes
    # automatic resumption safe rather than hopeful.
    stop_margin_m: float = 3.5
    # Nearest ground range the camera can resolve, from the cart origin.
    # Derived from mount height, pitch and the bottom image row; recompute it
    # with geometry.ground_range_m(cam.height - 1) if the bracket moves.
    blind_zone_m: float = 3.5

    # -- horizons ---------------------------------------------------------
    horizon_s: float = 6.0               # how far ahead we forecast
    horizon_m: float = 30.0              # how far along the route we look
    station_step_m: float = 0.25         # route sampling resolution
    arrival_window_s: float = 3.0        # timing tolerance around our arrival

    # -- speed shaping -----------------------------------------------------
    max_speed_mph: float = 5.5
    creep_speed_mph: float = 0.0         # floor once slowing (0 = allow a stop)
    accel_limit_ms2: float = 0.8         # how briskly we may speed back up
    decel_limit_ms2: float = 1.0         # measured full-brake capability
    jerk_limit_ms3: float = 1.0          # comfort: keep under ~1 m/s^3
    emergency_jerk_ms3: float = 4.0      # reflex layer may exceed comfort
    resume_hold_s: float = 1.5           # wait this long after a stop clears

    # -- L0 reflex ---------------------------------------------------------
    # Set just outside the blind zone so the reflex fires while its trigger is
    # still visible, rather than at the moment it disappears under the bonnet.
    reflex_range_m: float = 4.5          # anything nearer than this...
    # Narrower than the L1 corridor (cart_half_width + corridor_margin), and
    # deliberately so: the two layers ask different questions. L1 asks "will
    # this conflict with the space we intend to occupy", and can afford to be
    # generous because its answer is to slow down. L0 asks "is this about to
    # hit us", and its answer is a full stop -- so it is scoped to the cart's
    # actual body plus a little. At 1.2 m a car parked 2.6 m to one side
    # emergency-stops the cart as it drives past, every time.
    reflex_half_width_m: float = 0.9     # ...and this close to centreline
    # Hold the stop this long after the trigger clears. Without it, an object
    # that vanishes into the blind zone reads as "gone" and the cart pulls away
    # into the space it was last seen occupying.
    reflex_latch_s: float = 1.5

    # -- L2 degradation ----------------------------------------------------
    degraded_speed_mph: float = 2.0
    max_cue_disagreement: float = 0.35


@dataclass
class Conflict:
    """One reason we might slow down, with everything needed to explain it."""

    track_id: int
    cls: str
    station_m: float                  # arc-length along OUR route
    time_s: float                     # when the overlap happens
    gap_m: float                      # object range now
    closing_ms: float                 # rate of change of that range
    v_safe_ms: float                  # speed this conflict permits

    def describe(self) -> str:
        return (f"track #{self.track_id} ({self.cls}) blocks the route at "
                f"{self.station_m * M_TO_FT:.0f} ft in {self.time_s:.1f} s "
                f"-> {self.v_safe_ms * MPS_TO_MPH:.1f} mph")


@dataclass
class SpeedDecision:
    v_allowed_mph: float
    reason: str = "clear"
    limiting_track_id: Optional[int] = None
    emergency: bool = False
    degraded: bool = False
    conflicts: List[Conflict] = field(default_factory=list)
    layer: str = "nominal"            # clear | nominal | reflex | degraded

    def to_dict(self) -> dict:
        return {
            "v_allowed_mph": round(self.v_allowed_mph, 2),
            "reason": self.reason,
            "limiting_track_id": self.limiting_track_id,
            "emergency": self.emergency,
            "degraded": self.degraded,
            "layer": self.layer,
            "conflicts": [
                {"track_id": c.track_id, "cls": c.cls,
                 "station_m": round(c.station_m, 2),
                 "time_s": round(c.time_s, 2),
                 "gap_m": round(c.gap_m, 2),
                 "closing_ms": round(c.closing_ms, 2),
                 "v_safe_mph": round(c.v_safe_ms * MPS_TO_MPH, 2)}
                for c in self.conflicts[:8]
            ],
        }


# ---------------------------------------------------------------------------
# The envelope
# ---------------------------------------------------------------------------
def effective_reaction_s(cfg: PolicyConfig, decel_ms2: float) -> float:
    """Dead time, including the time spent ramping INTO the deceleration.

    The envelope's algebra assumes ``a`` is available the instant we decide to
    brake. It is not: the comfort jerk limit takes ``a / j`` seconds to reach
    it, and for most of that ramp we are still travelling at nearly full speed.
    Charging half the ramp as extra dead time is the standard trapezoidal-
    profile correction.

    This term is not a detail. With a = 1.2 m/s^2 and j = 1.0 m/s^3 the ramp is
    1.2 s -- comparable to the whole rest of the reaction chain. Omitting it
    makes the cart plan stops it physically cannot complete, which the
    simulator demonstrates the only way it can: by running into somebody.
    """
    return cfg.reaction_s + 0.5 * decel_ms2 / max(cfg.jerk_limit_ms3, 0.1)


def reflex_protected_speed_mph(cfg: PolicyConfig,
                               cart_front_m: float = 1.8) -> float:
    """Top speed from which the L0 reflex can still stop short of contact.

    Worth computing rather than assuming, because the answer is uncomfortable.
    The reflex fires at ``reflex_range_m`` from the cart origin, leaving
    ``reflex_range_m - cart_front_m`` metres in front of the bumper. With the
    braking authority the cart actually has, that bounds the speed at which a
    last-instant intrusion is survivable -- and with a measured 1.0 m/s^2 that
    bound lands well below ``max_speed_mph``.

    That is not a bug to be tuned away; it is the physics of a cart that takes
    nearly three seconds to stop. It means the layers do different jobs: L1
    keeps the cart slow enough for everything it can SEE, and L0 is a mitigation
    for what it could not, not a guarantee. If a guarantee against sudden
    step-outs is wanted, ``max_speed_mph`` has to come down to this number.
    """
    room = max(0.0, cfg.reflex_range_m - cart_front_m)
    a = cfg.decel_emergency_ms2
    rho = cfg.reaction_s + 0.5 * a / max(cfg.emergency_jerk_ms3, 0.1)
    return (-a * rho + math.sqrt((a * rho) ** 2 + 2.0 * a * room)) * MPS_TO_MPH


def safe_speed_for_distance(distance_m: float, cfg: PolicyConfig,
                            decel_ms2: Optional[float] = None,
                            approach_ms: float = 0.0) -> float:
    """Fastest speed from which we can still stop short of ``distance_m``.

    ``approach_ms`` is how fast the obstacle is coming towards us, and ignoring
    it is a genuine way to hit somebody. Braking is not instantaneous: it takes
    ``rho + v/a`` seconds, and a pedestrian walking head-on covers another four
    metres in that time. A formula that treats the gap as fixed will happily
    command a stop that completes exactly where the pedestrian now is.

    So the requirement is that our stopping distance plus their approach fits
    in the gap::

        v*rho + v^2/(2a) + v_o*(rho + v/a) <= d

    which is still a quadratic in v. With ``B = a*rho + v_o`` its positive root
    is ``-B + sqrt(B^2 + 2a*d - 2a*v_o*rho)``, collapsing to the stationary
    case when ``v_o = 0``. A receding obstacle is clamped to zero rather than
    credited: being handed extra speed because somebody is walking away is not
    a bet worth taking on a footpath.
    """
    a = decel_ms2 if decel_ms2 is not None else cfg.decel_comfort_ms2
    d = distance_m - cfg.stop_margin_m
    if d <= 0.0:
        return 0.0
    rho = effective_reaction_s(cfg, a)
    v_o = max(0.0, approach_ms)
    b = a * rho + v_o
    disc = b * b + 2.0 * a * d - 2.0 * a * v_o * rho
    if disc <= 0.0:
        return 0.0
    return max(0.0, -b + math.sqrt(disc))


def stopping_distance_m(speed_ms: float, cfg: PolicyConfig,
                        decel_ms2: Optional[float] = None) -> float:
    """Inverse of the above: how much room this speed needs, incl. reaction."""
    a = decel_ms2 if decel_ms2 is not None else cfg.decel_comfort_ms2
    rho = effective_reaction_s(cfg, a)
    return speed_ms * rho + (speed_ms ** 2) / (2.0 * a) + cfg.stop_margin_m


# ---------------------------------------------------------------------------
# Route handling
# ---------------------------------------------------------------------------
def route_stations(route_xy: Sequence[Tuple[float, float]],
                   ego_xy: Tuple[float, float],
                   cfg: PolicyConfig) -> List[Tuple[float, float, float]]:
    """Sample the route ahead of the cart as (arc_length, x, y).

    Arc length is measured from the cart's projection onto the route, so
    station 0 is directly under the cart and stations grow forward.
    """
    if len(route_xy) < 2:
        return []

    # Nearest segment to the cart.
    best_i, best_d, best_t = 0, float("inf"), 0.0
    for i in range(len(route_xy) - 1):
        ax, ay = route_xy[i]
        bx, by = route_xy[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            continue
        t = ((ego_xy[0] - ax) * dx + (ego_xy[1] - ay) * dy) / seg2
        t = max(0.0, min(1.0, t))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(ego_xy[0] - px, ego_xy[1] - py)
        if d < best_d:
            best_i, best_d, best_t = i, d, t

    # Walk forward from that projection, emitting evenly spaced stations.
    out: List[Tuple[float, float, float]] = []
    ax, ay = route_xy[best_i]
    bx, by = route_xy[best_i + 1]
    px, py = ax + best_t * (bx - ax), ay + best_t * (by - ay)
    s = 0.0
    out.append((0.0, px, py))
    cur = (px, py)
    i = best_i + 1
    while i < len(route_xy) and s < cfg.horizon_m:
        nx, ny = route_xy[i]
        seg = math.hypot(nx - cur[0], ny - cur[1])
        if seg < 1e-6:
            i += 1
            continue
        n_steps = int(seg / cfg.station_step_m)
        for k in range(1, n_steps + 1):
            f = (k * cfg.station_step_m) / seg
            out.append((s + k * cfg.station_step_m,
                        cur[0] + f * (nx - cur[0]),
                        cur[1] + f * (ny - cur[1])))
            if s + k * cfg.station_step_m >= cfg.horizon_m:
                return out
        s += seg
        cur = (nx, ny)
        i += 1
    return out


# ---------------------------------------------------------------------------
# Layers
# ---------------------------------------------------------------------------
def reflex_triggered(tracks: Sequence[Track], cfg: PolicyConfig
                     ) -> Optional[Track]:
    """L0: something close and roughly ahead. No forecasting, no route.

    Intentionally crude and intentionally short. This is the layer that has to
    keep working when everything above it is wrong, so it reasons only about a
    measured range and a measured lateral offset.
    """
    worst = None
    for t in tracks:
        # An emergency stop is a large, disruptive action, so it wants more
        # than one frame's evidence. Confirmation costs about 0.15 s at
        # detector rate -- a quarter of a metre at full speed -- and buys
        # immunity from single-frame detector noise slamming the brakes on a
        # public footpath. The nominal layer still slows for tentative tracks;
        # it is only the hard stop that waits.
        if not t.confirmed:
            continue
        fwd, lat = t.last_forward_m, t.last_lateral_m
        if fwd <= 0.0:
            continue
        # A clipped box's range is an upper bound, so "far" is not evidence of
        # safety -- if anything it means the object is close enough that its
        # feet have left the frame. Treat it as in range.
        if fwd > cfg.reflex_range_m and not t.clipped_bottom:
            continue
        if abs(lat) > cfg.reflex_half_width_m + 0.5 * t.width_m:
            continue
        if worst is None or fwd < worst.last_forward_m:
            worst = t
    return worst


def find_conflicts(tracks: Sequence[Track],
                   stations: Sequence[Tuple[float, float, float]],
                   ego: EgoPose, cfg: PolicyConfig) -> List[Conflict]:
    """L1: which tracks will be on our route around the time we get there."""
    conflicts: List[Conflict] = []
    if not stations:
        return conflicts

    half_w = cfg.cart_half_width_m + cfg.corridor_margin_m
    # Speed used to estimate when we would reach each station. Using the
    # current speed rather than the target keeps the estimate honest while we
    # are already slowing. The floor matters: at a dead stop, s/v is infinite
    # and nothing would ever be examined -- so the cart would resume straight
    # into whatever it stopped for. Floor it at a plausible resume speed.
    v_ref = max(ego.speed_ms, 1.0)

    for t in tracks:
        # Nothing alongside or behind us can be avoided by going slower, and
        # braking for it is both useless and alarming. A coasting track's
        # uncertainty grows, and without this a pedestrian we safely passed
        # half a second ago eventually inflates far enough to overlap the
        # station under our own wheels and command a full stop.
        if t.last_forward_m <= 0.0:
            continue
        best: Optional[Conflict] = None
        horizon = t.forecast_horizon_s(cfg.horizon_s)

        for (s, sx, sy) in stations:
            # tt = 0 -- where the object IS -- is always checked, and checked
            # first. Something standing on our route right now is a conflict
            # whatever the forecast says it is about to do; "they will probably
            # have moved" is not a thing to bet a pedestrian on. It also
            # removes the only stateful branch this function had: deciding
            # per-frame whether a track counted as moving made the decision
            # flip between two different answers as the velocity estimate
            # crossed the threshold, and a flickering target is one the speed
            # limiter cannot follow at all.
            times = [0.0]
            t_arrive = s / v_ref
            # Forecast samples are only added for objects that are actually
            # going somewhere. Extrapolating a stationary track still widens
            # its cone by the velocity uncertainty, and over a three-second
            # window that inflates a motionless pedestrian into a three-metre
            # blob -- enough to stop the cart for somebody standing well off
            # the path. For a stationary object the tt = 0 test above is not
            # merely sufficient, it is strictly better evidence.
            if t.is_moving and t_arrive <= cfg.horizon_s + cfg.arrival_window_s:
                # Then a spread of arrival times: slowing down makes us later,
                # so a single-instant test would not be conservative.
                t_hi = min(horizon, t_arrive + cfg.arrival_window_s)
                t_lo = max(0.0, min(t_arrive - cfg.arrival_window_s, t_hi))
                n = max(2, int((t_hi - t_lo) / 0.2) + 1)
                times += [t_lo + (t_hi - t_lo) * k / (n - 1) for k in range(n)]
            for tt in times:
                ox, oy = t.predict_at(tt)
                reach = t.radius_at(tt) + half_w
                if math.hypot(ox - sx, oy - sy) <= reach:
                    v_safe = safe_speed_for_distance(
                        s, cfg, approach_ms=_object_approach_ms(t, ego))
                    best = Conflict(
                        track_id=t.id, cls=t.cls, station_m=s, time_s=tt,
                        gap_m=math.hypot(t.last_forward_m, t.last_lateral_m),
                        closing_ms=_closing_rate(t, ego),
                        v_safe_ms=v_safe)
                    break
            if best is not None:
                break                      # nearest station wins; stop scanning
        if best is not None:
            conflicts.append(best)

    conflicts.sort(key=lambda c: c.v_safe_ms)
    return conflicts


def _object_approach_ms(t: Track, ego: EgoPose) -> float:
    """How fast the OBJECT is closing on us, with our own motion excluded.

    This is deliberately not ``_closing_rate``: the envelope is solving for our
    speed, so folding our current speed into the input would be circular. What
    the algebra needs is the part of the closure we cannot control.
    """
    dx, dy = t.x - ego.x, t.y - ego.y
    r = math.hypot(dx, dy)
    if r < 1e-6:
        return 0.0
    return -(t.vx * dx + t.vy * dy) / r


def _closing_rate(t: Track, ego: EgoPose) -> float:
    """Rate at which the straight-line gap to the cart is shrinking (m/s).

    Both the sight line and the relative velocity are taken in the WORLD frame.
    Mixing frames here -- world track velocity against a cart-frame sight line
    -- silently rotates the answer by the heading, which is the sort of bug
    that only shows up as a confusing number on an operator's screen.
    """
    dx, dy = t.x - ego.x, t.y - ego.y
    r = math.hypot(dx, dy)
    if r < 1e-6:
        return 0.0
    h = math.radians(ego.heading_deg)
    evx, evy = ego.speed_ms * math.sin(h), ego.speed_ms * math.cos(h)
    return -((t.vx - evx) * dx + (t.vy - evy) * dy) / r


def health_cap_mph(tracks: Sequence[Track], cfg: PolicyConfig,
                   detector_ok: bool = True, frame_age_s: float = 0.0
                   ) -> Tuple[float, str]:
    """L2: reasons to be slower that have nothing to do with any one object."""
    if not detector_ok:
        return 0.0, "perception offline"
    if frame_age_s > 0.5:
        return 0.0, f"stale camera frame ({frame_age_s:.1f}s)"
    if frame_age_s > 0.25:
        return cfg.degraded_speed_mph, f"slow camera ({frame_age_s:.2f}s)"
    # Clipped boxes are excluded: we already know why their two range cues
    # disagree -- the contact point is off the sensor -- and it says nothing
    # about the ground plane, which is what this check is for. Counting them
    # means the cart drops into degraded mode every time somebody walks past
    # closer than the blind zone, which on a footpath is constantly.
    usable = [t for t in tracks
              if t.cue_disagreement is not None and not t.clipped_bottom]
    bad = [t for t in usable if t.cue_disagreement > cfg.max_cue_disagreement]
    # A MAJORITY has to disagree, and at least two of them. What this check is
    # for -- a slope, a kerb, a bracket knocked out of calibration -- is a
    # property of the ground plane, so it shows up on everything at once. A
    # single disagreeing track is far more likely to be partially occluded,
    # which produces an identical signature and happens constantly on a
    # footpath: two people overlapping is enough. Degrading on one track means
    # crawling at 2 mph past every pair of pedestrians.
    #
    # The cost is that a genuine geometry fault with only one object in view
    # goes uncaught. That is accepted knowingly: the consequence there is an
    # over-estimated range, which the stop margin and the reflex layer are
    # sized to absorb, whereas the consequence of the alternative is a system
    # nobody leaves switched on.
    if len(bad) >= 2 and len(bad) * 2 >= len(usable):
        return (cfg.degraded_speed_mph,
                f"range cues disagree on {len(bad)} of {len(usable)} tracks — "
                f"ground plane assumption may be broken")
    return float("inf"), ""


# ---------------------------------------------------------------------------
# Top level
# ---------------------------------------------------------------------------
def evaluate(tracks: Sequence[Track],
             route_xy: Sequence[Tuple[float, float]],
             ego: EgoPose,
             cfg: Optional[PolicyConfig] = None,
             detector_ok: bool = True,
             frame_age_s: float = 0.0) -> SpeedDecision:
    """Combine all three layers into one allowed speed."""
    cfg = cfg or PolicyConfig()

    # L0 first: it can override everything and needs no route.
    hit = reflex_triggered(tracks, cfg)
    if hit is not None:
        return SpeedDecision(
            v_allowed_mph=0.0, layer="reflex", emergency=True,
            limiting_track_id=hit.id,
            reason=(f"REFLEX: {hit.cls} #{hit.id} at "
                    f"{hit.last_forward_m * M_TO_FT:.0f} ft, "
                    f"{hit.last_lateral_m * M_TO_FT:+.0f} ft lateral"))

    # L2 cap.
    cap_mph, cap_reason = health_cap_mph(tracks, cfg, detector_ok, frame_age_s)

    # L1 envelope.
    stations = route_stations(route_xy, (ego.x, ego.y), cfg)
    conflicts = find_conflicts(tracks, stations, ego, cfg)

    v_mph = cfg.max_speed_mph
    reason, limiting, layer = "clear", None, "clear"
    if conflicts:
        worst = conflicts[0]
        v_conf = worst.v_safe_ms * MPS_TO_MPH
        if v_conf > 0.0:
            v_conf = max(v_conf, cfg.creep_speed_mph)
        if v_conf < v_mph:
            v_mph, limiting, layer = v_conf, worst.track_id, "nominal"
            reason = worst.describe()

    degraded = False
    if cap_mph < v_mph:
        v_mph, layer, degraded = cap_mph, "degraded", True
        reason = cap_reason
    elif cap_reason:
        degraded = True

    return SpeedDecision(
        v_allowed_mph=max(0.0, v_mph), reason=reason,
        limiting_track_id=limiting, emergency=False, degraded=degraded,
        conflicts=conflicts, layer=layer)


# ---------------------------------------------------------------------------
# Smoothing
# ---------------------------------------------------------------------------
class SpeedLimiter:
    """Turns a possibly-jumpy allowed speed into something comfortable.

    Two things make a speed controller feel bad: changing acceleration abruptly
    (jerk), and hunting up and down as a detection flickers. This limits the
    first directly and damps the second with a hold-off before resuming.

    The emergency path deliberately bypasses the comfort limits. A jerk budget
    is a comfort preference; it is not worth a collision.
    """

    def __init__(self, cfg: Optional[PolicyConfig] = None,
                 initial_mph: float = 0.0):
        self.cfg = cfg or PolicyConfig()
        self.v_ms = initial_mph * MPH_TO_MPS
        self.a_ms2 = 0.0
        self._slow_since: Optional[float] = None
        self._held_ms = 0.0

    def reset(self, mph: float = 0.0) -> None:
        self.v_ms = mph * MPH_TO_MPS
        self.a_ms2 = 0.0
        self._slow_since = None
        self._held_ms = 0.0

    def step(self, target_mph: float, dt: float, now: float,
             emergency: bool = False) -> float:
        """Advance the smoothed setpoint one control tick. Returns mph."""
        cfg = self.cfg
        dt = max(1e-3, min(dt, 0.5))
        target_ms = max(0.0, target_mph) * MPH_TO_MPS

        # Resume hysteresis: after being held down, wait before climbing again
        # so a detection blinking in and out cannot pump the throttle. During
        # the hold we keep the LOW target rather than reverting to the current
        # speed. That distinction matters more than it looks: with a detection
        # alternating present/absent every frame, holding at the current speed
        # makes the braking ticks and the holding ticks cancel through the jerk
        # limiter, and the cart sails on at almost full speed past something it
        # has already decided to stop for.
        if target_ms <= self.v_ms + 1e-6:
            self._slow_since = now
            self._held_ms = target_ms
        elif self._slow_since is not None and (now - self._slow_since) < cfg.resume_hold_s:
            target_ms = min(target_ms, self._held_ms)

        err = target_ms - self.v_ms
        a_want = err / dt

        if emergency:
            a_max, jerk = cfg.decel_emergency_ms2, cfg.emergency_jerk_ms3
            a_want = max(a_want, -a_max)
            a_want = min(a_want, cfg.accel_limit_ms2)
        else:
            a_want = max(-cfg.decel_limit_ms2, min(a_want, cfg.accel_limit_ms2))
            jerk = cfg.jerk_limit_ms3

        # Approach ceiling: never carry more acceleration than can still be
        # ramped back to zero, at the jerk limit, within the speed error that
        # remains. Without this the filter arrives at its target still
        # accelerating and has to snap -- a discontinuity precisely at the
        # moment the ride is supposed to settle. sqrt(2*j*|err|) is the speed
        # error that a linear ramp-down of ``a`` consumes, and holding to it
        # produces the S-curve rather than a triangle.
        ceiling = math.sqrt(2.0 * jerk * abs(err))
        a_want = min(a_want, ceiling) if err >= 0 else max(a_want, -ceiling)

        # Jerk limit: the acceleration itself may only change so fast.
        da_max = jerk * dt
        self.a_ms2 += max(-da_max, min(a_want - self.a_ms2, da_max))

        # Land on the target rather than through it. Clamping the acceleration
        # to the value that exactly reaches it keeps the speed trajectory
        # continuous; snapping the speed afterwards would not.
        land = err / dt
        self.a_ms2 = min(self.a_ms2, land) if err >= 0 else max(self.a_ms2, land)

        self.v_ms = max(0.0, self.v_ms + self.a_ms2 * dt)
        return self.v_ms * MPS_TO_MPH


# ---------------------------------------------------------------------------
# The one object the follower talks to
# ---------------------------------------------------------------------------
class Governor:
    """Stateful front door: tracks in, one smoothed allowed speed out.

    Everything above this line is a pure function of its arguments, which is
    what makes it testable. The two things that genuinely need memory live
    here: the smoothing filter, and the reflex latch.

    The latch deserves an explanation, because it is the difference between a
    cart that stops and a cart that stops and then rolls forward anyway. A
    forward camera cannot see the ground within ``blind_zone_m`` of the cart.
    So the last thing that happens on every close approach is that the obstacle
    slides out of frame -- and to a stateless policy, "no longer detected" and
    "no longer there" are the same observation. The latch holds the stop across
    that window. It is time-based rather than clever on purpose: the honest
    statement is "we cannot see there right now", and the honest response is to
    wait rather than to infer.
    """

    def __init__(self, cfg: Optional[PolicyConfig] = None,
                 initial_mph: float = 0.0):
        self.cfg = cfg or PolicyConfig()
        self.limiter = SpeedLimiter(self.cfg, initial_mph)
        self.decision = SpeedDecision(v_allowed_mph=0.0)
        self._latch_until: float = -1e18

    def reset(self, mph: float = 0.0) -> None:
        self.limiter.reset(mph)
        self._latch_until = -1e18

    def step(self, tracks: Sequence[Track],
             route_xy: Sequence[Tuple[float, float]],
             ego: EgoPose, now: float, dt: float,
             detector_ok: bool = True,
             frame_age_s: float = 0.0) -> float:
        """Returns the smoothed allowed speed in mph. Also sets ``.decision``."""
        cfg = self.cfg
        dec = evaluate(tracks, route_xy, ego, cfg, detector_ok, frame_age_s)

        if dec.emergency:
            self._latch_until = now + cfg.reflex_latch_s
        elif now < self._latch_until:
            dec = SpeedDecision(
                v_allowed_mph=0.0, layer="reflex", emergency=True,
                degraded=dec.degraded, conflicts=dec.conflicts,
                reason=(f"holding {self._latch_until - now:.1f}s after a reflex "
                        f"stop — the object is in the camera's blind zone"))

        self.decision = dec
        return self.limiter.step(min(cfg.max_speed_mph, dec.v_allowed_mph),
                                 dt, now, dec.emergency)
