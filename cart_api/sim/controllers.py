"""
controllers — the OLD (recorded) steering law and the NEW smooth law, both
driving the simlib plant so they can be compared on identical routes.

Both expose:  compute(snap, v_ms, wheel_deg, dt, pos=None) -> column_target_deg
where `snap` is a simlib.Snap, `v_ms` the speed, `wheel_deg` the current
measured column angle (the ODrive encoder), `dt` the control period, and `pos`
the cart's (x, y) in the local frame (used for the GPS-track heading estimate).
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from simlib import Snap, WHEELBASE_M, STEER_RATIO, MAX_ROADWHEEL_DEG, angle_wrap


# ===========================================================================
# OLD controller — faithful reimplementation of cartlib/follow.py step()
# ===========================================================================
@dataclass
class OldConfig:
    lookahead_m: float = 3.0
    steer_gain: float = 5.4
    xtrack_gain: float = 1.5
    heading_gain: float = 3.0
    max_steer_deg: float = 320.0
    heading_min_speed_mph: float = 0.5


class OldController:
    """Heading-free cross-track PD (the current on-cart law)."""
    def __init__(self, cfg: OldConfig | None = None):
        self.cfg = cfg or OldConfig()
        self._prev_cross = None
        self._herr_ewma = None

    def reset(self):
        self._prev_cross = None
        self._herr_ewma = None

    def _smooth_herr(self, herr):
        if self._herr_ewma is None:
            self._herr_ewma = herr
        else:
            self._herr_ewma = 0.6 * self._herr_ewma + 0.4 * herr
        return self._herr_ewma

    def compute(self, snap: Snap, v_ms: float, wheel_deg: float, dt: float,
                pos=None) -> float:
        c = self.cfg
        cross = snap.signed_m
        cross_rate = 0.0
        if self._prev_cross is not None and dt > 1e-3:
            cross_rate = (cross - self._prev_cross) / dt
        self._prev_cross = cross
        speed_ms = max(v_ms, 0.3)
        raw_herr = math.degrees(math.asin(max(-1.0, min(1.0, cross_rate / speed_ms))))
        heading_err = self._smooth_herr(raw_herr)
        if v_ms < c.heading_min_speed_mph * 0.44704:
            heading_err = 0.0
        pull_deg = math.degrees(math.atan2(c.xtrack_gain * cross, max(c.lookahead_m, 0.5)))
        align_deg = c.heading_gain * heading_err
        correction = pull_deg + align_deg
        raw = c.steer_gain * correction
        return max(-c.max_steer_deg, min(raw, c.max_steer_deg))


# ===========================================================================
# NEW controller — smooth Stanley + curvature feedforward + fused heading
# ===========================================================================
@dataclass
class NewConfig:
    # --- Stanley feedback. Gentle enough to stay smooth (a 2 m error at 3 mph
    #     asks for ~15 deg road angle, not full lock) but firm enough to cut onto
    #     the line directly instead of drifting in on a long shallow tail. ---
    k_cross: float = 0.40         # Stanley cross-track gain in atan(k*e/(v+soft))
    cross_soft_mps: float = 0.9   # softening speed so low-v doesn't saturate
    k_heading: float = 1.2        # heading-alignment weight (damping to the line)
    max_cross_road_deg: float = 24.0  # clamp the cross term's road-angle demand
    # --- integral trim for steady bias (crown, miscalibration) ---
    k_int: float = 0.10           # road-wheel deg per (m*s) of cross error
    int_max_deg: float = 5.0
    int_enable_cross_m: float = 1.0   # only integrate once reasonably close
    # --- heading estimator (GPS track fused with wheel-angle yaw rate) ---
    gps_lookback_m: float = 0.9   # travel window used for the absolute GPS heading
    gps_correct_gain: float = 0.4 # 1/s pull of fused heading toward GPS heading.
                                  # Low on purpose: the wheel-angle yaw model is
                                  # lag-free, the backward-looking GPS track lags
                                  # ~0.3 s. A slow crossover (~0.06 Hz) lets the
                                  # prediction carry the fast heading changes so
                                  # the damping term stays in phase; GPS only
                                  # removes slow drift. A high gain here dragged
                                  # the heading a quarter-cycle late and the cart
                                  # weaved across the line.
    min_track_speed_mps: float = 0.4
    # --- command shaping ---
    tau_cmd_s: float = 0.25       # low-pass time constant on the column target
    slew_deg_s: float = 320.0     # controller-side slew clamp (<= actuator limit)
    max_steer_deg: float = 320.0


class NewController:
    """
    Smooth, continuous path tracking (Stanley + curvature feedforward) with a
    properly-anchored heading estimate.

        column = STEER_RATIO * (delta_ff + delta_heading + delta_cross) + trim
          delta_ff      = atan(L * kappa_path)            # curve feedforward
          delta_heading = -k_heading * heading_err        # align to path dir
          delta_cross   = atan(k_cross * e / (v + soft))  # Stanley pull to line

    heading_err = fused_heading - path_heading. `fused_heading` is a
    complementary filter: propagated at high frequency by the wheel-angle
    yaw-rate model (clean, zero lag) and corrected at low frequency toward the
    absolute GPS-track heading over a ~1.4 m travel window (no drift, robust to
    RTK noise). This is the piece the old law never had — it tried to damp using
    a raw 0.5 s-differentiated cross-rate, which was so noisy it limit-cycled.
    With a real heading, the Stanley law is inherently smooth: both terms are
    bounded atans, so it eases off as it reaches the line instead of sawing
    across it. The column target is then low-pass + slew limited so the wheel
    moves as one continuous motion.
    """
    def __init__(self, cfg: NewConfig | None = None):
        self.cfg = cfg or NewConfig()
        self.reset()

    def reset(self):
        self._integral = 0.0        # road-wheel deg
        self._cmd = 0.0             # filtered column target
        self._fused_heading = None  # rad, absolute (0 = north)
        self._hist = []             # [(cum_s, x, y)]
        self._cum_s = 0.0
        self._last_xy = None

    def _update_heading(self, x, y, v_ms, wheel_deg, dt):
        """Complementary filter: wheel-angle yaw integration + GPS-track heading."""
        c = self.cfg
        if self._last_xy is not None:
            self._cum_s += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        self._hist.append((self._cum_s, x, y))
        while len(self._hist) > 2 and self._cum_s - self._hist[0][0] > 3 * c.gps_lookback_m:
            self._hist.pop(0)

        # absolute heading from the GPS track over the lookback window
        gps_heading = None
        if v_ms > c.min_track_speed_mps:
            for s0, hx, hy in self._hist:
                if self._cum_s - s0 >= c.gps_lookback_m:
                    dx, dy = x - hx, y - hy
                    if math.hypot(dx, dy) > 0.3:
                        gps_heading = math.atan2(dx, dy)   # 0 = north
                    break

        if self._fused_heading is None:
            # No trustworthy heading yet (not enough motion to derive a GPS
            # track). Stay None so the heading term is SUPPRESSED — never pin it
            # to an arbitrary north, which would fabricate a huge heading error
            # and yank the wheel the wrong way for the first ~1 s. Matches the
            # on-cart follow.py behavior.
            self._fused_heading = gps_heading  # may stay None until we've moved
            return self._fused_heading

        # predict with the wheel-angle yaw-rate model (responsive, no lag)
        delta = math.radians(max(-MAX_ROADWHEEL_DEG,
                                 min(wheel_deg / STEER_RATIO, MAX_ROADWHEEL_DEG)))
        yaw_rate = v_ms / WHEELBASE_M * math.tan(delta)
        self._fused_heading = angle_wrap(self._fused_heading + yaw_rate * dt)
        # correct toward the absolute GPS heading (low frequency, no drift)
        if gps_heading is not None:
            k = min(1.0, c.gps_correct_gain * dt)
            self._fused_heading = angle_wrap(
                self._fused_heading + k * angle_wrap(gps_heading - self._fused_heading))
        return self._fused_heading

    def compute(self, snap: Snap, v_ms: float, wheel_deg: float, dt: float,
                pos=None) -> float:
        c = self.cfg
        e = snap.signed_m            # + = left of path
        v = max(v_ms, 0.05)

        if pos is not None:
            fused = self._update_heading(pos[0], pos[1], v_ms, wheel_deg, dt)
            heading_err = (angle_wrap(fused - snap.path_heading)
                           if fused is not None else 0.0)
        else:
            heading_err = 0.0

        delta_ff = math.atan(WHEELBASE_M * snap.curvature)
        delta_heading = -c.k_heading * heading_err
        raw_cross = math.atan(c.k_cross * e / (v + c.cross_soft_mps))
        cross_lim = math.radians(c.max_cross_road_deg)
        delta_cross = max(-cross_lim, min(raw_cross, cross_lim))

        if abs(e) < c.int_enable_cross_m and v_ms > 0.4:
            self._integral += c.k_int * e * dt
            self._integral = max(-c.int_max_deg, min(self._integral, c.int_max_deg))
        else:
            self._integral *= 0.98

        delta_road_deg = math.degrees(delta_ff + delta_heading + delta_cross)
        column_raw = STEER_RATIO * (delta_road_deg + self._integral)
        column_raw = max(-c.max_steer_deg, min(column_raw, c.max_steer_deg))

        # command shaping: low-pass then slew clamp for one smooth motion
        alpha = dt / (c.tau_cmd_s + dt)
        target = self._cmd + alpha * (column_raw - self._cmd)
        max_step = c.slew_deg_s * dt
        target = self._cmd + max(-max_step, min(target - self._cmd, max_step))
        self._cmd = target
        return target
