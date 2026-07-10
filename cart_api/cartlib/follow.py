"""
cartlib.follow — autonomous GPS path following for the cart.

A conservative cross-track controller that ties the RTK position to steering
and throttle WITHOUT ever computing a GPS heading:

  * Steering is a heading-free cross-track PD law. ``signed_distance_m`` from
    the path geometry tells us which side of the line we're on (and how far);
    its rate of change supplies the damping term that a heading would normally
    provide (cross_rate ~= v * sin(heading_error), so PD on
    (cross, cross_rate) is a Stanley-style law that needs no world heading).
    The cart's own steering-wheel angle sensor closes the inner loop.
  * Throttle ramps toward ``max_speed_mph`` using the live GPS speed as
    feedback, hard-capped at ``gas_cap`` and backed off in sharp turns.
  * Throttle is cut to zero (with brake) at the goal.

SAFETY — this AUTONOMOUSLY DRIVES THE CART. The follower:
  * caps gas at ``gas_cap`` (default = FSD_GAS_LIMIT, 0.25),
  * stops on e-stop, on lost/old GPS, on excessive cross-track error, and at
    the final waypoint,
  * defaults to DRY-RUN (computes + prints, sends nothing) unless armed.

Path format: a list of (lat, lon) tuples. ``load_path`` accepts JSON in the
drivelive ({lat,lng}), lane_tracker (fix dicts with lat/lon), or plain
[[lat,lon], ...] shapes.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import List, Optional

from . import config, geo
from .cart import Cart


def load_path(path_file: str) -> List[geo.LatLon]:
    """Load waypoints from a JSON file, tolerant of several shapes."""
    with open(path_file) as f:
        data = json.load(f)
    # Unwrap common containers.
    if isinstance(data, dict):
        for key in ("points", "path", "waypoints"):
            if key in data:
                data = data[key]
                break
    pts: List[geo.LatLon] = []
    for item in data:
        if isinstance(item, dict):
            lat = item.get("lat")
            lon = item.get("lon", item.get("lng"))
        else:  # [lat, lon] pair
            lat, lon = item[0], item[1]
        if lat is not None and lon is not None:
            pts.append((float(lat), float(lon)))
    if len(pts) < 2:
        raise ValueError(f"{path_file}: need >=2 waypoints, got {len(pts)}")
    return pts


@dataclass
class FollowConfig:
    lookahead_m: float = 2.0          # cross-track correction lookahead distance
    gas_cap: float = config.FSD_GAS_LIMIT   # hard ceiling the controller may push to
    max_speed_mph: float = 3.0        # target cruise speed (closed-loop setpoint)
    live_speed_mph: float = 0.0       # GPS-derived speed currently shown in UI
    # Closed-loop speed control: PI on the GPS speed (so we actually hit the
    # requested mph despite hills / load / open-loop miscalibration) plus a
    # brake term that bleeds overspeed on descents.
    speed_kp: float = 0.030           # gas pot added per mph of speed deficit
    speed_ki: float = 0.012           # gas pot per (mph*s) of accumulated error
    speed_i_max: float = 35.0         # clamp on the error integral (mph*s)
    brake_kp: float = 0.08            # brake pot per mph of overspeed
    brake_deadband_mph: float = 0.5   # tolerate this much overspeed before braking
    # --- LEGACY steering knobs (kept so the server/UI still construct a config;
    #     the smooth Stanley law below uses the `st_*` params, not these). ---
    steer_gain: float = 5.4           # (legacy) old PD proportional gain
    xtrack_gain: float = 1.5          # (legacy) old cross-track multiplier
    heading_gain: float = 3.0         # (legacy) old cross-rate damping gain
    heading_min_speed_mph: float = 0.5
    steer_sign: float = 1.0           # hardware steering sign convention (+1 = right)
    max_steer_deg: float = 320.0      # clamp on commanded column angle
    turn_slowdown: float = 0.0        # gas *= 1/(1+turn_slowdown*|steer|/max_steer)

    # --- SMOOTH steering law (Stanley + curvature feedforward + fused heading).
    #     Tuned in the off-vehicle simulator (cart_api/sim) against all three
    #     recorded RTK drives: ~4-10x lower steering rate, tighter tracking, no
    #     full-lock sawing, stable at 5.5 mph. See sim/controllers.py::NewController.
    st_k_cross: float = 0.40          # Stanley cross-track gain in atan(k*e/(v+soft))
    st_cross_soft_mps: float = 0.9    # softening speed so low speed doesn't saturate
    st_k_heading: float = 1.2         # heading-alignment weight (damping to the line)
    st_max_cross_road_deg: float = 24.0   # clamp the cross term's road-angle demand
    st_k_int: float = 0.10            # integral trim: road-wheel deg per (m*s)
    st_int_max_deg: float = 5.0
    st_int_enable_cross_m: float = 1.0
    st_gps_lookback_m: float = 0.9    # travel window for the absolute GPS heading
    st_gps_correct_gain: float = 0.4  # 1/s pull of fused heading toward GPS heading.
                                      # Deliberately low: the wheel-angle yaw model
                                      # is lag-free while the backward-looking GPS
                                      # track lags ~0.3 s, so a slow crossover
                                      # (~0.06 Hz) keeps the heading (hence the
                                      # damping term) in phase. A high gain dragged
                                      # the heading a quarter-cycle late and the
                                      # cart weaved across the line.
    st_min_track_speed_mph: float = 0.9
    st_tau_cmd_s: float = 0.25        # low-pass time constant on the column target
    st_slew_deg_s: float = 320.0      # controller-side slew clamp (<= actuator limit)
    goal_radius_m: float = 1.5        # within this of last point => arrived
    # Final-approach deceleration: ease the speed setpoint down to a crawl over
    # arrival_slowdown_m so we glide to the goal instead of cruising flat-out
    # until the cutoff and lurching, then hold a firm brake to a full stop.
    arrival_slowdown_m: float = 6.0   # start easing speed down within this of goal
    arrival_creep_mph: float = 1.0    # floor speed kept until inside goal_radius
    arrival_brake: float = 0.30       # brake pot held once arrived (full stop & park)
    max_crosstrack_m: float = 6.0     # abort if we stray this far off path
    gps_max_age_s: float = 2.5        # stop if fix older than this
    require_rtk: bool = False         # require RTK Fixed/Float to drive
    rate_hz: float = 12.0             # control loop rate (serial read fix lets us
                                      # actually hit this now — was ~2 Hz stalled)


@dataclass
class FollowState:
    phase: str = "init"               # init | tracking | done | abort
    reason: str = ""


class PathFollower:
    def __init__(self, cart: Cart, path: List[geo.LatLon],
                 cfg: Optional[FollowConfig] = None, armed: bool = False):
        self.cart = cart
        self.path = path
        self.cfg = cfg or FollowConfig()
        self.cfg.gas_cap = config.effective_gas_cap(self.cfg.gas_cap)
        self.armed = armed             # False => dry-run (no actuator output)
        self.state = FollowState()
        self._last_steer_read_ts = 0.0
        self._last_actual_steer_deg: Optional[float] = None
        # closed-loop speed control state
        self._speed_integral: float = 0.0
        self._prev_speed_ts: Optional[float] = None
        # --- smooth steering law state ---
        self._origin = self.path[0]        # local-frame origin for the heading est
        self._st_integral: float = 0.0     # road-wheel deg trim
        self._st_cmd: float = 0.0          # filtered column target
        self._fused_heading: Optional[float] = None   # deg, absolute compass (0=N)
        self._hist: list = []              # [(cum_s, x, y)] recent track
        self._cum_s: float = 0.0
        self._last_xy: Optional[tuple] = None
        self._prev_step_ts: Optional[float] = None

    # -- helpers -----------------------------------------------------------
    def _apply(self, gas: float, brake: float, steer_deg: Optional[float]) -> None:
        """Send (or, in dry-run, just record) actuator commands."""
        gas = max(0.0, min(gas, self.cfg.gas_cap))
        if not self.armed:
            return
        if steer_deg is not None and self.cart.steering and self.cart.steering._enabled:
            self.cart.steering.set_angle(steer_deg)
        if self.cart.pedals:
            self.cart.pedals.set_brake(brake)
            self.cart.pedals.set_gas(gas)

    def _stop(self, reason: str, brake: float = 0.15) -> None:
        self.state.reason = reason
        if self.armed and self.cart.pedals:
            self.cart.pedals.set_gas(0.0)
            self.cart.pedals.set_brake(brake)

    def _path_curvature(self, along_m: float, w: float = 2.5) -> float:
        """Signed path curvature (1/m) at arc-length ``along_m``.

        +curvature = path bending to the RIGHT (bearing increasing clockwise),
        which is the sign the road-wheel feedforward needs (positive column =
        right turn). Estimated from the bearing change over a ±w window so a
        single noisy vertex can't spike it.
        """
        a, _ = geo.point_at_distance(self.path, max(0.0, along_m - w))
        b, _ = geo.point_at_distance(self.path, along_m)
        c, _ = geo.point_at_distance(self.path, along_m + w)
        if a == b or b == c:
            return 0.0
        h1 = geo.bearing_deg(a, b)
        h2 = geo.bearing_deg(b, c)
        dh = geo.angle_diff_deg(h2, h1)   # signed, wrapped to (-180, 180]
        return math.radians(dh) / (2.0 * w)

    def _update_fused_heading(self, pos, v_ms: float, wheel_deg: float,
                              dt: float) -> Optional[float]:
        """Absolute cart heading (deg, compass) via a complementary filter:
        high-frequency wheel-angle yaw integration + low-frequency GPS track.

        The wheel-angle model is clean and lag-free; the GPS track over a short
        travel window is absolute and drift-free. Fusing them gives a smooth,
        trustworthy heading at any speed — the thing the old law never had, so it
        had to damp on a raw differentiated cross-track and limit-cycled.
        """
        c = self.cfg
        x, y = geo.local_xy(self._origin, pos)
        if self._last_xy is not None:
            self._cum_s += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        self._hist.append((self._cum_s, x, y))
        while len(self._hist) > 2 and self._cum_s - self._hist[0][0] > 3 * c.st_gps_lookback_m:
            self._hist.pop(0)

        gps_heading = None
        if v_ms > c.st_min_track_speed_mph * 0.44704:
            for s0, hx, hy in self._hist:
                if self._cum_s - s0 >= c.st_gps_lookback_m:
                    dx, dy = x - hx, y - hy
                    if math.hypot(dx, dy) > 0.3:
                        # bearing of travel: 0 = north, + = clockwise (east)
                        gps_heading = math.degrees(math.atan2(dx, dy)) % 360.0
                    break

        if self._fused_heading is None:
            self._fused_heading = gps_heading  # may stay None until we've moved
            return self._fused_heading

        # predict with the wheel-angle yaw-rate model (responsive, lag-free)
        road = math.radians(config.column_to_roadwheel_deg(wheel_deg))
        yaw_rate_deg = math.degrees(v_ms / config.WHEELBASE_M * math.tan(road))
        self._fused_heading = (self._fused_heading + yaw_rate_deg * dt) % 360.0
        # correct toward the absolute GPS heading (low frequency, no drift)
        if gps_heading is not None:
            k = min(1.0, c.st_gps_correct_gain * dt)
            self._fused_heading = (
                self._fused_heading
                + k * geo.angle_diff_deg(gps_heading, self._fused_heading)) % 360.0
        return self._fused_heading

    # -- one control step --------------------------------------------------
    def step(self) -> dict:
        """Run one control cycle. Returns a telemetry dict for logging/UI."""
        c = self.cfg
        fix = self.cart.gps.latest if self.cart.gps else None
        ped = self.cart.pedals.telemetry if self.cart.pedals else {}

        # --- safety gates ---
        if ped.get("estop"):
            self.state.phase = "abort"
            self._stop("E-STOP engaged")
            return self._telemetry(fix, None, 0.0)
        if not fix or (time.time() - fix["ts"]) > c.gps_max_age_s:
            self.state.phase = "abort"
            self._stop("GPS fix lost/stale")
            return self._telemetry(fix, None, 0.0)
        if c.require_rtk and fix["fix_code"] not in (4, 5):
            self.state.phase = "abort"
            self._stop(f"not RTK (fix={fix['fix_type']})")
            return self._telemetry(fix, None, 0.0)

        pos = (fix["lat"], fix["lon"])

        # --- goal check ---
        dist_to_goal = geo.haversine_m(pos, self.path[-1])
        near_last = geo.nearest_index(self.path, pos) >= len(self.path) - 1
        if near_last and dist_to_goal <= c.goal_radius_m:
            self.state.phase = "done"
            self._stop("arrived at goal", brake=c.arrival_brake)
            return self._telemetry(fix, None, 0.0, brake=c.arrival_brake)

        # --- smooth path tracking (Stanley + curvature FF + fused heading) ---
        self.state.phase = "tracking"
        snap = geo.nearest_point_on_path(self.path, pos)

        # cross-track abort
        xtrack = snap.distance_m
        if xtrack > c.max_crosstrack_m:
            self.state.phase = "abort"
            self._stop(f"off path ({xtrack:.1f} m > {c.max_crosstrack_m} m)")
            return self._telemetry(fix, None, 0.0)

        # Signed cross-track error (+ = cart is LEFT of the path direction).
        cross = snap.signed_distance_m

        # Direction the purple line is heading at the nearest segment (compass).
        seg_i = snap.segment_index
        a = self.path[seg_i]
        b = self.path[min(seg_i + 1, len(self.path) - 1)]
        path_bearing = geo.bearing_deg(a, b)

        # control-loop dt (for the heading filter, integral, and command shaping)
        now = time.time()
        dt = (now - self._prev_step_ts) if self._prev_step_ts is not None else 1.0 / c.rate_hz
        dt = max(1e-3, min(dt, 0.5))
        self._prev_step_ts = now
        v_ms = c.live_speed_mph * 0.44704

        # Absolute cart heading from the wheel-angle yaw model fused with the GPS
        # track (see _update_fused_heading). The measured wheel angle drives the
        # lag-free prediction; GPS anchors it. This replaces the old, noisy
        # cross-rate "heading" that made the loop saw across the line.
        wheel_deg = self._last_actual_steer_deg or 0.0
        fused = self._update_fused_heading(pos, v_ms, wheel_deg, dt)
        # heading error: + = cart pointing RIGHT of the path direction
        heading_err = geo.angle_diff_deg(fused, path_bearing) if fused is not None else 0.0

        # Stanley law, in ROAD-WHEEL degrees (POSITIVE = right turn):
        #   ff    — curvature feedforward: the steady angle that holds the curve.
        #   head  — align to the path direction (steer opposite the heading err).
        #   cross — bounded pull back toward the line; atan keeps it gentle so a
        #           2 m error at 3 mph asks for ~10 deg, never full lock.
        curvature = self._path_curvature(snap.along_m)
        ff_deg = math.degrees(math.atan(config.WHEELBASE_M * curvature))
        head_deg = -c.st_k_heading * heading_err
        v_soft = v_ms + c.st_cross_soft_mps
        cross_deg = math.degrees(math.atan(c.st_k_cross * cross / max(v_soft, 0.2)))
        cross_deg = max(-c.st_max_cross_road_deg, min(cross_deg, c.st_max_cross_road_deg))

        # integral trim — null a steady bias (road crown / calibration) only when
        # already close and moving; bleed it off otherwise so it can't wind up.
        if abs(cross) < c.st_int_enable_cross_m and v_ms > 0.4:
            self._st_integral += c.st_k_int * cross * dt
            self._st_integral = max(-c.st_int_max_deg, min(self._st_integral, c.st_int_max_deg))
        else:
            self._st_integral *= 0.98

        road_deg = ff_deg + head_deg + cross_deg + self._st_integral
        column_raw = c.steer_sign * config.STEER_RATIO * road_deg
        column_raw = max(-c.max_steer_deg, min(column_raw, c.max_steer_deg))

        # Command shaping: low-pass then slew clamp so the column moves as one
        # continuous motion instead of the old per-cycle jumps.
        alpha = dt / (c.st_tau_cmd_s + dt)
        target = self._st_cmd + alpha * (column_raw - self._st_cmd)
        max_step = c.st_slew_deg_s * dt
        target = self._st_cmd + max(-max_step, min(target - self._st_cmd, max_step))
        self._st_cmd = target
        steer_deg = target
        correction_deg = road_deg   # reported as `alpha` in telemetry

        # Map needle = the cart's actual (fused) heading; falls back to the path
        # bearing leaned by the wheel angle until the fused estimate is seeded.
        heading_abs = fused if fused is not None else (path_bearing + wheel_deg) % 360.0

        # --- throttle: closed-loop speed control (PI on GPS mph + brake) ---
        # Open-loop gas is miscalibrated (0.24 gave only ~2 mph) and blind to
        # grade, so we feed the GPS speed back: a feedforward guess gets us in
        # the ballpark, the integral adds gas to hold speed up a climb, and the
        # brake bleeds overspeed on a descent. The setpoint is max_speed_mph;
        # gas_cap is just the safety ceiling the controller may push to.
        now = time.time()
        dt_spd = (now - self._prev_speed_ts) if self._prev_speed_ts is not None else 0.0
        dt_spd = max(0.0, min(dt_spd, 1.0))   # ignore long gaps (startup/stalls)
        self._prev_speed_ts = now

        # Final-approach taper: ramp the setpoint linearly down to the creep
        # speed across arrival_slowdown_m so we decelerate smoothly into the
        # goal. The overspeed-brake term below bleeds the excess as the setpoint
        # drops; the goal cutoff above then holds the firm parking brake.
        target_mph = c.max_speed_mph
        if dist_to_goal < c.arrival_slowdown_m:
            frac = dist_to_goal / max(c.arrival_slowdown_m, 0.1)
            target_mph = max(c.arrival_creep_mph, c.max_speed_mph * frac)

        speed_error = target_mph - c.live_speed_mph          # + => want to speed up
        ff_gas = config.gas_for_mph(target_mph)               # open-loop ballpark
        i_cand = self._speed_integral + speed_error * dt_spd
        i_cand = max(-c.speed_i_max, min(i_cand, c.speed_i_max))

        gas_cmd = ff_gas + c.speed_kp * speed_error + c.speed_ki * i_cand
        gas_cmd /= (1.0 + c.turn_slowdown * abs(steer_deg) / max(c.max_steer_deg, 1.0))
        applied_gas = max(0.0, min(gas_cmd, c.gas_cap))

        # Conditional anti-windup: only commit the integral step when it isn't
        # pushing further into a saturated pedal (cap reached / gas already 0).
        sat_high = gas_cmd >= c.gas_cap and speed_error > 0
        sat_low = gas_cmd <= 0.0 and speed_error < 0
        if not (sat_high or sat_low):
            self._speed_integral = i_cand

        # Brake only once gas is cut and we're still over target past the
        # deadband (downhill); proportional to the overspeed.
        applied_brake = 0.0
        over = -speed_error - c.brake_deadband_mph
        if over > 0.0:
            applied_gas = 0.0
            applied_brake = min(c.brake_kp * over, config.BRAKE_POT_MAX)

        self._apply(gas=applied_gas, brake=applied_brake, steer_deg=steer_deg)
        return self._telemetry(fix, correction_deg, applied_gas, steer_deg,
                               snap.segment_index, xtrack, dist_to_goal, cross,
                               heading_abs, round(heading_err, 1), applied_brake)

    def _telemetry(self, fix, alpha, gas, steer_deg=0.0, look_i=-1,
                   xtrack=0.0, dist_to_goal=0.0, cross=None,
                   heading_deg=None, heading_err_deg=None, brake=0.0) -> dict:
        steering_actual = self._last_actual_steer_deg
        steering_target = None
        if self.cart.steering:
            steering_target = getattr(self.cart.steering, "target_deg", None)
            now = time.time()
            if now - self._last_steer_read_ts >= 0.25:
                self._last_steer_read_ts = now
                try:
                    steering_actual = self.cart.steering.angle_deg()
                    self._last_actual_steer_deg = steering_actual
                except Exception:
                    pass
        return {
            "phase": self.state.phase,
            "reason": self.state.reason,
            "fix": fix["fix_type"] if fix else None,
            "lat": fix["lat"] if fix else None,
            "lon": fix["lon"] if fix else None,
            "ts": fix["ts"] if fix else None,
            "alpha": round(alpha, 1) if alpha is not None else None,
            "steer_cmd": round(steer_deg, 1),
            "gas": round(gas, 3),
            "brake": round(brake, 3),
            "max_speed_mph": round(self.cfg.max_speed_mph, 1),
            "lookahead_i": look_i,
            "xtrack_m": round(xtrack, 2),
            "xtrack_signed_m": round(cross, 2) if cross is not None else None,
            "heading_deg": round(heading_deg, 1) if heading_deg is not None else None,
            "heading_err_deg": heading_err_deg,
            "dist_to_goal_m": round(dist_to_goal, 1),
            "steering_actual_deg": round(steering_actual, 1) if steering_actual is not None else None,
            "steering_target_deg": round(steering_target, 1) if steering_target is not None else None,
            "live_speed_mph": round(self.cfg.live_speed_mph, 1),
            "lookahead_m": round(self.cfg.lookahead_m, 2),
            "steer_gain": round(self.cfg.steer_gain, 2),
            "xtrack_gain": round(self.cfg.xtrack_gain, 2),
            "max_steer_deg": round(self.cfg.max_steer_deg, 1),
            "turn_slowdown": round(self.cfg.turn_slowdown, 2),
            "armed": self.armed,
        }

    # -- run loop ----------------------------------------------------------
    def run(self, on_step=None) -> str:
        """Drive the loop until done/abort. Returns the terminating phase.

        ``on_step(telemetry)`` is called every cycle (use it to print/log).
        Always releases the throttle on exit.
        """
        period = 1.0 / self.cfg.rate_hz
        try:
            while True:
                tele = self.step()
                if on_step:
                    on_step(tele)
                if self.state.phase in ("done", "abort"):
                    return self.state.phase
                time.sleep(period)
        finally:
            if self.armed and self.cart.pedals:
                self.cart.pedals.set_gas(0.0)
                self.cart.pedals.stop()
