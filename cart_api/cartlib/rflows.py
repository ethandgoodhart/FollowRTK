"""
cartlib.rflows — the cart as a Retriever dataflow graph.

The same autonomy that ``follow.py`` runs in one hand-rolled ``while`` loop,
expressed as five typed Flows:

    GpsSourceFlow  @Rate(10)      owns the u-blox        -> GpsFix
    FollowerFlow   @Rate(15)      the control law        -> DriveCmd
    SteeringFlow   @Trigger       owns the ODrive        -> SteerState  (feedback)
    PedalFlow      @Trigger       owns the Arduino       -> PedalState  (feedback)
    TelemetryFlow  @Trigger       UDP -> whoever listens

Why this shape:

* Retriever's real-time backends run **one flow per worker process**, so each
  hardware flow opens its own serial port inside its own process. No shared
  ``Cart`` handle, no cross-thread device access. A flow's ``init()`` runs in
  the worker; ``finalize()`` runs there on shutdown, which is where we cut gas
  and idle the motor.
* Flows are rebuilt in the worker from ``init_config()``, so every constructor
  argument (waypoints, gains) must be returned from it. That is not optional —
  a value only set in ``__init__`` never reaches the worker.
* ``FollowerFlow`` touches no hardware. It is pure control math, which is what
  makes it replayable against a recorded drive.

The control law itself is ported verbatim from ``follow.PathFollower.step`` —
heading-free Stanley PD on (cross-track, cross-track rate) for steering, PI on
GPS speed for throttle. ``FollowConfig`` is reused as-is so gains and tuning
stay in one place.

SAFETY — this AUTONOMOUSLY DRIVES THE CART.

* Dry-run is the default and is structural: when ``armed=False`` the actuator
  flows are never built, so no serial port to the ODrive or Arduino is ever
  opened. Dry-run cannot actuate because there is nothing to actuate with.
* The 20 Hz pedal heartbeat stays a plain thread inside ``PedalController``,
  living in the pedal worker process. It is deliberately NOT a flow: graph
  scheduling must never be able to starve the failsafe.
* If the pipeline dies for any reason, the pedal worker dies with it, the
  heartbeat stops, and the Arduino trips FAILSAFE in firmware within 300 ms —
  gas released, brake slammed. That firmware watchdog, not any Python here, is
  the bottom-most safety net.
"""

from __future__ import annotations

import json
import math
import socket
import time
from dataclasses import dataclass
from typing import List, Optional

from retriever import Flow, Rate, Trigger, io

from . import config, geo
from .follow import FollowConfig
from .gps import GpsReceiver
from .pedals import PedalController
from .steering import SteeringController

DEFAULT_TELEMETRY_PORT = 5077


# --------------------------------------------------------------------------
# payloads
# --------------------------------------------------------------------------
# Retriever fans several sources into one flow by giving that flow a single
# composite input type and mapping fields onto it edge by edge — hence the
# wide FollowIn below rather than one input type per upstream.

@io
@dataclass
class GpsFix:
    lat: Optional[float] = None
    lon: Optional[float] = None
    fix_type: Optional[str] = None
    fix_code: Optional[int] = None
    sats: Optional[int] = None
    speed_mph: Optional[float] = None
    ts: Optional[float] = None


@io
@dataclass
class FollowIn:
    """GPS + both actuator feedbacks, fanned into the controller."""
    lat: Optional[float] = None
    lon: Optional[float] = None
    fix_type: Optional[str] = None
    fix_code: Optional[int] = None
    speed_mph: Optional[float] = None
    ts: Optional[float] = None
    steering_actual_deg: Optional[float] = None
    steering_target_deg: Optional[float] = None
    estop: Optional[bool] = None


@io
@dataclass
class DriveCmd:
    """Controller output: the actuator commands plus everything the UI plots."""
    phase: Optional[str] = None
    reason: Optional[str] = None
    steer_deg: Optional[float] = None
    steer_enable: Optional[bool] = None
    gas: Optional[float] = None
    brake: Optional[float] = None
    alpha: Optional[float] = None       # road-wheel angle demand (deg)
    lat: Optional[float] = None
    lon: Optional[float] = None
    fix: Optional[str] = None
    ts: Optional[float] = None
    t: Optional[float] = None
    xtrack_m: Optional[float] = None
    xtrack_signed_m: Optional[float] = None
    heading_deg: Optional[float] = None
    heading_err_deg: Optional[float] = None
    dist_to_goal_m: Optional[float] = None
    live_speed_mph: Optional[float] = None
    max_speed_mph: Optional[float] = None
    steering_actual_deg: Optional[float] = None
    steering_target_deg: Optional[float] = None
    armed: Optional[bool] = None


@io
@dataclass
class SteerCmd:
    steer_deg: Optional[float] = None
    steer_enable: Optional[bool] = None
    phase: Optional[str] = None


@io
@dataclass
class SteerState:
    steering_actual_deg: Optional[float] = None
    steering_target_deg: Optional[float] = None


@io
@dataclass
class PedalCmd:
    gas: Optional[float] = None
    brake: Optional[float] = None
    phase: Optional[str] = None


@io
@dataclass
class PedalState:
    estop: Optional[bool] = None
    failsafe: Optional[bool] = None


# --------------------------------------------------------------------------
# GPS
# --------------------------------------------------------------------------

class GpsSourceFlow(Flow[None, GpsFix]):
    """Owns the u-blox receiver (and optionally the NTRIP corrections feed).

    Also derives ground speed from consecutive fixes. ``follow.py`` never did
    this — it took ``live_speed_mph`` from the browser, which meant the speed
    controller's feedback made a round trip through a websocket. Computing it
    at the source keeps the graph self-contained.
    """

    def __init__(self, *, ntrip_provider: Optional[str] = None):
        super().__init__()
        self.ntrip_provider = ntrip_provider

    def init_config(self) -> dict:
        return {"ntrip_provider": self.ntrip_provider}

    def reset(self) -> None:
        # reset() is the lifecycle hook: called once at startup AND again on any
        # Pipeline.reset(). So the device open is guarded — re-opening the GPS on
        # a state reset would be a bug. Only the speed estimator is re-zeroed.
        if getattr(self, "gps", None) is None:
            self.gps = GpsReceiver()
            self.gps.open()
            self.ntrip = None
            if self.ntrip_provider:
                from .ntrip import NtripClient
                self.ntrip = NtripClient(self.gps, provider=self.ntrip_provider).start()
        self._prev: Optional[tuple] = None   # (lat, lon, ts)
        self._speed_mph = 0.0

    def step(self, _) -> GpsFix:
        fix = self.gps.latest
        if not fix or fix.get("lat") is None:
            return GpsFix()

        pos, ts = (fix["lat"], fix["lon"]), fix["ts"]
        if self._prev is not None:
            plat, plon, pts = self._prev
            dt = ts - pts
            if dt > 1e-3:
                mps = geo.haversine_m((plat, plon), pos) / dt
                # EWMA: raw fix-to-fix speed is jumpy at walking pace, and this
                # feeds both the throttle PI and the heading-error estimate.
                self._speed_mph = 0.7 * self._speed_mph + 0.3 * (mps * 2.23694)
        self._prev = (pos[0], pos[1], ts)

        return GpsFix(
            lat=fix["lat"], lon=fix["lon"],
            fix_type=fix.get("fix_type"), fix_code=fix.get("fix_code"),
            sats=fix.get("sats"), speed_mph=round(self._speed_mph, 2), ts=ts,
        )

    def finalize(self) -> None:
        if self.ntrip:
            self.ntrip.stop()
        self.gps.close()


# --------------------------------------------------------------------------
# the control law  (ported from follow.PathFollower.step)
# --------------------------------------------------------------------------

class FollowerFlow(Flow[FollowIn, DriveCmd]):
    """Stanley + curvature feedforward + fused heading, and PI speed control.

    A straight lift of ``follow.PathFollower.step()`` — same law, same state,
    same gains (the ``st_*`` family; the ``steer_gain``/``xtrack_gain`` knobs are
    legacy and unused). ``tests/test_rflows_parity.py`` asserts step-for-step
    equality with the original, so the two cannot silently diverge.

    Touches no hardware: position and the measured wheel angle arrive as inputs,
    actuator commands leave as outputs. That's what makes it replayable.
    """

    def __init__(self, *, path: List[list], cfg: dict, armed: bool = False):
        super().__init__()
        self.path = [tuple(p) for p in path]
        self.cfg_dict = dict(cfg)
        self.armed = bool(armed)

    def init_config(self) -> dict:
        # Everything the worker needs to rebuild this flow. path/cfg live here
        # or they never cross the process boundary.
        return {"path": [list(p) for p in self.path],
                "cfg": self.cfg_dict,
                "armed": self.armed}

    def reset(self) -> None:
        self.cfg = FollowConfig(**self.cfg_dict)
        self.cfg.gas_cap = config.effective_gas_cap(self.cfg.gas_cap)
        self.phase = "init"
        self.reason = ""
        self.t0 = time.time()
        # speed control state
        self._speed_integral = 0.0
        self._prev_speed_ts: Optional[float] = None
        # smooth steering law state
        self._origin = self.path[0]
        self._st_integral = 0.0            # road-wheel deg trim
        self._st_cmd = 0.0                 # filtered column target
        self._fused_heading: Optional[float] = None
        self._hist: list = []              # [(cum_s, x, y)] recent track
        self._cum_s = 0.0
        self._last_xy: Optional[tuple] = None
        self._prev_step_ts: Optional[float] = None

    # -- helpers (ported verbatim from follow.py) --------------------------
    def _path_curvature(self, along_m: float, w: float = 2.5) -> float:
        """Signed path curvature (1/m); + = path bending RIGHT."""
        a, _ = geo.point_at_distance(self.path, max(0.0, along_m - w))
        b, _ = geo.point_at_distance(self.path, along_m)
        c, _ = geo.point_at_distance(self.path, along_m + w)
        if a == b or b == c:
            return 0.0
        dh = geo.angle_diff_deg(geo.bearing_deg(b, c), geo.bearing_deg(a, b))
        return math.radians(dh) / (2.0 * w)

    def _update_fused_heading(self, pos, v_ms: float, wheel_deg: float,
                              dt: float) -> Optional[float]:
        """Complementary filter: wheel-angle yaw integration (lag-free) corrected
        toward the backward-looking GPS track (absolute, drift-free)."""
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
                        gps_heading = math.degrees(math.atan2(dx, dy)) % 360.0
                    break

        if self._fused_heading is None:
            self._fused_heading = gps_heading   # stays None until we've moved
            return self._fused_heading

        road = math.radians(config.column_to_roadwheel_deg(wheel_deg))
        yaw_rate_deg = math.degrees(v_ms / config.WHEELBASE_M * math.tan(road))
        self._fused_heading = (self._fused_heading + yaw_rate_deg * dt) % 360.0
        if gps_heading is not None:
            k = min(1.0, c.st_gps_correct_gain * dt)
            self._fused_heading = (
                self._fused_heading
                + k * geo.angle_diff_deg(gps_heading, self._fused_heading)) % 360.0
        return self._fused_heading

    def _halt(self, phase: str, reason: str, inp: FollowIn,
              brake: float = 0.15) -> DriveCmd:
        self.phase, self.reason = phase, reason
        return self._out(inp, gas=0.0, brake=brake, steer_deg=0.0,
                         steer_enable=False)

    def _out(self, inp: FollowIn, *, gas: float, brake: float,
             steer_deg: float, steer_enable: bool, **extra) -> DriveCmd:
        return DriveCmd(
            phase=self.phase, reason=self.reason,
            steer_deg=round(steer_deg, 1), steer_enable=steer_enable,
            gas=round(gas, 3), brake=round(brake, 3),
            lat=inp.lat, lon=inp.lon, fix=inp.fix_type, ts=inp.ts,
            t=round(time.time() - self.t0, 3),
            live_speed_mph=round(inp.speed_mph or 0.0, 1),
            max_speed_mph=round(self.cfg.max_speed_mph, 1),
            steering_actual_deg=inp.steering_actual_deg,
            steering_target_deg=inp.steering_target_deg,
            armed=self.armed,
            **extra,
        )

    # -- one control cycle -------------------------------------------------
    def step(self, inp: FollowIn) -> DriveCmd:
        c = self.cfg

        if self.phase in ("done", "abort"):
            # Latch. Once we've stopped we stay stopped — the pipeline runner
            # tears the graph down, we don't quietly resume driving.
            return self._out(inp, gas=0.0,
                             brake=c.arrival_brake if self.phase == "done" else 0.15,
                             steer_deg=0.0, steer_enable=False)

        if inp.estop:
            return self._halt("abort", "E-STOP engaged", inp)
        if inp.lat is None or inp.ts is None:
            return self._out(inp, gas=0.0, brake=0.0, steer_deg=0.0,
                             steer_enable=False)
        if (time.time() - inp.ts) > c.gps_max_age_s:
            return self._halt("abort", "GPS fix lost/stale", inp)
        if c.require_rtk and inp.fix_code not in (4, 5):
            return self._halt("abort", f"not RTK (fix={inp.fix_type})", inp)

        pos = (inp.lat, inp.lon)
        speed_mph = inp.speed_mph or 0.0

        dist_to_goal = geo.haversine_m(pos, self.path[-1])
        near_last = geo.nearest_index(self.path, pos) >= len(self.path) - 1
        if near_last and dist_to_goal <= c.goal_radius_m:
            self.phase, self.reason = "done", "arrived at goal"
            return self._out(inp, gas=0.0, brake=c.arrival_brake, steer_deg=0.0,
                             steer_enable=False, dist_to_goal_m=round(dist_to_goal, 1))

        self.phase = "tracking"
        snap = geo.nearest_point_on_path(self.path, pos)
        xtrack = snap.distance_m
        if xtrack > c.max_crosstrack_m:
            return self._halt(
                "abort", f"off path ({xtrack:.1f} m > {c.max_crosstrack_m} m)", inp)

        cross = snap.signed_distance_m   # + => cart is LEFT of the path
        seg_i = snap.segment_index
        path_bearing = geo.bearing_deg(
            self.path[seg_i], self.path[min(seg_i + 1, len(self.path) - 1)])

        now = time.time()
        dt = (now - self._prev_step_ts) if self._prev_step_ts is not None else 1.0 / c.rate_hz
        dt = max(1e-3, min(dt, 0.5))
        self._prev_step_ts = now
        v_ms = speed_mph * 0.44704

        # Absolute heading from the wheel-angle yaw model fused with the GPS
        # track. The measured column angle arrives as feedback from SteeringFlow.
        wheel_deg = inp.steering_actual_deg or 0.0
        fused = self._update_fused_heading(pos, v_ms, wheel_deg, dt)
        # + heading_err = cart pointing RIGHT of the path direction
        heading_err = geo.angle_diff_deg(fused, path_bearing) if fused is not None else 0.0

        # Stanley, in ROAD-WHEEL degrees (+ = right):
        #   ff    — curvature feedforward: the steady angle that holds the curve
        #   head  — align to the path direction
        #   cross — bounded pull back to the line (atan keeps it gentle)
        curvature = self._path_curvature(snap.along_m)
        ff_deg = math.degrees(math.atan(config.WHEELBASE_M * curvature))
        head_deg = -c.st_k_heading * heading_err
        v_soft = v_ms + c.st_cross_soft_mps
        cross_deg = math.degrees(math.atan(c.st_k_cross * cross / max(v_soft, 0.2)))
        cross_deg = max(-c.st_max_cross_road_deg, min(cross_deg, c.st_max_cross_road_deg))

        # Integral trim: null a steady bias only when close and moving; otherwise
        # bleed it off so it can't wind up.
        if abs(cross) < c.st_int_enable_cross_m and v_ms > 0.4:
            self._st_integral += c.st_k_int * cross * dt
            self._st_integral = max(-c.st_int_max_deg,
                                    min(self._st_integral, c.st_int_max_deg))
        else:
            self._st_integral *= 0.98

        road_deg = ff_deg + head_deg + cross_deg + self._st_integral
        column_raw = c.steer_sign * config.STEER_RATIO * road_deg
        column_raw = max(-c.max_steer_deg, min(column_raw, c.max_steer_deg))

        # Command shaping: low-pass then slew clamp, so the column moves as one
        # continuous motion instead of per-cycle jumps.
        alpha = dt / (c.st_tau_cmd_s + dt)
        target = self._st_cmd + alpha * (column_raw - self._st_cmd)
        max_step = c.st_slew_deg_s * dt
        target = self._st_cmd + max(-max_step, min(target - self._st_cmd, max_step))
        self._st_cmd = target
        steer_deg = target

        heading_abs = fused if fused is not None else (path_bearing + wheel_deg) % 360.0

        # --- throttle: PI on GPS speed, brake bleeds overspeed downhill ---
        now = time.time()
        dt_spd = (now - self._prev_speed_ts) if self._prev_speed_ts is not None else 0.0
        dt_spd = max(0.0, min(dt_spd, 1.0))
        self._prev_speed_ts = now

        target_mph = c.max_speed_mph
        if dist_to_goal < c.arrival_slowdown_m:
            frac = dist_to_goal / max(c.arrival_slowdown_m, 0.1)
            target_mph = max(c.arrival_creep_mph, c.max_speed_mph * frac)

        speed_error = target_mph - speed_mph
        ff_gas = config.gas_for_mph(target_mph)
        i_cand = max(-c.speed_i_max,
                     min(self._speed_integral + speed_error * dt_spd, c.speed_i_max))

        gas_cmd = ff_gas + c.speed_kp * speed_error + c.speed_ki * i_cand
        gas_cmd /= (1.0 + c.turn_slowdown * abs(steer_deg) / max(c.max_steer_deg, 1.0))
        applied_gas = max(0.0, min(gas_cmd, c.gas_cap))

        # Conditional anti-windup: don't integrate further into a saturated pedal.
        sat_high = gas_cmd >= c.gas_cap and speed_error > 0
        sat_low = gas_cmd <= 0.0 and speed_error < 0
        if not (sat_high or sat_low):
            self._speed_integral = i_cand

        applied_brake = 0.0
        over = -speed_error - c.brake_deadband_mph
        if over > 0.0:
            applied_gas = 0.0
            applied_brake = min(c.brake_kp * over, config.BRAKE_POT_MAX)

        return self._out(
            inp, gas=applied_gas, brake=applied_brake, steer_deg=steer_deg,
            steer_enable=True,
            alpha=round(road_deg, 1),          # road-wheel demand, as follow.py reports it
            xtrack_m=round(xtrack, 2),
            xtrack_signed_m=round(cross, 2),
            heading_deg=round(heading_abs, 1),
            heading_err_deg=round(heading_err, 1),
            dist_to_goal_m=round(dist_to_goal, 1),
        )


# --------------------------------------------------------------------------
# actuators  (only ever built when armed=True)
# --------------------------------------------------------------------------

class SteeringFlow(Flow[SteerCmd, SteerState]):
    """Owns the ODrive. Reports the measured column angle back to the follower."""

    def reset(self) -> None:
        # Guarded: reset() runs again on Pipeline.reset(), and re-opening the
        # ODrive mid-drive would drop the motor out of closed loop.
        if getattr(self, "steering", None) is None:
            self.steering = SteeringController()
            self.steering.connect()
            self.enabled = self.steering.enable()
            if not self.enabled:
                print("[steering] FAILED to enter closed loop — not actuating",
                      flush=True)
        self._last_read = 0.0
        self._actual: Optional[float] = None

    def step(self, cmd: SteerCmd) -> SteerState:
        if self.enabled and cmd.steer_enable and cmd.steer_deg is not None:
            self.steering.set_angle(cmd.steer_deg)

        # The encoder read is a serial round trip; at 15 Hz it would dominate
        # the cycle, so poll it at ~4 Hz like follow.py did.
        now = time.time()
        if now - self._last_read >= 0.25:
            self._last_read = now
            try:
                self._actual = self.steering.angle_deg()
            except Exception:
                pass

        return SteerState(
            steering_actual_deg=round(self._actual, 1) if self._actual is not None else None,
            steering_target_deg=round(self.steering.target_deg, 1),
        )

    def finalize(self) -> None:
        try:
            self.steering.idle()
        finally:
            self.steering.close()


class PedalFlow(Flow[PedalCmd, PedalState]):
    """Owns the Arduino. The 20 Hz heartbeat thread lives inside PedalController
    (started by arm()) and stays a plain thread on purpose — see module docstring.
    """

    def __init__(self, *, gas_cap: float, arrival_brake: float):
        super().__init__()
        self.gas_cap = float(gas_cap)
        self.arrival_brake = float(arrival_brake)

    def init_config(self) -> dict:
        return {"gas_cap": self.gas_cap, "arrival_brake": self.arrival_brake}

    def reset(self) -> None:
        # Guarded: re-opening the Arduino would restart the heartbeat thread and
        # briefly drop the cart into FAILSAFE mid-drive.
        if getattr(self, "pedals", None) is None:
            self.pedals = PedalController(gas_cap=self.gas_cap)
            self.pedals.open()
            self.pedals.arm()
        self.phase = "init"

    def step(self, cmd: PedalCmd) -> PedalState:
        if cmd.phase:
            self.phase = cmd.phase
        # Brake before gas: if both are commanded this ordering can only ever
        # make us slower, never faster.
        self.pedals.set_brake(cmd.brake or 0.0)
        self.pedals.set_gas(cmd.gas or 0.0)
        tele = self.pedals.telemetry
        return PedalState(estop=tele.get("estop"), failsafe=tele.get("failsafe"))

    def finalize(self) -> None:
        try:
            self.pedals.set_gas(0.0)
            if self.phase == "done":
                self.pedals.set_brake(self.arrival_brake)   # park at the goal
            else:
                self.pedals.stop()
        finally:
            self.pedals.close()


# --------------------------------------------------------------------------
# telemetry
# --------------------------------------------------------------------------

class TelemetryFlow(Flow[DriveCmd, None]):
    """Fire-and-forget UDP to the parent process (and anyone else listening).

    UDP because the flow runs in a worker process: a Python callback can't reach
    the parent's asyncio loop, and a datagram to localhost doesn't care about
    fork-vs-spawn or block the control loop if nobody is listening.

    Takes DriveCmd — the follower's own output type — rather than a lookalike
    mirror type. An unmapped edge passes the value atomically, so a distinct
    (even field-identical) input type arrives as something that is not that
    dataclass, and every send fails. Same-type sink is the idiom the runtime
    expects; see the sim -> viz edge in golden-retriever's autopilot example.
    """

    def __init__(self, *, port: int = DEFAULT_TELEMETRY_PORT):
        super().__init__()
        self.port = int(port)

    def init_config(self) -> dict:
        return {"port": self.port}

    # A flow's step() receives an IOView, NOT the dataclass — attribute access
    # works, but dataclasses.asdict() raises. So the payload is built field by
    # field. Same names follow.py's telemetry already uses, so the UI and the
    # drive-log tooling need no changes.
    FIELDS = (
        "phase", "reason", "gas", "brake", "alpha", "lat", "lon", "fix", "ts", "t",
        "xtrack_m", "xtrack_signed_m", "heading_deg", "heading_err_deg",
        "dist_to_goal_m", "live_speed_mph", "max_speed_mph",
        "steering_actual_deg", "steering_target_deg", "armed",
    )

    def reset(self) -> None:
        if getattr(self, "sock", None) is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def step(self, t: DriveCmd) -> None:
        if getattr(t, "phase", None) is None:
            return None
        payload = {}
        for name in self.FIELDS:
            v = getattr(t, name, None)
            if v is not None:
                payload[name] = v
        steer = getattr(t, "steer_deg", None)
        if steer is not None:
            payload["steer_cmd"] = steer     # follow.py's name for it
        try:
            self.sock.sendto(json.dumps(payload).encode(), ("127.0.0.1", self.port))
        except OSError:
            pass   # telemetry must never take the cart down
        return None

    def finalize(self) -> None:
        self.sock.close()
