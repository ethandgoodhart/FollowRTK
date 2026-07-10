"""
simharness — run a controller over a recorded route through the simlib plant,
producing a trajectory trace + smoothness/tracking metrics.

Speed is handled by a SHARED model (ramp toward target, slow for curvature and
for the final approach) so that any difference between the OLD and NEW traces is
attributable to the STEERING law, not the throttle.
"""
from __future__ import annotations
import json, math
from dataclasses import dataclass, field
from typing import List, Optional

import simlib
from simlib import Vehicle, SteeringActuator, Path, from_local_xy

MPH = 0.44704


def load_route(path_json: str):
    d = json.load(open(path_json))
    raw = d["path"]
    pts = []
    for p in raw:
        if isinstance(p, dict):
            pts.append((p["lat"], p.get("lon", p.get("lng"))))
        else:
            pts.append((p[0], p[1]))
    steps = d["steps"]
    start = (steps[0]["lat"], steps[0]["lon"])
    # initial heading from the first few GPS displacements (fallback: path dir)
    origin = start
    xy = [simlib.local_xy(origin, (s["lat"], s["lon"])) for s in steps[:6] if s.get("lat")]
    if len(xy) >= 3:
        dx = xy[2][0] - xy[0][0]
        dy = xy[2][1] - xy[0][1]
        h0 = math.atan2(dx, dy)
    else:
        p = Path(pts, origin)
        h0 = p.seg_head[0]
    return {
        "name": path_json,
        "pts": pts,
        "origin": origin,
        "start": start,
        "h0": h0,
        "max_speed_mph": d["config"].get("max_speed_mph", 3.5),
        "recorded_steps": steps,
    }


@dataclass
class SpeedModel:
    """Shared throttle model: ramp to target, slow in turns and on approach."""
    max_mph: float = 3.5
    accel_mph_s: float = 2.2
    decel_mph_s: float = 3.5
    curve_slow: float = 0.55      # fraction of speed shed at full lock curvature
    arrival_slow_m: float = 6.0
    creep_mph: float = 1.0
    v: float = 0.0                # current mph (state)

    def target(self, wheel_deg: float, dist_to_goal: float) -> float:
        # slow for how hard the wheel is turned (proxy for lateral accel)
        lock = min(abs(wheel_deg) / 320.0, 1.0)
        t = self.max_mph * (1.0 - self.curve_slow * lock)
        if dist_to_goal < self.arrival_slow_m:
            t = max(self.creep_mph, self.max_mph * dist_to_goal / self.arrival_slow_m)
        return t

    def update(self, wheel_deg: float, dist_to_goal: float, dt: float) -> float:
        tgt = self.target(wheel_deg, dist_to_goal)
        if tgt > self.v:
            self.v = min(tgt, self.v + self.accel_mph_s * dt)
        else:
            self.v = max(tgt, self.v - self.decel_mph_s * dt)
        return self.v


def _noise_gen(sigma_m: float, seed: int):
    """Deterministic RTK-Float-like noise: slow correlated walk + white jitter."""
    state = {"wx": 0.0, "wy": 0.0, "s": seed or 1}
    def nxt():
        # simple LCG for reproducibility without global RNG state
        state["s"] = (1103515245 * state["s"] + 12345) & 0x7fffffff
        r1 = (state["s"] / 0x7fffffff) - 0.5
        state["s"] = (1103515245 * state["s"] + 12345) & 0x7fffffff
        r2 = (state["s"] / 0x7fffffff) - 0.5
        # correlated component (random walk, mean-reverting)
        state["wx"] = 0.92 * state["wx"] + 0.4 * r1
        state["wy"] = 0.92 * state["wy"] + 0.4 * r2
        nx = sigma_m * (0.7 * state["wx"] + 0.6 * r1)
        ny = sigma_m * (0.7 * state["wy"] + 0.6 * r2)
        return nx, ny
    return nxt


def simulate(route: dict, controller, rate_hz: float,
             max_time: float = 90.0, goal_radius_m: float = 1.5,
             abort_cross_m: float = 6.0, act_vel=simlib.ACT_VEL_LIM,
             act_acc=simlib.ACT_ACC_LIM, gps_noise_m: float = 0.0,
             noise_seed: int = 12345) -> dict:
    origin = route["origin"]
    path = Path(route["pts"], origin)
    sx, sy = simlib.local_xy(origin, route["start"])
    veh = Vehicle(sx, sy, route["h0"])
    act = SteeringActuator(0.0, vel_lim=act_vel, acc_lim=act_acc)
    spd = SpeedModel(max_mph=route["max_speed_mph"])
    controller.reset()

    dt = 1.0 / rate_hz
    phys_dt = 0.02
    t = 0.0
    trace = []
    aborted = False
    abort_t = None
    arrived = False
    goal_xy = path.pts[-1]
    noise = _noise_gen(gps_noise_m, noise_seed) if gps_noise_m > 0 else None

    while t < max_time:
        # position the CONTROLLER perceives (true state + optional RTK noise)
        if noise:
            nx, ny = noise()
            px, py = veh.x + nx, veh.y + ny
        else:
            px, py = veh.x, veh.y
        snap = path.snap(px, py)            # what the controller sees
        true_snap = path.snap(veh.x, veh.y) if noise else snap  # for honest metrics
        dist_goal = math.hypot(veh.x - goal_xy[0], veh.y - goal_xy[1])
        near_end = true_snap.along_m >= path.length - 0.5
        if near_end and dist_goal <= goal_radius_m:
            arrived = True
        if true_snap.dist_m > abort_cross_m and not aborted:
            aborted = True
            abort_t = t

        wheel = act.angle
        v_mph = spd.update(wheel, dist_goal, dt)
        v_ms = v_mph * MPH
        col_cmd = controller.compute(snap, v_ms, wheel, dt, pos=(px, py))

        lat, lon = from_local_xy(origin, veh.x, veh.y)
        trace.append({
            "t": round(t, 3),
            "x": round(veh.x, 3), "y": round(veh.y, 3),
            "lat": lat, "lon": lon,
            "heading_deg": round(math.degrees(veh.heading) % 360, 1),
            "wheel_cmd": round(col_cmd, 1),
            "wheel_act": round(wheel, 1),
            "v_mph": round(v_mph, 2),
            "xtrack_signed": round(true_snap.signed_m, 3),  # honest (true) error
            "xtrack": round(true_snap.dist_m, 3),
            "snap_x": round(true_snap.x, 3), "snap_y": round(true_snap.y, 3),
        })
        if arrived:
            break
        # integrate plant at fine dt over one control period
        steps_n = max(1, int(round(dt / phys_dt)))
        h = dt / steps_n
        for _ in range(steps_n):
            act.update(col_cmd, h)
            veh.step(v_ms, act.angle, h)
        t += dt

    return {
        "trace": trace,
        "metrics": metrics(trace, path, arrived, aborted, abort_t, t),
        "aborted": aborted, "abort_t": abort_t, "arrived": arrived,
        "time_s": round(t, 2),
    }


def metrics(trace, path: Path, arrived, aborted, abort_t, total_t) -> dict:
    xs = [p["xtrack"] for p in trace]
    rms = math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0.0
    maxx = max(xs) if xs else 0.0
    # steering smoothness: rate and jerk of the ACTUAL wheel
    rates, reversals, jerks = [], 0, []
    prev_rate = None
    for i in range(1, len(trace)):
        dt = trace[i]["t"] - trace[i - 1]["t"]
        if dt <= 0:
            continue
        r = (trace[i]["wheel_act"] - trace[i - 1]["wheel_act"]) / dt
        rates.append(r)
        if prev_rate is not None:
            if (r > 5 and prev_rate < -5) or (r < -5 and prev_rate > 5):
                reversals += 1
            jerks.append((r - prev_rate) / dt)
        prev_rate = r
    rate_rms = math.sqrt(sum(r * r for r in rates) / len(rates)) if rates else 0.0
    jerk_rms = math.sqrt(sum(j * j for j in jerks) / len(jerks)) if jerks else 0.0
    return {
        "xtrack_rms_m": round(rms, 3),
        "xtrack_max_m": round(maxx, 3),
        "steer_rate_rms_dps": round(rate_rms, 1),
        "steer_jerk_rms": round(jerk_rms, 0),
        "steer_reversals": reversals,
        "arrived": arrived,
        "aborted": aborted,
        "time_to_goal_s": round(total_t, 1) if arrived else None,
    }
