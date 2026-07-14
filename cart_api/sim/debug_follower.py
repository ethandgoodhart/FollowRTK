#!/usr/bin/env python3
"""
debug_follower.py — put FollowerFlow under a microscope, in the main process.

The cart runs the graph on the multiprocessing backend, which is right for real
time and awful for debugging: the flow lives in another process, so breakpoints
don't land, prints interleave, and nothing is reproducible. This runs the SAME
FollowerFlow in THIS process, on a deterministic clock, and prints the control
law's internals every step — the terms that never reach telemetry.

    ff_deg      curvature feedforward   (the steady angle that holds the curve)
    head_deg    heading alignment       (-k_heading * heading_err)
    cross_deg   Stanley pull to line    (bounded by atan)
    int_deg     integral trim
    -> road_deg -> column_raw (x STEER_RATIO, clamped) -> column_cmd (LPF + slew)

Two ways to run it (--mode):

  direct     call FollowerFlow.step(FollowIn(...)) in a plain loop. No Retriever
             runtime at all. The simplest thing that can possibly work — use this
             when you want to breakpoint the law itself.
  pipeline   build a real 2-flow graph (Plant <-> Follower) and drive it with
             pipe.step(), the in-process stepper. Same objects, same process, so
             breakpoints still land — but it exercises the actual Retriever
             wiring, IOViews and all.

Three data sources (--source):

  sim        CLOSED LOOP against the calibrated simlib plant (kinematic bicycle +
             rate/accel-limited steering). The cart responds to what the law
             commands. This is the one that answers "would it actually drive the
             route?"
  replay     OPEN LOOP over a recorded RTK drive: feed the real GPS positions and
             see what FollowerFlow would have commanded at each one, next to what
             the old law actually did command. Ground truth, no plant model.
  synthetic  straight path, cart offset to one side. No drive file needed.

Everything runs on a FAKE CLOCK (time.time is patched, advanced 1/rate per step),
so dt, the integral, and the fused-heading filter are exactly reproducible and it
finishes instantly instead of in real time.

Examples:
    python3 debug_follower.py                                  # sim, route1, direct
    python3 debug_follower.py --mode pipeline                  # through pipe.step()
    python3 debug_follower.py --source replay --drive route2_notgreat3.5
    python3 debug_follower.py --source sim --drive route2_failed5.5 --k-cross 0.6
    python3 debug_follower.py --break-at 40                    # drop into pdb at step 40
    python3 debug_follower.py --every 1                        # print every step
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import asdict
from typing import Optional
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                              # simlib, simharness
sys.path.insert(0, os.path.dirname(HERE))             # cartlib

import simlib
import simharness
from simlib import Path, SteeringActuator, Vehicle, from_local_xy, local_xy

from cartlib.follow import FollowConfig
from cartlib.rflows import DriveCmd, FollowIn, FollowerFlow

MPH = 0.44704
DRIVES = os.path.join(HERE, "drives")


class FakeClock:
    """FollowerFlow reads time.time() for dt / integral / heading filter. Own it,
    and the whole run becomes deterministic and instant."""

    def __init__(self, t0: float = 1_700_000_000.0):
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float) -> None:
        self.t += dt


# ---------------------------------------------------------------------------
# data sources
# ---------------------------------------------------------------------------

def load_drive(name: str) -> dict:
    f = os.path.join(DRIVES, name, "last_drive.json")
    if not os.path.exists(f):
        avail = sorted(os.listdir(DRIVES)) if os.path.isdir(DRIVES) else []
        sys.exit(f"no such drive {name!r}. available: {', '.join(avail)}")
    return simharness.load_route(f)


def synthetic_route(length_m: float = 30.0, offset_m: float = 1.5) -> dict:
    """Straight path north; cart starts offset_m to the east of it."""
    origin = (37.4275, -122.1697)

    def at(east, north):
        dlat = north / simlib.EARTH_R
        dlon = east / (simlib.EARTH_R * math.cos(math.radians(origin[0])))
        return (origin[0] + math.degrees(dlat), origin[1] + math.degrees(dlon))

    pts = [at(0.0, i) for i in range(int(length_m) + 1)]
    return {"name": "synthetic", "pts": pts, "origin": origin,
            "start": at(offset_m, 0.0), "h0": 0.0, "max_speed_mph": 3.0,
            "recorded_steps": []}


# ---------------------------------------------------------------------------
# checkpoint printing
# ---------------------------------------------------------------------------

HDR = (f"{'step':>4} {'t':>5} {'mph':>4} │ {'xtrack':>7} {'hd_err':>7} │ "
       f"{'ff':>6} {'head':>7} {'cross':>7} {'int':>5} │ {'road':>6} "
       f"{'col_raw':>8} {'col_cmd':>8} │ {'wheel':>7} {'gas':>5}")


def checkpoint(i: int, t: float, out: DriveCmd, dbg: dict, wheel: float) -> str:
    return (f"{i:>4} {t:>5.2f} {out.live_speed_mph or 0:>4.1f} │ "
            f"{out.xtrack_signed_m if out.xtrack_signed_m is not None else 0:>+7.2f} "
            f"{dbg.get('heading_err', 0):>+7.1f} │ "
            f"{dbg.get('ff_deg', 0):>+6.1f} {dbg.get('head_deg', 0):>+7.1f} "
            f"{dbg.get('cross_deg', 0):>+7.1f} {dbg.get('integral_deg', 0):>+5.1f} │ "
            f"{dbg.get('road_deg', 0):>+6.1f} {dbg.get('column_raw', 0):>+8.1f} "
            f"{dbg.get('column_cmd', 0):>+8.1f} │ {wheel:>+7.1f} {out.gas or 0:>5.3f}")


# ---------------------------------------------------------------------------
# runners
# ---------------------------------------------------------------------------

def run(args) -> int:
    route = (synthetic_route() if args.source == "synthetic"
             else load_drive(args.drive))
    cfg = FollowConfig(
        max_speed_mph=args.speed or route["max_speed_mph"],
        rate_hz=args.rate,
        require_rtk=False,
        st_k_cross=args.k_cross,
        st_k_heading=args.k_heading,
    )

    follower = FollowerFlow(path=[list(p) for p in route["pts"]],
                            cfg=asdict(cfg), armed=False)

    clock = FakeClock()
    dt = 1.0 / cfg.rate_hz

    print(f"source={args.source}  mode={args.mode}  route={route['name'].split('/')[-2] if '/' in route['name'] else route['name']}")
    print(f"path={len(route['pts'])} pts  speed={cfg.max_speed_mph} mph  rate={cfg.rate_hz} Hz  "
          f"k_cross={cfg.st_k_cross}  k_heading={cfg.st_k_heading}")
    print(f"clock=fake (deterministic)   dt={dt:.4f}s\n")
    print(HDR)
    print("─" * len(HDR))

    with mock.patch("time.time", clock):
        follower.reset()
        if args.mode == "pipeline":
            trace = _run_pipeline(args, route, cfg, follower, clock, dt)
        elif args.source == "replay":
            trace = _run_replay(args, route, cfg, follower, clock, dt)
        else:
            trace = _run_sim(args, route, cfg, follower, clock, dt)

    _report(args, route, cfg, trace)
    return 0


def _maybe_break(args, i: int) -> None:
    if args.break_at is not None and i == args.break_at:
        import pdb
        print(f"\n--- pdb at step {i}. `follower._dbg`, `follower._st_integral`, "
              f"`follower._fused_heading` are live. `c` to continue ---")
        pdb.set_trace()


def _run_sim(args, route, cfg, follower, clock, dt) -> list:
    """CLOSED LOOP: the simlib plant responds to what the law commands."""
    origin = route["origin"]
    path = Path(route["pts"], origin)
    sx, sy = local_xy(origin, route["start"])
    veh = Vehicle(sx, sy, route["h0"])
    act = SteeringActuator(0.0)
    spd = simharness.SpeedModel(max_mph=cfg.max_speed_mph)
    goal = path.pts[-1]

    trace, t = [], 0.0
    for i in range(args.max_steps):
        _maybe_break(args, i)
        dist_goal = math.hypot(veh.x - goal[0], veh.y - goal[1])
        v_mph = spd.update(act.angle, dist_goal, dt)
        lat, lon = from_local_xy(origin, veh.x, veh.y)

        out = follower.step(FollowIn(
            lat=lat, lon=lon, fix_type="RTK Fixed", fix_code=4,
            speed_mph=v_mph, ts=clock.t,
            steering_actual_deg=act.angle, steering_target_deg=act.angle,
            estop=False))
        dbg = follower._dbg

        snap = path.snap(veh.x, veh.y)
        if i % args.every == 0 and out.phase == "tracking":
            print(checkpoint(i, t, out, dbg, act.angle))
        trace.append({"t": round(t, 3), "wheel_act": act.angle,
                      "wheel_cmd": out.steer_deg or 0.0,
                      "xtrack": snap.dist_m, "xtrack_signed": snap.signed_m,
                      "v_mph": v_mph, "phase": out.phase})

        if out.phase in ("done", "abort"):
            print(f"\n>>> {out.phase}: {out.reason}  (step {i}, t={t:.1f}s)")
            break

        # integrate the plant at a finer dt across one control period
        n = max(1, int(round(dt / 0.02)))
        h = dt / n
        for _ in range(n):
            act.update(out.steer_deg or 0.0, h)
            veh.step(v_mph * MPH, act.angle, h)
        t += dt
        clock.tick(dt)
    return trace


def _run_replay(args, route, cfg, follower, clock, dt) -> list:
    """OPEN LOOP: feed the RECORDED GPS positions; compare against the old law."""
    steps = route["recorded_steps"]
    if not steps:
        sys.exit("this route has no recorded steps to replay")
    origin = route["origin"]
    path = Path(route["pts"], origin)

    print(f"{'':>62}   old_cmd  new_cmd   delta")
    trace, t = [], 0.0
    for i, rec in enumerate(steps[:args.max_steps]):
        _maybe_break(args, i)
        if rec.get("lat") is None:
            continue
        out = follower.step(FollowIn(
            lat=rec["lat"], lon=rec["lon"],
            fix_type=rec.get("fix"), fix_code=4,
            speed_mph=rec.get("live_speed_mph", 0.0), ts=clock.t,
            # the wheel angle the cart ACTUALLY had at that moment
            steering_actual_deg=rec.get("steering_actual_deg"),
            steering_target_deg=rec.get("steering_target_deg"),
            estop=False))
        dbg = follower._dbg
        wheel = rec.get("steering_actual_deg") or 0.0

        if i % args.every == 0 and out.phase == "tracking":
            old_cmd = rec.get("steer_cmd", 0.0)
            new_cmd = out.steer_deg or 0.0
            line = checkpoint(i, t, out, dbg, wheel)
            print(f"{line}   {old_cmd:>+7.1f} {new_cmd:>+8.1f} {new_cmd - old_cmd:>+7.1f}")

        x, y = local_xy(origin, (rec["lat"], rec["lon"]))
        snap = path.snap(x, y)
        trace.append({"t": round(t, 3), "wheel_act": wheel,
                      "wheel_cmd": out.steer_deg or 0.0,
                      "xtrack": snap.dist_m, "xtrack_signed": snap.signed_m,
                      "v_mph": rec.get("live_speed_mph", 0.0),
                      "old_cmd": rec.get("steer_cmd", 0.0), "phase": out.phase})
        if out.phase in ("done", "abort"):
            print(f"\n>>> {out.phase}: {out.reason}  (step {i})")
            break
        t += dt
        clock.tick(dt)
    return trace


def _run_pipeline(args, route, cfg, follower, clock, dt) -> list:
    """Same law, but driven through a REAL Retriever graph with the in-process
    stepper. Plant <-> Follower is a genuine cyclic graph; pipe.step() runs both
    in THIS process, so breakpoints still land and the objects stay inspectable.

    Note the stepper fires every flow once per logical step and ignores @Rate —
    which is exactly what we want here (lockstep, one plant tick per control
    tick), and exactly why it must never be used to drive real hardware.
    """
    from retriever import Flow, Pipeline, Rate, io
    from dataclasses import dataclass as dc

    origin = route["origin"]
    path = Path(route["pts"], origin)
    sx, sy = local_xy(origin, route["start"])

    # NOTE: these must be spelled Optional[X], not `X | None`. Retriever's IR
    # compares edge types by their annotation TEXT, so `float | None` does not
    # match FollowIn's `Optional[float]` and the graph refuses to build with
    # IR_VAL_TYPE_MISMATCH — even though they are the same type to Python.
    @io
    @dc
    class PlantOut:
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
    @dc
    class PlantIn:
        steer_deg: Optional[float] = None

    class PlantFlow(Flow[PlantIn, PlantOut]):
        """The cart: kinematic bicycle + rate/accel-limited steering column."""

        def reset(self) -> None:
            self.veh = Vehicle(sx, sy, route["h0"])
            self.act = SteeringActuator(0.0)
            self.spd = simharness.SpeedModel(max_mph=cfg.max_speed_mph)
            self.goal = path.pts[-1]
            self.v_mph = 0.0

        def step(self, cmd: PlantIn) -> PlantOut:
            # First tick the follower hasn't produced a command yet -> coast straight.
            target = getattr(cmd, "steer_deg", None) or 0.0
            dist_goal = math.hypot(self.veh.x - self.goal[0], self.veh.y - self.goal[1])
            self.v_mph = self.spd.update(self.act.angle, dist_goal, dt)
            n = max(1, int(round(dt / 0.02)))
            h = dt / n
            for _ in range(n):
                self.act.update(target, h)
                self.veh.step(self.v_mph * MPH, self.act.angle, h)
            lat, lon = from_local_xy(origin, self.veh.x, self.veh.y)
            return PlantOut(lat=lat, lon=lon, fix_type="RTK Fixed", fix_code=4,
                            speed_mph=self.v_mph, ts=clock.t,
                            steering_actual_deg=self.act.angle,
                            steering_target_deg=self.act.angle, estop=False)

    pipe = Pipeline("debug_follower")
    with pipe:
        plant = PlantFlow() @ Rate(hz=cfg.rate_hz)
        ctrl = follower @ Rate(hz=cfg.rate_hz)
        plant.then(ctrl, map={
            "lat": "lat", "lon": "lon", "fix_type": "fix_type",
            "fix_code": "fix_code", "speed_mph": "speed_mph", "ts": "ts",
            "steering_actual_deg": "steering_actual_deg",
            "steering_target_deg": "steering_target_deg", "estop": "estop"})
        ctrl.then(plant, map={"steer_deg": "steer_deg"})     # close the loop

    plant_flow = plant.flow
    trace, t = [], 0.0
    for i in range(args.max_steps):
        _maybe_break(args, i)
        # StepResult.outputs is keyed by flow name -> the real output payload, so
        # this is the graph's actual DriveCmd, not a reconstruction.
        res = pipe.step(dt=dt)               # <-- in-process; breakpoints land
        out = res.outputs.get("FollowerFlow")
        if out is None:                      # follower hasn't fired yet
            t += dt
            clock.tick(dt)
            continue
        dbg = follower._dbg
        snap = path.snap(plant_flow.veh.x, plant_flow.veh.y)

        if i % args.every == 0 and out.phase == "tracking":
            print(checkpoint(i, t, out, dbg, plant_flow.act.angle))
        trace.append({"t": round(t, 3), "wheel_act": plant_flow.act.angle,
                      "wheel_cmd": out.steer_deg or 0.0,
                      "xtrack": snap.dist_m, "xtrack_signed": snap.signed_m,
                      "v_mph": plant_flow.v_mph, "phase": out.phase})
        if out.phase in ("done", "abort"):
            print(f"\n>>> {out.phase}: {out.reason}  (step {i}, t={t:.1f}s)")
            break
        t += dt
        clock.tick(dt)
    pipe.close_stepper()
    return trace


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------

def _report(args, route, cfg, trace) -> None:
    if not trace:
        print("\n(no steps)")
        return
    xs = [abs(p["xtrack"]) for p in trace]
    rms = math.sqrt(sum(x * x for x in xs) / len(xs))
    rates, reversals, prev = [], 0, None
    for i in range(1, len(trace)):
        d = trace[i]["t"] - trace[i - 1]["t"]
        if d <= 0:
            continue
        r = (trace[i]["wheel_act"] - trace[i - 1]["wheel_act"]) / d
        rates.append(r)
        if prev is not None and ((r > 5 and prev < -5) or (r < -5 and prev > 5)):
            reversals += 1
        prev = r
    rate_rms = math.sqrt(sum(r * r for r in rates) / len(rates)) if rates else 0.0
    arrived = trace[-1]["phase"] == "done"

    print(f"\n{'─' * 60}")
    print(f"steps            {len(trace)}")
    print(f"xtrack rms       {rms:.3f} m")
    print(f"xtrack max       {max(xs):.3f} m")
    print(f"steer rate rms   {rate_rms:.1f} deg/s     (lower = smoother)")
    print(f"steer reversals  {reversals}")
    print(f"arrived          {arrived}")
    if args.source == "replay" and any("old_cmd" in p for p in trace):
        deltas = [abs(p["wheel_cmd"] - p["old_cmd"]) for p in trace if "old_cmd" in p]
        print(f"|new - old| cmd  mean {sum(deltas)/len(deltas):.1f} deg, "
              f"max {max(deltas):.1f} deg   (vs the recorded law)")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"config": asdict(cfg), "trace": trace}, f)
        print(f"\nwrote {args.json}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["direct", "pipeline"], default="direct")
    ap.add_argument("--source", choices=["sim", "replay", "synthetic"], default="sim")
    ap.add_argument("--drive", default="route1_bad3.5",
                    help="drive under sim/drives/ (route1_bad3.5, route2_notgreat3.5, "
                         "route2_failed5.5)")
    ap.add_argument("--speed", type=float, default=None,
                    help="cruise mph (default: the recorded drive's)")
    ap.add_argument("--rate", type=float, default=FollowConfig.rate_hz)
    ap.add_argument("--k-cross", type=float, default=FollowConfig.st_k_cross)
    ap.add_argument("--k-heading", type=float, default=FollowConfig.st_k_heading)
    ap.add_argument("--every", type=int, default=5, help="print every Nth step")
    ap.add_argument("--max-steps", type=int, default=600)
    ap.add_argument("--break-at", type=int, default=None,
                    help="drop into pdb at this step")
    ap.add_argument("--json", default=None, help="write the trace to this file")
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
