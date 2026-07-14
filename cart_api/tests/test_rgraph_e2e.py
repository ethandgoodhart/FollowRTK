#!/usr/bin/env python3
"""
test_rgraph_e2e.py — run the REAL graph in the REAL runtime, with a simulated GPS.

The parity test proves the control law is right; the graph test proves the wiring
is right. Neither proves the two work *together inside Retriever* — flows run in
worker processes, inputs arrive as IOViews rather than dataclasses, and edges are
per-field queues. Bugs live in that seam. (One did: TelemetryFlow used
dataclasses.asdict() on its input, which raises on an IOView, so every telemetry
send failed silently and the cart would never have detected arrival.)

So: swap GpsSourceFlow for a SimGps that marches along the path, keep FollowerFlow
and TelemetryFlow exactly as the cart runs them, and assert the drive actually
completes — tracking, throttle, bounded steering, and a clean arrival at the goal.

armed=False, so no actuator flow is built and no serial port is opened.

Run: python3 tests/test_rgraph_e2e.py   (~10 s)
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from retriever import Flow, Pipeline, Rate, Trigger

from cartlib import geo
from cartlib.follow import FollowConfig
from cartlib.rflows import DriveCmd, FollowerFlow, GpsFix, TelemetryFlow
from cartlib.rpipeline import TelemetryListener

ORIGIN = (37.4275, -122.1697)
PORT = 5089
SIM_MPH = 6.0
SIM_MS = SIM_MPH * 0.44704
PATH_LEN_M = 14.0

PASS, FAIL = "\033[32mPASS\033[0m", "\033[31mFAIL\033[0m"
results = []


def check(name, cond, detail=""):
    results.append(bool(cond))
    print(f"  [{PASS if cond else FAIL}] {name}" + (f"  ({detail})" if detail else ""))


def offset(origin, east_m, north_m):
    lat0 = origin[0]
    dlat = north_m / geo.EARTH_R
    dlon = east_m / (geo.EARTH_R * math.cos(math.radians(lat0)))
    return (lat0 + math.degrees(dlat), origin[1] + math.degrees(dlon))


PATH = [offset(ORIGIN, 0.0, i * 1.0) for i in range(int(PATH_LEN_M) + 1)]


class SimGps(Flow[None, GpsFix]):
    """Stands in for GpsSourceFlow: marches up the path, starting 0.5 m off it."""

    def __init__(self, *, hz: float = 10.0):
        super().__init__()
        self.hz = float(hz)

    def init_config(self) -> dict:
        return {"hz": self.hz}

    def reset(self) -> None:
        self.north = 0.0

    def step(self, _) -> GpsFix:
        self.north += SIM_MS / self.hz
        east = 0.5 * math.exp(-self.north / 6.0)   # converge onto the line
        lat, lon = offset(ORIGIN, east, self.north)
        return GpsFix(lat=lat, lon=lon, fix_type="RTK Fixed", fix_code=4,
                      sats=14, speed_mph=SIM_MPH, ts=time.time())


def main() -> int:
    cfg = FollowConfig(max_speed_mph=SIM_MPH, rate_hz=12.0, require_rtk=False)
    listener = TelemetryListener(port=PORT).start()

    pipe = Pipeline("cart_e2e")
    with pipe:
        gps = SimGps(hz=10.0) @ Rate(hz=10.0)
        follower = FollowerFlow(path=[list(p) for p in PATH],
                                cfg=cfg.__dict__.copy(), armed=False) @ Rate(hz=12.0)
        telemetry = TelemetryFlow(port=PORT) @ Trigger("phase")
        gps.then(follower, map={
            "lat": "lat", "lon": "lon", "fix_type": "fix_type",
            "fix_code": "fix_code", "speed_mph": "speed_mph", "ts": "ts",
        }).then(telemetry)

    print(f"running the real graph (sim GPS, {PATH_LEN_M:.0f} m path, "
          f"{SIM_MPH:.0f} mph) ...\n")
    engine = pipe.run(backend="multiprocessing", duration=14.0, blocking=False)

    t0 = time.time()
    while time.time() - t0 < 14.0:
        if listener.phase == "done":
            break
        time.sleep(0.05)
    engine.stop()          # NOT pipe.reset() — that re-inits flows, it doesn't stop them
    time.sleep(0.3)
    steps = list(listener.steps)
    listener.stop()

    tracking = [s for s in steps if s.get("phase") == "tracking"]
    phases = [s.get("phase") for s in steps]

    check("telemetry actually arrived", len(steps) > 20, f"{len(steps)} msgs")
    check("entered tracking", len(tracking) > 10, f"{len(tracking)} tracking steps")
    check("reached the goal (phase=done)", "done" in phases,
          f"final={phases[-1] if phases else None!r}")
    check("never aborted", "abort" not in phases,
          next((s.get("reason") for s in steps if s.get("phase") == "abort"), ""))

    if tracking:
        gases = [s.get("gas", 0.0) for s in tracking]
        steers = [abs(s.get("steer_cmd", 0.0)) for s in tracking]
        xtracks = [abs(s.get("xtrack_signed_m", 0.0)) for s in tracking]
        check("throttle came off zero", max(gases) > 0.0, f"peak gas {max(gases):.3f}")
        check("steering stayed within the clamp", max(steers) <= cfg.max_steer_deg,
              f"peak |steer| {max(steers):.1f}deg")
        check("tracked onto the line", xtracks[-1] < 0.5,
              f"final xtrack {xtracks[-1]:.2f} m (started 0.50)")
        check("telemetry carries the UI's key names",
              {"phase", "steer_cmd", "gas", "live_speed_mph"} <= set(tracking[-1]),
              ", ".join(sorted(tracking[-1])[:6]) + " ...")

    ok = all(results)
    print(f"\n{PASS if ok else FAIL} — {sum(results)}/{len(results)} checks")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
