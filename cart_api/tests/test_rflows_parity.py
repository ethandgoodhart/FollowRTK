#!/usr/bin/env python3
"""
test_rflows_parity.py — prove the Retriever port didn't change how the cart drives.

rflows.FollowerFlow is a straight lift of follow.PathFollower.step(). This marches
BOTH down the same synthetic trajectory, feeding them byte-identical inputs, and
asserts every actuator command matches. If someone edits one control law and not
the other, this fails.

The trajectory is a cart converging onto a straight path from 1.5 m off to the
side — which is the case that actually exercises the interesting terms: the
cross-track P term, the cross-rate D (damping) term, and the throttle PI.

Both laws call time.time() internally for their derivative/integral timebase, so
a fake clock is installed and advanced in lockstep. Without it the two would see
dts a few microseconds apart and drift.

No hardware. Run: python3 tests/test_rflows_parity.py
"""

import math
import os
import sys
from dataclasses import asdict
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib import geo
from cartlib.follow import FollowConfig, PathFollower
from cartlib.rflows import FollowIn, FollowerFlow

ORIGIN = (37.4275, -122.1697)
HZ = 15.0
DT = 1.0 / HZ
SPEED_MPH = 3.0
SPEED_MS = SPEED_MPH * 0.44704


def offset(origin, east_m, north_m):
    lat0 = origin[0]
    dlat = north_m / geo.EARTH_R
    dlon = east_m / (geo.EARTH_R * math.cos(math.radians(lat0)))
    return (lat0 + math.degrees(dlat), origin[1] + math.degrees(dlon))


class StubGps:
    latest = None


class StubPedals:
    telemetry: dict = {}

    def set_gas(self, v):
        pass

    def set_brake(self, v):
        pass

    def stop(self):
        pass


class StubCart:
    def __init__(self):
        self.gps = StubGps()
        self.pedals = StubPedals()
        self.steering = None


class FakeClock:
    """Both control laws read time.time(); give them one we control."""

    def __init__(self, t0=1_700_000_000.0):
        self.t = t0

    def __call__(self):
        return self.t

    def tick(self, dt):
        self.t += dt


def trajectory(n=90):
    """Cart converging onto the path: 1.5 m east, decaying, marching north."""
    for i in range(n):
        north = i * SPEED_MS * DT
        east = 1.5 * math.exp(-i / 20.0)
        yield offset(ORIGIN, east, north)


def main() -> int:
    path = [offset(ORIGIN, 0.0, i * 1.0) for i in range(40)]   # straight, north
    clock = FakeClock()

    with mock.patch("time.time", clock):
        cfg_old = FollowConfig(require_rtk=False, rate_hz=HZ, max_speed_mph=SPEED_MPH)
        cfg_new = FollowConfig(require_rtk=False, rate_hz=HZ, max_speed_mph=SPEED_MPH)

        cart = StubCart()
        old = PathFollower(cart, path, cfg_old, armed=False)

        new = FollowerFlow(path=[list(p) for p in path],
                           cfg=asdict(cfg_new), armed=False)
        new.init()

        mismatches = []
        steers: list = []
        gases: list = []
        compared = 0

        for lat, lon in trajectory():
            cart.gps.latest = {"lat": lat, "lon": lon, "fix_code": 4,
                               "fix_type": "RTK Fixed", "ts": clock.t}
            # follow.py takes live speed from the UI; rflows computes it at the
            # GPS source. Hand both the same number so only the LAW is compared.
            old.cfg.live_speed_mph = SPEED_MPH

            t_old = old.step()
            t_new = new.step(FollowIn(
                lat=lat, lon=lon, fix_code=4, fix_type="RTK Fixed",
                speed_mph=SPEED_MPH, ts=clock.t,
                steering_actual_deg=None, steering_target_deg=None, estop=False,
            ))

            compared += 1
            for label, a, b in [
                ("phase",       t_old["phase"],            t_new.phase),
                ("steer",       t_old["steer_cmd"],        t_new.steer_deg),
                ("alpha",       t_old["alpha"],            t_new.alpha),
                ("gas",         t_old["gas"],              t_new.gas),
                ("brake",       t_old["brake"],            t_new.brake),
                ("xtrack",      t_old["xtrack_signed_m"],  t_new.xtrack_signed_m),
                ("heading",     t_old["heading_deg"],      t_new.heading_deg),
                ("heading_err", t_old["heading_err_deg"],  t_new.heading_err_deg),
                ("goal_dist",   t_old["dist_to_goal_m"],   t_new.dist_to_goal_m),
            ]:
                if a != b:
                    mismatches.append((compared, label, a, b))

            steers.append(t_new.steer_deg or 0.0)
            gases.append(t_new.gas or 0.0)
            clock.tick(DT)

    print(f"compared {compared} control steps (converging trajectory)\n")

    if mismatches:
        print(f"\033[31mFAIL\033[0m — {len(mismatches)} mismatched field(s):")
        for step, label, a, b in mismatches[:15]:
            print(f"  step {step:3d}  {label:12} follow.py={a!r}  rflows={b!r}")
        if len(mismatches) > 15:
            print(f"  ... and {len(mismatches) - 15} more")
        return 1

    # Guard against a vacuous pass: two laws that both emit zeros agree
    # perfectly. The run has to have actually steered and actually throttled.
    if max(abs(s) for s in steers) < 5.0:
        print(f"\033[31mFAIL\033[0m — vacuous: peak steer only "
              f"{max(abs(s) for s in steers):.1f}deg, the law never engaged")
        return 1
    if max(gases) <= 0.0:
        print("\033[31mFAIL\033[0m — vacuous: throttle never came off zero")
        return 1

    print(f"\033[32mPASS\033[0m — rflows.FollowerFlow matches follow.PathFollower on "
          f"every step\n         (steer, alpha, gas, brake, phase, xtrack, heading, "
          f"heading_err, goal)")
    print(f"         peak steer {max(abs(s) for s in steers):.1f}deg, "
          f"peak gas {max(gases):.3f} — the law was actually exercised.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
