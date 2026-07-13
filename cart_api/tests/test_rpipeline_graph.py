#!/usr/bin/env python3
"""
test_rpipeline_graph.py — assert the drive graph is wired the way safety assumes.

Two claims worth a test:

  1. DRY-RUN CANNOT ACTUATE. rpipeline builds no SteeringFlow and no PedalFlow
     when armed=False, so the ODrive and the Arduino are never even opened. This
     is a structural guarantee, not an `if armed:` check that someone can later
     get wrong — and this test is what keeps it structural.

  2. ARMED CLOSES BOTH FEEDBACK LOOPS. The measured column angle has to come
     back from SteeringFlow into FollowerFlow (the fused-heading estimator is
     built on it — without it the cart drives on a heading model fed zeros), and
     e-stop has to come back from PedalFlow.

No hardware: constructing the flows opens nothing. Serial ports are only touched
in init(), which runs inside the worker process at pipeline start.

Run: python3 tests/test_rpipeline_graph.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.follow import FollowConfig
from cartlib.rpipeline import build_pipeline

PASS, FAIL = "\033[32mPASS\033[0m", "\033[31mFAIL\033[0m"
results = []


def check(name, cond, detail=""):
    results.append(cond)
    print(f"  [{PASS if cond else FAIL}] {name}" + (f"  ({detail})" if detail else ""))


def flow_names(pipe):
    """Class names of the flows in the graph (unwrapping the Rate/Trigger shell)."""
    names = []
    for node in pipe.get_flow_dict().values():
        inner = getattr(node, "flow", node)
        names.append(type(inner).__name__)
    return sorted(names)


def waypoints(n=10):
    return [(37.4275 + i * 1e-5, -122.1697) for i in range(n)]


def main() -> int:
    wp = waypoints()

    print("dry-run graph (armed=False)")
    dry = build_pipeline(wp, FollowConfig(), armed=False, name="dry")
    names = flow_names(dry)
    check("no SteeringFlow — ODrive never opened", "SteeringFlow" not in names,
          ", ".join(names))
    check("no PedalFlow — Arduino never opened", "PedalFlow" not in names)
    check("GPS + follower + telemetry present",
          {"GpsSourceFlow", "FollowerFlow", "TelemetryFlow"} == set(names))
    check("2 edges (gps->follower->telemetry)", len(dry.get_connections()) == 2,
          f"{len(dry.get_connections())} edges")

    print("\narmed graph (armed=True)")
    armed = build_pipeline(wp, FollowConfig(), armed=True, name="armed")
    names = flow_names(armed)
    check("all five flows built",
          {"GpsSourceFlow", "FollowerFlow", "SteeringFlow", "PedalFlow",
           "TelemetryFlow"} == set(names), ", ".join(names))
    check("6 edges incl. both feedback loops",
          len(armed.get_connections()) == 6, f"{len(armed.get_connections())} edges")

    print("\nconfig round-trip (must survive the process boundary)")
    follower = next(
        getattr(n, "flow", n) for n in armed.get_flow_dict().values()
        if type(getattr(n, "flow", n)).__name__ == "FollowerFlow")
    cfg = follower.init_config()
    check("init_config carries the waypoints",
          len(cfg.get("path", [])) == len(wp), f"{len(cfg.get('path', []))} pts")
    check("init_config carries the gains", "st_k_cross" in cfg.get("cfg", {}))
    check("init_config carries armed=True", cfg.get("armed") is True)

    print("\nbad input")
    try:
        build_pipeline([(37.4, -122.1)], FollowConfig(), armed=False, name="short")
        check("rejects a 1-waypoint path", False, "no error raised")
    except ValueError:
        check("rejects a 1-waypoint path", True)

    ok = all(results)
    print(f"\n{PASS if ok else FAIL} — {sum(results)}/{len(results)} checks")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
