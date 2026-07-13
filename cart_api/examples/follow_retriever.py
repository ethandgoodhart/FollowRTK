#!/usr/bin/env python3
"""
follow_retriever.py — follow a recorded RTK path, with the autonomy running as
a Retriever dataflow graph instead of the hand-rolled loop in follow.py.

!!! THIS DRIVES THE CART AUTONOMOUSLY (steering + throttle). !!!
Defaults to DRY-RUN. In dry-run the actuator flows are never built, so the
ODrive and Arduino are not even opened — it computes and prints the control it
*would* send. Add --go to actually drive. Always keep a hand on the e-stop.

Usage:
    python3 examples/follow_retriever.py paths/loop.json                  # dry-run
    python3 examples/follow_retriever.py paths/loop.json --go             # DRIVE
    python3 examples/follow_retriever.py paths/loop.json --go --speed 3.0
    python3 examples/follow_retriever.py paths/loop.json --go --ntrip --require-rtk

--speed is the cruise SETPOINT in mph. The PI speed controller drives the cart
to it; the throttle is separately hard-capped by the FSD/global governors.

Stopping (Ctrl-C, arrival, or abort) tears the graph down. Each flow's
finalize() cuts gas, parks the brake, and idles the steering in its own worker
process; if a worker is wedged, its death stops the pedal heartbeat and the
Arduino firmware slams the brake within 300 ms.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib import config
from cartlib.follow import FollowConfig, load_path
from cartlib.rpipeline import run_drive


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--go", action="store_true",
                    help="actually drive (default: dry-run, actuators not opened)")
    ap.add_argument("--speed", type=float, default=3.0,
                    help="cruise setpoint in mph (default 3.0)")
    ap.add_argument("--ntrip", action="store_true", help="feed RTK corrections")
    ap.add_argument("--require-rtk", action="store_true",
                    help="refuse to drive without an RTK fix")
    # The live steering knobs are the st_* family (Stanley + fused heading). The
    # old steer_gain/xtrack_gain/heading_gain are legacy and no longer read by
    # the law, so they are deliberately not exposed here.
    ap.add_argument("--k-cross", type=float, default=FollowConfig.st_k_cross,
                    help="Stanley cross-track gain")
    ap.add_argument("--k-heading", type=float, default=FollowConfig.st_k_heading,
                    help="heading-alignment (damping) weight")
    ap.add_argument("--rate", type=float, default=FollowConfig.rate_hz,
                    help="control loop rate (Hz)")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after N seconds regardless of progress")
    args = ap.parse_args()

    waypoints = load_path(args.path)
    cfg = FollowConfig(
        max_speed_mph=args.speed,
        st_k_cross=args.k_cross,
        st_k_heading=args.k_heading,
        rate_hz=args.rate,
        require_rtk=args.require_rtk,
    )

    eff_cap = config.effective_gas_cap(cfg.gas_cap)
    print(f"Loaded {len(waypoints)} waypoints from {args.path}")
    print(f"Mode: {'### LIVE DRIVE ###' if args.go else 'dry-run (actuators not opened)'}")
    print(f"speed={cfg.max_speed_mph} mph (gas cap {eff_cap})  rate={cfg.rate_hz} Hz  "
          f"k_cross={cfg.st_k_cross}  k_heading={cfg.st_k_heading}  "
          f"require_rtk={cfg.require_rtk}")
    if args.go:
        print("\n*** ARMED — the cart will move. Hand on the e-stop. ***\n")

    def show(t: dict) -> None:
        sys.stdout.write(
            f"\r[{t.get('phase','?'):8}] fix={t.get('fix')} "
            f"steer={t.get('steer_deg', 0.0):+6.1f} gas={t.get('gas', 0.0):.3f} "
            f"brake={t.get('brake', 0.0):.3f} "
            f"mph={t.get('live_speed_mph', 0.0):4.1f} "
            f"xtrack={t.get('xtrack_signed_m', 0.0):+5.2f}m "
            f"goal={t.get('dist_to_goal_m', 0.0):5.1f}m   ")
        sys.stdout.flush()

    run_drive(
        waypoints, cfg,
        armed=args.go,
        duration=args.duration,
        ntrip_provider="pointone" if args.ntrip else None,
        on_step=show,
    )


if __name__ == "__main__":
    main()
