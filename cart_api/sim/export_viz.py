#!/usr/bin/env python3
"""
export_viz.py — run the OLD and NEW steering controllers through the calibrated
simulator on each recorded route and emit a single JSON payload the standalone
visualization (steering_sim.html) embeds/loads.

Usage:  python3 export_viz.py            # writes viz_data.json
        python3 export_viz.py --embed    # also injects it into steering_sim.html
"""
import json, math, os, sys

from simharness import load_route, simulate
from controllers import OldController, OldConfig, NewController, NewConfig

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES = [
    ("route1_bad3.5", "Route 1 — 3.5 mph (was: bad)"),
    ("route2_notgreat3.5", "Route 2 — 3.5 mph (was: not great)"),
    ("route2_failed5.5", "Route 2 — 5.5 mph (was: FAILED)"),
]


def seg_metrics(trace, thr=0.7):
    """Steady-state (post-acquisition) tracking + smoothness summary."""
    acq = next((i for i, p in enumerate(trace) if p["xtrack"] < thr), None)
    seg = trace[acq:] if acq is not None else trace
    xs = [p["xtrack"] for p in seg]
    rms = math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else 0
    rates = []
    for i in range(1, len(seg)):
        dt = seg[i]["t"] - seg[i - 1]["t"]
        if dt > 0:
            rates.append((seg[i]["wheel_act"] - seg[i - 1]["wheel_act"]) / dt)
    rate_rms = math.sqrt(sum(r * r for r in rates) / len(rates)) if rates else 0
    return {
        "acq_t": round(trace[acq]["t"], 1) if acq is not None else None,
        "ss_xtrack_rms": round(rms, 3),
        "ss_xtrack_max": round(max(xs), 3) if xs else 0,
        "ss_steer_rate": round(rate_rms, 1),
    }


def slim(trace):
    """Keep only the fields the viz needs, at full rate."""
    return [{
        "t": p["t"], "x": p["x"], "y": p["y"],
        "h": p["heading_deg"], "wa": p["wheel_act"], "wc": p["wheel_cmd"],
        "v": p["v_mph"], "xt": p["xtrack_signed"],
        "sx": p["snap_x"], "sy": p["snap_y"],
    } for p in trace]


def build(gps_noise_m=0.0):
    out = {"routes": [], "gps_noise_m": gps_noise_m}
    for key, label in ROUTES:
        route = load_route(os.path.join(HERE, "drives", key, "last_drive.json"))
        old = simulate(route, OldController(OldConfig()), rate_hz=1.9,
                       gps_noise_m=gps_noise_m)
        new = simulate(route, NewController(NewConfig()), rate_hz=10.0,
                       gps_noise_m=gps_noise_m)
        # path in the same local frame the traces use
        from simlib import Path
        path = Path(route["pts"], route["origin"])
        out["routes"].append({
            "key": key, "label": label,
            "max_mph": route["max_speed_mph"],
            "path": [[round(x, 3), round(y, 3)] for (x, y) in path.pts],
            "old": {"trace": slim(old["trace"]), "arrived": old["arrived"],
                    "aborted": old["aborted"], "abort_t": old["abort_t"],
                    "time_s": old["time_s"], "metrics": seg_metrics(old["trace"]),
                    "rate_hz": 1.9},
            "new": {"trace": slim(new["trace"]), "arrived": new["arrived"],
                    "aborted": new["aborted"], "abort_t": new["abort_t"],
                    "time_s": new["time_s"], "metrics": seg_metrics(new["trace"]),
                    "rate_hz": 10.0},
        })
    return out


def main():
    data = build()
    with open(os.path.join(HERE, "viz_data.json"), "w") as f:
        json.dump(data, f)
    print(f"wrote viz_data.json  ({len(json.dumps(data))//1024} KB)")
    for r in data["routes"]:
        o, n = r["old"]["metrics"], r["new"]["metrics"]
        print(f"  {r['key']:20s}  OLD rms={o['ss_xtrack_rms']}m rate={o['ss_steer_rate']}dps"
              f"   NEW rms={n['ss_xtrack_rms']}m rate={n['ss_steer_rate']}dps")

    if "--embed" in sys.argv:
        html_path = os.path.join(HERE, "steering_sim.html")
        with open(html_path) as f:
            html = f.read()
        payload = json.dumps(data)
        start = html.index("/*DATA_START*/") + len("/*DATA_START*/")
        end = html.index("/*DATA_END*/")
        html = html[:start] + payload + html[end:]
        with open(html_path, "w") as f:
            f.write(html)
        print("embedded data into steering_sim.html")


if __name__ == "__main__":
    main()
