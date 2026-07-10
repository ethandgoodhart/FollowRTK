# Steering simulator & controller lab

Off-vehicle simulation used to redesign the path-following steering for a smooth
ride. Everything here is calibrated to the **2006 Club Car Precedent + ODrive S1**
from the three recorded RTK drives in `drives/`.

## What's here

| file | purpose |
|------|---------|
| `simlib.py` | calibrated plant: kinematic bicycle + rate/accel-limited steering actuator + path geometry |
| `controllers.py` | `OldController` (faithful reimpl of the on-cart law) and `NewController` (the new smooth law) |
| `simharness.py` | runs a controller over a recorded route through the plant; shared speed model so only steering differs |
| `export_viz.py` | runs old + new on all 3 routes and writes `viz_data.json`; `--embed` injects it into the HTML |
| `steering_sim.html` | **standalone visualization** — open in a browser, press play, watch old vs new |
| `steering_sim_artifact.html` | body-only copy published as a claude.ai artifact |

```bash
python3 export_viz.py --embed   # regenerate data + rebuild the page
open steering_sim.html          # or just double-click it
```

## What was wrong with the old steering

1. **The control loop ran at ~1.9 Hz, not 15 Hz.** `steering._query` did
   `serial.read(256)`, which blocked for the full 0.4 s port timeout on every
   wheel-angle read (the ODrive reply is ~10 bytes and never filled the buffer).
   The loop paid that ~0.45 s *every* cycle. Fixed in `cartlib/steering.py` with
   `read_until(b"\n")` → the loop can now hit its 10–15 Hz target. This is the
   single biggest smoothness win (matches the smooth ~10 Hz manual PS5 driving).
2. **High-gain proportional steering + no real heading.** The old law damped on a
   raw 0.5 s-differentiated cross-track "heading", which was so noisy the loop
   limit-cycled — the wheels sawed full-lock across the line and back (measured
   steering-rate RMS up to 240 °/s; the wheel sat at full lock ~48 % of the time
   at 5.5 mph, which is why 5.5 mph failed).

## The new law (`NewController` → `cartlib/follow.py`)

Stanley cross-track + curvature feedforward, with a **properly-anchored heading
estimate**: a complementary filter fusing the wheel-angle yaw-rate model
(lag-free) with the absolute GPS-track heading over a ~0.9 m window (drift-free).
The column target is then low-pass + slew limited so the wheel moves as one
continuous motion. Gains are gentle by design (a 2 m error at 3 mph asks for
~15° of road wheel, never full lock) but firm enough to cut onto the line
directly instead of drifting in on a long tail.

### Simulated results (steady-state, vs the old law)

| route | old rate | new rate | old xtrack RMS | new xtrack RMS |
|-------|---------:|---------:|---------------:|---------------:|
| Route 1, 3.5 mph | 92 °/s | **33 °/s** | 0.28 m | **0.18 m** |
| Route 2, 3.5 mph | 25 °/s | **24 °/s** | 0.24 m | **0.18 m** |
| Route 2, 5.5 mph | 243 °/s | **30 °/s** | 0.84 m | **0.18 m** |

Smoother on every route, tighter tracking, and stable at 5.5 mph (which the old
law failed). Verified robust to ±0.2 m RTK-Float position noise.

## Plant calibration notes

- `C_yaw ≈ 0.025 deg/s per (m/s · column-deg)` fit from the drives; the RTK-Float
  data (~0.5 m noise over 0.5 s steps at 3 mph) is too noisy for a tight
  open-loop twin, so the plant uses physically-grounded geometry
  (`WHEELBASE 1.65 m`, `STEER_RATIO 10:1`, `MAX_ROADWHEEL 33°`) and is validated
  by **closed-loop** replay: running the old law in-sim reproduces the recorded
  xtrack/steering profiles (max cross-track matches to ~0.1 m).
- Actuator peak slew ~480 °/s at the column matches the ODrive trap-traj vel
  limit (4 turns/s ÷ 3:1 belt) and the peak slew measured in the drives.
