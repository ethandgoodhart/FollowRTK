#!/usr/bin/env python3
"""
brake_char.py — measure how the cart actually stops.

!!! THIS DRIVES A LIVE CART. !!!  Open area, clear of people, hand on the
e-stop. You drive manually with a PS5 (DualSense) controller; the script only
takes over the pedals for the duration of a braking trial, then hands back.

Everything in the pedestrian/vehicle speed-control plan hangs off four numbers
that nobody can guess from first principles, so we measure them:

  * actuator lag   — command sent -> brake pot actually starts moving, and ->
                     reaches the commanded value. The linear actuator pushing
                     the pedal is slow, and this is very likely the dominant
                     term in the reaction time rho.
  * motion lag     — command sent -> cart speed actually starts dropping.
  * deceleration   — m/s^2 achieved, per brake pot level.
  * stop distance  — metres travelled from command to standstill, per entry
                     speed and brake level.

A brake level of 0.0 is a COAST-DOWN trial: no brake, just gas off. That
measures the free deceleration from rolling resistance, which is what you get
during the reaction window before the brake bites. It is a genuinely useful
data point, not a wasted run.

Controls (DualSense, default mapping)
------------------------------------
    R2                  gas   (requires L1 held as a dead-man)
    L2                  brake (manual, always available)
    left stick X        steering  -- only with --steer
    Cross  (X)          START A BRAKING TRIAL at the current speed
    Circle (O)          abort trial / release everything
    D-pad up/down       select the brake level for the next trial
    Options             quit cleanly

Usage
-----
    # Check the controller mapping indoors, no hardware touched:
    python3 tools/brake_char.py --probe
    python3 tools/brake_char.py --no-cart

    # Real run:
    python3 tools/brake_char.py
    python3 tools/brake_char.py --levels 0,0.1,0.2,0.3 --gas-cap 0.20

Results land in ``brake_char/<timestamp>/`` as ``samples.jsonl`` (every loop
sample) and ``trials.json`` (one analysed record per trial), and a summary
table is printed on exit.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib import Cart, config, geo  # noqa: E402

MPS_TO_MPH = 2.2369363


# ---------------------------------------------------------------------------
# Speed estimation
# ---------------------------------------------------------------------------
# The GGA sentences the receiver gives us carry position but NO velocity, and
# cartlib's follower gets its ``live_speed_mph`` from the browser. For a
# braking measurement we need our own, better estimate: least-squares slope of
# cumulative arc length against time over a short sliding window. With RTK
# fixed at 10 Hz, a 0.4 s window is 4 samples of centimetre-grade position,
# which resolves speed far more finely than we need.
class SpeedEstimator:
    def __init__(self, window_s: float = 0.4):
        self.window_s = window_s
        self._pts: deque = deque()      # (ts, lat, lon, cumulative_m)
        self._cum = 0.0
        self._last_ts: Optional[float] = None

    def add(self, ts: float, lat: float, lon: float) -> None:
        """Feed one fix. Ignores repeats of a fix we've already seen."""
        if self._last_ts is not None and ts <= self._last_ts:
            return
        if self._pts:
            _, plat, plon, _ = self._pts[-1]
            self._cum += geo.haversine_m((plat, plon), (lat, lon))
        self._pts.append((ts, lat, lon, self._cum))
        self._last_ts = ts
        cutoff = ts - self.window_s
        while len(self._pts) > 2 and self._pts[0][0] < cutoff:
            self._pts.popleft()

    @property
    def speed_ms(self) -> float:
        """Least-squares d(arclength)/dt over the window, in m/s."""
        if len(self._pts) < 2:
            return 0.0
        t0 = self._pts[0][0]
        ts = [p[0] - t0 for p in self._pts]
        ds = [p[3] for p in self._pts]
        n = len(ts)
        mt = sum(ts) / n
        md = sum(ds) / n
        num = sum((t - mt) * (d - md) for t, d in zip(ts, ds))
        den = sum((t - mt) ** 2 for t in ts)
        if den <= 1e-9:
            return 0.0
        return max(0.0, num / den)

    @property
    def speed_mph(self) -> float:
        return self.speed_ms * MPS_TO_MPH


# ---------------------------------------------------------------------------
# Gamepad
# ---------------------------------------------------------------------------
# evdev codes for a DualSense on the kernel's hid-playstation driver. If your
# pad enumerates differently, run --probe: it prints every event code live so
# you can correct these in one place.
AXIS_GAS = "ABS_RZ"      # R2 trigger
AXIS_BRAKE = "ABS_Z"     # L2 trigger
AXIS_STEER = "ABS_X"     # left stick X
BTN_DEADMAN = "BTN_TL"   # L1
BTN_TRIAL = "BTN_SOUTH"  # Cross
BTN_ABORT = "BTN_EAST"   # Circle
BTN_QUIT = "BTN_START"   # Options
AXIS_DPAD_Y = "ABS_HAT0Y"

PAD_NAME_HINTS = ("dualsense", "wireless controller", "sony", "playstation", "dualshock")


class Gamepad:
    """Background evdev reader exposing the pad as a few normalised floats."""

    def __init__(self, path: Optional[str] = None, deadzone: float = 0.08):
        import evdev  # imported here so --no-cart works without a pad attached

        self.evdev = evdev
        self.deadzone = deadzone
        self.dev = self._find(path)
        self._absinfo = {}
        caps = self.dev.capabilities(absinfo=True)
        for code, info in caps.get(evdev.ecodes.EV_ABS, []):
            name = evdev.ecodes.ABS[code]
            if isinstance(name, list):
                name = name[0]
            self._absinfo[name] = info

        self.gas = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.deadman = False
        self._edges = deque()          # button-press events, consumed by the loop
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _find(self, path):
        if path:
            return self.evdev.InputDevice(path)
        for p in self.evdev.list_devices():
            d = self.evdev.InputDevice(p)
            if any(h in d.name.lower() for h in PAD_NAME_HINTS):
                # The pad also exposes a motion-sensor node with no buttons;
                # take the one that actually has a Cross button.
                caps = d.capabilities()
                keys = caps.get(self.evdev.ecodes.EV_KEY, [])
                if self.evdev.ecodes.BTN_SOUTH in keys:
                    return d
        raise RuntimeError(
            "no DualSense found. Connect it by USB or Bluetooth, then check "
            "`ls /dev/input/by-id/`. Pass --pad /dev/input/eventN to force one."
        )

    def _norm(self, name: str, value: int, signed: bool) -> float:
        info = self._absinfo.get(name)
        if not info:
            return 0.0
        lo, hi = info.min, info.max
        if hi <= lo:
            return 0.0
        frac = (value - lo) / (hi - lo)          # 0..1
        out = frac * 2.0 - 1.0 if signed else frac
        return 0.0 if abs(out) < self.deadzone else out

    def _loop(self) -> None:
        ecodes = self.evdev.ecodes
        try:
            for ev in self.dev.read_loop():
                if self._stop.is_set():
                    return
                if ev.type == ecodes.EV_ABS:
                    name = ecodes.ABS[ev.code]
                    if isinstance(name, list):
                        name = name[0]
                    if name == AXIS_GAS:
                        self.gas = self._norm(name, ev.value, False)
                    elif name == AXIS_BRAKE:
                        self.brake = self._norm(name, ev.value, False)
                    elif name == AXIS_STEER:
                        self.steer = self._norm(name, ev.value, True)
                    elif name == AXIS_DPAD_Y and ev.value != 0:
                        self._push("DPAD_UP" if ev.value < 0 else "DPAD_DOWN")
                elif ev.type == ecodes.EV_KEY:
                    names = ecodes.KEY.get(ev.code) or ecodes.BTN.get(ev.code)
                    if isinstance(names, list):
                        names = names[0]
                    if names == BTN_DEADMAN:
                        self.deadman = ev.value != 0
                    elif ev.value == 1 and names in (BTN_TRIAL, BTN_ABORT, BTN_QUIT):
                        self._push(names)
        except OSError:
            # Pad unplugged mid-run. Surface it as a hard abort: the main loop
            # sees the disconnect and releases the pedals.
            self._push("DISCONNECT")

    def _push(self, name: str) -> None:
        with self._lock:
            self._edges.append(name)

    def poll_events(self) -> List[str]:
        with self._lock:
            out = list(self._edges)
            self._edges.clear()
        return out

    def close(self) -> None:
        self._stop.set()


def probe_gamepad(path: Optional[str]) -> None:
    """Print live event codes so a non-standard pad can be re-mapped."""
    import evdev

    pad = Gamepad(path)
    print(f"reading {pad.dev.path}  ({pad.dev.name})")
    print("press buttons / pull triggers; Ctrl-C to stop\n")
    ecodes = evdev.ecodes
    for ev in pad.dev.read_loop():
        if ev.type == ecodes.EV_ABS:
            name = ecodes.ABS[ev.code]
            if isinstance(name, list):
                name = name[0]
            print(f"  AXIS {name:<12} = {ev.value}")
        elif ev.type == ecodes.EV_KEY and ev.value in (0, 1):
            name = ecodes.KEY.get(ev.code) or ecodes.BTN.get(ev.code)
            if isinstance(name, list):
                name = name[0]
            print(f"  BTN  {name:<12} {'down' if ev.value else 'up'}")


# ---------------------------------------------------------------------------
# Trial recording + analysis
# ---------------------------------------------------------------------------
@dataclass
class Sample:
    t: float                # seconds since trial command
    speed_mph: float
    speed_ms: float
    brake_pot: float        # measured, from Mega telemetry
    brake_cmd: float
    gas_pot: float
    lat: Optional[float]
    lon: Optional[float]
    fix: Optional[str]


@dataclass
class Trial:
    index: int
    brake_level: float
    v0_mph: float = 0.0
    v0_ms: float = 0.0
    start_lat: Optional[float] = None
    start_lon: Optional[float] = None
    samples: List[Sample] = field(default_factory=list)
    # analysis outputs
    actuator_lag_s: Optional[float] = None
    actuator_full_s: Optional[float] = None
    motion_lag_s: Optional[float] = None
    decel_ms2: Optional[float] = None
    stop_time_s: Optional[float] = None
    stop_dist_integrated_m: Optional[float] = None
    stop_dist_gps_m: Optional[float] = None
    note: str = ""


STOP_MPH = 0.25          # below this we call it stopped
STOP_HOLD_S = 0.6        # ...sustained this long
MOTION_DROP_MPH = 0.15   # speed drop that counts as "the cart responded"


def analyse(tr: Trial) -> Trial:
    s = tr.samples
    if len(s) < 5:
        tr.note = "too few samples"
        return tr

    base_pot = s[0].brake_pot
    cmd = tr.brake_level

    # Actuator lag: when did the measured pot leave its resting value, and when
    # did it arrive. Skipped for a coast-down trial, which commands no brake.
    if cmd > 0.02:
        for smp in s:
            if smp.brake_pot > base_pot + 0.02:
                tr.actuator_lag_s = smp.t
                break
        for smp in s:
            if smp.brake_pot >= 0.9 * cmd:
                tr.actuator_full_s = smp.t
                break

    # Motion lag: first sustained drop below v0. Requires two consecutive
    # samples so a single noisy fix can't trigger it.
    for i in range(len(s) - 1):
        if (s[i].speed_mph < tr.v0_mph - MOTION_DROP_MPH
                and s[i + 1].speed_mph < tr.v0_mph - MOTION_DROP_MPH):
            tr.motion_lag_s = s[i].t
            break

    # Stop detection: first time speed stays under STOP_MPH for STOP_HOLD_S.
    stop_i = None
    for i, smp in enumerate(s):
        if smp.speed_mph >= STOP_MPH:
            continue
        if s[-1].t - smp.t < STOP_HOLD_S:
            break
        if all(s[j].speed_mph < STOP_MPH
               for j in range(i, len(s)) if s[j].t - smp.t <= STOP_HOLD_S):
            stop_i = i
            break
    if stop_i is None:
        tr.note = "never reached a full stop in the recording"
        stop_i = len(s) - 1
    tr.stop_time_s = s[stop_i].t

    # Deceleration: least-squares slope of speed vs time over the meat of the
    # stop -- from 90% of entry speed down to 15%. Trimming both ends keeps the
    # actuator ramp-in and the final creep out of the fit.
    hi, lo = 0.90 * tr.v0_ms, 0.15 * tr.v0_ms
    seg = [smp for smp in s[:stop_i + 1] if lo <= smp.speed_ms <= hi]
    if len(seg) >= 3:
        n = len(seg)
        mt = sum(x.t for x in seg) / n
        mv = sum(x.speed_ms for x in seg) / n
        num = sum((x.t - mt) * (x.speed_ms - mv) for x in seg)
        den = sum((x.t - mt) ** 2 for x in seg)
        if den > 1e-9:
            tr.decel_ms2 = -(num / den)     # positive = slowing down

    # Stopping distance, two independent ways. They should agree within a few
    # per cent; if they don't, distrust the GPS for that trial.
    dist = 0.0
    for a, b in zip(s[:stop_i], s[1:stop_i + 1]):
        dist += 0.5 * (a.speed_ms + b.speed_ms) * (b.t - a.t)
    tr.stop_dist_integrated_m = dist
    if tr.start_lat is not None and s[stop_i].lat is not None:
        tr.stop_dist_gps_m = geo.haversine_m(
            (tr.start_lat, tr.start_lon), (s[stop_i].lat, s[stop_i].lon))
    return tr


def summarise(trials: List[Trial]) -> str:
    if not trials:
        return "no trials recorded"
    head = (f"{'#':>2}  {'brake':>5}  {'v0 mph':>6}  {'act lag':>7}  "
            f"{'full':>5}  {'motion':>6}  {'decel':>7}  {'stop t':>6}  "
            f"{'dist(int)':>9}  {'dist(gps)':>9}")
    lines = [head, "-" * len(head)]

    def f(v, spec=".2f"):
        return format(v, spec) if v is not None else "  --"

    for t in trials:
        lines.append(
            f"{t.index:>2}  {t.brake_level:>5.2f}  {t.v0_mph:>6.2f}  "
            f"{f(t.actuator_lag_s):>7}  {f(t.actuator_full_s):>5}  "
            f"{f(t.motion_lag_s):>6}  {f(t.decel_ms2):>7}  "
            f"{f(t.stop_time_s):>6}  {f(t.stop_dist_integrated_m):>9}  "
            f"{f(t.stop_dist_gps_m):>9}")
        if t.note:
            lines.append(f"      note: {t.note}")

    lines.append("")
    lines.append("units: lag/full/motion/stop t = seconds, decel = m/s^2, dist = metres")

    # The headline number for the plan: reaction time rho, and the decel to
    # design the speed envelope around.
    lags = [t.actuator_lag_s for t in trials if t.actuator_lag_s is not None]
    motions = [t.motion_lag_s for t in trials if t.motion_lag_s is not None]
    if lags:
        lines.append(f"\nactuator lag: min {min(lags):.2f}s  max {max(lags):.2f}s  "
                     f"mean {sum(lags)/len(lags):.2f}s")
    if motions:
        lines.append(f"motion lag  : min {min(motions):.2f}s  max {max(motions):.2f}s  "
                     f"mean {sum(motions)/len(motions):.2f}s   <- this is rho")
    by_level = {}
    for t in trials:
        if t.decel_ms2 is not None:
            by_level.setdefault(t.brake_level, []).append(t.decel_ms2)
    if by_level:
        lines.append("\ndeceleration by brake level:")
        for lvl in sorted(by_level):
            vals = by_level[lvl]
            lines.append(f"  pot {lvl:.2f} -> {sum(vals)/len(vals):.2f} m/s^2 "
                         f"(n={len(vals)}, spread {min(vals):.2f}..{max(vals):.2f})")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--levels", default="0,0.10,0.15,0.20,0.30,0.45",
                    help="brake pot levels to sweep (0 = coast-down trial)")
    ap.add_argument("--gas-cap", type=float, default=0.22,
                    help="hard gas ceiling for manual driving (default 0.22)")
    ap.add_argument("--steer", action="store_true",
                    help="steer with the left stick (default: hand-steer the wheel)")
    ap.add_argument("--max-steer-deg", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=50.0, help="loop Hz")
    ap.add_argument("--out", default=None, help="output directory")
    ap.add_argument("--pad", default=None, help="force an evdev path")
    ap.add_argument("--probe", action="store_true",
                    help="print gamepad events and exit (no cart, no hardware)")
    ap.add_argument("--no-cart", action="store_true",
                    help="gamepad + logging only; never opens the cart")
    args = ap.parse_args()

    if args.probe:
        probe_gamepad(args.pad)
        return 0

    levels = [float(x) for x in args.levels.split(",") if x.strip()]
    if not levels:
        print("no brake levels given", file=sys.stderr)
        return 2
    for lvl in levels:
        if lvl > config.BRAKE_POT_MAX:
            print(f"brake level {lvl} exceeds BRAKE_POT_MAX "
                  f"({config.BRAKE_POT_MAX})", file=sys.stderr)
            return 2

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "brake_char", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    samples_path = os.path.join(out_dir, "samples.jsonl")
    trials_path = os.path.join(out_dir, "trials.json")

    print(f"[brake_char] logging to {out_dir}")
    pad = Gamepad(args.pad)
    print(f"[brake_char] gamepad: {pad.dev.name}")

    cart = None
    if not args.no_cart:
        cart = Cart(use_steering=args.steer, gas_cap=args.gas_cap).open()
        print("[brake_char] waiting for a GPS fix...")
        fix = cart.gps.wait_for_fix(timeout=30.0)
        if not fix:
            print("no GPS fix after 30s -- aborting", file=sys.stderr)
            cart.close()
            return 1
        print(f"[brake_char] fix: {fix['fix_type']}  sats={fix['sats']}")
        cart.arm()
        if args.steer:
            cart.steering.enable()

    speed = SpeedEstimator()
    trials: List[Trial] = []
    active: Optional[Trial] = None
    level_i = 0
    running = True
    period = 1.0 / args.rate

    def release() -> None:
        if cart and cart.pedals:
            cart.pedals.set_gas(0.0)
            cart.pedals.stop()

    def on_signal(_sig, _frm):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print(f"\n  brake level for next trial: {levels[level_i]:.2f}"
          f"   (D-pad up/down to change)")
    print("  hold L1 + R2 to drive, L2 to brake, Cross to run a trial, "
          "Options to quit\n")

    sfile = open(samples_path, "w")
    last_status = 0.0
    try:
        while running:
            loop_t = time.time()

            fix = cart.gps.latest if cart else None
            if fix and fix.get("lat") is not None:
                speed.add(fix["ts"], fix["lat"], fix["lon"])
            tele = cart.pedals.telemetry if cart else {}

            if tele.get("estop") or tele.get("failsafe"):
                if active:
                    active.note = "e-stop / failsafe during trial"
                    trials.append(analyse(active))
                    active = None
                release()
                print("\n!! E-STOP / FAILSAFE -- pedals released. "
                      "Clear it, then continue.\n")

            for ev in pad.poll_events():
                if ev == "DISCONNECT":
                    print("\n!! gamepad disconnected -- releasing and quitting\n")
                    release()
                    running = False
                elif ev == BTN_QUIT:
                    running = False
                elif ev == "DPAD_UP" and active is None:
                    level_i = (level_i + 1) % len(levels)
                    print(f"  brake level for next trial: {levels[level_i]:.2f}")
                elif ev == "DPAD_DOWN" and active is None:
                    level_i = (level_i - 1) % len(levels)
                    print(f"  brake level for next trial: {levels[level_i]:.2f}")
                elif ev == BTN_ABORT:
                    if active:
                        active.note = "aborted by operator"
                        trials.append(analyse(active))
                        active = None
                        print("  trial aborted")
                    release()
                elif ev == BTN_TRIAL and active is None:
                    v0 = speed.speed_ms
                    if v0 * MPS_TO_MPH < 0.8:
                        print("  ignoring trial: cart is barely moving "
                              f"({v0 * MPS_TO_MPH:.2f} mph)")
                        continue
                    lvl = levels[level_i]
                    active = Trial(index=len(trials) + 1, brake_level=lvl,
                                   v0_mph=v0 * MPS_TO_MPH, v0_ms=v0,
                                   start_lat=fix["lat"] if fix else None,
                                   start_lon=fix["lon"] if fix else None)
                    active._t0 = loop_t          # type: ignore[attr-defined]
                    # Command both at the same instant: gas off AND brake on.
                    # That is exactly what the follower's emergency layer will
                    # do, so this measures the lag that policy will actually
                    # experience -- not an idealised brake-only step.
                    if cart:
                        cart.pedals.set_gas(0.0)
                        cart.pedals.set_brake(lvl)
                    kind = "COAST-DOWN" if lvl == 0.0 else f"brake {lvl:.2f}"
                    print(f"\n  >> trial {active.index}: {kind} "
                          f"from {active.v0_mph:.2f} mph")

            # --- pedal command ---------------------------------------------
            if active is None:
                gas = pad.gas if pad.deadman else 0.0
                brake = pad.brake * config.BRAKE_POT_MAX
                if brake > 0.02:
                    gas = 0.0            # brake always wins
                if cart:
                    cart.pedals.set_gas(gas * args.gas_cap)
                    cart.pedals.set_brake(brake)
                    if args.steer:
                        cart.steering.set_angle(pad.steer * args.max_steer_deg)
            else:
                # Trial owns the pedals; hold the commanded level.
                if cart:
                    cart.pedals.set_brake(active.brake_level)

            # --- record -----------------------------------------------------
            row = {
                "ts": loop_t,
                "speed_mph": round(speed.speed_mph, 3),
                "speed_ms": round(speed.speed_ms, 3),
                "brake_pot": tele.get("brake"),
                "brake_target": tele.get("brake_target"),
                "gas_pot": tele.get("gas"),
                "lat": fix["lat"] if fix else None,
                "lon": fix["lon"] if fix else None,
                "fix": fix["fix_type"] if fix else None,
                "trial": active.index if active else None,
            }
            sfile.write(json.dumps(row) + "\n")

            if active is not None:
                t_rel = loop_t - active._t0      # type: ignore[attr-defined]
                active.samples.append(Sample(
                    t=t_rel,
                    speed_mph=speed.speed_mph,
                    speed_ms=speed.speed_ms,
                    brake_pot=tele.get("brake", 0.0) or 0.0,
                    brake_cmd=active.brake_level,
                    gas_pot=tele.get("gas", 0.0) or 0.0,
                    lat=fix["lat"] if fix else None,
                    lon=fix["lon"] if fix else None,
                    fix=fix["fix_type"] if fix else None,
                ))
                # End the trial once stopped and held, or after a hard timeout.
                stopped = (speed.speed_mph < STOP_MPH
                           and t_rel > 1.0
                           and all(smp.speed_mph < STOP_MPH
                                   for smp in active.samples
                                   if smp.t >= t_rel - STOP_HOLD_S))
                if stopped or t_rel > 30.0:
                    if not stopped:
                        active.note = "timed out after 30s"
                    done = analyse(active)
                    trials.append(done)
                    active = None
                    if cart:
                        cart.pedals.set_brake(0.0)
                    print(f"     stopped: decel {done.decel_ms2 or float('nan'):.2f} m/s^2, "
                          f"{done.stop_dist_integrated_m or float('nan'):.2f} m, "
                          f"act lag {done.actuator_lag_s if done.actuator_lag_s is not None else float('nan'):.2f}s, "
                          f"motion lag {done.motion_lag_s if done.motion_lag_s is not None else float('nan'):.2f}s")
                    print(f"  brake level for next trial: {levels[level_i]:.2f}")
            elif loop_t - last_status >= 0.5:
                last_status = loop_t
                sys.stdout.write(
                    f"\r  {speed.speed_mph:5.2f} mph   gas {tele.get('gas', 0.0) or 0.0:.2f}  "
                    f"brake {tele.get('brake', 0.0) or 0.0:.2f}  "
                    f"next trial @ {levels[level_i]:.2f}   ")
                sys.stdout.flush()

            time.sleep(max(0.0, period - (time.time() - loop_t)))
    finally:
        release()
        sfile.close()
        if active:
            active.note = "interrupted"
            trials.append(analyse(active))
        with open(trials_path, "w") as f:
            json.dump([{k: v for k, v in asdict(t).items() if k != "samples"}
                       for t in trials], f, indent=2)
        # Keep the full per-trial traces too -- the summary numbers are only as
        # trustworthy as the curves behind them.
        with open(os.path.join(out_dir, "trial_traces.json"), "w") as f:
            json.dump([asdict(t) for t in trials], f, indent=2)
        if cart:
            if args.steer and cart.steering:
                try:
                    cart.steering.set_angle(0.0)
                except Exception:
                    pass
            cart.close()
        pad.close()
        print("\n\n" + summarise(trials))
        print(f"\nwrote {trials_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
