#!/usr/bin/env python3
"""Drive with the normal PS5 controls and record RTK stopping distances.

The running FollowRTK cart server remains the only process that opens the
ODrive, pedal Arduino, and GPS. This tool sends short-lived manual commands to
that server and uses its RTK position stream for speed/distance measurement.

Controls:
  Left stick / R2 / L2   steer, gas, brake
  D-pad up/down          select manual/coast/gentle/medium/full stop
  Cross (X)              start the selected stop at the current speed
  Circle                 toggle full-brake soft e-stop
  Options or Esc/Q       stop and quit

LIVE CART: use a clear, level test area, a spotter, and the hardware e-stop.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from websockets.sync.client import connect


DEFAULT_DRIVER = Path("/home/caddy/PRODUCTION/scripts/ps5_drive.py")
DEFAULT_OUT_ROOT = Path(__file__).resolve().parents[1] / "stop_tests"
MPS_PER_MPH = 0.44704
MPH_PER_MPS = 2.2369363
EARTH_M = 6_371_000.0
STOP_MPH = 0.25
STOP_HOLD_S = 0.6
BUTTON_CROSS = 1
BUTTON_OPTIONS = 6
BUTTON_DPAD_UP = 11
BUTTON_DPAD_DOWN = 12


def load_ps5_driver(path: Path):
    if not path.is_file():
        raise RuntimeError(f"production PS5 driver not found: {path}")
    spec = importlib.util.spec_from_file_location("production_ps5_drive", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import production PS5 driver: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_M * math.asin(min(1.0, math.sqrt(h)))


class MotionTracker:
    """RTK speed from a 0.5 s least-squares position window."""

    def __init__(self, window_s: float = 0.5):
        self.window_s = window_s
        self.points: deque[tuple[float, float]] = deque()  # timestamp, cumulative metres
        self.last_position: tuple[float, float] | None = None
        self.last_fix: dict | None = None
        self.cumulative_m = 0.0
        self.lock = threading.Lock()

    def add(self, fix: dict) -> None:
        try:
            ts = float(fix["ts"])
            position = (float(fix["lat"]), float(fix["lon"]))
        except (KeyError, TypeError, ValueError):
            return
        with self.lock:
            if self.last_fix and ts <= float(self.last_fix["ts"]):
                return
            if self.last_position is not None:
                self.cumulative_m += distance_m(self.last_position, position)
            self.last_position = position
            self.last_fix = dict(fix)
            self.points.append((ts, self.cumulative_m))
            while len(self.points) > 2 and self.points[0][0] < ts - self.window_s:
                self.points.popleft()

    def snapshot(self) -> tuple[float | None, dict | None]:
        with self.lock:
            fix = dict(self.last_fix) if self.last_fix else None
            points = list(self.points)
        if fix is None or time.time() - float(fix["ts"]) > 1.0 or len(points) < 2:
            return None, fix
        t0 = points[0][0]
        times = [t - t0 for t, _ in points]
        distances = [d for _, d in points]
        mean_t = sum(times) / len(times)
        mean_d = sum(distances) / len(distances)
        denominator = sum((t - mean_t) ** 2 for t in times)
        if denominator <= 1e-9:
            return None, fix
        speed_mps = sum((t - mean_t) * (d - mean_d) for t, d in zip(times, distances)) / denominator
        return max(0.0, speed_mps) * MPH_PER_MPS, fix


class CartServer:
    def __init__(self, uri: str):
        self.ws = connect(uri, open_timeout=3)
        self.motion = MotionTracker()
        self.error: str | None = None
        self.closed = False
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _receive(self) -> None:
        try:
            for raw in self.ws:
                msg = json.loads(raw)
                if msg.get("type") == "position":
                    self.motion.add(msg.get("data", {}))
                elif msg.get("type") == "manual_ack" and not msg.get("data", {}).get("ok"):
                    self.error = msg.get("data", {}).get("reason", "manual command rejected")
        except Exception as exc:
            if not self.closed:
                self.error = f"cart server connection lost: {exc}"

    def command(self, gas: float, brake: float, steer_deg: float) -> None:
        self.ws.send(json.dumps({
            "type": "manual", "gas": gas, "brake": brake, "steer_deg": steer_deg,
        }))

    def stop(self, emergency: bool) -> None:
        try:
            self.ws.send(json.dumps({"type": "stop", "emergency": emergency}))
        except Exception:
            pass

    def close(self) -> None:
        self.closed = True
        self.ws.close()
        self.thread.join(timeout=1.0)


@dataclass(frozen=True)
class StopMode:
    name: str
    brake: float | None


@dataclass
class ActiveTest:
    number: int
    mode: StopMode
    started_wall: float
    started_mono: float
    entry_speed_mph: float
    distance_m: float = 0.0
    peak_brake: float = 0.0
    last_mono: float = 0.0
    last_speed_mph: float = 0.0
    below_stop_since: float | None = None


def parse_modes(text: str, brake_max: float) -> list[StopMode]:
    modes = []
    for item in text.split(","):
        name, sep, raw = item.strip().partition(":")
        if not name:
            continue
        brake = None if not sep or raw.strip().lower() == "manual" else float(raw)
        if brake is not None and not 0.0 <= brake <= brake_max:
            raise ValueError(f"{name} brake {brake} is outside 0..{brake_max}")
        modes.append(StopMode(name, brake))
    if not modes:
        raise ValueError("at least one stop mode is required")
    return modes


def save_result(path: Path, test: ActiveTest, duration_s: float) -> None:
    fields = ["test", "timestamp", "stop_type", "brake_command", "entry_speed_mph",
              "stopping_distance_m", "stop_time_s", "peak_brake_command"]
    new_file = not path.exists()
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if new_file:
            writer.writeheader()
        writer.writerow({
            "test": test.number,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(test.started_wall)),
            "stop_type": test.mode.name,
            "brake_command": "manual" if test.mode.brake is None else f"{test.mode.brake:.3f}",
            "entry_speed_mph": f"{test.entry_speed_mph:.3f}",
            "stopping_distance_m": f"{test.distance_m:.3f}",
            "stop_time_s": f"{duration_s:.3f}",
            "peak_brake_command": f"{test.peak_brake:.3f}",
        })
        f.flush()
        os.fsync(f.fileno())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--driver", type=Path, default=Path(os.getenv("FOLLOWRTK_PS5_DRIVER", DEFAULT_DRIVER)))
    parser.add_argument("--server", default="ws://localhost:8765")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="read controls/GPS but send no cart commands")
    parser.add_argument("--stops", default="manual:manual,coast:0,gentle:0.15,medium:0.30,full:0.45")
    args = parser.parse_args()

    ps5 = load_ps5_driver(args.driver.resolve())
    try:
        modes = parse_modes(args.stops, ps5.BRAKE_POT_MAX)
    except ValueError as exc:
        parser.error(str(exc))

    out_dir = args.out or DEFAULT_OUT_ROOT / time.strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    results_path, samples_path = out_dir / "stops.csv", out_dir / "samples.jsonl"
    gas_cap = ps5.effective_gas_cap(ps5.PS5_GAS_LIMIT)
    cart = CartServer(args.server)
    joystick = ps5.init_controller(args.index)
    ps5.pygame.display.set_mode((560, 120))
    clock = ps5.pygame.time.Clock()

    selected = completed = 0
    active: ActiveTest | None = None
    circle_estop = panic = False
    steer_target = previous_lx = 0.0
    previous_mono = time.monotonic()
    last_status = 0.0
    running = True

    print(f"\nLogging to {out_dir}")
    print("D-pad selects; Cross starts; Circle e-stop; Options quits.")
    try:
        with samples_path.open("a", buffering=1) as samples:
            while running:
                for event in ps5.pygame.event.get():
                    if event.type == ps5.pygame.QUIT:
                        running = False
                    elif event.type == ps5.pygame.KEYDOWN and event.key in (ps5.pygame.K_ESCAPE, ps5.pygame.K_q):
                        running = False
                    elif event.type == ps5.pygame.JOYDEVICEREMOVED:
                        panic, running = True, False
                    elif event.type == ps5.pygame.JOYHATMOTION and active is None and event.value[1]:
                        selected = (selected + (1 if event.value[1] > 0 else -1)) % len(modes)
                        print(f"\nSelected: {modes[selected].name}")
                    elif event.type == ps5.pygame.JOYBUTTONDOWN:
                        if event.button == BUTTON_OPTIONS:
                            running = False
                        elif event.button in (BUTTON_DPAD_UP, BUTTON_DPAD_DOWN) and active is None:
                            selected = (selected + (1 if event.button == BUTTON_DPAD_UP else -1)) % len(modes)
                            print(f"\nSelected: {modes[selected].name}")
                        elif event.button == ps5.BUTTON_CIRCLE:
                            circle_estop = not circle_estop
                            print(f"\nE-stop {'ENGAGED' if circle_estop else 'released'}")
                            if circle_estop:
                                active = None
                        elif event.button == BUTTON_CROSS and active is None:
                            speed, _ = cart.motion.snapshot()
                            if speed is None:
                                print("\nCannot start: waiting for fresh RTK speed")
                            elif speed < 0.8:
                                print(f"\nCannot start: only {speed:.2f} mph")
                            else:
                                active = ActiveTest(completed + 1, modes[selected], time.time(),
                                                    time.monotonic(), speed, last_mono=time.monotonic(),
                                                    last_speed_mph=speed)
                                print(f"\n>> Test {active.number}: {active.mode.name} from {speed:.2f} mph")

                alive, reason = ps5.controller_alive(joystick)
                if not alive or cart.error:
                    print(f"\nFAULT: {cart.error or reason}")
                    panic = True
                    break

                now = time.monotonic()
                dt = now - previous_mono
                previous_mono = now
                if dt <= 0.0 or dt > 0.25:
                    dt = 1.0 / ps5.CONTROL_HZ
                lx = ps5.apply_deadzone(joystick.get_axis(ps5.AXIS_LEFT_X), ps5.STEER_STICK_DEADZONE)
                l2 = ps5.read_trigger(joystick, ps5.AXIS_L2, ps5.TRIGGER_MAX_L2)
                r2 = ps5.read_trigger(joystick, ps5.AXIS_R2, ps5.TRIGGER_MAX_R2)
                speed, fix = cart.motion.snapshot()
                steer_speed = speed if speed is not None else max(0.0, (r2 - l2) * 20.0)
                scale = ps5.steering_speed_scale(steer_speed)
                if lx:
                    steer_target = ps5.clamp(
                        steer_target + lx * ps5.STEER_INTEGRATE_RATE_DPS * scale * dt,
                        -ps5.PS5_STEERING_MAX_DEG, ps5.PS5_STEERING_MAX_DEG)
                else:
                    speed_t = ps5.clamp(steer_speed / ps5.STEER_RETURN_REF_MPH, 0.0, 1.0)
                    tau = ps5.STEER_RETURN_TAU_REST_S * (1 - speed_t) + ps5.STEER_RETURN_TAU_FAST_S * speed_t
                    steer_target *= math.exp(-dt / tau)

                gas, brake = r2 * gas_cap, l2 * ps5.BRAKE_POT_MAX
                if active:
                    gas = 0.0
                    if active.mode.brake is not None:
                        brake = active.mode.brake
                if circle_estop:
                    gas, brake = 0.0, ps5.BRAKE_POT_MAX
                if not args.dry_run:
                    cart.command(gas, brake, steer_target)

                samples.write(json.dumps({"ts": time.time(), "speed_mph": speed,
                    "gas": gas, "brake": brake, "steer_deg": steer_target,
                    "fix": fix, "test": active.number if active else None}) + "\n")

                if active and speed is not None:
                    sample_dt = max(0.0, now - active.last_mono)
                    active.distance_m += 0.5 * (active.last_speed_mph + speed) * MPS_PER_MPH * sample_dt
                    active.last_mono, active.last_speed_mph = now, speed
                    active.peak_brake = max(active.peak_brake, brake)
                    active.below_stop_since = (active.below_stop_since or now) if speed < STOP_MPH else None
                    if active.below_stop_since and now - active.below_stop_since >= STOP_HOLD_S:
                        duration = active.below_stop_since - active.started_mono
                        save_result(results_path, active, duration)
                        print(f"\n<< {active.distance_m:.2f} m, {duration:.2f} s, entry {active.entry_speed_mph:.2f} mph")
                        completed += 1
                        active = None

                if now - last_status >= 0.25:
                    last_status = now
                    speed_text = f"{speed:5.2f} mph" if speed is not None else "NO RTK SPEED"
                    next_text = active.mode.name if active else modes[selected].name
                    sys.stdout.write(f"\r{speed_text} | {'test' if active else 'next'}: {next_text:<10}")
                    sys.stdout.flush()
                previous_lx = lx
                clock.tick(ps5.CONTROL_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        cart.stop(emergency=panic or circle_estop)
        cart.close()
        ps5.pygame.quit()
        print(f"\nResults: {results_path}")
    return 1 if panic else 0


if __name__ == "__main__":
    raise SystemExit(main())
