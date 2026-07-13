"""
cartlib.rpipeline — build and run the cart's Retriever graph.

    from cartlib.rpipeline import run_drive
    run_drive(waypoints, FollowConfig(max_speed_mph=3.0), armed=False)

Graph (edges are field maps; ``<--`` are the feedback edges):

    GpsSourceFlow @Rate(10) ──lat,lon,fix,speed,ts──┐
                                                    ▼
    SteeringFlow  <──steer_deg,steer_enable── FollowerFlow @Rate(15)
         └──────steering_actual_deg───────────────▶ │
    PedalFlow     <──gas,brake,phase──────────────  │
         └──────estop────────────────────────────▶  │
                                                    ▼
                                            TelemetryFlow ──UDP──▶ parent

Backend: ``multiprocessing``. This is not a preference — it is the only backend
that honours ``Rate`` in wall-clock time. The ``in-process`` backend is a
debug/replay surface that spins the graph as fast as it can (its own source
says "Intentionally run at logical-step / simulation speed"), which on real
hardware would mean thousands of serial writes per second instead of 15.

Stopping the pipeline is itself the emergency stop: the pedal worker dies, its
heartbeat stops, and the Arduino trips FAILSAFE in firmware within 300 ms —
gas released, brake slammed. That path does not depend on any Python here
still being responsive.
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from dataclasses import asdict
from typing import Callable, List, Optional

from retriever import Latest, Pipeline, Rate, Trigger

from . import config
from .follow import FollowConfig
from .rflows import (
    DEFAULT_TELEMETRY_PORT,
    FollowerFlow,
    GpsSourceFlow,
    PedalFlow,
    SteeringFlow,
    TelemetryFlow,
)

LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "last_drive.json")


def build_pipeline(
    waypoints: List[tuple],
    cfg: FollowConfig,
    *,
    armed: bool = False,
    telemetry_port: int = DEFAULT_TELEMETRY_PORT,
    ntrip_provider: Optional[str] = None,
    name: str = "cart_drive",
) -> Pipeline:
    """Wire the graph. ``armed=False`` omits the actuator flows entirely, so
    dry-run never opens the ODrive or Arduino at all."""
    if len(waypoints) < 2:
        raise ValueError(f"need >=2 waypoints, got {len(waypoints)}")

    gas_cap = config.effective_gas_cap(cfg.gas_cap)

    pipe = Pipeline(name)
    with pipe:
        gps = GpsSourceFlow(ntrip_provider=ntrip_provider) @ Rate(hz=10.0)
        follower = FollowerFlow(
            path=[list(p) for p in waypoints],
            cfg=asdict(cfg),
            armed=armed,
        ) @ Rate(hz=cfg.rate_hz)
        telemetry = TelemetryFlow(port=telemetry_port) @ Trigger("phase")

        pipe.connect(gps, follower, sync=Latest(), map={
            "lat": "lat", "lon": "lon", "fix_type": "fix_type",
            "fix_code": "fix_code", "speed_mph": "speed_mph", "ts": "ts",
        })
        pipe.connect(follower, telemetry, sync=Latest())

        if armed:
            steering = SteeringFlow() @ Trigger("steer_deg")
            pedals = PedalFlow(
                gas_cap=gas_cap, arrival_brake=cfg.arrival_brake,
            ) @ Trigger("gas")

            pipe.connect(follower, steering, sync=Latest(), map={
                "steer_deg": "steer_deg", "steer_enable": "steer_enable",
                "phase": "phase",
            })
            pipe.connect(follower, pedals, sync=Latest(), map={
                "gas": "gas", "brake": "brake", "phase": "phase",
            })
            # Feedback: measured column angle and e-stop back into the law.
            pipe.connect(steering, follower, sync=Latest(), map={
                "steering_actual_deg": "steering_actual_deg",
                "steering_target_deg": "steering_target_deg",
            })
            pipe.connect(pedals, follower, sync=Latest(), map={"estop": "estop"})

    return pipe


class TelemetryListener:
    """Receives the UDP telemetry the graph emits, in the parent process."""

    def __init__(self, port: int = DEFAULT_TELEMETRY_PORT,
                 on_step: Optional[Callable[[dict], None]] = None):
        self.port = port
        self.on_step = on_step
        self.steps: List[dict] = []
        self.last: dict = {}
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "TelemetryListener":
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", self.port))
        self._sock.settimeout(0.2)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                tele = json.loads(self._sock.recv(8192).decode())
            except socket.timeout:
                continue
            except (OSError, ValueError):
                continue
            self.last = tele
            self.steps.append(tele)
            if self.on_step:
                try:
                    self.on_step(tele)
                except Exception:
                    pass

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass

    @property
    def phase(self) -> str:
        return self.last.get("phase", "init")


def write_drive_log(listener: TelemetryListener, cfg: FollowConfig,
                    waypoints: List[tuple], reason: str,
                    path: str = LOG_PATH) -> None:
    """Same shape as follow.py's log, so the tuning tools keep working."""
    try:
        with open(path, "w") as f:
            json.dump({
                "runtime": "retriever",
                "phase": listener.phase,
                "reason": reason,
                "config": asdict(cfg),
                "waypoints": [list(p) for p in waypoints],
                "steps": listener.steps,
            }, f)
    except Exception as e:
        print(f"[log] failed to write {path}: {e}", flush=True)


def run_drive(
    waypoints: List[tuple],
    cfg: FollowConfig,
    *,
    armed: bool = False,
    duration: Optional[float] = None,
    telemetry_port: int = DEFAULT_TELEMETRY_PORT,
    ntrip_provider: Optional[str] = None,
    on_step: Optional[Callable[[dict], None]] = None,
    log_path: Optional[str] = LOG_PATH,
) -> str:
    """Run one drive to completion. Returns the terminating phase.

    Blocks until the follower reaches done/abort, ``duration`` elapses, or
    Ctrl-C. Teardown (cut gas, park brake, idle steering) happens in each
    flow's finalize() inside its own worker process.
    """
    listener = TelemetryListener(port=telemetry_port, on_step=on_step).start()
    pipe = build_pipeline(
        waypoints, cfg, armed=armed,
        telemetry_port=telemetry_port, ntrip_provider=ntrip_provider,
    )

    # The graph has no "I'm finished" signal of its own, so the parent watches
    # the telemetry for a terminal phase and tears the pipeline down.
    done = threading.Event()

    def watch() -> None:
        while not done.wait(0.05):
            if listener.phase in ("done", "abort"):
                return

    watcher = threading.Thread(target=watch, daemon=True)
    watcher.start()

    reason = ""
    try:
        pipe.run(backend="multiprocessing", duration=duration, blocking=False)
        t0 = time.time()
        while True:
            if listener.phase in ("done", "abort"):
                reason = listener.last.get("reason", "")
                break
            if duration is not None and (time.time() - t0) > duration:
                reason = "duration elapsed"
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        done.set()
        # Tearing the graph down runs finalize() in every worker: gas to zero,
        # park brake if we arrived, motor idled. If a worker is wedged and never
        # gets there, its death stops the heartbeat and the Arduino firmware
        # brakes for us.
        try:
            pipe.reset()
        except Exception:
            pass
        time.sleep(0.3)   # let finalize() land before we stop listening
        if log_path:
            write_drive_log(listener, cfg, waypoints, reason, path=log_path)
        listener.stop()

    phase = listener.phase
    print(f"\n[drive] {phase}: {reason or '—'}  ({len(listener.steps)} steps)")
    return phase
