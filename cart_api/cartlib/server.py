"""
cartlib.server — WebSocket bridge between the cart and the drivelive web UI.

It does two things over a single WebSocket on ws://localhost:8765 (the URL the
drivelive ``useGps`` hook already connects to):

  1. Streams the cart's live RTK position to the browser as
        {"type": "position", "data": {lat, lon, fix, ...}}
     so the cart shows up on the map (same shape lane_tracker.py emitted).

  2. Accepts drive commands from the browser's "Drive Route" button:
        {"type": "drive", "path": [{lat,lng}, ...], "max_speed": 0.12}
        {"type": "stop"}
     On "drive" it arms the cart and runs cartlib.follow.PathFollower along the
     purple route, broadcasting follow telemetry each control step:
        {"type": "follow",     "data": {phase, gas, steer_cmd, xtrack_m, ...}}
        {"type": "follow_end", "data": {phase, reason}}

SAFETY — "drive" arms and moves the cart immediately (per the operator's choice
in the web UI). Motion is hard-capped by ``max_speed`` (and the global
governor), and "stop" releases throttle/brake and idles steering. Keep a hand
on the hardware e-stop for emergency stops.

Run:
    python3 -m cartlib.server                 # full cart (gps + pedals + steering)
    python3 -m cartlib.server --gps-only      # just stream position to the map
    python3 -m cartlib.server --ntrip         # also feed RTK corrections
    python3 -m cartlib.server --max-speed 0.15
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import os
import threading
import time

import websockets

from . import config
from .cart import Cart
from .follow import PathFollower, FollowConfig
from .livepub import set_command_handler

WS_PORT = 8765

# Every "Drive Route" overwrites this with a full step-by-step trace of the
# drive (positions, commanded + actual steering, gas, cross-track error, ...)
# so a failed run can be replayed/analyzed afterwards.
_DRIVE_LOG = os.path.join(os.path.dirname(os.path.dirname(__file__)), "last_drive.json")

# --- shared state -----------------------------------------------------------
_loop: asyncio.AbstractEventLoop | None = None
_clients: set = set()
_cart: Cart | None = None
_default_max_speed = 0.12
_default_max_speed_mph = 4.0

MAX_UI_SPEED_MPH = 20.0
# Cross-track recovery usually wants a gentle turn radius, but the slider is
# allowed all the way to the rack's mechanical limit (±320° column) so tight
# maneuvers are possible. Above ~110° the cart can carve tight enough to
# overshoot the line and limit-cycle — that's on the operator to tune.
MAX_UI_STEER_DEG = 320.0

_drive_thread: threading.Thread | None = None
_stop_event: threading.Event | None = None
_active_follower: PathFollower | None = None
_drive_lock = threading.Lock()


# --- GPS fix -> web shape ---------------------------------------------------
def _to_web(fix: dict) -> dict:
    dt = datetime.datetime.fromtimestamp(fix["ts"])
    return {
        "lat": fix["lat"],
        "lon": fix["lon"],
        "fix": fix["fix_type"],
        "fix_code": fix["fix_code"],
        "sats": fix.get("sats", 0),
        "hdop": fix.get("hdop", 0.0),
        "alt": fix.get("alt", 0.0),
        "ts": fix["ts"],
        "datetime": dt.strftime("%Y-%m-%d %H:%M:%S"),
        "utc_time": dt.strftime("%H:%M:%S"),
    }


# --- broadcasting -----------------------------------------------------------
async def _broadcast(obj: dict) -> None:
    if _clients:
        msg = json.dumps(obj)
        await asyncio.gather(*[c.send(msg) for c in _clients], return_exceptions=True)


def _broadcast_threadsafe(obj: dict) -> None:
    if _loop is not None:
        asyncio.run_coroutine_threadsafe(_broadcast(obj), _loop)


# --- drive control ----------------------------------------------------------
def _coords_from_path(path: list) -> list:
    pts = []
    for p in path:
        if isinstance(p, dict):
            lat = p.get("lat")
            lon = p.get("lon", p.get("lng"))
        else:
            lat, lon = p[0], p[1]
        if lat is not None and lon is not None:
            pts.append((float(lat), float(lon)))
    return pts


def _drive_loop(follower: PathFollower, stop_event: threading.Event) -> None:
    period = 1.0 / follower.cfg.rate_hz
    steps: list = []
    t0 = time.time()
    try:
        while not stop_event.is_set():
            tele = follower.step()
            tele = dict(tele, t=round(time.time() - t0, 3))
            steps.append(tele)
            _broadcast_threadsafe({"type": "follow", "data": tele})
            if follower.state.phase in ("done", "abort"):
                break
            stop_event.wait(period)
    except Exception as e:  # never let the drive thread die silently
        _broadcast_threadsafe({"type": "follow_end",
                               "data": {"phase": "abort", "reason": f"error: {e}"}})
    finally:
        # Always cut the throttle. If we arrived under our own control, HOLD the
        # firm arrival brake so the cart parks at the destination instead of
        # coasting through it (the pedal heartbeat keeps the target latched); any
        # other exit (user stop / abort / error) releases both pedals.
        if follower.armed and _cart and _cart.pedals:
            _cart.pedals.set_gas(0.0)
            if follower.state.phase == "done":
                _cart.pedals.set_brake(follower.cfg.arrival_brake)
            else:
                _cart.pedals.stop()
        if follower.armed and _cart and _cart.steering:
            _cart.steering.idle()
        reason = follower.state.reason or ("stopped" if stop_event.is_set() else "")
        _write_drive_log(follower, steps, reason)
        _broadcast_threadsafe({"type": "follow_end",
                               "data": {"phase": follower.state.phase, "reason": reason}})


def _write_drive_log(follower: PathFollower, steps: list, reason: str) -> None:
    """Dump a full trace of the just-finished drive to last_drive.json."""
    try:
        log = {
            "saved_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "armed": follower.armed,
            "phase": follower.state.phase,
            "reason": reason,
            "config": _config_snapshot(follower.cfg),
            "path": [{"lat": lat, "lon": lon} for (lat, lon) in follower.path],
            "n_steps": len(steps),
            "steps": steps,
        }
        with open(_DRIVE_LOG, "w") as f:
            json.dump(log, f, indent=1)
        print(f"[server] drive trace ({len(steps)} steps) -> {_DRIVE_LOG}")
    except Exception as e:
        print(f"[server] failed to write drive log: {e}")


def _float_setting(msg: dict, key: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(msg.get(key, default))
    except (TypeError, ValueError):
        value = default
    return max(lo, min(value, hi))


def _gas_cap_from_mph(speed_mph: float) -> float:
    # Hard autonomy ceiling. The requested speed is the closed-loop SETPOINT
    # (the PI speed controller in follow.py hits it); the cap is only how far
    # gas may be pushed when hills/load need more than the open-loop guess.
    # Pinning the cap to gas_for_mph(speed) starved the controller — that's why
    # asking for 8 mph only produced ~2.
    return config.effective_gas_cap(config.FSD_GAS_LIMIT)


def _speed_mph_from_msg(msg: dict) -> float:
    if "max_speed_mph" in msg:
        return _float_setting(msg, "max_speed_mph", _default_max_speed_mph, 1.0, MAX_UI_SPEED_MPH)

    # Backward compatibility for older clients that sent normalized gas as
    # max_speed. The UI now sends mph explicitly.
    gas = _float_setting(msg, "max_speed", _default_max_speed, 0.0, config.GLOBAL_SPEED_LIMIT)
    return max(1.0, min(config.mph_from_gas(gas), MAX_UI_SPEED_MPH))


def _follow_config_from_msg(msg: dict, max_speed_mph: float) -> FollowConfig:
    gas_cap = _gas_cap_from_mph(max_speed_mph)
    return FollowConfig(
        gas_cap=gas_cap,
        max_speed_mph=max_speed_mph,
        live_speed_mph=_float_setting(msg, "current_speed_mph", 0.0, 0.0, MAX_UI_SPEED_MPH),
        lookahead_m=_float_setting(msg, "lookahead_m", FollowConfig.lookahead_m, 0.3, 4.0),
        steer_gain=_float_setting(msg, "steer_gain", FollowConfig.steer_gain, 0.5, 8.0),
        xtrack_gain=_float_setting(msg, "xtrack_gain", FollowConfig.xtrack_gain, 0.0, 5.0),
        heading_gain=_float_setting(msg, "heading_gain", FollowConfig.heading_gain, 0.0, 5.0),
        max_steer_deg=_float_setting(msg, "max_steer_deg", FollowConfig.max_steer_deg, 10.0, MAX_UI_STEER_DEG),
        turn_slowdown=_float_setting(msg, "turn_slowdown", 0.0, 0.0, 4.0),
    )


def tune_active_follower(msg: dict) -> dict:
    with _drive_lock:
        if _active_follower is None:
            return {"ok": False, "reason": "no active follower"}
        cfg = _active_follower.cfg
        cfg.live_speed_mph = _float_setting(msg, "current_speed_mph", cfg.live_speed_mph, 0.0, MAX_UI_SPEED_MPH)
        cfg.lookahead_m = _float_setting(msg, "lookahead_m", cfg.lookahead_m, 0.3, 4.0)
        cfg.steer_gain = _float_setting(msg, "steer_gain", cfg.steer_gain, 0.5, 8.0)
        cfg.xtrack_gain = _float_setting(msg, "xtrack_gain", cfg.xtrack_gain, 0.0, 5.0)
        cfg.heading_gain = _float_setting(msg, "heading_gain", cfg.heading_gain, 0.0, 5.0)
        cfg.max_steer_deg = _float_setting(msg, "max_steer_deg", cfg.max_steer_deg, 10.0, MAX_UI_STEER_DEG)
        cfg.turn_slowdown = _float_setting(msg, "turn_slowdown", cfg.turn_slowdown, 0.0, 4.0)
        return {"ok": True, "config": _config_snapshot(cfg)}


def _config_snapshot(cfg: FollowConfig) -> dict:
    return {
        "max_speed_mph": cfg.max_speed_mph,
        "live_speed_mph": cfg.live_speed_mph,
        "lookahead_m": cfg.lookahead_m,
        "steer_gain": cfg.steer_gain,
        "xtrack_gain": cfg.xtrack_gain,
        "heading_gain": cfg.heading_gain,
        "max_steer_deg": cfg.max_steer_deg,
        "turn_slowdown": cfg.turn_slowdown,
    }


def start_drive(path: list, max_speed_mph: float, armed: bool, msg: dict) -> dict:
    """Begin following ``path``. Returns a status dict echoed back to the UI."""
    global _drive_thread, _stop_event, _active_follower
    with _drive_lock:
        _stop_drive_locked(emergency=False)

        waypoints = _coords_from_path(path)
        if len(waypoints) < 2:
            return {"ok": False, "reason": "need >=2 waypoints"}

        # Can only actuate if the actuators are actually present.
        can_drive = bool(_cart and _cart.pedals and _cart.steering)
        armed = armed and can_drive

        cfg = _follow_config_from_msg(msg, max_speed_mph)
        if armed:
            _cart.arm()
            if not _cart.steering.enable():
                return {"ok": False, "reason": "steering failed to enter closed-loop"}

        follower = PathFollower(_cart, waypoints, cfg, armed=armed)
        _active_follower = follower
        _stop_event = threading.Event()
        _drive_thread = threading.Thread(
            target=_drive_loop, args=(follower, _stop_event), daemon=True)
        _drive_thread.start()
        return {"ok": True, "armed": armed, "waypoints": len(waypoints),
                "max_speed_mph": max_speed_mph,
                "gas_cap": cfg.gas_cap,
                "config": _config_snapshot(cfg),
                "note": "" if can_drive else "actuators absent -> dry-run preview"}


def _stop_drive_locked(emergency: bool) -> None:
    global _drive_thread, _stop_event, _active_follower
    # Emergency: slam the brake to full NOW, before tearing the drive thread
    # down, so a panic stop is felt immediately and not after the ~2 s join.
    if emergency and _cart:
        _cart.emergency_brake()
    if _stop_event is not None:
        _stop_event.set()
    if _drive_thread is not None:
        _drive_thread.join(timeout=2.0)
    _drive_thread = None
    _stop_event = None
    _active_follower = None
    if emergency and _cart:
        # Re-assert after the drive loop's own teardown ran, so full brake is
        # the final committed state (the loop may have released on its way out).
        _cart.emergency_brake()
        if _cart.steering:
            _cart.steering.idle()


def stop_drive(emergency: bool = False) -> None:
    with _drive_lock:
        _stop_drive_locked(emergency=emergency)


# --- websocket handling -----------------------------------------------------
async def _ws_handler(ws):
    _clients.add(ws)
    try:
        if _cart and _cart.gps and _cart.gps.latest:
            await ws.send(json.dumps({"type": "position", "data": _to_web(_cart.gps.latest)}))
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except (ValueError, TypeError):
                continue
            t = msg.get("type")
            if t == "drive":
                status = start_drive(
                    msg.get("path", []),
                    _speed_mph_from_msg(msg),
                    bool(msg.get("armed", True)),   # web UI arms immediately
                    msg,
                )
                await ws.send(json.dumps({"type": "drive_ack", "data": status}))
            elif t == "tune":
                status = tune_active_follower(msg)
                await ws.send(json.dumps({"type": "tune_ack", "data": status}))
            elif t == "stop":
                emergency = bool(msg.get("emergency", False))
                stop_drive(emergency=emergency)
                await ws.send(json.dumps({"type": "drive_ack",
                                          "data": {"ok": True, "stopped": True,
                                                   "emergency": emergency}}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        _clients.discard(ws)


async def _gps_pump() -> None:
    last_ts = None
    while True:
        fix = _cart.gps.latest if (_cart and _cart.gps) else None
        if fix and fix.get("ts") != last_ts:
            last_ts = fix["ts"]
            await _broadcast({"type": "position", "data": _to_web(fix)})
        await asyncio.sleep(0.01)


def _open_cart(gps_only: bool) -> Cart:
    """Open whatever hardware is present; never let one missing device kill
    the map feed."""
    cart = Cart(use_gps=True, use_pedals=not gps_only, use_steering=not gps_only)
    for name in ("gps", "pedals", "steering"):
        sub = getattr(cart, name)
        if sub is None:
            continue
        try:
            if name == "steering":
                sub.connect()
            else:
                sub.open()
        except Exception as e:
            print(f"[server] {name} unavailable ({e}); continuing without it")
            setattr(cart, name, None)
    return cart


def _handle_remote_command(cmd: str, data: dict) -> dict:
    """Handle commands from the Cloudflare tunnel (livepub HTTP endpoints)."""
    if cmd == "start":
        dest = data.get("destination")
        if not dest:
            return {"ok": False, "reason": "no destination set"}
        fix = _cart.gps.latest if (_cart and _cart.gps) else None
        if not fix:
            return {"ok": False, "reason": "no GPS fix"}
        path = [
            {"lat": fix["lat"], "lon": fix["lon"]},
            {"lat": dest[0], "lon": dest[1]},
        ]
        print(f"[server] remote start: ({fix['lat']:.6f},{fix['lon']:.6f}) -> ({dest[0]:.6f},{dest[1]:.6f})")
        return start_drive(path, _default_max_speed_mph, True, {})

    elif cmd == "stop":
        emergency = bool(data.get("emergency", False))
        stop_drive(emergency=emergency)
        return {"ok": True, "stopped": True, "emergency": emergency}

    elif cmd == "status":
        with _drive_lock:
            driving = _active_follower is not None
            phase = _active_follower.state.phase if _active_follower else None
        return {"ok": True, "driving": driving, "phase": phase}

    return {"ok": False, "reason": f"unknown command: {cmd}"}


async def _main_async(args) -> None:
    global _loop, _cart, _default_max_speed, _default_max_speed_mph
    _loop = asyncio.get_running_loop()
    _default_max_speed = args.max_speed
    _default_max_speed_mph = max(1.0, min(config.mph_from_gas(args.max_speed), MAX_UI_SPEED_MPH))

    _cart = _open_cart(gps_only=args.gps_only)
    have = [n for n in ("gps", "pedals", "steering") if getattr(_cart, n)]
    print(f"[server] cart subsystems online: {', '.join(have) or 'none'}")

    set_command_handler(_handle_remote_command)
    print("[server] remote command handler registered (tunnel API ready)")

    ntrip = None
    if args.ntrip and _cart.gps:
        from .ntrip import NtripClient
        ntrip = NtripClient(_cart.gps).start()
        print("[server] NTRIP corrections started")

    print(f"[server] WebSocket bridge on ws://localhost:{WS_PORT}")
    print("[server] open the drivelive UI, click Set Start/End, then Drive Route.")
    try:
        async with websockets.serve(_ws_handler, "", WS_PORT):
            await _gps_pump()
    finally:
        # Process shutdown should release throttle and disarm cleanly. Do not
        # command the emergency brake here; a normal Ctrl-C/script exit should
        # not floor the brake actuator.
        stop_drive(emergency=False)
        if ntrip:
            ntrip.stop()
        if _cart:
            _cart.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Cart <-> drivelive WebSocket bridge")
    ap.add_argument("--gps-only", action="store_true",
                    help="stream position only; don't open pedals/steering")
    ap.add_argument("--ntrip", action="store_true", help="feed RTK corrections")
    ap.add_argument("--max-speed", type=float, default=0.12,
                    help="default normalized gas cap for legacy clients")
    args = ap.parse_args()
    try:
        asyncio.run(_main_async(args))
    except KeyboardInterrupt:
        print("\n[server] shutting down")


if __name__ == "__main__":
    main()
