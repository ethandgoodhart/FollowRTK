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
_ntrip = None  # NtripClient when corrections are running; lets the UI switch source
# PerceptionService when --percept is on, else None. Every drive reads it, so
# it is deliberately a module global rather than threaded through the UI
# commands: a camera and a model take seconds to load and must not be opened
# per drive. None means "no perception", which the follower treats as "no
# opinion on speed" -- exactly the behaviour today.
_percept = None
_default_max_speed = 0.12
_default_max_speed_mph = 4.0

MAX_UI_SPEED_MPH = 20.0
# Cross-track recovery usually wants a gentle turn radius. The default follower
# caps near normal driving angles, but the slider is allowed all the way to the
# rack's mechanical limit (±320° column) so tight maneuvers remain possible.
# Above ~110° the cart can carve tight enough to overshoot the line and weave.
MAX_UI_STEER_DEG = 320.0

_drive_thread: threading.Thread | None = None
_stop_event: threading.Event | None = None
_active_follower: PathFollower | None = None
_drive_lock = threading.Lock()

# Duplicate-drive guard: if the identical path is already being driven and the
# drive started within this window, a second "drive" is ignored instead of
# stopping and restarting the cart (the start/1s/restart jerk). A genuine
# re-drive (after a stop, or a different route) is never blocked.
_active_drive_key: tuple | None = None
_active_drive_t: float = 0.0
DUP_DRIVE_WINDOW_S = 4.0

# Manual (PS5/tool) commands share this process so it remains the sole owner of
# the serial devices. A stale client gets a full-brake stop within 300 ms.
_manual_active = False
_manual_owner = None
_manual_last_cmd = 0.0
MANUAL_TIMEOUT_S = 0.30


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
async def _send_to_client(c, msg: str) -> None:
    """Send to one client, but never let a slow/stuck one stall the broadcast.
    A backgrounded or laggy browser tab fills its TCP send buffer; an unbounded
    ``await c.send()`` would then block the GPS pump for EVERY client, freezing
    the whole map feed after ~a minute. So we cap each send and drop clients that
    can't keep up — the UI auto-reconnects, so the feed self-heals."""
    try:
        await asyncio.wait_for(c.send(msg), timeout=1.0)
    except Exception:
        _clients.discard(c)
        try:
            await c.close()
        except Exception:
            pass


async def _broadcast(obj: dict) -> None:
    if _clients:
        msg = json.dumps(obj)
        # Iterate a snapshot: _send_to_client may discard from _clients.
        await asyncio.gather(*[_send_to_client(c, msg) for c in list(_clients)],
                             return_exceptions=True)


def _broadcast_threadsafe(obj: dict) -> None:
    if _loop is not None:
        asyncio.run_coroutine_threadsafe(_broadcast(obj), _loop)


def broadcast_perception(payload: dict) -> None:
    """Push one perception frame to the UI minimap.

    Public because the perception loop runs in its own thread (it is paced by
    the camera, not by the GPS pump) and needs a thread-safe way in. Building
    the payload is cartlib.percept.telemetry's job; this only posts it.
    """
    _broadcast_threadsafe({"type": "perception", "data": payload})


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
        steer_trim_deg=_float_setting(msg, "steer_trim_deg", FollowConfig.steer_trim_deg, -40.0, 40.0),
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
        cfg.steer_trim_deg = _float_setting(msg, "steer_trim_deg", cfg.steer_trim_deg, -40.0, 40.0)
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
        "steer_trim_deg": cfg.steer_trim_deg,
        "xtrack_gain": cfg.xtrack_gain,
        "heading_gain": cfg.heading_gain,
        "max_steer_deg": cfg.max_steer_deg,
        "turn_slowdown": cfg.turn_slowdown,
    }


def start_drive(path: list, max_speed_mph: float, armed: bool, msg: dict) -> dict:
    """Begin following ``path``. Returns a status dict echoed back to the UI."""
    global _drive_thread, _stop_event, _active_follower
    global _active_drive_key, _active_drive_t
    with _drive_lock:
        waypoints = _coords_from_path(path)
        if len(waypoints) < 2:
            return {"ok": False, "reason": "need >=2 waypoints"}

        # Ignore an identical drive that lands right on top of one already
        # running — a duplicate command must not tear down and restart the cart.
        key = tuple(waypoints)
        if (_active_follower is not None and _active_drive_key == key
                and time.time() - _active_drive_t < DUP_DRIVE_WINDOW_S):
            print("[server] ignoring duplicate drive (same path, already driving)")
            return {"ok": True, "duplicate": True,
                    "note": "duplicate drive ignored (already driving this route)"}

        _stop_drive_locked(emergency=False)

        # Can only actuate if the actuators are actually present.
        can_drive = bool(_cart and _cart.pedals and _cart.steering)
        armed = armed and can_drive

        cfg = _follow_config_from_msg(msg, max_speed_mph)
        if armed:
            _cart.arm()
            if not _cart.steering.enable():
                return {"ok": False, "reason": "steering failed to enter closed-loop"}

        follower = PathFollower(_cart, waypoints, cfg, armed=armed,
                                percept=_percept)
        _active_follower = follower
        _active_drive_key = key
        _active_drive_t = time.time()
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


def _stop_manual_locked(emergency: bool) -> None:
    global _manual_active, _manual_owner
    if not _manual_active:
        return
    if _cart:
        if emergency:
            _cart.emergency_brake()
        else:
            _cart.stop()
        if _cart.steering:
            _cart.steering.idle()
    _manual_active = False
    _manual_owner = None


def manual_command(msg: dict, owner) -> dict:
    """Apply one short-lived manual control command from a local client."""
    global _manual_active, _manual_owner, _manual_last_cmd
    with _drive_lock:
        if not (_cart and _cart.pedals and _cart.steering):
            return {"ok": False, "reason": "cart actuators unavailable"}
        if _manual_owner is not None and _manual_owner is not owner:
            return {"ok": False, "reason": "manual control already in use"}
        if _active_follower is not None:
            _stop_drive_locked(emergency=False)
        if not _manual_active:
            _cart.arm()
            if not _cart.steering.enable():
                return {"ok": False, "reason": "steering failed to enter closed-loop"}
            _manual_active = True
            _manual_owner = owner

        gas = _float_setting(msg, "gas", 0.0, 0.0, config.GLOBAL_SPEED_LIMIT)
        brake = _float_setting(msg, "brake", 0.0, 0.0, config.BRAKE_POT_MAX)
        steer = _float_setting(
            msg, "steer_deg", 0.0, config.STEERING_MIN_DEG, config.STEERING_MAX_DEG)
        if brake > 0.02:
            gas = 0.0
        _cart.pedals.set_gas(gas)
        _cart.pedals.set_brake(brake)
        _cart.steering.set_angle(steer)
        _manual_last_cmd = time.monotonic()
        return {"ok": True}


# --- websocket handling -----------------------------------------------------
async def _ws_handler(ws):
    _clients.add(ws)
    try:
        if _cart and _cart.gps and _cart.gps.latest:
            await ws.send(json.dumps({"type": "position", "data": _to_web(_cart.gps.latest)}))
        if _ntrip is not None:
            await ws.send(json.dumps({"type": "ntrip", "data": _ntrip.status()}))
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
                with _drive_lock:
                    _stop_manual_locked(emergency=emergency)
                await ws.send(json.dumps({"type": "drive_ack",
                                          "data": {"ok": True, "stopped": True,
                                                   "emergency": emergency}}))
            elif t == "manual":
                status = manual_command(msg, ws)
                if not status.get("ok"):
                    await ws.send(json.dumps({"type": "manual_ack", "data": status}))
            elif t == "ntrip":
                # Switch the live correction source; tell everyone the new state.
                if _ntrip is not None:
                    provider = str(msg.get("provider", ""))
                    ok = _ntrip.switch(provider)
                    status = _ntrip.status()
                    await ws.send(json.dumps({"type": "ntrip_ack",
                                              "data": {"ok": ok, "requested": provider, **status}}))
                    await _broadcast({"type": "ntrip", "data": status})
                else:
                    await ws.send(json.dumps({"type": "ntrip_ack",
                                              "data": {"ok": False, "reason": "NTRIP is not running"}}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        with _drive_lock:
            if _manual_owner is ws:
                _stop_manual_locked(emergency=True)
        _clients.discard(ws)


async def _gps_pump() -> None:
    last_ts = None
    last_ntrip = None
    last_ntrip_t = 0.0
    while True:
        with _drive_lock:
            if (_manual_active
                    and time.monotonic() - _manual_last_cmd > MANUAL_TIMEOUT_S):
                print("[server] manual command timed out -- emergency braking")
                _stop_manual_locked(emergency=True)
        fix = _cart.gps.latest if (_cart and _cart.gps) else None
        if fix and fix.get("ts") != last_ts:
            last_ts = fix["ts"]
            await _broadcast({"type": "position", "data": _to_web(fix)})
        # Push NTRIP source/connection status ~1 Hz (and immediately on change)
        # so the UI toggle reflects whether the chosen caster is actually flowing.
        if _ntrip is not None:
            now = _loop.time() if _loop else 0.0
            status = _ntrip.status()
            if status != last_ntrip or now - last_ntrip_t >= 1.0:
                last_ntrip, last_ntrip_t = status, now
                await _broadcast({"type": "ntrip", "data": status})
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
    if cmd in ("start", "destination"):
        dest = data.get("destination")
        if not dest:
            return {"ok": False, "reason": "no destination set"}
        # Don't drive a raw straight line here. Push the destination to the
        # drivelive UI: it drops the pin, plans the purple route along the lane
        # network, shows it on screen, and (for "start") drives THAT computed
        # route back to us via the normal "drive" WebSocket command. So the cart
        # follows exactly the line the operator sees, not a naive point-to-point.
        autostart = cmd == "start"
        if not _clients:
            return {"ok": False,
                    "reason": "no drivelive UI connected — open the map so the route can be planned & shown"}
        _broadcast_threadsafe({"type": "remote_route",
                               "data": {"lat": dest[0], "lon": dest[1], "autostart": autostart}})
        action = "start" if autostart else "set destination"
        print(f"[server] remote {action} -> pushed ({dest[0]:.6f},{dest[1]:.6f}) to UI for routing "
              f"({len(_clients)} client(s))")
        return {"ok": True, "dispatched": True, "autostart": autostart,
                "clients": len(_clients),
                "note": "routing through drivelive UI (purple route)"}

    elif cmd == "stop":
        # A remote stop/pause must bring the cart to a real halt: fully apply the
        # brake and hold it, not just release the accelerator and coast. Force
        # the emergency (full-brake) stop path regardless of the incoming flag.
        stop_drive(emergency=True)
        return {"ok": True, "stopped": True, "emergency": True}

    elif cmd == "status":
        with _drive_lock:
            driving = _active_follower is not None
            phase = _active_follower.state.phase if _active_follower else None
        return {"ok": True, "driving": driving, "phase": phase}

    return {"ok": False, "reason": f"unknown command: {cmd}"}


def _start_perception(args):
    """Open the camera and the detector, or return None and say why.

    A perception failure must never stop the cart from being drivable. The
    system worked without a camera yesterday and it works without one now; the
    only thing lost is the speed cap, and the follower already treats a missing
    cap as "no opinion" rather than as zero. So every failure here is a printed
    warning and a None, not an exception into the server's startup path.
    """
    from .percept import calib as calib_mod
    from .percept.policy import PolicyConfig, reflex_protected_speed_mph
    from .percept.service import PerceptionService

    try:
        cam, calib = calib_mod.load_camera(args.percept_calib)
    except Exception as e:
        print(f"[percept] calibration unusable, perception disabled: {e}")
        return None
    print(calib_mod.describe(cam, calib))

    live = args.percept_live
    if live and not calib_mod.mount_measured(calib):
        # Refusing here is the whole point of the measured flag. Live mode is
        # the mode where a wrong range becomes a wrong speed on a real cart.
        print("[percept] --percept-live refused: the mount geometry has not "
              "been measured, so the ranges it would brake on are guesses. "
              "Run tools/percept_ground_calib.py --write first. Falling back "
              "to shadow mode.")
        live = False

    cfg = PolicyConfig()
    if live:
        guaranteed = reflex_protected_speed_mph(cfg)
        if cfg.max_speed_mph > guaranteed + 0.05:
            print(f"[percept] NOTE: max_speed_mph is {cfg.max_speed_mph:.1f} "
                  f"but at {cfg.decel_emergency_ms2:.1f} m/s2 the reflex layer "
                  f"only guarantees a stop from {guaranteed:.1f} mph. Above "
                  f"that speed the cart depends on seeing an obstacle before "
                  f"it enters the blind zone.")

    try:
        svc = PerceptionService(
            cam=cam, cfg=cfg, weights=args.percept_model,
            imgsz=args.percept_imgsz, conf=args.percept_conf,
            shadow=not live, publish=broadcast_perception,
            cap_width=cam.width, cap_height=cam.height,
            cap_fps=calib_mod.capture_fps(calib),
            cam_device=args.percept_device)
        print(f"[percept] loading {args.percept_model} @ {args.percept_imgsz} "
              f"(this takes a few seconds) ...")
        svc.start()
    except Exception as e:
        print(f"[percept] could not start, perception disabled: {e}")
        return None

    mode = "LIVE (capping speed)" if live else "SHADOW (watching only)"
    print(f"[percept] running in {mode}")
    return svc


async def _main_async(args) -> None:
    global _loop, _cart, _ntrip, _percept
    global _default_max_speed, _default_max_speed_mph
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
        _ntrip = ntrip
        print(f"[server] NTRIP corrections started (source: {ntrip.status()['label']})")

    if args.percept:
        _percept = _start_perception(args)

    print(f"[server] WebSocket bridge on ws://localhost:{WS_PORT}")
    print("[server] open the drivelive UI, click Set Start/End, then Drive Route.")
    try:
        # ping_interval keeps dead peers detectable; a generous ping_timeout
        # avoids dropping a healthy-but-briefly-busy browser tab.
        async with websockets.serve(_ws_handler, "", WS_PORT,
                                    ping_interval=20, ping_timeout=60):
            await _gps_pump()
    finally:
        # Process shutdown should release throttle and disarm cleanly. Do not
        # command the emergency brake here; a normal Ctrl-C/script exit should
        # not floor the brake actuator.
        stop_drive(emergency=False)
        with _drive_lock:
            _stop_manual_locked(emergency=False)
        if _percept:
            # Release the V4L2 stream. Being killed with it open is what leaves
            # the camera wedged until somebody replugs the USB.
            _percept.stop()
        if ntrip:
            ntrip.stop()
        if _cart:
            _cart.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Cart <-> drivelive WebSocket bridge")
    ap.add_argument("--gps-only", action="store_true",
                    help="stream position only; don't open pedals/steering")
    # RTK corrections are ON by default now; pass --no-ntrip to disable.
    ap.add_argument("--ntrip", dest="ntrip", action="store_true", default=True,
                    help="feed RTK corrections (default: on)")
    ap.add_argument("--no-ntrip", dest="ntrip", action="store_false",
                    help="disable RTK corrections")
    ap.add_argument("--max-speed", type=float, default=0.12,
                    help="default normalized gas cap for legacy clients")
    # Perception. Off unless asked for, and watching-only unless asked twice:
    # --percept runs the camera and feeds the minimap, --percept-live is the
    # separate decision to let it touch the cart's speed.
    ap.add_argument("--percept", action="store_true",
                    help="run the pedestrian/vehicle perception stack")
    ap.add_argument("--percept-live", action="store_true",
                    help="let perception cap the speed (default: shadow only)")
    ap.add_argument("--percept-calib", default=None,
                    help="calibration JSON (default calibration/front_camera.json)")
    ap.add_argument("--percept-device", type=int, default=0,
                    help="/dev/videoN for the front camera")
    ap.add_argument("--percept-model", default="yolo11m.pt")
    ap.add_argument("--percept-imgsz", type=int, default=960)
    ap.add_argument("--percept-conf", type=float, default=0.35)
    args = ap.parse_args()
    if args.percept_live:
        args.percept = True
    try:
        asyncio.run(_main_async(args))
    except KeyboardInterrupt:
        print("\n[server] shutting down")


if __name__ == "__main__":
    main()
