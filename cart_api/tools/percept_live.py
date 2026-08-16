#!/usr/bin/env python3
"""
percept_live.py — run perception on the real camera, drive nothing.

This is the first thing to run on the cart, and it should be run for a while
before anything is allowed to touch the throttle. It opens the front camera,
detects, ranges, tracks and evaluates the speed policy exactly as the live
system will, publishes to the drivelive minimap, and commands absolutely
nothing: no pedals, no steering, no ODrive. The cart can be pushed by hand or
left parked.

What to look for
----------------
  * Do the ranges match a tape measure? Stand someone at 5, 10 and 20 m. This
    is the single most important check, because every speed decision is
    downstream of it, and it is what tells you whether the calibration in
    --cam-* is right. A systematic error here reads as "the minimap is a bit
    off" and behaves as "the cart brakes two metres late".
  * Does a stationary person hold still on the minimap, or drift? Drift means
    the pose or the pitch is wrong.
  * How far out do people first appear, and does the disc shrink as they
    approach? It should: uncertainty is range-dependent.

GPS is used only for the pose. Without a fix the tracker cannot work in world
coordinates, so --no-gps substitutes a stationary pose, which is fine for
range checks on a parked cart and wrong for anything moving.

Usage
-----
    python3 tools/percept_live.py                    # publishes to the UI
    python3 tools/percept_live.py --no-gps           # parked, no fix needed
    python3 tools/percept_live.py --print            # numbers on the console
    python3 tools/percept_live.py --model yolo11s.pt --imgsz 640
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.percept import calib as calib_mod            # noqa: E402
from cartlib.percept.policy import PolicyConfig           # noqa: E402
from cartlib.percept.service import PerceptionService     # noqa: E402

WS_PORT = 8766          # not 8765: the live cart server owns that one


# ---------------------------------------------------------------------------
# A minimal websocket fan-out, so the minimap has something to connect to
# without needing the whole cart server running.
# ---------------------------------------------------------------------------
class Publisher:
    def __init__(self, port: int):
        self.port = port
        self.clients: set = set()
        self.loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self.on_command = None  # optional Callable[[dict], None]

    def start(self) -> "Publisher":
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        for _ in range(50):
            if self.loop is not None:
                break
            time.sleep(0.05)
        return self

    def _serve(self) -> None:
        import websockets

        async def handler(ws):
            self.clients.add(ws)
            try:
                async for raw in ws:
                    if not self.on_command:
                        continue
                    try:
                        msg = json.loads(raw)
                    except (ValueError, TypeError):
                        continue
                    if msg.get("type") == "camera_mount":
                        self.on_command(msg)
            except Exception:
                pass
            finally:
                self.clients.discard(ws)

        async def main():
            self.loop = asyncio.get_running_loop()
            async with websockets.serve(handler, "", self.port, ping_interval=20):
                print(f"[live] minimap feed on ws://localhost:{self.port}")
                await asyncio.Future()

        try:
            asyncio.run(main())
        except Exception as e:
            print(f"[live] websocket server failed: {e}")

    def send(self, payload: dict) -> None:
        if self.loop is None or not self.clients:
            return
        msg = json.dumps({"type": "perception", "data": payload})

        async def fan():
            await asyncio.gather(*[c.send(msg) for c in list(self.clients)],
                                 return_exceptions=True)

        asyncio.run_coroutine_threadsafe(fan(), self.loop)


def _on_sigterm(signum, frame):
    raise KeyboardInterrupt()


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="yolo11m.pt")
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--port", type=int, default=WS_PORT)
    ap.add_argument("--no-gps", action="store_true",
                    help="use a fixed stationary pose (parked range checks)")
    ap.add_argument("--print", dest="show", action="store_true",
                    help="print tracks to the console as well")
    # Calibration comes from a file, not from flags: the live service and the
    # calibration tool have to be looking at the same lens.
    ap.add_argument("--calib", default=None,
                    help="calibration JSON (default calibration/front_camera.json)")
    ap.add_argument("--device", type=int, default=0, help="/dev/videoN")
    args = ap.parse_args()

    # SIGTERM must unwind through the same path as ctrl-C. Being killed with
    # the V4L2 stream still open is what leaves the camera wedged -- it keeps
    # enumerating and keeps opening, but never delivers another frame until
    # the USB is replugged. `timeout`, systemd and a plain `kill` all send
    # SIGTERM, so this is the normal way this tool stops, not an edge case.
    signal.signal(signal.SIGTERM, _on_sigterm)

    cam, calib = calib_mod.load_camera(args.calib)
    print(calib_mod.describe(cam, calib))
    horizon = cam.horizon_y()
    near = cam.ground_range_m(cam.height - 1.0)
    print(f"[live] horizon at row {horizon:.0f}; nearest visible ground "
          f"{(near or float('nan')) + cam.offset_forward_m:.2f} m from the cart origin")

    pub = Publisher(args.port).start()
    cfg = PolicyConfig()

    cart = None
    if not args.no_gps:
        from cartlib.cart import Cart
        # GPS only. No pedals, no steering: this tool cannot move the cart even
        # if something in it goes wrong.
        cart = Cart(use_gps=True, use_pedals=False, use_steering=False).open()
        print("[live] GPS open (no pedals, no steering — this tool cannot drive)")

    svc = PerceptionService(cam=cam, cfg=cfg, weights=args.model,
                            imgsz=args.imgsz, conf=args.conf,
                            shadow=True, publish=pub.send,
                            cap_width=cam.width, cap_height=cam.height,
                            cap_fps=calib_mod.capture_fps(calib),
                            cam_device=args.device)
    pub.on_command = lambda msg: svc.set_mount(
        height_m=msg.get("height_m"), pitch_deg=msg.get("pitch_deg"))
    print(f"[live] loading {args.model} @ {args.imgsz} ...")
    try:
        svc.start()
    except Exception as e:
        print(f"[live] cannot start: {e}")
        return 1
    print("[live] running — open the UI with "
          f"NEXT_PUBLIC_GPS_WS_URL=ws://localhost:{args.port}   (ctrl-C to stop)")

    last_print = 0.0
    try:
        while True:
            if args.no_gps:
                svc.set_context((0.0, 0.0), 0.0, 0.0, [])
            elif cart and cart.gps and cart.gps.latest:
                fix = cart.gps.latest
                svc.set_context((fix["lat"], fix["lon"]), 0.0, 0.0, [])

            if svc.error:
                print(f"[live] perception stopped: {svc.error}")
                return 1

            now = time.monotonic()
            if args.show and now - last_print > 0.5:
                last_print = now
                d = svc.decision
                print(f"{svc.hz:5.1f} Hz  det {svc.detector.last_ms:5.1f} ms  "
                      f"[{d.get('layer', '?')}] "
                      f"allow {d.get('v_allowed_mph', 0):4.1f} mph  "
                      f"{d.get('reason', '')[:60]}")
            time.sleep(1.0 / 15.0)
    except KeyboardInterrupt:
        print("\n[live] stopping")
    finally:
        svc.stop()
        if cart:
            cart.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
