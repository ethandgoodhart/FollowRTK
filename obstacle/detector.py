"""
obstacle.detector — camera-based "should we brake right now?" service.

Watches the front camera feed (for now: a recorded eval video played back in
real time), runs YOLO object detection on each frame, and checks whether any
obstacle's footprint lands inside the BRAKE ZONE — a trapezoid covering the
lower-middle of the image, i.e. the patch of ground directly ahead of the
cart. If something is standing in that zone, the answer is "brake".

Serves the decision over HTTP (CORS open, for the drivelive UI):

    GET /status     {"brake": bool, "detections": [...], "video_t": s, ...}
    GET /frame.jpg  latest annotated frame (zone + boxes + verdict banner)

Run (eval mode against the recorded Stanford drive):

    obstacle/.venv/bin/python obstacle/detector.py \
        --video ~/Downloads/pi-stanford-test-data/front.mp4 --loop

The zone and thresholds are normalized to frame size, so the same code works
on the live camera later — swap --video for a camera index with --camera 0.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import threading
import time

import cv2
import numpy as np
from aiohttp import web
from ultralytics import YOLO

# COCO classes that count as "something in the way". Anything big enough to
# matter that YOLO can name; skateboards/strollers ride under "person" anyway.
OBSTACLE_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "bus", "truck",
    "dog", "cat", "horse", "bench", "backpack", "suitcase",
    "sports ball", "stop sign", "fire hydrant", "potted plant",
}

CONF_THRESHOLD = 0.35

# Brake zone: trapezoid in the lower-middle of the frame (normalized x,y).
# Bottom edge spans the cart's width plus margin; the top edge is narrower
# and sits just above the visual horizon of "stuff we'd hit within ~2s".
ZONE_NORM = [
    (0.22, 1.00),   # bottom-left
    (0.78, 1.00),   # bottom-right
    (0.56, 0.68),   # top-right
    (0.44, 0.68),   # top-left
]

# Fraction of a detection's footprint (bottom strip of its bbox — where it
# touches the ground) that must overlap the zone to count as "in the way".
FOOTPRINT_OVERLAP = 0.15
FOOTPRINT_HEIGHT_FRAC = 0.25  # bottom 25% of the bbox is the footprint

# Debounce: need N consecutive brake frames to latch, then hold for HOLD_S
# after the zone clears so the signal doesn't flicker.
TRIGGER_FRAMES = 2
HOLD_S = 1.0


class Detector:
    def __init__(self, model_path: str, device: str | None = None):
        self.model = YOLO(model_path)
        self.device = device
        self.lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.status: dict = {"brake": False, "detections": [], "connected": False}
        self._consec = 0
        self._brake_until = 0.0
        self._zone_mask: np.ndarray | None = None
        self._zone_px: np.ndarray | None = None

    def _ensure_zone(self, w: int, h: int) -> None:
        if self._zone_mask is not None:
            return
        pts = np.array([(int(x * w), int(y * h)) for x, y in ZONE_NORM], dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        self._zone_mask = mask
        self._zone_px = pts

    def reset(self) -> None:
        """Clear debounce state (call when the video source restarts)."""
        self._consec = 0
        self._brake_until = 0.0

    def process(self, frame: np.ndarray, video_t: float) -> np.ndarray:
        h, w = frame.shape[:2]
        self._ensure_zone(w, h)

        results = self.model(frame, verbose=False, device=self.device)[0]
        names = results.names

        detections = []
        zone_hit = False
        for box in results.boxes:
            conf = float(box.conf[0])
            cls = names[int(box.cls[0])]
            if conf < CONF_THRESHOLD or cls not in OBSTACLE_CLASSES:
                continue
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            # Footprint = bottom strip of the bbox, clamped to the frame.
            fy1 = max(0, int(y2 - (y2 - y1) * FOOTPRINT_HEIGHT_FRAC))
            fx1, fx2, fy2 = max(0, x1), min(w, x2), min(h, y2)
            foot_area = max(1, (fx2 - fx1) * (fy2 - fy1))
            overlap = int(np.count_nonzero(self._zone_mask[fy1:fy2, fx1:fx2])) / foot_area
            in_zone = overlap >= FOOTPRINT_OVERLAP
            zone_hit = zone_hit or in_zone
            detections.append({
                "cls": cls, "conf": round(conf, 2),
                "box": [x1, y1, x2, y2], "in_zone": in_zone,
                "overlap": round(overlap, 2),
            })

        # Debounce on *video* time so decisions are identical whether frames
        # arrive in real time (live service) or as fast as we can render.
        if zone_hit:
            self._consec += 1
            if self._consec >= TRIGGER_FRAMES:
                self._brake_until = video_t + HOLD_S
        else:
            self._consec = 0
        brake = video_t < self._brake_until

        annotated = self._annotate(frame, detections, brake)
        ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])

        with self.lock:
            self.status = {
                "brake": brake,
                "detections": detections,
                "video_t": round(video_t, 2),
                "connected": True,
                "t": time.time(),
            }
            if ok:
                self.latest_jpeg = jpeg.tobytes()
        return annotated

    def _annotate(self, frame: np.ndarray, detections: list[dict], brake: bool) -> np.ndarray:
        out = frame.copy()
        zone_color = (0, 0, 255) if brake else (0, 200, 0)
        overlay = out.copy()
        cv2.fillPoly(overlay, [self._zone_px], zone_color)
        cv2.addWeighted(overlay, 0.25, out, 0.75, 0, out)
        cv2.polylines(out, [self._zone_px], True, zone_color, 2)

        for d in detections:
            x1, y1, x2, y2 = d["box"]
            color = (0, 0, 255) if d["in_zone"] else (0, 200, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            cv2.putText(out, f'{d["cls"]} {d["conf"]:.2f}', (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        banner = "BRAKE" if brake else "CLEAR"
        cv2.rectangle(out, (0, 0), (110, 26), (0, 0, 255) if brake else (0, 130, 0), -1)
        cv2.putText(out, banner, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
        return out


def playback_loop(det: Detector, args: argparse.Namespace) -> None:
    """Read frames (eval video paced to wall-clock, or a live camera) forever."""
    while True:
        src = args.camera if args.camera is not None else args.video
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print(f"cannot open {src}, retrying in 2s")
            time.sleep(2)
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        det.reset()
        start = time.monotonic()
        n = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            n += 1
            video_t = n / fps
            if args.camera is None:
                # Pace playback to real time; drop frames we're too slow for.
                behind = video_t - (time.monotonic() - start)
                if behind > 0:
                    time.sleep(behind)
                elif behind < -0.5 and n % 2 == 1:
                    continue  # skip odd frames to catch back up
            det.process(frame, video_t)
        cap.release()
        if not args.loop or args.camera is not None:
            with det.lock:
                det.status = {**det.status, "connected": False, "ended": True}
            print("video ended")
            break
        print("video ended — looping")


CORS = {
    "Access-Control-Allow-Origin": "*",
    "Cache-Control": "no-store",
}


async def handle_status(request: web.Request) -> web.Response:
    det: Detector = request.app["detector"]
    with det.lock:
        body = json.dumps(det.status)
    return web.Response(text=body, content_type="application/json", headers=CORS)


async def handle_frame(request: web.Request) -> web.Response:
    det: Detector = request.app["detector"]
    with det.lock:
        jpeg = det.latest_jpeg
    if jpeg is None:
        return web.Response(status=503, text="no frame yet", headers=CORS)
    return web.Response(body=jpeg, content_type="image/jpeg", headers=CORS)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--video", default="/Users/georgv.manstein/Downloads/pi-stanford-test-data/front.mp4")
    p.add_argument("--camera", type=int, default=None,
                   help="live camera index (overrides --video)")
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--port", type=int, default=8766)
    p.add_argument("--loop", action="store_true", help="loop the eval video")
    args = p.parse_args()

    det = Detector(args.model)
    threading.Thread(target=playback_loop, args=(det, args), daemon=True).start()

    app = web.Application()
    app["detector"] = det
    app.router.add_get("/status", handle_status)
    app.router.add_get("/frame.jpg", handle_frame)
    print(f"obstacle detector on http://localhost:{args.port}  (status, frame.jpg)")
    web.run_app(app, port=args.port, print=None)


if __name__ == "__main__":
    main()
