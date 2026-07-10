"""
Render the eval video with obstacle-avoidance overlays baked in: brake zone
(green=clear, red=brake), detection boxes, and the BRAKE/CLEAR banner — the
exact frames the live service serves, written out as a watchable .mp4.

    obstacle/.venv/bin/python obstacle/render_overlay.py --device mps
    obstacle/.venv/bin/python obstacle/render_overlay.py --start 230 --end 300  # just a slice
"""

from __future__ import annotations

import argparse
import os
import subprocess
import time

import cv2

from detector import Detector


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--video", default="/Users/georgv.manstein/Downloads/pi-stanford-test-data/front.mp4")
    p.add_argument("--out", default="obstacle/out/front_obstacle_overlay.mp4")
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--device", default=None, help="e.g. mps / cpu (default: ultralytics auto)")
    p.add_argument("--start", type=float, default=0.0, help="start time (s)")
    p.add_argument("--end", type=float, default=None, help="end time (s)")
    args = p.parse_args()

    det = Detector(args.model, device=args.device)
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.start:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(args.start * fps))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    raw_path = args.out.replace(".mp4", ".raw.mp4")
    writer = cv2.VideoWriter(raw_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    n = int(args.start * fps)
    end_frame = int(args.end * fps) if args.end else total
    done = 0
    brake_frames = 0
    t0 = time.time()
    while n < end_frame:
        ret, frame = cap.read()
        if not ret:
            break
        n += 1
        done += 1
        video_t = n / fps
        annotated = det.process(frame, video_t)
        # Timestamp in the top-right corner so brake windows are easy to cite.
        cv2.putText(annotated, f"t={video_t:6.1f}s", (w - 118, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        if det.status["brake"]:
            brake_frames += 1
        writer.write(annotated)
        if done % int(30 * fps) == 0:
            rate = done / (time.time() - t0)
            print(f"{video_t:6.1f}s / {end_frame / fps:.0f}s  ({rate:.0f} fps, "
                  f"~{(end_frame - n) / rate:.0f}s left)", flush=True)

    cap.release()
    writer.release()

    # Re-encode to H.264/yuv420p so QuickTime & browsers play it.
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", raw_path,
                    "-c:v", "libx264", "-preset", "fast", "-crf", "23",
                    "-pix_fmt", "yuv420p", args.out], check=True)
    os.remove(raw_path)

    print(f"\nwrote {args.out}: {done} frames, {done / fps:.0f}s, "
          f"brake on for {brake_frames / fps:.1f}s ({100 * brake_frames / max(1, done):.1f}%) "
          f"in {time.time() - t0:.0f}s wall")


if __name__ == "__main__":
    main()
