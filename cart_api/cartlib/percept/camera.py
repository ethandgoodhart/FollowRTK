"""
cartlib.percept.camera — a camera thread that always hands you the newest frame.

The benchmark said the detector runs at 55 Hz (p95) and the camera at 27.8 Hz,
so the camera is the pacing item. Grabbing inline would make the pipeline cost
``camera + inference`` when it should cost ``max(camera, inference)`` -- on this
hardware that is the difference between roughly 27 Hz and roughly 18 Hz, i.e.
about 20 ms of extra reaction latency for nothing.

There is a second, sharper reason for the thread. V4L2 buffers frames, so a
consumer that reads more slowly than the driver produces gets served the OLDEST
queued frame. A perception system quietly working on 200 ms-old images is a
perception system that brakes late for reasons nobody can see in the logs. This
grabber therefore keeps exactly one frame -- the newest -- and drops the rest,
and stamps it so the policy's health layer can refuse to trust a stale one.

Two things learned the hard way and encoded here:

  * FPS MUST BE REQUESTED. Without ``CAP_PROP_FPS`` the driver serves its
    slowest advertised mode: 15 fps at 1920x1200, or 67 ms per frame, even
    though the sensor does 90. Asking nearly doubles real throughput.

  * ONLY ONE PROCESS MAY HOLD THE DEVICE. A second opener does not get an
    error, it gets ten-second ``select()`` timeouts, which look exactly like a
    hung model load. ``fuser -v /dev/video0`` is the diagnostic.

  * AN ADVERTISED MODE IS NOT A WORKING MODE. ``v4l2-ctl`` lists 1600x1200 at
    90 fps, and asking for it delivers *nothing at all* -- not slow frames,
    zero frames, with the same 10 s select() timeouts as a wedged device. Both
    cameras share one USB hub and the bandwidth is not there. 30 fps at the
    same resolution is solid. So the requested rate is treated as a wish:
    ``open()`` proves it with a real frame and steps down the ladder if it
    cannot, because a perception stack that silently fails to start is worse
    than one running at a third of the frame rate.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import numpy as np

FRONT_CAM = 0


class CameraStream:
    """Background MJPG grabber. ``read()`` returns (frame, capture_ts)."""

    def __init__(self, device: int = FRONT_CAM, width: int = 1920,
                 height: int = 1200, fps: int = 90):
        self.device, self.width, self.height, self.fps = device, width, height, fps
        self._frame: Optional[np.ndarray] = None
        self._ts: float = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._cap = None
        self.frames = 0
        self.fails = 0
        self.reopens = 0
        self.max_reopens = 3
        self.error: Optional[str] = None

    # -- lifecycle --------------------------------------------------------
    def _configure(self, fps: int):
        import cv2

        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _fps_ladder(self):
        """Requested rate first, then the rates this sensor also advertises."""
        rates = [self.fps] + [r for r in (60, 30, 15) if r < self.fps]
        return rates

    def open(self) -> "CameraStream":
        cap = None
        for i, fps in enumerate(self._fps_ladder()):
            cap = self._configure(fps)
            if not cap.isOpened():
                cap.release()
                raise RuntimeError(
                    f"cannot open /dev/video{self.device} — is something else "
                    f"holding it? check: fuser -v /dev/video{self.device}")
            # One real frame is the only proof the mode works. A read that
            # fails here has already burned its 10 s select() timeout, which
            # is why the ladder is short and starts at what was asked for.
            ok, _ = cap.read()
            if ok:
                if fps != self.fps:
                    print(f"[camera] {self.width}x{self.height}@{self.fps} "
                          f"delivered no frames (USB bandwidth) — running at "
                          f"{fps} fps instead")
                    self.fps = fps
                break
            cap.release()
            cap = None
        if cap is None:
            raise RuntimeError(
                f"/dev/video{self.device} opened but delivered no frames at "
                f"{self.width}x{self.height} at any of "
                f"{self._fps_ladder()} fps. Either another process holds it "
                f"(fuser -v /dev/video{self.device}) or the device is wedged "
                f"— unplug and replug its USB.")
        self._cap = cap
        self._thread = threading.Thread(target=self._run, name="camera",
                                        daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "CameraStream":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- the thread -------------------------------------------------------
    def _reopen(self) -> bool:
        """Close and reopen the device. Recovers the common wedged state.

        This camera drops off its USB hub from time to time (the same rig has
        form for it -- the Pi GPS bridge does the same thing), and it also
        wedges if a previous process was killed mid-stream without releasing
        the V4L2 buffers. In both cases the device still enumerates and still
        opens; it simply never delivers a frame, and every read costs a 10 s
        select() timeout. A reopen fixes the second case outright and the first
        one often enough to be worth trying before giving up on the drive.
        """
        try:
            self._cap.release()
        except Exception:
            pass
        time.sleep(0.5)
        cap = self._configure(self.fps)
        if not cap.isOpened():
            return False
        self._cap = cap
        self.reopens += 1
        return True

    def _run(self) -> None:
        consecutive = 0
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                consecutive += 1
                self.fails += 1
                # Each failed read costs a 10 s select() timeout, so these
                # thresholds are in units of tens of seconds, not frames.
                if consecutive == 2 and self.reopens < self.max_reopens:
                    print(f"[camera] /dev/video{self.device} stalled — reopening")
                    if self._reopen():
                        consecutive = 0
                        continue
                if consecutive >= 3:
                    self.error = (
                        f"/dev/video{self.device} is not delivering frames "
                        f"({self.frames} good frames so far, {self.reopens} "
                        f"reopen attempts). The device is wedged: unplug and "
                        f"replug the camera's USB, or check "
                        f"`fuser -v /dev/video{self.device}` for another holder.")
                    return
                continue
            consecutive = 0
            with self._lock:
                self._frame, self._ts = frame, time.monotonic()
                self.frames += 1

    # -- consumers --------------------------------------------------------
    def read(self) -> Tuple[Optional[np.ndarray], float]:
        """Newest frame and the monotonic time it was captured."""
        with self._lock:
            return self._frame, self._ts

    def wait_for_frame(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.error:
                return False
            with self._lock:
                if self._frame is not None:
                    return True
            time.sleep(0.02)
        return False

    @property
    def healthy(self) -> bool:
        return self.error is None and self._frame is not None
