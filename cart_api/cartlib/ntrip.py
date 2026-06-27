"""
cartlib.ntrip — optional NTRIP corrections feeder for an RTK fix.

The u-blox receiver only reaches "RTK Fixed" when it's fed RTCM correction
data from an NTRIP caster. This module connects to the caster and streams
corrections into the GPS serial port in a background thread. It periodically
echoes the receiver's latest GGA back to the caster (VRS mountpoints need it).

It reuses the same caster credentials the project's lane_tracker uses. Run it
alongside a ``GpsReceiver`` pointed at the same device:

    gps = GpsReceiver().open()
    ntrip = NtripClient(gps).start()
    ...
    gps.wait_for_fix()   # should climb to "RTK Fixed" once corrections flow
"""

from __future__ import annotations

import base64
import socket
import threading
import time
from typing import Optional

from .gps import GpsReceiver

NTRIP_HOST = "rtk.rtkdata.com"
NTRIP_PORT = 2101
MOUNTPOINT = "AUTO"
USERNAME = "rtkethangoo41d"
PASSWORD = "9ce1743e4074"
FALLBACK_GGA = "$GPGGA,120000.00,3725.5900,N,12209.8400,W,1,12,0.8,30.0,M,-30.0,M,,*5E"


class NtripClient:
    def __init__(self, gps: GpsReceiver, host: str = NTRIP_HOST, port: int = NTRIP_PORT,
                 mountpoint: str = MOUNTPOINT, username: str = USERNAME,
                 password: str = PASSWORD):
        self.gps = gps
        self.host, self.port, self.mountpoint = host, port, mountpoint
        self.username, self.password = username, password
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.connected = False

    def start(self) -> "NtripClient":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _gga(self) -> str:
        return self.gps.last_gga_raw or FALLBACK_GGA

    def _run(self) -> None:
        while not self._stop.is_set():
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10)
                sock.connect((self.host, self.port))
                creds = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
                req = (
                    f"GET /{self.mountpoint} HTTP/1.1\r\n"
                    f"Host: {self.host}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"User-Agent: NTRIP cartlib/1.0\r\n"
                    f"Authorization: Basic {creds}\r\n"
                    f"Accept: */*\r\n\r\n"
                )
                sock.sendall(req.encode())
                sock.sendall((self._gga() + "\r\n").encode())

                resp = b""
                while b"\r\n\r\n" not in resp:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break
                    resp += chunk
                header, _, remainder = resp.partition(b"\r\n\r\n")
                if b"200" not in header:
                    self.connected = False
                    time.sleep(5)
                    continue
                self.connected = True
                if remainder and self.gps._ser:
                    self.gps._ser.write(remainder)

                sock.settimeout(5)
                last_gga = time.time()
                while not self._stop.is_set():
                    try:
                        data = sock.recv(4096)
                        if not data:
                            break
                        if self.gps._ser:
                            self.gps._ser.write(data)
                        if time.time() - last_gga > 10:
                            last_gga = time.time()
                            sock.sendall((self._gga() + "\r\n").encode())
                    except socket.timeout:
                        continue
            except Exception:
                self.connected = False
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
            time.sleep(3)
