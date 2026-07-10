"""
simlib — a calibrated simulator of the FollowRTK cart so we can develop and
compare steering controllers off-vehicle.

Plant model (physically grounded in the 2006 Club Car Precedent + ODrive S1):
  * Kinematic bicycle. Road-wheel angle delta = column_angle / STEER_RATIO,
    clamped to the rack limit. yaw_rate = v / L * tan(delta).
  * Steering actuator: the ODrive runs trap-traj position control. We model it
    as a velocity+acceleration limited follower of the commanded column angle.
    Peak slew (~480 deg/s at the column) and accel come straight from the
    ODrive trap-traj limits (vel 4 turns/s, accel 8 turns/s^2, 3:1 belt) and
    match the peak slew measured in the recorded drives.
  * Control-loop rate is a parameter (the recorded drives ran at ~1.9 Hz due to
    a blocking serial read; a fixed loop runs ~10 Hz).

Geometry constants are calibrated so the model's turn radius and actuator slew
land in the physically-correct ballpark; the recorded RTK-Float data is too
noisy (0.5 m position noise over 0.5 s steps at 3 mph) to support a tighter
open-loop fit, so we validate by CLOSED-LOOP replay of the old law instead.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ----------------------------------------------------------------------------
# geodesy (mirrors cartlib.geo)
# ----------------------------------------------------------------------------
EARTH_R = 6371000.0
LatLon = Tuple[float, float]

def local_xy(origin: LatLon, p: LatLon) -> Tuple[float, float]:
    dlat = math.radians(p[0] - origin[0])
    dlon = math.radians(p[1] - origin[1])
    x = dlon * math.cos(math.radians(origin[0])) * EARTH_R  # east
    y = dlat * EARTH_R                                       # north
    return x, y

def from_local_xy(origin: LatLon, x: float, y: float) -> LatLon:
    lat = origin[0] + math.degrees(y / EARTH_R)
    lon = origin[1] + math.degrees(x / (math.cos(math.radians(origin[0])) * EARTH_R))
    return lat, lon

# ----------------------------------------------------------------------------
# plant constants
# ----------------------------------------------------------------------------
WHEELBASE_M = 1.65        # Club Car Precedent wheelbase (~65 in)
STEER_RATIO = 10.0        # steering-column deg per road-wheel deg (full lock 320 -> 32 deg)
MAX_ROADWHEEL_DEG = 33.0  # rack limit -> min turn radius ~2.5 m
ACT_VEL_LIM = 480.0       # column deg/s  (ODrive 4 turns/s / 3:1 belt * 360)
ACT_ACC_LIM = 960.0       # column deg/s^2 (ODrive 8 turns/s^2 / 3:1 * 360)


class SteeringActuator:
    """Vel+accel limited follower of a commanded column angle (deg)."""
    def __init__(self, angle=0.0, vel_lim=ACT_VEL_LIM, acc_lim=ACT_ACC_LIM):
        self.angle = angle
        self.vel = 0.0
        self.vel_lim = vel_lim
        self.acc_lim = acc_lim

    def update(self, target_deg: float, dt: float) -> float:
        err = target_deg - self.angle
        # trapezoidal: cap approach velocity so we can still decel to a stop
        v_stop = math.sqrt(2 * self.acc_lim * abs(err)) if err else 0.0
        v_des = math.copysign(min(self.vel_lim, v_stop), err) if err else 0.0
        dv = max(-self.acc_lim * dt, min(v_des - self.vel, self.acc_lim * dt))
        self.vel += dv
        self.angle += self.vel * dt
        return self.angle


class Vehicle:
    """Kinematic bicycle. State in local metric frame (x east, y north)."""
    def __init__(self, x=0.0, y=0.0, heading=0.0):
        self.x = x
        self.y = y
        self.heading = heading   # rad, 0 = north (+y), + = clockwise toward east

    def step(self, v_ms: float, column_deg: float, dt: float) -> None:
        delta = max(-MAX_ROADWHEEL_DEG, min(column_deg / STEER_RATIO, MAX_ROADWHEEL_DEG))
        yaw_rate = v_ms / WHEELBASE_M * math.tan(math.radians(delta))  # rad/s
        self.heading += yaw_rate * dt
        self.x += v_ms * math.sin(self.heading) * dt
        self.y += v_ms * math.cos(self.heading) * dt


# ----------------------------------------------------------------------------
# path geometry: cumulative length, bearing, signed cross-track, curvature
# ----------------------------------------------------------------------------
@dataclass
class Snap:
    x: float
    y: float
    seg_i: int
    along_m: float
    dist_m: float          # unsigned cross-track
    signed_m: float        # + = left of path direction
    path_heading: float    # rad (heading of the path at the snap, 0=north)
    curvature: float       # 1/m, + = path turning left


class Path:
    """Polyline in the local metric frame with geometry helpers."""
    def __init__(self, latlon_pts: List[LatLon], origin: Optional[LatLon] = None):
        self.origin = origin or latlon_pts[0]
        self.pts = [local_xy(self.origin, p) for p in latlon_pts]
        self.n = len(self.pts)
        # cumulative arc length
        self.cum = [0.0]
        for i in range(1, self.n):
            self.cum.append(self.cum[-1] + math.dist(self.pts[i - 1], self.pts[i]))
        self.length = self.cum[-1]
        # per-segment heading (rad, 0=north)
        self.seg_head = []
        for i in range(self.n - 1):
            dx = self.pts[i + 1][0] - self.pts[i][0]
            dy = self.pts[i + 1][1] - self.pts[i][1]
            self.seg_head.append(math.atan2(dx, dy))

    def snap(self, x: float, y: float) -> Snap:
        best = None
        for i in range(self.n - 1):
            ax, ay = self.pts[i]
            bx, by = self.pts[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 == 0:
                continue
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            px, py = ax + dx * t, ay + dy * t
            d = math.hypot(x - px, y - py)
            cross = dx * (y - ay) - dy * (x - ax)   # >0 => left of path dir
            signed = d if cross > 0 else -d
            along = self.cum[i] + math.sqrt(L2) * t
            if best is None or d < best[0]:
                best = (d, signed, i, along, px, py)
        d, signed, i, along, px, py = best
        return Snap(px, py, i, along, d, signed,
                    self.seg_head[i], self.curvature_at(along))

    def heading_at(self, along_m: float) -> float:
        along_m = max(0.0, min(along_m, self.length))
        for i in range(self.n - 1):
            if self.cum[i + 1] >= along_m or i == self.n - 2:
                return self.seg_head[i]
        return self.seg_head[-1]

    def curvature_at(self, along_m: float) -> float:
        """Signed curvature 1/m via heading change over a smoothing window."""
        w = 2.5  # m window each side
        h1 = self.heading_at(along_m - w)
        h2 = self.heading_at(along_m + w)
        dh = (h2 - h1 + math.pi) % (2 * math.pi) - math.pi
        return dh / (2 * w)

    def point_at(self, along_m: float) -> Tuple[float, float]:
        along_m = max(0.0, min(along_m, self.length))
        for i in range(self.n - 1):
            if self.cum[i + 1] >= along_m:
                seg = self.cum[i + 1] - self.cum[i]
                t = (along_m - self.cum[i]) / seg if seg else 0.0
                ax, ay = self.pts[i]
                bx, by = self.pts[i + 1]
                return ax + (bx - ax) * t, ay + (by - ay) * t
        return self.pts[-1]


def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
