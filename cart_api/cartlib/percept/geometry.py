"""
cartlib.percept.geometry — pixels to metres, in closed form.

This is the module that decides how far away a person is, and it is
deliberately the least clever part of the perception stack. Every number it
produces comes from a formula you can check with a tape measure and a
calculator, because a distance estimate that silently drifts is the failure
mode that hurts somebody.

The primary cue is the GROUND CONTACT POINT: where the bottom of a detection
box meets the ground. For a camera at a known height and pitch, the image row
of that contact point determines distance exactly. Published work on
ground-vehicle monocular ranging finds this cue carries the large majority of
the predictive power (~77% vs ~23% for apparent size) and lands within ~13-14
cm of calibrated ranging methods.

Derivation (world: X right, Y up, Z forward; camera at height h, pitched down
by theta; image +y is down):

    optical axis   = (0, -sin th,  cos th)
    image-down axis= (0, -cos th, -sin th)

For a ground point at forward distance Z and lateral offset X, the vector from
camera to point is (X, -h, Z), so

    z_c = h sin th + Z cos th
    y_c = h cos th - Z sin th
    k   = (v_img - cy) / fy = y_c / z_c

and inverting for Z gives the one expression this module is built around:

    Z = h (cos th - k sin th) / (k cos th + sin th)                        (*)

At th = 0 that collapses to the familiar Z = h*fy/(v - cy). The horizon sits
where the denominator vanishes, v_horizon = cy - fy*tan(th), and (*) blows up
as a detection approaches it — which is the honest behaviour: a contact point
near the horizon genuinely carries almost no distance information.

BECAUSE OF THAT, ``range_uncertainty_m`` is not optional decoration. Range
error grows with the square of distance, so the same 1 degree of pitch wobble
that costs 6% at 5 m costs 20% at 15 m. The policy layer is expected to use
the CONSERVATIVE (near) bound, never the point estimate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

# Typical real-world heights (metres), used only for the secondary size cue.
# These are deliberately coarse: the size cue is a cross-check, not a source of
# truth, and pretending to know a pedestrian's height to the centimetre would
# be false precision.
CLASS_HEIGHT_M = {
    "person": 1.70,
    "bicycle": 1.10,
    "car": 1.50,
    "motorcycle": 1.40,
    "bus": 3.20,
    "truck": 3.00,
    "dog": 0.55,
}

# Widths, used to sanity-check lateral extent.
CLASS_WIDTH_M = {
    "person": 0.55,
    "bicycle": 0.60,
    "car": 1.85,
    "motorcycle": 0.80,
    "bus": 2.55,
    "truck": 2.50,
    "dog": 0.35,
}


@dataclass
class CameraModel:
    """Intrinsics + mounting geometry for the front camera.

    ``pitch_deg`` is POSITIVE DOWNWARD. ``height_m`` is lens centre to ground.
    ``yaw_deg`` is the mount's rotation about vertical relative to the cart's
    forward axis (positive = pointing right); it is zero for a true forward
    mount but is here so a mis-aimed bracket can be corrected in software
    rather than by loosening bolts on a working cart.
    """

    width: int = 1920
    height: int = 1200
    fx: float = 1000.0
    fy: float = 1000.0
    cx: Optional[float] = None       # defaults to width/2
    cy: Optional[float] = None       # defaults to height/2
    # Brown-Conrady radial/tangential distortion. The front camera shows
    # visible barrel distortion, so these must be filled in from a real
    # calibration before any of this is trustworthy at the frame edges.
    dist: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    height_m: float = 1.50           # lens above ground
    pitch_deg: float = 0.0           # positive = looking down
    yaw_deg: float = 0.0             # positive = aimed right of straight ahead

    # Where the lens sits relative to the cart origin used by the follower
    # (positive forward / right). Small, but it matters at 3 m.
    offset_forward_m: float = 0.0
    offset_lateral_m: float = 0.0

    # How well we trust the pitch. Every bump and every brake application
    # rotates the camera; this is the budget the uncertainty model spends.
    pitch_sigma_deg: float = 1.0

    def __post_init__(self) -> None:
        if self.cx is None:
            self.cx = self.width / 2.0
        if self.cy is None:
            self.cy = self.height / 2.0

    # -- basics ----------------------------------------------------------
    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pitch_deg)

    def horizon_y(self, pitch_deg: Optional[float] = None) -> float:
        """Image row of the horizon. Ground points must lie strictly below it."""
        th = math.radians(self.pitch_deg if pitch_deg is None else pitch_deg)
        return self.cy - self.fy * math.tan(th)

    def nearest_ground_m(self) -> Optional[float]:
        """Forward range from the cart origin to the nearest visible ground.

        The bottom-centre pixel is the closest ground the camera can see; the
        mount offset then puts that in the same frame the follower uses. None
        if even the bottom row is on or above the horizon (aimed too high).
        """
        pt = self.ground_point(self.width / 2.0, float(self.height - 1))
        return None if pt is None else pt[0]

    def hfov_deg(self) -> float:
        return 2.0 * math.degrees(math.atan(self.width / (2.0 * self.fx)))

    def vfov_deg(self) -> float:
        return 2.0 * math.degrees(math.atan(self.height / (2.0 * self.fy)))

    # -- distortion ------------------------------------------------------
    def undistort(self, u: float, v: float, max_iters: int = 50,
                  tol_px: float = 1e-3) -> Tuple[float, float]:
        """Map a distorted pixel to where a pinhole camera would have put it.

        Iterative inverse of the Brown-Conrady forward model, run to a measured
        residual rather than a fixed iteration count. The usual textbook "five
        iterations and stop" converges fine near the optical axis but is still
        most of a pixel out at the frame corners under the barrel distortion
        this camera actually has -- and the corners are exactly where a
        pedestrian first appears when stepping into view from the side. So we
        iterate until re-distorting the answer reproduces the input to within
        ``tol_px``, and only then stop.
        """
        k1, k2, p1, p2, k3 = self.dist
        if not any(self.dist):
            return u, v
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        x0, y0 = x, y
        for _ in range(max_iters):
            r2 = x * x + y * y
            radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
            if radial <= 1e-9:
                break
            dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            x = (x0 - dx) / radial
            y = (y0 - dy) / radial
            # Convergence is measured in the space we care about: pixels.
            ru, rv = self.distort(x * self.fx + self.cx, y * self.fy + self.cy)
            if abs(ru - u) < tol_px and abs(rv - v) < tol_px:
                break
        return x * self.fx + self.cx, y * self.fy + self.cy

    def distort(self, u: float, v: float) -> Tuple[float, float]:
        """Forward Brown-Conrady. Mostly here so the inverse can be tested."""
        k1, k2, p1, p2, k3 = self.dist
        if not any(self.dist):
            return u, v
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return xd * self.fx + self.cx, yd * self.fy + self.cy

    # -- the ground-plane cue -------------------------------------------
    def ground_range_m(self, v_img: float,
                       pitch_deg: Optional[float] = None) -> Optional[float]:
        """Forward distance to a ground point imaged at row ``v_img``.

        Returns None when the row is at or above the horizon, i.e. when the
        pixel simply does not correspond to a point on the ground plane. That
        is a real answer, not an error: refusing to guess is the whole point.
        """
        th = math.radians(self.pitch_deg if pitch_deg is None else pitch_deg)
        k = (v_img - self.cy) / self.fy
        den = k * math.cos(th) + math.sin(th)
        if den <= 1e-9:                       # at or above the horizon
            return None
        z = self.height_m * (math.cos(th) - k * math.sin(th)) / den
        return z if z > 0 else None

    def ground_lateral_m(self, u_img: float, forward_m: float,
                         pitch_deg: Optional[float] = None) -> float:
        """Lateral offset (positive right) of a ground point at ``forward_m``."""
        th = math.radians(self.pitch_deg if pitch_deg is None else pitch_deg)
        z_c = self.height_m * math.sin(th) + forward_m * math.cos(th)
        return (u_img - self.cx) / self.fx * z_c

    def ground_point(self, u_img: float, v_img: float,
                     pitch_deg: Optional[float] = None
                     ) -> Optional[Tuple[float, float]]:
        """Pixel -> (forward_m, lateral_m) in the CART frame.

        Applies undistortion, the ground-plane inversion, then the mount's yaw
        and translation offsets.
        """
        u, v = self.undistort(u_img, v_img)
        fwd = self.ground_range_m(v, pitch_deg)
        if fwd is None:
            return None
        lat = self.ground_lateral_m(u, fwd, pitch_deg)
        # Rotate by the mount yaw into the cart's forward axis, then translate.
        psi = math.radians(self.yaw_deg)
        f2 = fwd * math.cos(psi) - lat * math.sin(psi)
        l2 = fwd * math.sin(psi) + lat * math.cos(psi)
        return f2 + self.offset_forward_m, l2 + self.offset_lateral_m

    def image_point(self, forward_m: float, lateral_m: float,
                    up_m: float = 0.0,
                    pitch_deg: Optional[float] = None
                    ) -> Optional[Tuple[float, float]]:
        """Ground/world point -> pixel. The forward direction of the inversion.

        Used to draw the route corridor and range rings onto the debug overlay,
        and to test that ``ground_point`` actually inverts what it claims to.
        """
        th = math.radians(self.pitch_deg if pitch_deg is None else pitch_deg)
        h = self.height_m - up_m
        z_c = h * math.sin(th) + forward_m * math.cos(th)
        if z_c <= 1e-6:                       # behind the image plane
            return None
        y_c = h * math.cos(th) - forward_m * math.sin(th)
        u = self.cx + self.fx * (lateral_m / z_c)
        v = self.cy + self.fy * (y_c / z_c)
        return self.distort(u, v)

    # -- the apparent-size cue ------------------------------------------
    def range_from_height_m(self, bbox_h_px: float,
                            real_height_m: float) -> Optional[float]:
        """Secondary distance cue from apparent height.

        Weaker than the contact point (it inherits the spread of real human
        heights, and collapses entirely for a partially-visible box) but it
        keeps working when the feet are occluded, which is exactly when the
        primary cue fails. Used for cross-checking, never alone.
        """
        if bbox_h_px <= 1.0 or real_height_m <= 0.0:
            return None
        return self.fy * real_height_m / bbox_h_px

    # -- honesty about error ---------------------------------------------
    def range_uncertainty_m(self, forward_m: float,
                            pitch_sigma_deg: Optional[float] = None
                            ) -> Tuple[float, float]:
        """(near_bound, far_bound) for a nominal range under pitch wobble.

        Perturbs the pitch by +/- sigma and re-inverts. Range error grows
        roughly with distance squared, so this spread widens fast: at 5 m it is
        a few per cent, at 15 m it is tens of per cent. The policy must plan
        against ``near`` — assuming an obstacle is at the closest distance
        consistent with the measurement is what makes the system safe rather
        than merely accurate on average.
        """
        sigma = self.pitch_sigma_deg if pitch_sigma_deg is None else pitch_sigma_deg
        if forward_m <= 0:
            return 0.0, 0.0
        # Find the image row this range corresponds to at nominal pitch, then
        # re-invert that same row at pitch +/- sigma.
        th = self.pitch_rad
        z_c = self.height_m * math.sin(th) + forward_m * math.cos(th)
        y_c = self.height_m * math.cos(th) - forward_m * math.sin(th)
        v = self.cy + self.fy * (y_c / z_c)
        lo = self.ground_range_m(v, self.pitch_deg + sigma)
        hi = self.ground_range_m(v, self.pitch_deg - sigma)
        cands = [c for c in (lo, hi, forward_m) if c is not None and c > 0]
        if not cands:
            return forward_m, forward_m
        # An unresolvable far bound (detection near the horizon) is reported as
        # infinity rather than silently clipped -- the caller should notice.
        far = float("inf") if (lo is None or hi is None) else max(cands)
        return min(cands), far


@dataclass
class GroundDetection:
    """One detection resolved onto the ground plane, with its error bar."""

    cls: str
    conf: float
    forward_m: float
    lateral_m: float
    forward_near_m: float            # conservative (closest plausible) range
    forward_far_m: float
    width_m: float                   # lateral extent implied by the box
    bbox: Tuple[float, float, float, float]
    size_range_m: Optional[float] = None      # apparent-height cross-check
    cue_disagreement: Optional[float] = None  # |ground - size| / ground
    on_horizon: bool = False
    clipped_bottom: bool = False              # feet below the frame: range is an
                                              # UPPER bound, not a measurement


def project_detection(cam: CameraModel, bbox: Sequence[float], cls: str,
                      conf: float, pitch_deg: Optional[float] = None
                      ) -> Optional[GroundDetection]:
    """Resolve one image-space detection to a ground-plane detection.

    ``bbox`` is (x1, y1, x2, y2) in pixels. Returns None if the box's contact
    point is not on the ground plane at all.
    """
    x1, y1, x2, y2 = (float(b) for b in bbox)
    u_mid = 0.5 * (x1 + x2)

    pt = cam.ground_point(u_mid, y2, pitch_deg)
    if pt is None:
        return None
    fwd, lat = pt
    if fwd <= 0.1 or fwd > 200.0:
        return None

    near, far = cam.range_uncertainty_m(fwd)

    # A box resting on the bottom edge of the frame has no observed contact
    # point: the feet are somewhere below the sensor, so ``fwd`` is an UPPER
    # bound on the range, not a measurement of it. This matters because it is
    # the one clipping case where BOTH cues fail the same way -- the box is
    # short as well as high, so the apparent-size cross-check also reads "far"
    # and cue_disagreement stays quiet. Widen the near bound to say so.
    clipped = y2 >= cam.height - 2.0
    if clipped:
        near = 0.0

    # Lateral extent of the box, projected at the object's own distance.
    ux1, _ = cam.undistort(x1, y2)
    ux2, _ = cam.undistort(x2, y2)
    width_m = abs(cam.ground_lateral_m(ux2, fwd, pitch_deg)
                  - cam.ground_lateral_m(ux1, fwd, pitch_deg))

    # Cross-check against apparent height. Disagreement is a health signal:
    # it usually means the ground assumption broke (a kerb, a slope, the feet
    # cut off by the frame edge) rather than that the detector was wrong.
    size_range = cam.range_from_height_m(abs(y2 - y1), CLASS_HEIGHT_M.get(cls, 0.0))
    disagreement = None
    if size_range is not None and fwd > 0.5:
        disagreement = abs(size_range - fwd) / fwd

    return GroundDetection(
        cls=cls, conf=conf, forward_m=fwd, lateral_m=lat,
        forward_near_m=near, forward_far_m=far, width_m=width_m,
        bbox=(x1, y1, x2, y2), size_range_m=size_range,
        cue_disagreement=disagreement,
        on_horizon=(y2 - cam.horizon_y(pitch_deg)) < 0.02 * cam.height,
        clipped_bottom=clipped,
    )
