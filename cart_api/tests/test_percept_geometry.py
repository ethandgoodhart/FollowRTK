#!/usr/bin/env python3
"""
test_percept_geometry.py — prove the pixels-to-metres maths actually inverts.

No hardware, no camera, no model. We place points on the ground at known
distances, project them to pixels with the forward model, and require the
inverse to give the metres back. If this file passes, a range error in the live
system is a calibration or pitch problem — not an algebra problem. That
separation is the entire reason the geometry is closed-form.

Run: python3 -m pytest tests/test_percept_geometry.py -q
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.percept.geometry import (  # noqa: E402
    CLASS_HEIGHT_M, CameraModel, project_detection,
)


def cam(**kw) -> CameraModel:
    base = dict(width=1920, height=1200, fx=1000.0, fy=1000.0,
                height_m=1.5, pitch_deg=0.0)
    base.update(kw)
    return CameraModel(**base)


# ---------------------------------------------------------------------------
# The core inversion
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("pitch", [0.0, 2.0, 5.0, 10.0, -3.0])
@pytest.mark.parametrize("fwd", [2.0, 5.0, 10.0, 20.0, 40.0])
@pytest.mark.parametrize("lat", [0.0, -1.5, 3.0])
def test_ground_point_round_trip(pitch, fwd, lat):
    """Project a ground point to a pixel, invert it, get the metres back."""
    c = cam(pitch_deg=pitch)
    px = c.image_point(fwd, lat)
    assert px is not None, "point should be imageable"
    got = c.ground_point(px[0], px[1])
    assert got is not None, "inversion should land on the ground plane"
    gf, gl = got
    assert gf == pytest.approx(fwd, rel=1e-6, abs=1e-6)
    assert gl == pytest.approx(lat, rel=1e-6, abs=1e-6)


def test_zero_pitch_matches_the_simple_formula():
    """At pitch 0 the general inversion must collapse to h*fy/(v-cy)."""
    c = cam(pitch_deg=0.0)
    for v in (700.0, 800.0, 1000.0, 1199.0):
        simple = c.height_m * c.fy / (v - c.cy)
        assert c.ground_range_m(v) == pytest.approx(simple, rel=1e-9)


def test_horizon_is_refused_not_guessed():
    """Rows at or above the horizon are not ground points. Say so."""
    for pitch in (0.0, 3.0, 8.0):
        c = cam(pitch_deg=pitch)
        h = c.horizon_y()
        assert c.ground_range_m(h) is None
        assert c.ground_range_m(h - 1.0) is None      # above horizon
        assert c.ground_range_m(h - 200.0) is None
        assert c.ground_range_m(h + 5.0) is not None  # just below: valid
    # Pitching down moves the horizon up the image.
    assert cam(pitch_deg=5.0).horizon_y() < cam(pitch_deg=0.0).horizon_y()


def test_range_grows_as_contact_point_rises():
    """Monotonicity: higher in the image (smaller v) == further away."""
    c = cam(pitch_deg=3.0)
    rows = [1150.0, 1000.0, 900.0, 800.0, 750.0]
    ranges = [c.ground_range_m(v) for v in rows]
    assert all(r is not None for r in ranges)
    assert ranges == sorted(ranges), "range must increase as v decreases"


def test_mount_yaw_rotates_into_the_cart_frame():
    """A camera aimed 10 deg right must report straight-ahead objects as right."""
    straight = cam(yaw_deg=0.0)
    skewed = cam(yaw_deg=10.0)
    px = straight.image_point(10.0, 0.0)
    f_s, l_s = straight.ground_point(*px)
    f_k, l_k = skewed.ground_point(*px)
    assert l_s == pytest.approx(0.0, abs=1e-6)
    assert l_k == pytest.approx(10.0 * math.sin(math.radians(10.0)), abs=1e-3)
    assert f_k == pytest.approx(10.0 * math.cos(math.radians(10.0)), abs=1e-3)


def test_mount_offset_is_applied():
    c = cam(offset_forward_m=1.2, offset_lateral_m=-0.3)
    px = cam().image_point(8.0, 0.0)
    f, l = c.ground_point(*px)
    assert f == pytest.approx(9.2, abs=1e-6)
    assert l == pytest.approx(-0.3, abs=1e-6)


# ---------------------------------------------------------------------------
# Distortion
# ---------------------------------------------------------------------------
def test_undistort_inverts_distort():
    """The iterative undistort must invert the forward model sub-pixel."""
    c = cam(dist=(-0.32, 0.11, 0.0005, -0.0004, -0.02))
    for u, v in [(960, 600), (100, 100), (1850, 1150), (50, 1100), (1900, 80)]:
        du, dv = c.distort(u, v)
        ru, rv = c.undistort(du, dv)
        assert ru == pytest.approx(u, abs=0.05)
        assert rv == pytest.approx(v, abs=0.05)


def test_distortion_actually_moves_edge_pixels():
    """Guard against a no-op: barrel distortion must matter at the edges.

    This is the reason undistortion is mandatory rather than nice-to-have --
    if it were negligible the test would be pointless, so assert it isn't.
    """
    c = cam(dist=(-0.32, 0.11, 0.0, 0.0, 0.0))
    du, dv = c.distort(1850, 1150)
    assert math.hypot(du - 1850, dv - 1150) > 20.0, "edge shift should be large"
    cu, cv = c.distort(965, 605)
    assert math.hypot(cu - 965, cv - 605) < 1.0, "centre should barely move"


def test_ground_point_uses_undistortion():
    """A distorted pixel must not be read as if it were pinhole."""
    c = cam(dist=(-0.30, 0.10, 0.0, 0.0, 0.0), pitch_deg=2.0)
    px = c.image_point(12.0, 4.0)            # forward model applies distortion
    f, l = c.ground_point(*px)
    assert f == pytest.approx(12.0, rel=1e-4)
    assert l == pytest.approx(4.0, rel=1e-4)


# ---------------------------------------------------------------------------
# Uncertainty — the part the policy depends on
# ---------------------------------------------------------------------------
def test_uncertainty_brackets_the_estimate_and_widens_with_range():
    c = cam(pitch_deg=2.0, pitch_sigma_deg=1.0)
    spreads = []
    for d in (5.0, 10.0, 15.0, 25.0):
        near, far = c.range_uncertainty_m(d)
        assert near <= d <= far, f"{near} <= {d} <= {far}"
        spreads.append((far - near) / d)
    assert spreads == sorted(spreads), "relative spread must grow with distance"


def test_uncertainty_magnitude_is_in_the_expected_ballpark():
    """The numbers the plan was written around: a few % at 5 m, tens at 15 m.

    Pinned deliberately. If a refactor makes ranging look implausibly precise,
    that is a bug in the error model and it should break the build here rather
    than quietly make the speed policy over-confident.
    """
    c = cam(pitch_deg=0.0, pitch_sigma_deg=1.0)
    n5, f5 = c.range_uncertainty_m(5.0)
    n15, f15 = c.range_uncertainty_m(15.0)
    assert 0.02 < (f5 - n5) / 5.0 < 0.20
    assert 0.20 < (f15 - n15) / 15.0 < 0.90
    # And the near bound is what keeps us safe: always an under-estimate.
    assert n5 < 5.0 and n15 < 15.0


def test_uncertainty_reports_infinity_near_the_horizon():
    """If a small pitch error puts the object beyond the horizon, say infinity."""
    c = cam(pitch_deg=0.5, pitch_sigma_deg=1.0)
    near, far = c.range_uncertainty_m(120.0)
    assert far == float("inf")
    assert near < 120.0


# ---------------------------------------------------------------------------
# Whole-detection projection
# ---------------------------------------------------------------------------
def _person_bbox(c: CameraModel, fwd: float, lat: float):
    """Synthesise the box a 1.70 m person at (fwd, lat) would produce."""
    feet = c.image_point(fwd, lat, up_m=0.0)
    head = c.image_point(fwd, lat, up_m=CLASS_HEIGHT_M["person"])
    half_w = abs(c.fx * 0.275 / fwd)
    return (feet[0] - half_w, head[1], feet[0] + half_w, feet[1])


@pytest.mark.parametrize("fwd", [3.0, 6.0, 12.0, 20.0])
def test_project_detection_recovers_a_synthetic_person(fwd):
    c = cam(pitch_deg=2.0)
    det = project_detection(c, _person_bbox(c, fwd, 1.0), "person", 0.9)
    assert det is not None
    assert det.forward_m == pytest.approx(fwd, rel=1e-3)
    assert det.lateral_m == pytest.approx(1.0, rel=1e-2)
    assert det.width_m == pytest.approx(0.55, rel=0.1)
    # Both cues agree for a synthetic person standing on flat ground.
    assert det.cue_disagreement is not None
    assert det.cue_disagreement < 0.05
    assert det.forward_near_m <= det.forward_m <= det.forward_far_m


def test_cue_disagreement_flags_a_broken_ground_assumption():
    """A person standing on a kerb breaks the flat-ground assumption.

    The whole box translates upward: height is preserved, so the size cue still
    reads the true distance, while the contact-point cue confidently reports
    something FURTHER away -- the dangerous direction. The two cues must
    visibly disagree, which is the entire reason the second cue exists.
    """
    c = cam(pitch_deg=2.0)
    x1, y1, x2, y2 = _person_bbox(c, 8.0, 0.0)
    on_kerb = (x1, y1 - 60.0, x2, y2 - 60.0)   # translated, height preserved
    det = project_detection(c, on_kerb, "person", 0.9)
    assert det is not None
    assert det.forward_m > 8.0, "contact-point cue over-estimates range here"
    assert det.size_range_m == pytest.approx(8.0, rel=0.05), "size cue stays right"
    assert det.cue_disagreement > 0.15, "disagreement must be visible"


def test_occluded_feet_are_a_known_blind_spot_of_the_cross_check():
    """Documented limitation, asserted so nobody assumes more coverage than we have.

    When the feet are hidden behind something, the box bottom rises AND the box
    shrinks. Both cues then move the same way and AGREE on a too-far distance,
    so ``cue_disagreement`` cannot catch it. Detecting this needs the occlusion
    reasoning in the policy layer, not more geometry -- and the test exists so
    that if someone later claims the cross-check covers occlusion, it doesn't
    silently pass.
    """
    c = cam(pitch_deg=2.0)
    x1, y1, x2, y2 = _person_bbox(c, 8.0, 0.0)
    feet_hidden = (x1, y1, x2, y2 - 60.0)      # bottom cropped, top unchanged
    det = project_detection(c, feet_hidden, "person", 0.9)
    assert det is not None
    assert det.forward_m > 8.0, "range is over-estimated..."
    assert det.cue_disagreement < 0.15, "...and the cross-check does NOT catch it"


def test_detection_above_the_horizon_is_rejected():
    c = cam(pitch_deg=0.0)
    h = c.horizon_y()
    assert project_detection(c, (900, h - 200, 1000, h - 10), "person", 0.9) is None


def test_on_horizon_flag_marks_untrustworthy_far_detections():
    c = cam(pitch_deg=1.0)
    far_det = project_detection(c, _person_bbox(c, 150.0, 0.0), "person", 0.8)
    near_det = project_detection(c, _person_bbox(c, 6.0, 0.0), "person", 0.8)
    assert near_det is not None and not near_det.on_horizon
    if far_det is not None:
        assert far_det.on_horizon


def test_fov_matches_the_real_camera_geometry():
    """Sanity: 1920 px at fx=1000 is a ~87 deg horizontal field of view."""
    c = cam()
    assert 80.0 < c.hfov_deg() < 95.0
    assert 55.0 < c.vfov_deg() < 70.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
