"""
Tests for the calibration loader.

The interesting cases here are all about refusing rather than computing. A
calibration loader that quietly does the wrong thing produces ranges that are
smoothly, plausibly wrong at every distance, and nothing downstream can detect
it -- the tracker will happily track a person who is 20% closer than reported
and the policy will happily brake 20% too late.
"""

from __future__ import annotations

import copy
import json
import math
import os

import pytest

from cartlib.percept import calib as calib_mod


@pytest.fixture
def base():
    return calib_mod.load()


def test_the_shipped_calibration_loads_and_is_a_real_calibration(base):
    assert base["intrinsics"]["measured"] is True
    assert base["intrinsics"]["rms_reprojection_error_px"] < 0.5
    # Distortion must be non-zero: a "calibration" of all zeros is a placeholder
    # wearing a calibration's clothes.
    assert any(abs(d) > 1e-6 for d in base["intrinsics"]["dist"])


def test_intrinsics_scale_to_the_capture_resolution(base):
    cam = calib_mod.camera_model(base)
    ins = base["intrinsics"]
    s = cam.width / ins["calibrated_width"]
    assert cam.fx == pytest.approx(ins["fx"] * s)
    assert cam.cy == pytest.approx(ins["cy"] * s)
    # Field of view is a property of the lens, so scaling must not change it.
    ref = calib_mod.camera_model(base, ins["calibrated_width"],
                                 ins["calibrated_height"])
    assert cam.hfov_deg() == pytest.approx(ref.hfov_deg(), abs=1e-9)
    assert cam.vfov_deg() == pytest.approx(ref.vfov_deg(), abs=1e-9)


def test_a_different_aspect_ratio_is_refused_not_guessed(base):
    # 1920x1200 is 16:10 against a 4:3 calibration. Scaling fx by 3.0 and fy by
    # 2.5 would be inventing a lens that was never calibrated.
    with pytest.raises(ValueError, match="aspect"):
        calib_mod.camera_model(base, 1920, 1200)


def test_the_aspect_refusal_can_be_overridden_deliberately(base):
    cam = calib_mod.camera_model(base, 1920, 1200, allow_aspect_mismatch=True)
    assert cam.fx == pytest.approx(base["intrinsics"]["fx"] * 3.0)


def test_capture_size_matches_the_calibrated_aspect(base):
    w, h = calib_mod.capture_size(base)
    ins = base["intrinsics"]
    assert w / h == pytest.approx(ins["calibrated_width"]
                                  / ins["calibrated_height"], rel=1e-6)


def test_an_unmeasured_mount_is_announced_in_the_description(base):
    cam = calib_mod.camera_model(base)
    text = calib_mod.describe(cam, base)
    if calib_mod.mount_measured(base):
        assert "NOT MEASURED" not in text
    else:
        assert "NOT MEASURED" in text
        assert "percept_ground_calib" in text


def test_the_ground_plane_is_consistent_with_the_loaded_geometry(base):
    """A row below the horizon must give a positive, monotonically nearer range."""
    cam = calib_mod.camera_model(base)
    horizon = cam.horizon_y()
    assert 0 < horizon < cam.height          # horizon must be in frame at all
    rows = [horizon + 50, horizon + 200, cam.height - 1]
    ranges = [cam.ground_range_m(r) for r in rows]
    assert all(r is not None and r > 0 for r in ranges)
    assert ranges[0] > ranges[1] > ranges[2]  # lower in frame == closer


def test_a_row_above_the_horizon_returns_no_range(base):
    cam = calib_mod.camera_model(base)
    assert cam.ground_range_m(cam.horizon_y() - 1.0) is None


def test_a_calibration_missing_intrinsics_raises(tmp_path, base):
    bad = copy.deepcopy(base)
    del bad["intrinsics"]["fx"]
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(bad))
    with pytest.raises(KeyError):
        calib_mod.camera_model(calib_mod.load(str(p)))


def test_default_calib_path_points_at_the_repo_file():
    assert os.path.isfile(calib_mod.DEFAULT_CALIB)
    assert calib_mod.DEFAULT_CALIB.endswith(
        os.path.join("calibration", "front_camera.json"))
