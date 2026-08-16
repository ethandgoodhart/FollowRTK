#!/usr/bin/env python3
"""
test_percept_preview.py — the operator preview must encode, and live mount
tuning must actually move the geometry the radar and ranging use.

Run: python3 -m pytest tests/test_percept_preview.py -q
"""

import base64
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.percept.geometry import CameraModel  # noqa: E402
from cartlib.percept.preview import encode_preview  # noqa: E402
from cartlib.percept.service import PerceptionService  # noqa: E402


def _cam(**kw) -> CameraModel:
    base = dict(width=160, height=120, fx=100.0, fy=100.0,
                height_m=1.5, pitch_deg=6.0, offset_forward_m=1.6)
    base.update(kw)
    return CameraModel(**base)


def test_encode_preview_is_a_jpeg():
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    frame[:] = (40, 40, 40)
    dets = [((10.0, 20.0, 40.0, 80.0), "person", 0.9)]
    b64 = encode_preview(frame, dets, _cam(), width=80)
    assert b64
    raw = base64.b64decode(b64)
    assert raw[:2] == b"\xff\xd8"          # JPEG SOI
    assert raw[-2:] == b"\xff\xd9"         # JPEG EOI


def test_encode_preview_survives_an_empty_frame():
    assert encode_preview(np.zeros((0, 0, 3), dtype=np.uint8)) == ""


def test_set_mount_updates_height_pitch_and_blind_zone():
    svc = PerceptionService(cam=_cam(pitch_deg=6.0, height_m=1.5))
    svc.set_mount(height_m=1.5, pitch_deg=6.0)
    bz_level = svc.cfg.blind_zone_m
    out = svc.set_mount(height_m=1.8, pitch_deg=15.0)
    assert out["ok"] is True
    assert svc.cam.height_m == pytest.approx(1.8)
    assert svc.cam.pitch_deg == pytest.approx(15.0)
    assert out["height_m"] == pytest.approx(1.8)
    assert out["pitch_deg"] == pytest.approx(15.0)
    assert out["blind_zone_m"] == svc.cfg.blind_zone_m
    assert svc.cfg.blind_zone_m < bz_level


def test_set_mount_clamps_out_of_range_values():
    svc = PerceptionService(cam=_cam())
    svc.set_mount(height_m=99.0, pitch_deg=-20.0)
    assert svc.cam.height_m == 4.0
    assert svc.cam.pitch_deg == -5.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
