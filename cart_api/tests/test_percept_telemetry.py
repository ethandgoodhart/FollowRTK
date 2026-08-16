#!/usr/bin/env python3
"""
test_percept_telemetry.py — the payload the minimap draws must mean what it says.

These are cheap tests for an easy thing to get wrong. The minimap's whole value
is that an operator can glance at it and know why the cart slowed down; a
lateral sign flip or a stale frame rendered as fresh turns it into a confident
lie, which is worse than no minimap at all.

Run: python3 -m pytest tests/test_percept_telemetry.py -q
"""

import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import percept_sim as sim  # noqa: E402
from percept_sim import Actor, SimConfig  # noqa: E402

from cartlib.percept.geometry import project_detection  # noqa: E402
from cartlib.percept.policy import Governor, PolicyConfig, evaluate  # noqa: E402
from cartlib.percept.telemetry import perception_payload  # noqa: E402
from cartlib.percept.track import (  # noqa: E402
    CLASS_GROUP, EgoPose, Track, Tracker,
)

STRAIGHT = [(0.0, 0.0), (0.0, 60.0)]


def mk_track(x, y, vx=0.0, vy=0.0, cls="person", tid=1, **kw) -> Track:
    P = np.diag([0.04, 0.04, 0.04, 0.04]).astype(float)
    kw.setdefault("last_forward_m", y)      # holds for a north-facing cart
    kw.setdefault("last_lateral_m", x)
    return Track(id=tid, cls=cls, group=CLASS_GROUP.get(cls, "vru"),
                 state=np.array([x, y, vx, vy], dtype=float), P=P,
                 confirmed=True, width_m=0.55, first_ts=0.0, last_ts=5.0,
                 hits=9, **kw)


def payload_for(tracks, ego, cfg=None, **kw) -> dict:
    cfg = cfg or PolicyConfig()
    dec = evaluate(tracks, STRAIGHT, ego, cfg)
    return perception_payload(tracks, dec, ego, cfg, **kw)


def test_the_payload_is_json_serialisable():
    """It goes straight down a websocket. A stray numpy float breaks the feed."""
    ego = EgoPose(0, 0, 0.0, 2.4)
    p = payload_for([mk_track(0.5, 12.0, vx=1.4)], ego)
    round_tripped = json.loads(json.dumps(p))
    assert round_tripped["tracks"][0]["cls"] == "person"


def test_lateral_sign_means_right_of_the_cart():
    """The one convention the whole minimap hangs on."""
    ego = EgoPose(0, 0, 0.0, 2.0)
    p = payload_for([mk_track(3.0, 10.0, tid=1), mk_track(-3.0, 10.0, tid=2)], ego)
    by_id = {t["id"]: t for t in p["tracks"]}
    assert by_id[1]["lateral_m"] > 0
    assert by_id[2]["lateral_m"] < 0


def test_velocity_is_reported_in_the_cart_frame_not_the_world():
    """A cart facing east must see a north-walking pedestrian going LEFT.

    Reporting the world velocity would draw the arrow pointing off the top of
    a minimap whose axes are the cart's, which is precisely the kind of error
    nobody notices until it matters.
    """
    east = EgoPose(0, 0, 90.0, 2.0)
    # Pedestrian 10 m ahead of an east-facing cart is at world (10, 0). The
    # cart-relative fields are set explicitly here because mk_track's shortcut
    # of reading them off (x, y) only holds for a north-facing cart.
    p = payload_for([mk_track(10.0, 0.0, vx=0.0, vy=1.5,
                              last_forward_m=10.0, last_lateral_m=0.0)], east)
    tr = p["tracks"][0]
    assert tr["forward_m"] == pytest.approx(10.0, abs=0.01)
    assert tr["vf_ms"] == pytest.approx(0.0, abs=0.01)
    assert tr["vl_ms"] == pytest.approx(-1.5, abs=0.01), "north is the cart's left"


def test_conflicting_tracks_are_flagged_for_highlighting():
    ego = EgoPose(0, 0, 0.0, 2.4)
    p = payload_for([mk_track(0.0, 8.0, tid=1), mk_track(9.0, 8.0, tid=2)], ego)
    by_id = {t["id"]: t for t in p["tracks"]}
    assert by_id[1]["conflict"] is True
    assert by_id[2]["conflict"] is False
    assert p["decision"]["limiting_track_id"] == 1


def test_the_furniture_the_minimap_draws_is_present_and_sane():
    p = payload_for([], EgoPose(0, 0, 0.0, 2.4), fov_deg=87.0)
    assert p["range_m"] > p["reflex_range_m"] > 0
    assert p["blind_zone_m"] > 0
    assert 0 < p["fov_deg"] < 180
    assert p["stopping_distance_m"] > 0


def test_mount_geometry_is_included_when_supplied():
    p = payload_for([], EgoPose(0, 0, 0.0, 2.4), height_m=1.783, pitch_deg=15.0)
    assert p["height_m"] == pytest.approx(1.783)
    assert p["pitch_deg"] == pytest.approx(15.0)
    assert "preview_jpeg" not in p


def test_stopping_distance_tracks_the_actual_speed():
    cfg = PolicyConfig()
    slow = payload_for([], EgoPose(0, 0, 0.0, 0.5), cfg)["stopping_distance_m"]
    fast = payload_for([], EgoPose(0, 0, 0.0, 2.4), cfg)["stopping_distance_m"]
    assert fast > slow >= cfg.stop_margin_m


def test_shadow_mode_is_reported_honestly():
    ego = EgoPose(0, 0, 0.0, 2.4)
    assert payload_for([], ego, shadow=True)["shadow"] is True
    assert payload_for([], ego, shadow=False)["shadow"] is False


def test_a_coasting_track_is_marked_so_the_ui_can_dash_it():
    """"Predicted" and "seen" must not look the same on screen."""
    ego = EgoPose(0, 0, 0.0, 2.4)
    seen = mk_track(0.0, 12.0, tid=1)
    ghosted = mk_track(2.0, 12.0, tid=2, coast_s=0.4)
    by_id = {t["id"]: t for t in payload_for([seen, ghosted], ego)["tracks"]}
    assert by_id[1]["coasting"] is False
    assert by_id[2]["coasting"] is True


def test_a_clipped_track_is_marked_so_the_ui_can_warn():
    ego = EgoPose(0, 0, 0.0, 2.4)
    p = payload_for([mk_track(0.0, 12.0, clipped_bottom=True)], ego)
    assert p["tracks"][0]["clipped"] is True


def test_the_reflex_layer_reaches_the_operator():
    ego = EgoPose(0, 0, 0.0, 2.4)
    p = payload_for([mk_track(0.0, 3.0)], ego)
    assert p["decision"]["layer"] == "reflex"
    assert p["decision"]["emergency"] is True
    assert p["decision"]["v_allowed_mph"] == 0.0
    assert "REFLEX" in p["decision"]["reason"]


# ---------------------------------------------------------------------------
# End to end: a real scenario's frames must all be drawable
# ---------------------------------------------------------------------------
def test_every_frame_of_a_live_scenario_produces_a_drawable_payload():
    """Runs the step-out-from-behind-a-car scenario and validates each frame.

    This is the one that catches NaNs: a track briefly at zero range, or an
    infinite far bound near the horizon, propagates into the payload as
    something JSON cannot encode and the minimap silently stops updating.
    """
    cfg, sc = PolicyConfig(), SimConfig()
    cam = sim.CameraModel(width=1920, height=1200, fx=1000.0, fy=1000.0,
                          dist=(-0.28, 0.09, 0.0, 0.0, 0.0), height_m=1.45,
                          pitch_deg=6.0, offset_forward_m=1.6)
    actors = [Actor("car", -2.6, 21.0),
              Actor("person", -2.6, 20.6, vx=1.4, t_start=2.0)]
    route = sim.straight_route()
    import random
    rng = random.Random(3)
    tracker, gov = Tracker(), Governor(cfg, initial_mph=cfg.max_speed_mph)

    s, v = 0.0, cfg.max_speed_mph / 2.2369363
    seen_conflict = False
    for k in range(400):
        t = k * sc.dt
        x, y, h = route.at(s)
        ego = EgoPose(x, y, h, v, t)
        boxes = sim.render_frame(cam, ego, actors, t,
                                 cam.pitch_deg + rng.gauss(0, 0.6), rng)
        dets = [d for b in boxes
                if (d := project_detection(cam, b.bbox, b.cls, b.conf))]
        tracks = tracker.update(dets, ego, t)
        v_cmd = gov.step(tracks, list(route.pts), ego, t, sc.dt)

        p = perception_payload(tracks, gov.decision, ego, cfg,
                               detector_hz=20.0, frame_age_s=sc.dt, ts=t,
                               fov_deg=cam.hfov_deg())
        blob = json.dumps(p)            # raises on NaN-free? no -- check by hand
        assert "NaN" not in blob and "Infinity" not in blob, f"t={t:.2f}: {blob[:300]}"
        for tr in p["tracks"]:
            assert math.isfinite(tr["forward_m"]) and math.isfinite(tr["lateral_m"])
            assert tr["radius_m"] >= 0.0
        seen_conflict |= any(tr["conflict"] for tr in p["tracks"])

        v = max(0.0, v + max(-sc.plant_decel_ms2,
                             min((v_cmd / 2.2369363 - v) / sc.plant_tau_s,
                                 sc.plant_accel_ms2)) * sc.dt)
        s = min(s + v * sc.dt, route.length)

    assert seen_conflict, "the scenario never produced a conflict to draw"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
