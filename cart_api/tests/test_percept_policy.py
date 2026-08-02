#!/usr/bin/env python3
"""
test_percept_policy.py — unit tests for the speed envelope and its layers.

The scenario suite proves the whole stack does not hit people. This file proves
the individual pieces mean what they say, so that when a scenario fails there is
somewhere to look. Everything here is a pure function of its arguments except
the two classes at the end, which is deliberate: the policy keeps its state in
exactly two places.

Run: python3 -m pytest tests/test_percept_policy.py -q
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cartlib.percept.policy import (  # noqa: E402
    MPH_TO_MPS, MPS_TO_MPH, Governor, PolicyConfig, SpeedLimiter,
    effective_reaction_s, evaluate, find_conflicts, health_cap_mph,
    reflex_protected_speed_mph, reflex_triggered, route_stations,
    safe_speed_for_distance, stopping_distance_m,
)
from cartlib.percept.track import CLASS_GROUP, EgoPose, Track  # noqa: E402


def cfg(**kw) -> PolicyConfig:
    c = PolicyConfig()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def mk_track(x, y, vx=0.0, vy=0.0, cls="person", tid=1, confirmed=True,
             pos_sigma=0.2, vel_sigma=0.2, width=0.55, observed_s=5.0,
             **kw) -> Track:
    """A track placed exactly where we want it, with tidy covariances.

    The cart-relative fields are filled in for a north-facing cart at the
    origin, matching ``north()`` below, so (x, y) reads as (lateral, forward).
    """
    P = np.diag([pos_sigma ** 2, pos_sigma ** 2,
                 vel_sigma ** 2, vel_sigma ** 2]).astype(float)
    t = Track(id=tid, cls=cls, group=CLASS_GROUP.get(cls, "vru"),
              state=np.array([x, y, vx, vy], dtype=float), P=P,
              confirmed=confirmed, width_m=width,
              first_ts=0.0, last_ts=observed_s, hits=9,
              last_forward_m=y, last_lateral_m=x)
    t.last_lat_sigma_m = 0.15
    for k, v in kw.items():
        setattr(t, k, v)
    return t


def north(x=0.0, y=0.0, speed=2.46) -> EgoPose:
    return EgoPose(x=x, y=y, heading_deg=0.0, speed_ms=speed, ts=0.0)


STRAIGHT = [(0.0, 0.0), (0.0, 60.0)]


# ---------------------------------------------------------------------------
# The envelope
# ---------------------------------------------------------------------------
def test_effective_reaction_charges_for_the_jerk_ramp():
    c = cfg(reaction_s=0.8, jerk_limit_ms3=1.0)
    assert effective_reaction_s(c, 1.0) == pytest.approx(1.3)
    # Softer jerk limit means a longer ramp means more effective dead time.
    assert effective_reaction_s(cfg(jerk_limit_ms3=0.5), 1.0) > \
        effective_reaction_s(cfg(jerk_limit_ms3=2.0), 1.0)


def test_inside_the_margin_means_stop():
    c = cfg(stop_margin_m=3.5)
    assert safe_speed_for_distance(3.5, c) == 0.0
    assert safe_speed_for_distance(2.0, c) == 0.0
    assert safe_speed_for_distance(0.0, c) == 0.0
    assert safe_speed_for_distance(-5.0, c) == 0.0


def test_safe_speed_rises_with_distance():
    c = cfg()
    v = [safe_speed_for_distance(d, c) for d in (4, 6, 10, 20, 40, 80)]
    assert v == sorted(v)
    assert v[0] < v[-1]


@pytest.mark.parametrize("d", [5.0, 8.0, 15.0, 30.0, 60.0])
def test_safe_speed_and_stopping_distance_are_inverses(d):
    """The two directions of the same physics must agree, or one is wrong."""
    c = cfg()
    v = safe_speed_for_distance(d, c)
    assert stopping_distance_m(v, c) == pytest.approx(d, rel=1e-6)


def test_an_approaching_obstacle_lowers_the_allowed_speed():
    """The head-on case: the gap closes while we are braking."""
    c = cfg()
    still = safe_speed_for_distance(15.0, c, approach_ms=0.0)
    walking = safe_speed_for_distance(15.0, c, approach_ms=1.4)
    running = safe_speed_for_distance(15.0, c, approach_ms=3.0)
    assert running < walking < still
    assert walking < 0.75 * still, "walking pace should matter a lot"


def test_a_receding_obstacle_earns_no_bonus():
    """Being handed extra speed because somebody is walking away is a bet on
    them not turning round. Refuse it."""
    c = cfg()
    assert safe_speed_for_distance(15.0, c, approach_ms=-2.0) == \
        pytest.approx(safe_speed_for_distance(15.0, c, approach_ms=0.0))


def test_a_fast_enough_approach_forces_a_full_stop():
    c = cfg()
    assert safe_speed_for_distance(6.0, c, approach_ms=11.0) == 0.0


def test_weaker_brakes_demand_lower_speeds():
    c = cfg()
    assert safe_speed_for_distance(20.0, c, decel_ms2=0.6) < \
        safe_speed_for_distance(20.0, c, decel_ms2=2.0)


def test_the_reflex_layer_cannot_guarantee_a_stop_at_full_speed():
    """A documented limitation, asserted so it cannot be quietly forgotten.

    With the braking the cart actually has (~1.0 m/s^2, measured), the reflex
    layer only guarantees a stop from about 3.5 mph -- well under the 5.5 mph
    default. Above that, L0 reduces the severity of a sudden step-out rather
    than preventing it, and the real protection is L1 having already slowed
    down for everything visible.

    If this assertion starts failing because the number went UP, someone has
    found more braking authority and the default max speed should be revisited.
    """
    c = cfg()
    guaranteed = reflex_protected_speed_mph(c)
    assert 3.0 < guaranteed < 4.0, f"reflex-protected speed is {guaranteed:.2f} mph"
    assert guaranteed < c.max_speed_mph, (
        "if the reflex covers full speed, say so and delete this test")


# ---------------------------------------------------------------------------
# Route stations
# ---------------------------------------------------------------------------
def test_stations_start_under_the_cart_and_step_evenly():
    c = cfg(station_step_m=0.25, horizon_m=10.0)
    st = route_stations(STRAIGHT, (0.0, 3.0), c)
    assert st[0][0] == 0.0
    assert st[0][1:] == pytest.approx((0.0, 3.0))
    gaps = [st[i + 1][0] - st[i][0] for i in range(len(st) - 1)]
    assert all(g == pytest.approx(0.25) for g in gaps)
    assert st[-1][0] <= c.horizon_m + 0.25


def test_stations_measure_arc_length_not_straight_line():
    """A right-angle route: the station 20 m along is not 20 m away."""
    c = cfg(station_step_m=0.5, horizon_m=25.0)
    st = route_stations([(0, 0), (0, 10), (20, 10)], (0.0, 0.0), c)
    s20 = min(st, key=lambda p: abs(p[0] - 20.0))
    assert s20[1:] == pytest.approx((10.0, 10.0), abs=0.6)
    assert math.hypot(s20[1], s20[2]) < 20.0


def test_stations_are_relative_to_the_projection_not_the_route_start():
    c = cfg(horizon_m=10.0)
    st = route_stations(STRAIGHT, (0.0, 25.0), c)
    assert st[0][2] == pytest.approx(25.0)


def test_a_degenerate_route_yields_nothing_rather_than_crashing():
    assert route_stations([], (0, 0), cfg()) == []
    assert route_stations([(0, 0)], (0, 0), cfg()) == []


# ---------------------------------------------------------------------------
# L0 reflex
# ---------------------------------------------------------------------------
def test_reflex_fires_for_something_close_and_ahead():
    c = cfg(reflex_range_m=4.5)
    t = mk_track(0, 3, last_forward_m=3.0, last_lateral_m=0.1)
    assert reflex_triggered([t], c) is t


def test_reflex_ignores_things_beside_and_beyond():
    c = cfg(reflex_range_m=4.5, reflex_half_width_m=1.2)
    far = mk_track(0, 9, tid=1, last_forward_m=9.0, last_lateral_m=0.0)
    wide = mk_track(3, 3, tid=2, last_forward_m=3.0, last_lateral_m=3.0)
    behind = mk_track(0, -3, tid=3, last_forward_m=-3.0, last_lateral_m=0.0)
    assert reflex_triggered([far, wide, behind], c) is None


def test_reflex_takes_the_nearest_of_several():
    c = cfg()
    a = mk_track(0, 4, tid=1, last_forward_m=4.0, last_lateral_m=0.0)
    b = mk_track(0, 2, tid=2, last_forward_m=2.0, last_lateral_m=0.0)
    assert reflex_triggered([a, b], c) is b


def test_reflex_will_not_slam_the_brakes_on_an_unconfirmed_track():
    c = cfg()
    t = mk_track(0, 3, confirmed=False, last_forward_m=3.0, last_lateral_m=0.0)
    assert reflex_triggered([t], c) is None


def test_reflex_distrusts_a_range_read_off_a_clipped_box():
    """Feet below the frame: the measured range is an upper bound, so a large
    value is not evidence of safety. It usually means the opposite."""
    c = cfg(reflex_range_m=4.5)
    t = mk_track(0, 20, last_forward_m=20.0, last_lateral_m=0.0,
                 clipped_bottom=True)
    assert reflex_triggered([t], c) is t


# ---------------------------------------------------------------------------
# L2 health
# ---------------------------------------------------------------------------
def test_health_layer_stops_us_when_perception_is_offline():
    assert health_cap_mph([], cfg(), detector_ok=False)[0] == 0.0


def test_health_layer_stops_us_on_a_stale_frame():
    assert health_cap_mph([], cfg(), frame_age_s=1.0)[0] == 0.0


def test_health_layer_slows_us_on_a_merely_slow_frame():
    c = cfg()
    cap, why = health_cap_mph([], c, frame_age_s=0.3)
    assert cap == c.degraded_speed_mph and why


def test_health_layer_slows_us_when_most_range_cues_disagree():
    """Cue disagreement across the scene means the flat-ground assumption is
    broken -- a slope, a kerb, a mis-set pitch. Every range is then suspect."""
    c = cfg()
    bad = [mk_track(0, 10 + i, tid=i, cue_disagreement=0.9) for i in range(3)]
    cap, why = health_cap_mph(bad, c)
    assert cap == c.degraded_speed_mph and "ground plane" in why


def test_health_layer_does_not_degrade_on_one_odd_track():
    """One disagreeing track among several is occlusion, not geometry.

    Two pedestrians overlapping in bearing is enough to produce it, and that
    happens constantly on a footpath. Degrading here would have the cart crawl
    past every pair of people it meets.
    """
    c = cfg()
    tracks = [mk_track(0, 10, tid=1, cue_disagreement=0.9),
              mk_track(2, 12, tid=2, cue_disagreement=0.03),
              mk_track(-2, 14, tid=3, cue_disagreement=0.05)]
    assert health_cap_mph(tracks, c)[0] == float("inf")


def test_health_layer_ignores_disagreement_it_can_already_explain():
    """A clipped box's cues disagree for a known reason that says nothing
    about the ground plane."""
    c = cfg()
    clipped = [mk_track(0, 5 + i, tid=i, cue_disagreement=0.9,
                        clipped_bottom=True) for i in range(3)]
    assert health_cap_mph(clipped, c)[0] == float("inf")


def test_a_healthy_system_imposes_no_cap():
    assert health_cap_mph([mk_track(0, 10, cue_disagreement=0.02)],
                          cfg())[0] == float("inf")


# ---------------------------------------------------------------------------
# Conflicts
# ---------------------------------------------------------------------------
def test_something_standing_on_the_route_is_a_conflict():
    c = cfg()
    st = route_stations(STRAIGHT, (0, 0), c)
    con = find_conflicts([mk_track(0.0, 12.0)], st, north(), c)
    assert len(con) == 1
    # The reported station is where the corridor first touches the object's
    # extent, not its centre -- about a metre and a half nearer. That is the
    # distance the envelope must plan against, so it is the one to report.
    assert 10.0 < con[0].station_m < 12.1


def test_something_standing_well_off_the_route_is_not():
    c = cfg()
    st = route_stations(STRAIGHT, (0, 0), c)
    assert find_conflicts([mk_track(3.5, 12.0)], st, north(), c) == []


def test_a_conflict_further_away_permits_more_speed():
    c = cfg()
    st = route_stations(STRAIGHT, (0, 0), c)
    near = find_conflicts([mk_track(0.0, 8.0)], st, north(), c)[0]
    far = find_conflicts([mk_track(0.0, 25.0)], st, north(), c)[0]
    assert far.v_safe_ms > near.v_safe_ms


def test_conflicts_come_back_worst_first():
    c = cfg()
    st = route_stations(STRAIGHT, (0, 0), c)
    con = find_conflicts([mk_track(0.0, 22.0, tid=1),
                          mk_track(0.0, 7.0, tid=2)], st, north(), c)
    assert [x.track_id for x in con] == [2, 1]


def test_a_crosser_who_will_be_gone_is_not_a_conflict():
    """3 m/s across, 30 m ahead: clear of the corridor many seconds before we
    could possibly arrive. Braking here is the behaviour that gets the whole
    system turned off."""
    c = cfg()
    st = route_stations(STRAIGHT, (0, 0), c)
    con = find_conflicts([mk_track(-1.0, 30.0, vx=3.0, vel_sigma=0.2)],
                         st, north(), c)
    assert all(x.v_safe_ms * MPS_TO_MPH > c.max_speed_mph for x in con)


def test_no_route_means_no_nominal_conflicts():
    """L1 needs a route. L0 does not, which is the point of L0."""
    assert find_conflicts([mk_track(0, 5)], [], north(), cfg()) == []


# ---------------------------------------------------------------------------
# Combining the layers
# ---------------------------------------------------------------------------
def test_reflex_beats_everything_else():
    d = evaluate([mk_track(0, 3, last_forward_m=3.0, last_lateral_m=0.0)],
                 STRAIGHT, north(), cfg())
    assert d.v_allowed_mph == 0.0 and d.emergency and d.layer == "reflex"


def test_an_empty_scene_allows_full_speed():
    c = cfg()
    d = evaluate([], STRAIGHT, north(), c)
    assert d.v_allowed_mph == c.max_speed_mph
    assert d.layer == "clear" and not d.emergency and not d.degraded


def test_no_layer_can_authorise_more_than_another_allows():
    """The whole architecture rests on this: the layers combine by min."""
    c = cfg(degraded_speed_mph=1.0)
    d = evaluate([mk_track(0.0, 60.0)], STRAIGHT, north(), c, frame_age_s=0.3)
    assert d.v_allowed_mph <= 1.0 and d.degraded


def test_the_decision_explains_itself():
    d = evaluate([mk_track(0.0, 9.0, tid=42)], STRAIGHT, north(), cfg())
    assert d.limiting_track_id == 42
    assert "42" in d.reason
    js = d.to_dict()
    assert js["limiting_track_id"] == 42 and js["conflicts"]


# ---------------------------------------------------------------------------
# Smoothing
# ---------------------------------------------------------------------------
def test_the_limiter_respects_its_acceleration_and_jerk_budgets():
    c = cfg()
    lim = SpeedLimiter(c, initial_mph=0.0)
    target_ms = c.max_speed_mph * MPH_TO_MPS
    prev_v, prev_a, t = 0.0, 0.0, 0.0
    for _ in range(400):
        v = lim.step(c.max_speed_mph, 0.05, t) * MPH_TO_MPS
        a = (v - prev_v) / 0.05
        assert a <= c.accel_limit_ms2 + 1e-6
        # Checked while there is still a speed error worth shaping. A discrete
        # jerk-limited profile cannot land exactly on its target without a
        # final sub-tick adjustment; asserting through it would be testing the
        # integrator's rounding, not the ride.
        if abs(target_ms - v) > 0.02:
            assert abs(a - prev_a) / 0.05 <= c.jerk_limit_ms3 + 1e-6
        prev_v, prev_a, t = v, a, t + 0.05
    assert prev_v == pytest.approx(c.max_speed_mph * MPH_TO_MPS, rel=1e-3)


def test_the_limiter_respects_its_deceleration_budget():
    c = cfg()
    lim = SpeedLimiter(c, initial_mph=c.max_speed_mph)
    prev_v, t = c.max_speed_mph * MPH_TO_MPS, 0.0
    for _ in range(200):
        v = lim.step(0.0, 0.05, t) * MPH_TO_MPS
        assert (v - prev_v) / 0.05 >= -c.decel_limit_ms2 - 1e-6
        prev_v, t = v, t + 0.05
    assert prev_v == pytest.approx(0.0, abs=1e-3)


def test_the_limiter_never_commands_a_negative_speed():
    lim = SpeedLimiter(cfg(), initial_mph=1.0)
    for i in range(100):
        assert lim.step(0.0, 0.05, i * 0.05) >= 0.0


def test_a_blinking_detection_cannot_pump_the_throttle():
    """Alternate 'stop' and 'go' every tick for four seconds. The hold-off
    means the cart settles low instead of surging on every other frame."""
    c = cfg()
    lim = SpeedLimiter(c, initial_mph=c.max_speed_mph)
    out = [lim.step(c.max_speed_mph if i % 2 else 0.0, 0.05, i * 0.05)
           for i in range(80)]
    assert out[-1] < 1.0, f"ended at {out[-1]:.2f} mph"
    assert max(out[k + 1] - out[k] for k in range(len(out) - 1)) < 0.2


def test_the_emergency_path_ignores_the_comfort_budget():
    """Jerk limits are a preference. They are not worth a collision."""
    c = cfg()
    comfy = SpeedLimiter(c, initial_mph=c.max_speed_mph)
    panic = SpeedLimiter(c, initial_mph=c.max_speed_mph)
    for i in range(12):
        comfy.step(0.0, 0.05, i * 0.05, emergency=False)
        panic.step(0.0, 0.05, i * 0.05, emergency=True)
    assert panic.v_ms < comfy.v_ms


def test_resuming_waits_out_the_hold_off():
    c = cfg(resume_hold_s=1.5)
    lim = SpeedLimiter(c, initial_mph=0.0)
    lim.step(0.0, 0.05, 0.0)                     # arms the hold-off
    assert lim.step(c.max_speed_mph, 0.05, 0.5) == pytest.approx(0.0)
    assert lim.step(c.max_speed_mph, 0.05, 1.4) == pytest.approx(0.0)
    assert lim.step(c.max_speed_mph, 0.05, 2.0) > 0.0


# ---------------------------------------------------------------------------
# The governor's latch
# ---------------------------------------------------------------------------
def test_the_latch_holds_the_stop_after_the_trigger_disappears():
    """The blind-zone case. An obstacle that vanishes under the bonnet must not
    read as an obstacle that left."""
    c = cfg(reflex_latch_s=1.5)
    g = Governor(c, initial_mph=2.0)
    close = mk_track(0, 3, last_forward_m=3.0, last_lateral_m=0.0)
    g.step([close], STRAIGHT, north(), 0.0, 0.05)
    assert g.decision.emergency

    g.step([], STRAIGHT, north(speed=0.0), 0.5, 0.05)   # vanished
    assert g.decision.v_allowed_mph == 0.0
    assert "blind zone" in g.decision.reason

    g.step([], STRAIGHT, north(speed=0.0), 1.4, 0.05)
    assert g.decision.v_allowed_mph == 0.0

    g.step([], STRAIGHT, north(speed=0.0), 1.7, 0.05)   # latch expired
    assert g.decision.v_allowed_mph > 0.0


def test_the_governor_is_the_only_thing_holding_state():
    c = cfg()
    g = Governor(c, initial_mph=c.max_speed_mph)
    for i in range(20):
        g.step([], STRAIGHT, north(), i * 0.05, 0.05)
    assert g.limiter.v_ms > 0
    g.reset()
    assert g.limiter.v_ms == 0.0 and g._latch_until < 0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
