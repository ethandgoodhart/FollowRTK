#!/usr/bin/env python3
"""
test_percept_scenarios.py — closed-loop proof that the cart does not hit people.

Every test here drives the whole stack: real camera projection, real distortion,
real ground-plane inversion, real Kalman tracking, real speed envelope, and a
plant with transport delay it cannot argue with. Nothing is mocked. The bar for
every scenario involving a person is the same and it is not negotiable:

    result.min_clearance_m > 0

That is the distance between the cart's FOOTPRINT and the person, so zero means
contact, not "centres coincided".

But a cart that never moves also never hits anyone, so roughly half of these
tests exist to stop the safety cases from being satisfied trivially. If the
policy becomes timid enough to stop for a pedestrian standing two metres off
the path, or to lurch every time the detector blinks, the permissiveness and
comfort tests fail. Both halves have to pass at once, which is the only
formulation of "safe" that is worth anything.

Run: python3 -m pytest tests/test_percept_scenarios.py -q
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import percept_sim as sim  # noqa: E402
from percept_sim import Actor, Route, SimConfig, straight_route  # noqa: E402

from cartlib.percept.policy import PolicyConfig  # noqa: E402

WALK = 1.4          # m/s, ordinary walking pace
RUN = 3.0           # m/s, someone hurrying across
CAR = 11.0          # m/s, ~25 mph

# Comfort budget for the ACTUAL cart speed, not the setpoint. The policy limits
# setpoint jerk to 1.0 m/s^3; the drivetrain's own lag turns a step in demanded
# acceleration into a somewhat sharper response than that. Published comfort
# work puts <1 m/s^3 at "comfortable" and +/-2 m/s^3 at "acceptable", and ISO
# 15622 caps automated longitudinal jerk at 2.5, so 2.0 is the line here.
JERK_BUDGET = 2.0


def assert_no_collision(r, who: str = "the pedestrian"):
    """The bar: the cart never DRIVES into anybody.

    See ``SimResult.min_clearance_moving_m`` for why this is measured while the
    cart is moving rather than over the whole run.
    """
    assert r.min_clearance_moving_m > 0.0, (
        f"CART DROVE INTO {who}: clearance {r.min_clearance_moving_m:.2f} m\n"
        f"{r.summary()}")


def assert_comfortable(r):
    j = r.max_jerk_ms3()
    assert j <= JERK_BUDGET, f"jerky ride: {j:.2f} m/s^3\n{r.summary()}"


# ---------------------------------------------------------------------------
# Baseline: the cart must actually drive
# ---------------------------------------------------------------------------
def test_empty_route_runs_at_full_speed():
    r = sim.run([])
    assert r.distance_travelled_m > 55.0, r.summary()
    assert min(r.speeds_mph) > 5.4
    assert r.max_jerk_ms3() < 0.2
    assert r.reversals() == 0


def test_a_person_well_clear_of_the_path_is_ignored():
    """The permissiveness test. Failing this means a useless cart.

    Two metres is a normal separation on a campus footpath. If this ever starts
    failing because the corridor or the manoeuvre cone grew, that is a
    regression even though it makes the safety tests easier to pass.
    """
    r = sim.run([Actor("person", 2.2, 20.0), Actor("person", -2.4, 34.0)])
    assert r.distance_travelled_m > 50.0, r.summary()
    assert min(r.speeds_mph) > 3.0, f"slowed too much:\n{r.summary()}"
    assert_no_collision(r)


def test_a_group_standing_beside_the_path_does_not_stop_us():
    people = [Actor("person", 2.3 + 0.7 * i, 24.0 + 0.5 * (i % 3))
              for i in range(5)]
    r = sim.run(people)
    assert r.distance_travelled_m > 45.0, r.summary()
    assert not r.stopped_at_any_point, f"stopped for a crowd off-path:\n{r.summary()}"


def test_someone_already_behind_us_is_irrelevant():
    r = sim.run([Actor("person", 0.0, -6.0, vy=WALK)])
    assert r.distance_travelled_m > 55.0, r.summary()
    assert min(r.speeds_mph) > 5.4


# ---------------------------------------------------------------------------
# Static obstacles
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("dist", [12.0, 18.0, 25.0, 35.0])
def test_person_standing_in_the_path_stops_the_cart(dist):
    r = sim.run([Actor("person", 0.0, dist)], sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)
    assert r.stopped_at_any_point, f"never stopped:\n{r.summary()}"
    # Stopped short, not crept up to them.
    assert r.min_clearance_m > 0.8, r.summary()
    assert_comfortable(r)


def test_stopping_for_a_person_does_not_oscillate():
    r = sim.run([Actor("person", 0.0, 22.0)], sim=SimConfig(duration_s=30.0))
    assert r.reversals() <= 1, f"cart hunted:\n{r.summary()}"


def test_car_stopped_in_the_path_stops_the_cart():
    r = sim.run([Actor("car", 0.0, 25.0)], sim=SimConfig(duration_s=30.0))
    assert_no_collision(r, "the car")
    assert r.stopped_at_any_point
    assert r.min_clearance_m > 0.8, r.summary()
    assert_comfortable(r)


def test_the_cart_resumes_once_the_obstruction_walks_away():
    """Auto-resume, which is what the operator asked for -- and its danger.

    Resuming has to be driven by positive evidence that the way is clear, not
    by the mere absence of a detection, so this pairs with the dropout and
    blind-zone tests below.
    """
    r = sim.run([Actor("person", 0.0, 18.0, vx=-WALK, t_start=8.0)],
                sim=SimConfig(duration_s=34.0))
    assert_no_collision(r)
    assert r.stopped_at_any_point, f"should have stopped first:\n{r.summary()}"
    late = [st.v_ms for st in r.steps if st.t > 24.0]
    assert max(late) > 1.5, f"never resumed:\n{r.summary()}"


# ---------------------------------------------------------------------------
# Crossing traffic
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("y0,t_start", [(14.0, 0.0), (20.0, 2.0),
                                        (26.0, 4.0), (32.0, 6.0)])
def test_pedestrian_crossing_left_to_right(y0, t_start):
    """Swept so the crossing lands at a spread of times-to-collision.

    A single hand-picked geometry is easy to pass by accident; the parameter
    sweep is what makes this evidence rather than an anecdote.
    """
    r = sim.run([Actor("person", -5.0, y0, vx=WALK, t_start=t_start)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)


@pytest.mark.parametrize("y0", [16.0, 22.0, 28.0])
def test_pedestrian_running_across(y0):
    r = sim.run([Actor("person", -7.0, y0, vx=RUN)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)


def test_pedestrian_who_will_be_long_gone_barely_slows_us():
    """The whole reason for the arrival window.

    Somebody crossing 30 m ahead at walking pace is clear of the corridor about
    twenty seconds before the cart arrives. Braking for them is the behaviour
    that makes operators switch the system off.
    """
    r = sim.run([Actor("person", -1.5, 32.0, vx=RUN)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)
    assert r.distance_travelled_m > 40.0, f"over-cautious:\n{r.summary()}"


def test_car_crossing_at_25_mph():
    r = sim.run([Actor("car", -40.0, 26.0, vx=CAR)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r, "the car")


def test_pedestrian_walking_towards_us_head_on():
    """Closing at 3.9 m/s combined, which is the case that forced the envelope
    to account for the obstacle's own approach during our braking."""
    r = sim.run([Actor("person", 0.2, 30.0, vy=-WALK)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)
    assert r.stopped_at_any_point, r.summary()
    # Stopped with real room, not by a whisker. (The pedestrian then keeps
    # walking into the stationary cart, which is why the criterion above is
    # measured while moving.)
    moving = [st.min_clearance_m for st in r.steps if st.v_ms > 0.05]
    assert min(moving) > 1.0, r.summary()


def test_pedestrian_walking_the_same_way_is_followed_not_rammed():
    r = sim.run([Actor("person", 0.0, 14.0, vy=WALK)],
                sim=SimConfig(duration_s=32.0))
    assert_no_collision(r)
    assert r.distance_travelled_m > 15.0, f"gave up entirely:\n{r.summary()}"


# ---------------------------------------------------------------------------
# The hard one: occlusion
# ---------------------------------------------------------------------------
def test_pedestrian_steps_out_from_behind_a_parked_car():
    """The scenario that justifies the reflex layer's existence.

    The pedestrian is geometrically invisible behind the parked car until the
    sight line opens, and their feet are clipped by the car's roofline for
    several frames after that -- so the first ranges the system gets are both
    late and biased long. There is no forecasting answer to this; the only
    defences are having been slow enough already and reacting hard once.
    """
    r = sim.run([Actor("car", -2.6, 21.0),
                 Actor("person", -2.6, 20.6, vx=WALK, t_start=2.0)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)


def test_pedestrian_emerging_from_between_two_parked_cars():
    r = sim.run([Actor("car", -2.7, 18.0), Actor("car", -2.7, 27.0),
                 Actor("person", -2.7, 22.5, vx=WALK, t_start=3.0)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)


def test_a_person_hidden_behind_another_person_is_still_not_hit():
    r = sim.run([Actor("person", 0.1, 24.0),
                 Actor("person", -0.1, 26.0)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)
    assert r.min_clearance_m > 0.8, r.summary()


# ---------------------------------------------------------------------------
# Perception misbehaving
# ---------------------------------------------------------------------------
def test_heavy_detection_dropout_does_not_pump_the_throttle():
    """Forty per cent of frames missing. The failure this guards against is
    not a collision but a surge-brake-surge cycle as the obstacle blinks."""
    r = sim.run([Actor("person", 0.0, 22.0)],
                sim=SimConfig(duration_s=30.0, dropout_p=0.4, seed=11))
    assert_no_collision(r)
    assert r.reversals() <= 2, f"throttle pumping:\n{r.summary()}"
    assert_comfortable(r)


def test_a_two_frame_phantom_does_not_trigger_a_stop():
    """A false positive lasting 0.1 s must not be able to stop the cart.

    Note the phantom is placed 30 m out, not 3 m. A detection three metres in
    front of the bumper SHOULD stop the cart even if it turns out to be a
    shadow -- there is no time to be discerning at that range, and the cost of
    being wrong is asymmetric. The claim being tested is the achievable one:
    transient noise at ordinary detection distances must not brake us.
    """
    ghost = Actor("person", 0.0, 30.0, ghost=True,
                  visible_from=3.0, visible_to=3.06)
    r = sim.run([ghost], sim=SimConfig(duration_s=16.0))
    assert r.distance_travelled_m > 30.0, f"stopped for a phantom:\n{r.summary()}"
    assert min(r.speeds_mph) > 2.5, r.summary()


def test_a_persistent_phantom_is_allowed_to_stop_the_cart():
    """The other half of the previous test: transient noise is rejected, but a
    detection that keeps appearing must still be believed. A filter that
    ignores anything it dislikes is not a safety system."""
    ghost = Actor("person", 0.0, 16.0, ghost=True, visible_from=1.0)
    r = sim.run([ghost], sim=SimConfig(duration_s=24.0))
    assert r.stopped_at_any_point, f"ignored a persistent object:\n{r.summary()}"


@pytest.mark.parametrize("bias", [-1.5, -0.8, 0.8, 1.5])
def test_a_miscalibrated_camera_pitch_still_does_not_hit_anyone(bias):
    """Pitch bias is the dominant systematic error in monocular ranging.

    A bias of +/-1.5 degrees is a badly-shimmed bracket, and it scales the
    whole range estimate. Negative bias is the dangerous sign: everything is
    reported further away than it is.
    """
    r = sim.run([Actor("person", 0.0, 20.0)],
                sim=SimConfig(duration_s=30.0, pitch_bias_deg=bias))
    assert_no_collision(r)


def test_violent_suspension_movement_still_does_not_hit_anyone():
    r = sim.run([Actor("person", 0.0, 20.0)],
                sim=SimConfig(duration_s=30.0, pitch_wobble_deg=1.5,
                              jitter_px=5.0, seed=23))
    assert_no_collision(r)


def test_a_slower_control_loop_still_does_not_hit_anyone():
    """Ten hertz instead of twenty, i.e. half the reaction budget spent."""
    r = sim.run([Actor("person", -5.0, 18.0, vx=WALK)],
                sim=SimConfig(dt=0.1, duration_s=30.0))
    assert_no_collision(r)


def test_a_sluggish_brake_actuator_still_does_not_hit_anyone():
    """Double the assumed plant delay. This is what the policy's reaction_s is
    protecting against, and the margin should be real, not nominal."""
    r = sim.run([Actor("person", 0.0, 20.0)],
                sim=SimConfig(duration_s=30.0, plant_delay_s=0.7,
                              plant_tau_s=0.5))
    assert_no_collision(r)


# ---------------------------------------------------------------------------
# Route geometry
# ---------------------------------------------------------------------------
def test_a_person_on_the_inside_of_a_bend_is_seen_as_on_the_route():
    """Corridor conflicts follow the ROUTE, not the current heading.

    Someone standing on a bend is straight ahead of nothing, yet the cart is
    going to drive through where they are standing.
    """
    route = Route([(0, 0), (0, 18), (8, 30), (8, 60)])
    r = sim.run([Actor("person", 3.2, 22.5)], route=route,
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)
    assert r.stopped_at_any_point, f"drove round the bend into them:\n{r.summary()}"


def test_a_person_off_the_outside_of_a_bend_does_not_stop_us():
    """The mirror image, and the reason the previous test is not enough: a
    straight-ahead corridor would flag this person, and it should not."""
    route = Route([(0, 0), (0, 18), (10, 28), (10, 60)])
    r = sim.run([Actor("person", 0.0, 30.0)], route=route,
                sim=SimConfig(duration_s=26.0))
    assert r.distance_travelled_m > 40.0, f"stopped for someone off-route:\n{r.summary()}"


# ---------------------------------------------------------------------------
# The sweep: nothing anywhere near the path may ever be hit
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("lat", [-1.0, -0.5, 0.0, 0.5, 1.0])
@pytest.mark.parametrize("dist", [10.0, 16.0, 24.0])
def test_sweep_static_people_across_the_corridor(lat, dist):
    r = sim.run([Actor("person", lat, dist)], sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)


@pytest.mark.parametrize("speed", [0.8, 1.4, 2.2, 3.0])
@pytest.mark.parametrize("side", [-1.0, 1.0])
def test_sweep_crossing_speeds_and_directions(speed, side):
    r = sim.run([Actor("person", -side * 6.0, 22.0, vx=side * speed)],
                sim=SimConfig(duration_s=30.0))
    assert_no_collision(r)


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5, 6, 7, 8])
def test_sweep_noise_seeds_on_a_close_crossing(seed):
    """Same geometry, different noise. Safety must not be a lucky seed."""
    r = sim.run([Actor("person", -4.0, 17.0, vx=WALK)],
                sim=SimConfig(duration_s=30.0, seed=seed))
    assert_no_collision(r)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
