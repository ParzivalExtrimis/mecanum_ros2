# Copyright 2026 Aryan Hegde
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests of the safety rules (no ROS)."""

import math

from mecanum_safety.safety_logic import (
    corner_guard, decide, EStop, face_guard, is_cliff, Layout, Twist2D, Watchdog, ZERO)
import pytest

CX, CY, BX, HW = 0.3535, 0.2078, 0.3915, 0.255
LAYOUT = Layout(
    cliffs={'front_left': (CX, CY), 'front_right': (CX, -CY),
            'rear_left': (-CX, CY), 'rear_right': (-CX, -CY)},
    bumpers={'front': BX, 'rear': -BX},
    ultrasonics={'front_left': 0.408, 'front_right': 0.408,
                 'rear_left': -0.401, 'rear_right': -0.401},
    half_width=HW)
FULL_STOP = ('/scan', '/l2/points_filtered', '/d435i/depth/color/points', '/joint_states',
             '/odom')

FORWARD, BACK = Twist2D(0.3, 0, 0), Twist2D(-0.3, 0, 0)
LEFT, RIGHT = Twist2D(0, 0.3, 0), Twist2D(0, -0.3, 0)
CCW, CW = Twist2D(0, 0, 0.8), Twist2D(0, 0, -0.8)


def topic_of(kind, name):
    return f'/{kind}/{name}'


def run(estop=False, lost=(), cliffs=None, bumpers=None):
    ranges = {name: 0.119 for name in LAYOUT.cliffs}
    ranges.update(cliffs or {})
    return decide(LAYOUT, estop_latched=estop, lost=lost, full_stop_topics=FULL_STOP,
                  cliff_ranges=ranges, floor_distance=0.119, cliff_margin=0.05,
                  bumpers_pressed=bumpers or {}, topic_of=topic_of)


def allowed(decision, cmd):
    return decision.gate(cmd) == cmd


def test_clear_passes_everything():
    decision = run()
    assert decision.reasons == ()
    for cmd in (FORWARD, BACK, LEFT, RIGHT, CCW, CW, Twist2D(0.2, -0.1, 0.5)):
        assert allowed(decision, cmd)


@pytest.mark.parametrize('value, cliff', [
    (0.119, False), (0.168, False), (0.17, True), (math.inf, True), (-math.inf, False),
    (math.nan, True)])
def test_cliff_threshold(value, cliff):
    assert is_cliff(value, 0.119, 0.05) is cliff


@pytest.mark.parametrize('corner, away, toward', [
    ('front_left', (BACK, RIGHT), (FORWARD, LEFT, CCW, CW)),
    ('front_right', (BACK, LEFT), (FORWARD, RIGHT, CCW, CW)),
    ('rear_left', (FORWARD, RIGHT), (BACK, LEFT, CCW, CW)),
    ('rear_right', (FORWARD, LEFT), (BACK, RIGHT, CCW, CW)),
])
def test_cliff_allows_only_motion_away_from_its_corner(corner, away, toward):
    decision = run(cliffs={corner: math.inf})
    assert decision.reasons == (f'cliff {corner}',)
    assert not decision.stopped
    for cmd in away:
        assert allowed(decision, cmd), cmd
    for cmd in toward:
        assert decision.gate(cmd) == ZERO, cmd


def test_two_front_cliffs_leave_only_reverse():
    decision = run(cliffs={'front_left': math.inf, 'front_right': math.inf})
    assert allowed(decision, BACK)
    for cmd in (FORWARD, LEFT, RIGHT, CCW, CW):
        assert decision.gate(cmd) == ZERO


@pytest.mark.parametrize('plank, away, toward', [
    ('front', BACK, FORWARD), ('rear', FORWARD, BACK)])
def test_pressed_plank_allows_only_moving_off_it(plank, away, toward):
    decision = run(bumpers={plank: True})
    assert decision.reasons == (f'bumper {plank}',)
    assert allowed(decision, away)
    assert decision.gate(toward) == ZERO
    # Turning on the spot swings one end of the plank into the wall.
    assert decision.gate(CCW) == ZERO and decision.gate(CW) == ZERO
    # Sliding along the wall keeps the plank in place: no speed along its normal.
    assert allowed(decision, LEFT)


def test_backing_off_a_plank_may_turn_if_both_ends_leave():
    decision = run(bumpers={'front': True})
    # The plank ends are at y = +-0.255: vx - wz * y < 0 at both needs |wz| < 0.3 / 0.255.
    assert allowed(decision, Twist2D(-0.3, 0, 1.0))
    assert decision.gate(Twist2D(-0.3, 0, 1.5)) == ZERO


@pytest.mark.parametrize('topic', FULL_STOP)
def test_lost_full_stop_sensor_stops_everything(topic):
    decision = run(lost=[topic])
    assert decision.stopped and decision.reasons == (f'lost {topic}',)
    for cmd in (FORWARD, BACK, LEFT, CCW):
        assert decision.gate(cmd) == ZERO


def test_lost_cliff_sensor_acts_as_a_cliff():
    decision = run(lost=['/cliff/rear_right'])
    assert decision.reasons == ('lost /cliff/rear_right',)
    assert allowed(decision, FORWARD) and allowed(decision, LEFT)
    assert decision.gate(BACK) == ZERO and decision.gate(RIGHT) == ZERO


def test_lost_ultrasonic_blocks_its_face():
    decision = run(lost=['/ultrasonic/front_left'])
    assert decision.reasons == ('lost /ultrasonic/front_left',)
    assert decision.gate(FORWARD) == ZERO
    assert allowed(decision, BACK) and allowed(decision, LEFT)


def test_lost_bumper_acts_as_pressed():
    decision = run(lost=['/bumper/rear'])
    assert decision.gate(BACK) == ZERO and allowed(decision, FORWARD)


def test_estop_stops_everything():
    decision = run(estop=True)
    assert decision.stopped and decision.reasons == ('estop',)
    assert decision.gate(BACK) == ZERO


def test_reasons_are_sorted_and_combined():
    decision = run(estop=True, lost=['/scan'], cliffs={'front_left': math.inf},
                   bumpers={'rear': True})
    assert decision.reasons == ('bumper rear', 'cliff front_left', 'estop', 'lost /scan')


def test_zero_always_passes_a_guard():
    for guard in (corner_guard(CX, CY), face_guard(-BX, HW)):
        assert guard.allows(ZERO)


def test_estop_latches_and_resets_only_when_released():
    estop = EStop()
    estop.update(True)
    assert estop.latched
    ok, _ = estop.reset()
    assert not ok and estop.latched
    estop.update(False)
    assert estop.latched  # still latched until reset
    ok, _ = estop.reset()
    assert ok and not estop.latched


def test_watchdog_lost_until_first_message_and_after_timeout():
    dog = Watchdog({'/a': 0.2, '/b': 0.5})
    assert dog.lost(0.0) == ['/a', '/b']
    dog.feed('/a', 1.0)
    dog.feed('/b', 1.0)
    dog.feed('/unknown', 1.0)
    assert dog.lost(1.2) == []
    assert dog.lost(1.21) == ['/a']
    assert dog.lost(1.51) == ['/a', '/b']
    dog.feed('/a', 1.6)
    assert dog.lost(1.7) == ['/b']
