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

"""Unit tests for the pure helpers of mecanum_base.motion_test."""

import math

from mecanum_base.motion_test import (
    build_pattern, displacement_in_start_frame, judge_step, nominal_duration, segment_progress,
    unwrap_yaw)
import pytest


def test_unwrap_crosses_pi():
    yaw = unwrap_yaw(math.radians(179.0), math.radians(179.0), math.radians(-179.0))
    assert math.degrees(yaw) == pytest.approx(181.0)


def test_displacement_is_in_start_frame():
    # Start facing +y; moving +y in the world is "forward" for the robot.
    dx, dy, dyaw = displacement_in_start_frame((1.0, 1.0, math.pi / 2), (1.0, 1.5, math.pi / 2))
    assert (dx, dy, dyaw) == pytest.approx((0.5, 0.0, 0.0))


def test_rotation_beyond_half_turn_is_ccw():
    _, _, dyaw = displacement_in_start_frame((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(190.0)))
    ok, text = judge_step('rotate_ccw', (0.0, 0.0, math.radians(183.0)), (0.0, 0.0, dyaw),
                          0.05, 0.05)
    assert ok and 'CCW' in text


def test_forward_drift_fails():
    ok, _ = judge_step('forward', (0.45, 0.0, 0.0), (0.45, 0.05, 0.0), 0.05, 0.05)
    assert not ok


def test_strafe_wrong_direction_fails():
    ok, _ = judge_step('strafe_left', (0.0, 0.45, 0.0), (0.0, -0.45, 0.0), 0.05, 0.05)
    assert not ok


def test_rotation_with_translation_fails():
    ok, _ = judge_step('rotate_ccw', (0.0, 0.0, 1.5), (0.1, 0.0, 1.5), 0.05, 0.05)
    assert not ok


def test_axes_pattern_unchanged():
    steps = build_pattern('axes', 0.15, 0.5)
    assert [s[0] for s in steps] == ['forward', 'forward_return', 'strafe_left',
                                     'strafe_left_return', 'rotate_ccw', 'rotate_ccw_return']
    assert all(target is None for *_, target in steps)
    assert [s[2] for s in steps] == [True, False] * 3
    assert len(build_pattern('axes', 0.15, 0.5, return_to_start=False)) == 3


def simulate(steps, start=(0.0, -1.0, 0.0)):
    """Integrate each step exactly to its target from ``start``; return the final pose."""
    x, y, yaw = start
    for _, (vx, vy, wz), _, target in steps:
        t = nominal_duration((vx, vy, wz), target)
        if wz:
            yaw += wz * t
        else:
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * vx - s * vy) * t
            y += (s * vx + c * vy) * t
    return x, y, yaw


def test_loop_closes():
    steps = build_pattern('loop', 0.15, 0.5, loop_length=4.0, loop_width=2.0, laps=2)
    assert len(steps) == 12
    assert steps[0][0] == 'lap1_1_forward' and steps[-1][0] == 'lap2_6_strafe_right'
    x, y, yaw = simulate(steps)
    assert (x, y) == pytest.approx((0.0, -1.0), abs=1e-9)
    assert math.degrees(yaw) == pytest.approx(720.0)
    distance = sum(t for _, (vx, vy, wz), _, t in steps if not wz)
    assert distance == pytest.approx(24.0)


def test_loop_stays_in_corridor_and_lobby():
    # Corners of the lap, from the restroom spawn: (4, -1), then back to (0, -1), up to (0, 1).
    steps = build_pattern('loop', 0.15, 0.5, loop_length=4.0, loop_width=2.0)
    corners = [simulate(steps[:k]) for k in range(1, 7)]
    assert corners[0][:2] == pytest.approx((4.0, -1.0))
    assert corners[3][:2] == pytest.approx((0.0, 1.0))


def test_corridor_returns():
    steps = build_pattern('corridor', 0.2, 0.5, loop_length=8.0, laps=3)
    assert len(steps) == 6
    assert simulate(steps)[:2] == pytest.approx((0.0, -1.0))


def test_unknown_pattern():
    with pytest.raises(ValueError):
        build_pattern('square', 0.1, 0.1)


def test_segment_progress():
    # Strafing right: progress counts displacement along -y of the start frame.
    assert segment_progress((0.0, -0.15, 0.0), (0.02, -0.5, 0.1)) == pytest.approx(0.5)
    assert segment_progress((-0.2, 0.0, 0.0), (-1.0, 0.0, 0.0)) == pytest.approx(1.0)
    assert segment_progress((0.0, 0.0, 0.5), (0.0, 0.0, 3.5)) == pytest.approx(3.5)
    assert segment_progress((0.0, 0.0, -0.5), (0.0, 0.0, -1.0)) == pytest.approx(1.0)
    assert nominal_duration((0.0, 0.0, 0.5), math.pi) == pytest.approx(2 * math.pi)
