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

from mecanum_base.motion_test import displacement_in_start_frame, judge_step, unwrap_yaw
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
