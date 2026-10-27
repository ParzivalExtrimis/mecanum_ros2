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

"""Unit tests for mecanum_localization.planar_math and the adapter's covariance helper."""

import math

from mecanum_localization.lio_odom_adapter import diagonal_covariance
from mecanum_localization.planar_math import (
    compose, inverse, PlanarFlattener, quaternion_from_yaw, relative, wrap_angle,
    yaw_from_quaternion)
import pytest


def quaternion_rpy(roll, pitch, yaw):
    """Return the (x, y, z, w) quaternion of Z-Y-X Euler angles."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_yaw_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert yaw_from_quaternion(*quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_yaw_ignores_small_roll_and_pitch():
    q = quaternion_rpy(math.radians(3.0), math.radians(-2.0), 1.2)
    assert yaw_from_quaternion(*q) == pytest.approx(1.2)


def test_compose_and_inverse():
    a = (1.0, 2.0, math.pi / 2)
    b = (1.0, 0.0, 0.0)
    # One metre forward when facing +y moves +y in the parent frame.
    assert compose(a, b) == pytest.approx((1.0, 3.0, math.pi / 2))
    assert compose(a, inverse(a)) == pytest.approx((0.0, 0.0, 0.0))


def test_relative_is_in_origin_frame():
    origin = (2.0, -1.0, math.pi)
    pose = (1.0, -1.0, -math.pi / 2)
    # Facing -x, a point 1 m further along -x is 1 m ahead; the heading turned +90 deg.
    assert relative(origin, pose) == pytest.approx((1.0, 0.0, math.pi / 2))


def test_flattener_starts_at_origin():
    flatten = PlanarFlattener()
    q0 = quaternion_rpy(0.01, -0.02, 0.7)
    assert flatten(3.0, 4.0, *q0) == pytest.approx((0.0, 0.0, 0.0))
    # Then drive 1 m forward along the start heading and turn 0.3 rad.
    x = 3.0 + math.cos(0.7)
    y = 4.0 + math.sin(0.7)
    assert flatten(x, y, *quaternion_rpy(0.0, 0.0, 1.0)) == pytest.approx((1.0, 0.0, 0.3))


def test_flattener_reset():
    flatten = PlanarFlattener()
    flatten(1.0, 1.0, *quaternion_from_yaw(0.0))
    flatten.reset()
    assert flatten(5.0, 5.0, *quaternion_from_yaw(2.0)) == pytest.approx((0.0, 0.0, 0.0))


def test_diagonal_covariance():
    cov = diagonal_covariance([1, 2, 3, 4, 5, 6])
    assert len(cov) == 36
    assert [cov[i * 7] for i in range(6)] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert sum(cov) == 21.0
    with pytest.raises(ValueError):
        diagonal_covariance([1.0])
