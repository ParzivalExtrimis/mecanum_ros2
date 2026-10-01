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

"""Unit tests for mecanum_base.kinematics."""

import itertools
import math

from mecanum_base.kinematics import (clamp_twist, forward, inverse, MecanumGeometry,
                                     scale_to_limits)
import pytest

GEOM = MecanumGeometry(lx=0.2255, ly=0.207, wheel_radius=0.05)
K = 0.2255 + 0.207


def approx(values, expected, tol=1e-9):
    return all(math.isclose(a, b, abs_tol=tol) for a, b in zip(values, expected))


def test_inverse_matches_spec_formulas():
    vx, vy, wz = 0.12, -0.07, 0.3
    r = GEOM.wheel_radius
    expected = ((vx - vy - K * wz) / r, (vx + vy + K * wz) / r,
                (vx + vy - K * wz) / r, (vx - vy + K * wz) / r)
    assert approx(inverse(vx, vy, wz, GEOM), expected)


def test_forward_all_wheels_equal():
    # 0.1 m/s forward -> every wheel at 0.1 / 0.05 = 2 rad/s
    assert approx(inverse(0.1, 0.0, 0.0, GEOM), (2.0, 2.0, 2.0, 2.0))


def test_strafe_left_sign_pattern():
    fl, fr, rl, rr = inverse(0.0, 0.1, 0.0, GEOM)
    assert fl < 0 and fr > 0 and rl > 0 and rr < 0
    assert approx((abs(fl), abs(fr), abs(rl), abs(rr)), (2.0, 2.0, 2.0, 2.0))


def test_rotate_ccw_sign_pattern():
    fl, fr, rl, rr = inverse(0.0, 0.0, 0.5, GEOM)
    # Counter-clockwise: left side backward, right side forward.
    assert fl < 0 and rl < 0 and fr > 0 and rr > 0
    assert math.isclose(fr, K * 0.5 / GEOM.wheel_radius)


@pytest.mark.parametrize('twist', list(itertools.product((-0.3, 0.0, 0.17), repeat=3)))
def test_forward_inverts_inverse(twist):
    assert approx(forward(inverse(*twist, GEOM), GEOM), twist)


def test_geometry_rejects_bad_values():
    with pytest.raises(ValueError):
        MecanumGeometry(lx=0.0, ly=0.2, wheel_radius=0.05)
    with pytest.raises(ValueError):
        MecanumGeometry(lx=0.2, ly=0.2, wheel_radius=float('nan'))


def test_clamp_twist_scales_uniformly():
    (vx, vy, wz), factor = clamp_twist(0.6, 0.15, 0.4, 0.3, 0.3, 0.8)
    assert math.isclose(factor, 0.5)
    assert approx((vx, vy, wz), (0.3, 0.075, 0.2))


def test_clamp_twist_within_limits_is_unchanged():
    twist, factor = clamp_twist(0.1, -0.2, 0.5, 0.3, 0.3, 0.8)
    assert factor == 1.0 and twist == (0.1, -0.2, 0.5)


def test_scale_to_limits_preserves_ratios():
    wheels = (10.0, -20.0, 5.0, 0.0)
    scaled, factor = scale_to_limits(wheels, (15.0, 15.0, 15.0, 15.0))
    assert math.isclose(factor, 0.75)
    assert approx(scaled, (7.5, -15.0, 3.75, 0.0))
    assert max(abs(w) for w in scaled) <= 15.0 + 1e-12


def test_scale_to_limits_per_wheel_limits():
    scaled, factor = scale_to_limits((10.0, 10.0, 10.0, 10.0), (20.0, 5.0, 20.0, 20.0))
    assert math.isclose(factor, 0.5)
    assert approx(scaled, (5.0, 5.0, 5.0, 5.0))


def test_saturated_command_keeps_direction():
    # A fast diagonal + rotation command, saturated, must map to the same body direction.
    twist = (2.0, 1.0, 1.5)
    scaled, factor = scale_to_limits(inverse(*twist, GEOM), (15.0,) * 4)
    assert factor < 1.0
    vx, vy, wz = forward(scaled, GEOM)
    assert approx((vx / factor, vy / factor, wz / factor), twist, tol=1e-9)
