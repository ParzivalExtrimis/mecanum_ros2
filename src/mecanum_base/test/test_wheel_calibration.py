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

"""Unit tests for mecanum_base.wheel_calibration."""

import math

from mecanum_base.wheel_calibration import WheelCalibration
import pytest


def cal(**kwargs):
    params = {'motor_id': 0, 'sign': 1, 'scale': 0.05, 'deadband': 0.1, 'max_value': 1.0}
    params.update(kwargs)
    return WheelCalibration(**params)


def test_forward_and_reverse():
    c = cal()
    assert c.to_command(4.0) == (1, pytest.approx(0.1 + 0.05 * 4.0))
    assert c.to_command(-4.0) == (-1, pytest.approx(0.3))


def test_sign_flips_direction_only():
    c = cal(sign=-1)
    assert c.to_command(4.0) == (-1, pytest.approx(0.3))


def test_zero_and_tiny_speeds_stop():
    c = cal()
    assert c.to_command(0.0) == (0, 0.0)
    assert c.to_command(1e-6) == (0, 0.0)


def test_small_speed_starts_at_deadband():
    direction, value = cal().to_command(0.01)
    assert direction == 1 and value >= 0.1


def test_value_capped_at_max():
    c = cal()
    assert c.to_command(1000.0) == (1, 1.0)
    assert math.isclose(c.max_omega(), (1.0 - 0.1) / 0.05)


def test_non_finite_speed_stops():
    c = cal()
    assert c.to_command(float('nan')) == (0, 0.0)
    assert c.to_command(float('inf')) == (0, 0.0)


@pytest.mark.parametrize('omega', [-17.0, -3.2, 0.5, 9.9])
@pytest.mark.parametrize('sign', [1, -1])
def test_round_trip(omega, sign):
    c = cal(sign=sign)
    assert c.to_omega(*c.to_command(omega)) == pytest.approx(omega)


@pytest.mark.parametrize('bad', [{'sign': 0}, {'scale': 0.0}, {'deadband': -0.1},
                                 {'max_value': 0.05}, {'motor_id': -1}])
def test_rejects_invalid_calibration(bad):
    with pytest.raises(ValueError):
        cal(**bad)
