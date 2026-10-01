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

"""Unit tests for mecanum_base.drive_core (timeout, clamping, saturation, calibration)."""

import math

from mecanum_base.drive_core import DriveCore, DriveLimits
from mecanum_base.kinematics import forward, MecanumGeometry, WHEEL_NAMES
from mecanum_base.wheel_calibration import WheelCalibration
import pytest

GEOM = MecanumGeometry(lx=0.2255, ly=0.207, wheel_radius=0.05)
LIMITS = DriveLimits(max_vx=0.3, max_vy=0.3, max_wz=0.8)


def make_core(max_value=1.0, scale=1.0 / 15.0, deadband=0.0, timeout=0.25, signs=None):
    signs = signs or {}
    cals = {name: WheelCalibration(motor_id=i, sign=signs.get(name, 1), scale=scale,
                                   deadband=deadband, max_value=max_value)
            for i, name in enumerate(WHEEL_NAMES)}
    return DriveCore(GEOM, cals, LIMITS, timeout)


def test_no_command_means_zeros():
    out = make_core().compute(now=10.0)
    assert out.timed_out
    assert all(c.direction == 0 and c.value == 0.0 for c in out.commands)


def test_fresh_command_drives_and_stale_command_stops():
    core = make_core()
    core.set_command(0.1, 0.0, 0.0, stamp=10.0)
    out = core.compute(now=10.2)
    assert not out.timed_out
    assert all(c.direction == 1 and c.value > 0.0 for c in out.commands)
    out = core.compute(now=10.3)  # 0.3 s > 0.25 s timeout
    assert out.timed_out
    assert all(c.direction == 0 for c in out.commands)


def test_clock_jump_backwards_stops():
    core = make_core()
    core.set_command(0.1, 0.0, 0.0, stamp=10.0)
    assert core.compute(now=2.0).timed_out


def test_non_finite_command_is_ignored():
    core = make_core()
    core.set_command(0.1, 0.0, 0.0, stamp=10.0)
    assert not core.set_command(float('nan'), 0.0, 0.0, stamp=10.1)
    assert core.compute(now=10.2).twist == pytest.approx((0.1, 0.0, 0.0))


def test_twist_is_clamped_to_limits():
    core = make_core()
    core.set_command(0.6, 0.0, 0.0, stamp=0.0)
    out = core.compute(now=0.1)
    assert out.twist == pytest.approx((0.3, 0.0, 0.0))


def test_saturation_scales_all_wheels_and_keeps_direction():
    # max_value 0.2 at scale 1/15 -> max 3 rad/s per wheel; 0.3 m/s needs 6 rad/s.
    core = make_core(max_value=0.2)
    core.set_command(0.3, 0.1, 0.0, stamp=0.0)
    out = core.compute(now=0.1)
    assert out.saturation_factor < 1.0
    assert max(abs(w) for w in out.omega_sent) == pytest.approx(3.0)
    vx, vy, _ = forward(out.omega_sent, GEOM)
    assert math.atan2(vy, vx) == pytest.approx(math.atan2(0.1, 0.3))
    assert all(c.value <= 0.2 + 1e-12 for c in out.commands)


def test_calibration_sign_applied_per_wheel():
    core = make_core(signs={'front_right': -1})
    core.set_command(0.1, 0.0, 0.0, stamp=0.0)
    out = core.compute(now=0.1)
    directions = {name: c.direction for name, c in zip(WHEEL_NAMES, out.commands)}
    assert directions == {'front_left': 1, 'front_right': -1, 'rear_left': 1,
                          'rear_right': 1}


def test_motor_ids_follow_calibration():
    out = make_core().zero_output()
    assert [c.motor_id for c in out.commands] == [0, 1, 2, 3]
    assert all(c.direction == 0 and c.value == 0.0 for c in out.commands)


def test_duplicate_motor_ids_rejected():
    cals = {name: WheelCalibration(motor_id=0, sign=1, scale=0.1, deadband=0.0,
                                   max_value=1.0) for name in WHEEL_NAMES}
    with pytest.raises(ValueError):
        DriveCore(GEOM, cals, LIMITS, 0.25)
