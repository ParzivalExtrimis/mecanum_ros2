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

"""The segment controller on an ideal holonomic base: every kind of move arrives."""

import math

from demo_harness import load_config
from mecanum_demo.geometry import wrap
from mecanum_demo.segment_control import SegmentController
import pytest

SPEEDS = load_config()['speeds']


def simulate(controller, pose, dt=0.02, limit=60.0, lag=0.0):
    """Integrate body-frame commands; return (final pose, seconds, peak speed, peak wz)."""
    x, y, yaw = pose
    t = peak_v = peak_w = 0.0
    applied = (0.0, 0.0, 0.0)
    while t < limit:
        cmd, done = controller.step((x, y, yaw), dt)
        if done:
            return (x, y, yaw), t, peak_v, peak_w
        applied = tuple(a + (c - a) * (1.0 - lag) for a, c in zip(applied, cmd))
        vx, vy, wz = applied
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw += wz * dt
        t += dt
        peak_v = max(peak_v, math.hypot(vx, vy))
        peak_w = max(peak_w, abs(wz))
    raise AssertionError(f'did not arrive: {(x, y, yaw)}')


@pytest.mark.parametrize('lag', [0.0, 0.3])
def test_move_with_a_heading_change(lag):
    start = (-0.5, -0.16, math.pi)
    ctl = SegmentController('move', start, target=(0.46, -0.235, 0.0), speeds=SPEEDS)
    (x, y, yaw), t, peak_v, peak_w = simulate(ctl, start, lag=lag)
    assert math.hypot(x - 0.46, y + 0.235) < 0.02
    assert abs(wrap(yaw)) < 0.03
    assert peak_v <= SPEEDS['linear'] + 1e-6 and peak_w <= SPEEDS['angular'] + 1e-6
    assert t < 10.0


def test_move_keeping_the_heading_stays_on_the_line():
    start = (0.0, 0.0, 0.3)
    ctl = SegmentController('move', start, target=(0.0, 0.5, 2.0), keep_heading=True,
                            slow=True, speeds=SPEEDS)
    (x, y, yaw), _, peak_v, _ = simulate(ctl, start)
    assert abs(y - 0.5) < 0.02 and abs(x) < 0.02 and abs(yaw - 0.3) < 0.03
    assert peak_v <= SPEEDS['linear_slow'] + 1e-6


def test_turn_takes_the_shorter_way():
    start = (0.4, -0.4, 3.0)
    ctl = SegmentController('turn', start, target=(0, 0, -3.0), speeds=SPEEDS)
    (x, y, yaw), t, _, _ = simulate(ctl, start)
    assert abs(wrap(yaw + 3.0)) < 0.03 and t < 3.0
    assert math.hypot(x - 0.4, y + 0.4) < 0.01


def test_spin_turns_one_and_a_half_times():
    start = (0.46, -0.425, math.pi)
    ctl = SegmentController('spin', start, delta_yaw=-3 * math.pi, speeds=SPEEDS)
    (_, _, yaw), t, _, _ = simulate(ctl, start)
    assert yaw == pytest.approx(math.pi - 3 * math.pi, abs=0.03)
    assert t > 3 * math.pi / SPEEDS['angular']


def test_remaining_time_falls_as_the_move_goes():
    start = (0.0, 0.0, 0.0)
    ctl = SegmentController('move', start, target=(1.0, 0.0, 0.0), speeds=SPEEDS)
    assert ctl.remaining_s(start) == pytest.approx(1.0 / SPEEDS['linear'])
    assert ctl.remaining_s((0.5, 0.0, 0.0)) == pytest.approx(0.5 / SPEEDS['linear'])
