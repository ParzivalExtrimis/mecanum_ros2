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

"""Unit tests for the drift metrics of mecanum_localization.gt_compare."""

import math

from mecanum_localization.gt_compare import DriftTracker, GroundTruthBuffer, interpolate_pose
import pytest


def test_interpolate_takes_short_way_round():
    pose = interpolate_pose(0.5, 0.0, (0.0, 0.0, math.radians(170.0)),
                            1.0, (2.0, 0.0, math.radians(-170.0)))
    assert pose[0] == pytest.approx(1.0)
    assert abs(pose[2]) == pytest.approx(math.pi)


def test_buffer_interpolates_and_bounds():
    buf = GroundTruthBuffer()
    buf.add(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    buf.add(2.0, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    pose, twist = buf.at(1.25)
    assert pose == pytest.approx((0.25, 0.0, 0.0))
    assert twist == (1.0, 0.0, 0.0)
    assert buf.at(0.5) is None and buf.at(2.5) is None
    assert buf.at(2.0)[0] == (1.0, 0.0, 0.0)


def test_buffer_drops_old_samples():
    buf = GroundTruthBuffer(keep_seconds=1.0)
    for i in range(30):
        buf.add(i * 0.1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert buf.at(0.5) is None
    assert buf.at(2.5) is not None


def test_perfect_estimate_in_other_frame_has_no_drift():
    tracker = DriftTracker()
    # The estimate starts at its own origin; the ground truth starts at (5, -1) facing +y.
    for i in range(11):
        d = i * 0.5
        tracker.add(i, (d, 0.0, 0.0), (5.0, -1.0 + d, math.pi / 2), (0.5, 0, 0), (0.5, 0, 0))
    assert tracker.path_length == pytest.approx(5.0)
    assert tracker.final_error == pytest.approx(0.0, abs=1e-9)
    assert tracker.jumps == 0
    assert tracker.passed(1.0, 2.0)


def test_drift_percent_and_yaw():
    tracker = DriftTracker()
    tracker.add(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    tracker.add(1.0, (10.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    tracker.add(2.0, (20.15, 0.0, math.radians(3.0)), (20.0, 0.0, 0.0))
    assert tracker.drift_percent == pytest.approx(0.75)
    assert math.degrees(tracker.max_yaw_error) == pytest.approx(3.0)
    assert not tracker.passed(1.0, 2.0)
    assert tracker.passed(1.0, 5.0)
    assert 'PASS' in tracker.summary(1.0, 5.0)


def test_jump_is_counted():
    tracker = DriftTracker(jump_threshold=0.2)
    tracker.add(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    tracker.add(0.2, (1.0, 0.0, 0.0), (0.05, 0.0, 0.0))
    assert tracker.jumps == 1
    assert not tracker.passed(100.0, 180.0)


def test_out_and_back_that_never_moved_fails():
    tracker = DriftTracker()
    for i, gx in enumerate((0.0, 1.0, 2.0, 1.0, 0.0)):
        tracker.add(i, (0.0, 0.0, 0.0), (gx, 0.0, 0.0))
    assert tracker.final_error == pytest.approx(0.0)
    assert tracker.max_drift_percent == pytest.approx(50.0)
    assert not tracker.passed(1.0, 2.0)
