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

"""Unit tests of the pure helpers of the sim-only sensor nodes."""

import math

from mecanum_simulation.sim_bumper import pressed
from mecanum_simulation.sim_depth_cloud import depth_to_points
from mecanum_simulation.sim_range_sensor import cloud_ranges, cone_range
import numpy as np
import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def test_cone_range_is_the_nearest_return():
    assert cone_range([3.0, 1.5, math.inf, 2.0], 0.03, 4.5) == 1.5


def test_cone_range_without_a_return_is_inf():
    assert cone_range([math.inf, math.inf], 0.03, 4.5) == math.inf
    assert cone_range([], 0.03, 4.5) == math.inf


def test_cone_range_ignores_nan():
    assert cone_range([math.nan, 2.0], 0.03, 4.5) == 2.0
    assert cone_range([math.nan], 0.03, 4.5) == math.inf


def test_cone_range_clamps_noise_to_the_limits():
    assert cone_range([0.01], 0.03, 4.5) == 0.03
    assert cone_range([4.6], 0.03, 4.5) == 4.5


def test_cloud_ranges_with_a_ray_that_hit_nothing():
    cloud = point_cloud2.create_cloud_xyz32(
        Header(), [(3.0, 4.0, 0.0), (0.0, 0.0, 2.0), (math.inf, math.inf, math.inf)])
    assert list(cloud_ranges(cloud)) == [5.0, 2.0, math.inf]


def test_pressed_before_any_contact():
    assert not pressed(None, 10.0, 0.1)


def test_pressed_holds_then_releases():
    assert pressed(10.0, 10.0, 0.1)
    assert pressed(10.0, 10.1, 0.1)
    assert not pressed(10.0, 10.11, 0.1)


def test_depth_centre_pixel_is_on_the_optical_axis():
    depth = np.full((3, 3), 2.0, dtype=np.float32)
    points = depth_to_points(depth, 100.0, 100.0, 1.0, 1.0)
    assert len(points) == 9
    assert points[4] == pytest.approx([0.0, 0.0, 2.0])


def test_depth_axes_are_optical():
    """Pixels right of centre have +x, and pixels below it +y, at the pinhole's scale."""
    depth = np.full((3, 3), 2.0, dtype=np.float32)
    points = depth_to_points(depth, 100.0, 50.0, 1.0, 1.0)
    assert points[5] == pytest.approx([0.02, 0.0, 2.0])  # row 1, column 2
    assert points[7] == pytest.approx([0.0, 0.04, 2.0])  # row 2, column 1


def test_depth_drops_invalid_pixels():
    depth = np.array([[1.0, np.inf], [np.nan, 0.0]], dtype=np.float32)
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0)
    assert len(points) == 1
    assert points.dtype == np.float32


def test_depth_stride_keeps_the_pixel_coordinates():
    depth = np.full((4, 4), 1.0, dtype=np.float32)
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, stride=2)
    assert len(points) == 4
    assert points[3] == pytest.approx([2.0, 2.0, 1.0])
