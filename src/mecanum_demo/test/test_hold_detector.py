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

"""The hold detector: people near the robot count, the booth's own walls do not."""

from demo_harness import load_config
from mecanum_demo.hold_detector import scan_points, StaticMap, unexplained_near
import numpy as np
import pytest

FOOTPRINT = load_config()['footprint']


def wall_map():
    """Return a 4 x 4 m map, 5 cm cells, origin (-2, -2), with a wall along x = 1."""
    occupied = np.zeros((80, 80), dtype=bool)
    occupied[:, 60] = True
    return StaticMap(occupied, 0.05, (-2.0, -2.0))


def column(x, y, count=10, spread=0.02):
    return np.column_stack([np.full(count, x), np.linspace(y - spread, y + spread, count)])


def test_a_person_in_front_holds():
    close, nearest, count = unexplained_near(column(0.75, 0.0), (0.0, 0.0, 0.0), FOOTPRINT,
                                             0.45, wall_map())
    assert close and nearest == pytest.approx(0.75 - FOOTPRINT['front']) and count == 10


def test_a_person_beyond_the_radius_does_not():
    close, _, _ = unexplained_near(column(0.0, 0.8), (0.0, 0.0, 0.0), FOOTPRINT, 0.45,
                                   wall_map())
    assert not close


def test_the_map_wall_never_holds_even_a_little_off():
    pose = (0.5, 0.0, 0.0)
    wall = column(1.0 + 0.04, 0.0, count=40, spread=0.5)      # 4 cm localization error
    close, _, count = unexplained_near(wall, pose, FOOTPRINT, 0.45, wall_map())
    assert not close and count == 0


def test_a_single_return_is_noise():
    close, _, count = unexplained_near(column(0.6, 0.0, count=2), (0.0, 0.0, 0.0), FOOTPRINT,
                                       0.45, wall_map())
    assert not close and count == 2


def test_the_rear_and_heading_are_respected():
    # Facing -x: a point at x = +0.7 is behind the robot, 0.7 - rear from the outline.
    close, nearest, _ = unexplained_near(column(0.7, 0.0), (0.0, 0.0, np.pi), FOOTPRINT, 0.45)
    assert close and nearest == pytest.approx(0.7 - FOOTPRINT['rear'], abs=1e-6)


def test_scan_points_skip_invalid_returns():
    points = scan_points([1.0, float('inf'), 0.01, 2.0], 0.0, np.pi / 2, 0.05, 12.0,
                         (1.0, 0.0, np.pi / 2))
    assert points == pytest.approx(np.array([[1.0, 1.0], [3.0, 0.0]]), abs=1e-9)


def test_off_map_points_count_as_free():
    close, _, _ = unexplained_near(column(-1.6, -1.9), (-1.6, -2.6, np.pi / 2), FOOTPRINT,
                                   0.45, wall_map())
    assert close


def test_points_in_a_static_zone_are_ignored():
    occupied = np.zeros((80, 80), dtype=bool)
    zone = [[0.6, -0.2], [0.9, -0.2], [0.9, 0.2], [0.6, 0.2]]
    with_zone = StaticMap(occupied, 0.05, (-2.0, -2.0), zones=[zone])
    close, _, _ = unexplained_near(column(0.75, 0.0), (0.0, 0.0, 0.0), FOOTPRINT, 0.45,
                                   with_zone)
    assert not close


def test_the_top_of_furniture_over_unknown_cells_does_not_hold():
    # A 0.5 m deep counter beside the robot: only its outline is occupied, its inside unknown
    # (-1), as in a lidar map. L2 points on its top, 0.25 m from the outline, are the counter.
    grid = np.zeros((80, 80), dtype=np.int16)
    grid[20:31, 30:51] = 100            # rows y -1.0..-0.45, cols x -0.5..0.55
    grid[21:30, 31:50] = -1
    smap = StaticMap.from_occupancy_grid(80, 80, 0.05, (-2.0, -2.0), grid.ravel())
    top = column(0.0, -0.72, spread=0.1)
    close, _, _ = unexplained_near(top, (0.0, 0.0, 0.0), FOOTPRINT, 0.6, smap)
    assert not close
    # The same points with the inside marked free would hold.
    grid[21:30, 31:50] = 0
    smap = StaticMap.from_occupancy_grid(80, 80, 0.05, (-2.0, -2.0), grid.ravel())
    close, _, _ = unexplained_near(top, (0.0, 0.0, 0.0), FOOTPRINT, 0.6, smap)
    assert close


def test_the_booth_needs_ten_points_to_hold():
    cfg = load_config()
    few = column(0.0, 0.0, count=cfg['hold_min_points'] - 1) + [FOOTPRINT['front'] + 0.2, 0]
    close, _, count = unexplained_near(few, (0.0, 0.0, 0.0), FOOTPRINT, cfg['hold_radius'],
                                       wall_map(), cfg['hold_map_margin'],
                                       cfg['hold_min_points'])
    assert count == cfg['hold_min_points'] - 1 and not close
    many = column(0.0, 0.0, count=cfg['hold_min_points']) + [FOOTPRINT['front'] + 0.2, 0]
    close, _, _ = unexplained_near(many, (0.0, 0.0, 0.0), FOOTPRINT, cfg['hold_radius'],
                                   wall_map(), cfg['hold_map_margin'], cfg['hold_min_points'])
    assert close
