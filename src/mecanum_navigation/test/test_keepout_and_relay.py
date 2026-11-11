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

"""Unit tests for the keepout mask geometry and the ultrasonic range relay."""

import math
import os

from mecanum_navigation import keepout
from mecanum_navigation.range_relay import relay_range
import numpy as np
import pytest
from sensor_msgs.msg import Range
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, '..', 'maps')
WORLD = os.path.join(HERE, '..', '..', 'mecanum_simulation', 'worlds', 'restroom.sdf')


def test_points_in_polygon():
    square = [[0, 0], [1, 0], [1, 1], [0, 1]]
    inside = keepout.points_in_polygon([[0.5, 0.5], [1.5, 0.5], [-0.1, 0.2], [0.9, 0.99]],
                                       square)
    assert inside.tolist() == [True, False, False, True]


def test_rasterize_marks_cells_and_thin_zones():
    # A 10 x 10 grid of 0.1 m cells from (0, 0).
    zones = [{'polygon': [[0.21, 0.21], [0.39, 0.21], [0.39, 0.39], [0.21, 0.39]]},
             {'polygon': [[0.71, 0.71], [0.72, 0.71], [0.72, 0.72], [0.71, 0.72]]}]
    grid = keepout.rasterize(zones, 10, 10, 0.1, [0.0, 0.0, 0.0])
    # Row 0 is the top (y 0.9..1.0). Centres 0.25 and 0.35 are inside the first zone.
    assert grid[9 - 2, 2] and grid[9 - 3, 3]
    assert not grid[9 - 1, 1] and not grid[9 - 4, 4]
    # The thin zone has no cell centre inside, but its corner cell is marked.
    assert grid[9 - 7, 7]
    assert grid.sum() == 5


def test_world_to_map():
    assert keepout.world_to_map([(1.0, 0.0)], (0.0, -1.0, 0.0)) == [(1.0, 1.0)]
    x, y = keepout.world_to_map([(0.0, 1.0)], (0.0, 0.0, math.pi / 2))[0]
    assert x == pytest.approx(1.0) and y == pytest.approx(0.0, abs=1e-12)


def test_restroom_zones_from_world():
    map_compare = pytest.importorskip('mecanum_localization.map_compare')
    shapes = map_compare.load_world_shapes(WORLD)
    zones = keepout.zones_from_world(shapes, 0.13, (0.0, 0.9), 0.05, (0.0, -1.0, 0.0))
    names = sorted(z['name'] for z in zones)
    # What the 2D map misses (Phase 5) and the robot can hit; not the door header (2.1 m up)
    # or the ceiling.
    assert names == sorted(['building/stall_partition_side', 'building/stall_partition_front',
                            'building/toilet_1_tank', 'threshold/link', 'glass_screen/link'])


def test_committed_mask_matches_its_zones():
    zones = keepout.load_zones(os.path.join(MAPS, 'restroom_keepout_zones.yaml'))
    with open(os.path.join(MAPS, 'restroom_keepout.yaml')) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(MAPS, 'restroom.yaml')) as f:
        map_meta = yaml.safe_load(f)
    assert meta['origin'][:2] == map_meta['origin'][:2]
    assert meta['resolution'] == map_meta['resolution']
    with open(os.path.join(MAPS, meta['image']), 'rb') as f:
        data = f.read()
    header = data.split(b'\n', 4)
    width, height = (int(v) for v in header[2].split())
    pixels = np.frombuffer(data[-width * height:], dtype=np.uint8).reshape(height, width)
    grid = keepout.rasterize(zones, width, height, meta['resolution'], meta['origin'])
    assert np.array_equal(pixels == 0, grid)
    assert set(np.unique(pixels)) <= {0, keepout.UNKNOWN_PIXEL}


def _range(value, min_range=0.03, max_range=4.5):
    msg = Range()
    msg.header.frame_id = 'ultrasonic_front_left_link'
    msg.min_range, msg.max_range, msg.range = min_range, max_range, value
    return msg


def test_relay_no_echo_becomes_max_range():
    out = relay_range(_range(math.inf))
    assert out.range == 4.5 and out.max_range == 4.5
    assert out.header.frame_id == 'ultrasonic_front_left_link'


def test_relay_passes_echoes_and_clamps():
    assert relay_range(_range(1.2)).range == pytest.approx(1.2)
    assert relay_range(_range(-math.inf)).range == pytest.approx(0.03)
    assert relay_range(_range(math.nan)) is None


def test_relay_cap_marks_only_near_echoes():
    near = relay_range(_range(0.3), mark_max_range=0.4)
    assert near.range == pytest.approx(0.3) and near.max_range == pytest.approx(0.4)
    far = relay_range(_range(2.0), mark_max_range=0.4)
    assert far.range == pytest.approx(0.4) and far.max_range == pytest.approx(0.4)
    none = relay_range(_range(math.inf), mark_max_range=0.4)
    assert none.range == pytest.approx(0.4)
