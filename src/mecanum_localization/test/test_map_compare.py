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

"""Check the map-versus-world geometry used by tools/map_check.py."""

import math
import os

from mecanum_localization import map_compare as mc
import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RESTROOM = os.path.join(HERE, '..', '..', 'mecanum_simulation', 'worlds', 'restroom.sdf')

WORLD = """<?xml version="1.0"?>
<sdf version="1.9"><world name="w">
  <model name="room"><static>true</static><pose>1 0 0 0 0 0</pose>
    <link name="wall"><pose>0 2 1 0 0 0</pose>
      <collision name="c"><geometry><box><size>4 0.1 2</size></box></geometry></collision>
    </link>
    <link name="post"><pose>0 0 0.5 0 0 0</pose>
      <collision name="c"><geometry><cylinder><radius>0.2</radius><length>1</length>
      </cylinder></geometry></collision>
    </link>
    <link name="shelf"><pose>2 0 1 0 0 0</pose>
      <collision name="c"><geometry><box><size>0.5 0.5 0.2</size></box></geometry></collision>
    </link>
  </model>
  <model name="glass"><static>true</static><pose>-2 0 1 0 0 1.5707963</pose>
    <link name="l">
      <collision name="c"><geometry><box><size>1 0.02 2</size></box></geometry></collision>
      <visual name="v"><visibility_flags>268435456</visibility_flags></visual>
    </link>
  </model>
  <model name="robot"><pose>0 0 0 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry>
    </collision></link>
  </model>
</world></sdf>
"""


@pytest.fixture
def world(tmp_path):
    path = tmp_path / 'w.sdf'
    path.write_text(WORLD)
    return mc.load_world_shapes(str(path))


def by_name(shapes, name):
    return next(s for s in shapes if s.name == name)


def test_parses_static_shapes_with_composed_poses(world):
    assert {s.name for s in world} == {'room/wall', 'room/post', 'room/shelf', 'glass/l'}
    wall = by_name(world, 'room/wall')
    assert (wall.kind, wall.x, wall.y, wall.z) == ('box', 1.0, 2.0, 1.0)
    post = by_name(world, 'room/post')
    assert post.kind == 'cylinder' and post.size == (0.2, 1.0)


def test_glass_is_hidden_and_shelf_above_the_plane(world):
    assert by_name(world, 'glass/l').hidden
    expected = {s.name for s in mc.expected_shapes(world, 0.13)}
    assert expected == {'room/wall', 'room/post'}


def test_signed_distance_box_and_cylinder(world):
    wall = by_name(world, 'room/wall')
    d = wall.signed_distance([[1.0, 1.0], [1.0, 2.0], [4.0, 2.0]])
    assert d == pytest.approx([0.95, -0.05, 1.0])
    post = by_name(world, 'room/post')
    assert post.signed_distance([[2.0, 0.0]]) == pytest.approx([0.8])


def test_rotated_box_distance_and_faces(world):
    glass = by_name(world, 'glass/l')    # 1 m long along y after the 90 degree yaw
    assert glass.signed_distance([[-2.0, 0.6]]) == pytest.approx([0.1], abs=1e-6)
    normals = {label: n for label, _, _, n in glass.faces()}
    assert normals['+x'] == pytest.approx((0.0, 1.0), abs=1e-6)


def test_closest_points_on_the_outline(world):
    wall = by_name(world, 'room/wall')
    assert wall.closest_points([[1.0, 1.0]])[0] == pytest.approx([1.0, 1.95])
    assert wall.closest_points([[1.0, 2.01]])[0] == pytest.approx([1.0, 2.05])


def test_face_offsets_signed_out_of_the_box(world):
    wall = by_name(world, 'room/wall')
    xs = np.linspace(-0.5, 2.5, 31)
    cells = np.column_stack([xs, np.full_like(xs, 1.92)])   # 3 cm into the room
    faces = mc.face_offsets(cells, [wall])
    south = next(f for f in faces if f['face'] == '-y')
    assert south['cells'] == 31 and south['judged']
    assert south['median'] == pytest.approx(0.03)
    assert all(f['cells'] == 0 for f in faces if f['face'] != '-y')


def test_short_faces_are_not_judged(world):
    wall = by_name(world, 'room/wall')
    ys = np.linspace(1.96, 2.04, 9)
    cells = np.column_stack([np.full_like(ys, 3.02), ys])
    east = next(f for f in mc.face_offsets(cells, [wall]) if f['face'] == '+x')
    assert east['cells'] == 9 and not east['judged']


def test_transform_round_trip():
    pts = np.array([[1.0, 0.0], [0.0, 2.0]])
    moved = mc.transform(pts, (1.0, -1.0, math.pi / 2))
    assert moved == pytest.approx(np.array([[1.0, 0.0], [-1.0, -1.0]]))


def test_best_fit_recovers_a_known_offset(world):
    shapes = mc.expected_shapes(world, 0.13)
    wall, post = shapes
    xs = np.linspace(-0.8, 2.8, 50)
    on_wall = np.column_stack([xs, np.full_like(xs, 1.95)])
    angles = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    on_post = np.column_stack([1.0 + 0.2 * np.cos(angles), 0.2 * np.sin(angles)])
    truth = np.vstack([on_wall, on_post])
    offset = (0.04, -0.03, math.radians(1.0))
    # Points as a map would hold them: the inverse of the offset applied.
    c, s = math.cos(offset[2]), math.sin(offset[2])
    shifted = (truth - offset[:2]) @ np.array([[c, -s], [s, c]])
    pose, residual = mc.best_fit(shifted, shapes)
    assert pose == pytest.approx(offset, abs=2e-3)
    assert residual.max() < 0.005


def test_coverage_counts_only_near_cells(world):
    shapes = mc.expected_shapes(world, 0.13)
    counts = mc.shape_coverage([[1.0, 1.9], [1.0, 0.25], [5.0, 5.0]], shapes)
    assert counts == [1, 1]


def test_reads_a_map_server_map(tmp_path):
    # 3 x 2 map, origin (-1, -1); top-left pixel occupied (0), the rest free (254).
    pixels = bytes([0, 254, 254, 254, 254, 254])
    (tmp_path / 'm.pgm').write_bytes(b'P5\n# comment\n3 2\n255\n' + pixels)
    (tmp_path / 'm.yaml').write_text(yaml.safe_dump({
        'image': 'm.pgm', 'mode': 'trinary', 'resolution': 0.5,
        'origin': [-1.0, -1.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.25}))
    cells, resolution = mc.load_occupied_cells(str(tmp_path / 'm.yaml'))
    assert resolution == 0.5
    # Top row is the higher y: centre (-1 + 0.25, -1 + 0.5 + 0.25).
    assert cells.tolist() == [[-0.75, -0.25]]


def test_restroom_world_expectations():
    shapes = mc.load_world_shapes(RESTROOM)
    expected = {s.name for s in mc.expected_shapes(shapes, 0.13)}
    # The C1's plane passes under the stall partitions, over the threshold, and the glass is
    # hidden from it; the post, the pedestal and the bowl are in it.
    assert 'building/stall_partition_post' in expected
    assert 'building/toilet_1_pedestal' in expected
    assert 'building/toilet_1_bowl' in expected
    for missed in ('building/stall_partition_side', 'building/stall_partition_front',
                   'threshold/link', 'glass_screen/link', 'building/toilet_1_tank',
                   'ceiling/link'):
        assert missed not in expected
    assert by_name(shapes, 'glass_screen/link').hidden
