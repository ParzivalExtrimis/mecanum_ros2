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

"""
The booth layout: every prewritten move, swept with the real footprint through booth.sdf.

The poses in config/booth_demo.yaml were fitted by hand to the 2 x 2 m booth. This test fails
when a pose or the world changes so that a move comes closer to a wall or fixture than the
clearances below (owner, 2026-11-15: 5 cm; the spin at centre stage cannot have more than
about 4 cm between the counter and the bowl).
"""

import os

from ament_index_python.packages import get_package_share_directory
from demo_harness import Harness, load_config
from mecanum_demo.geometry import footprint_corners, sweep_poses, wrap
from mecanum_localization import map_compare as mc
import numpy as np
import pytest

CLEARANCE = 0.05
SPIN_CLEARANCE = 0.04
ROBOT_TOP = 0.75      # the body reaches 0.74 m above the floor


def world_shapes():
    world = os.path.join(get_package_share_directory('mecanum_simulation'), 'worlds',
                         'booth.sdf')
    return [s for s in mc.load_world_shapes(world) if s.z_range[0] < ROBOT_TOP]


def outline(pose, footprint, spacing=0.01):
    """Return points every ``spacing`` along the footprint's outline at ``pose``."""
    corners = footprint_corners(pose, footprint)
    points = []
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        count = max(2, int(np.hypot(*(b - a)) / spacing))
        points.append(a + (b - a) * np.linspace(0.0, 1.0, count, endpoint=False)[:, None])
    return np.vstack(points)


def clearance(start, end, shapes, footprint):
    least = np.inf
    for pose in sweep_poses(start, end):
        points = outline(pose, footprint)
        for shape in shapes:
            least = min(least, float(shape.signed_distance(points).min()))
    return least


def moves_of(harness, command):
    """Run ``command`` to the end of its first pass; return (label, kind, start, end)."""
    logic = harness.logic
    harness.ok(*command)
    out = []
    for _ in range(200):
        if logic.state != 'running' or logic.runs > 1:
            break
        motion = logic.motion
        if motion is None:
            logic.tick(0.5)
            continue
        start = logic.pose
        if motion['kind'] == 'move':
            yaw = start[2] if motion['keep_heading'] else start[2] + wrap(
                motion['yaw'] - start[2])
            end = (motion['x'], motion['y'], yaw)
        elif motion['kind'] == 'turn':
            end = (start[0], start[1], start[2] + wrap(motion['yaw'] - start[2]))
        else:
            end = (start[0], start[1], start[2] + motion['delta_yaw'])
        out.append((motion['label'], motion['kind'], start, end))
        harness.finish_motion()
    return out


CASES = [
    ('home', 'W', ('start_mode', '1')),
    ('dock', 'E', ('start_mode', '2')),
    ('a2', 'E', ('start_mode', '2')),
    ('a1', 'E', ('start_mode', '2')),
    ('b1', 'E', ('start_mode', '2')),
    ('b1', 'W', ('start_mode', '2')),
    ('stage', 'E', ('start_mode', '2')),
    ('stage', 'S', ('start_mode', '2')),
    ('w1', 'W', ('start_mode', '2')),
    ('w1', 'E', ('start_mode', '2')),
    ('stage', 'W', ('run_feature', 'greet')),
    ('stage', 'S', ('run_feature', 'spin')),
    ('dock', 'E', ('run_feature', 'back_out')),
]


@pytest.mark.parametrize('spot,heading,command', CASES)
def test_every_move_keeps_its_clearance(spot, heading, command):
    harness = Harness(at=spot, heading=heading)
    footprint = load_config()['footprint']
    shapes = world_shapes()
    moves = moves_of(harness, command)
    assert moves, 'the command produced no base moves'
    for label, kind, start, end in moves:
        least = clearance(start, end, shapes, footprint)
        needed = SPIN_CLEARANCE if kind in ('spin', 'turn') else CLEARANCE
        assert least >= needed, (
            f'"{label}" from {spot}/{heading}: clearance {least * 100:.1f} cm, '
            f'needs {needed * 100:.0f} cm')


def test_the_world_has_the_booth_fixtures():
    names = {s.name for s in world_shapes()}
    assert {'booth/toilet_1_bowl', 'booth/counter', 'booth/chair',
            'booth/wall_back'} <= names
