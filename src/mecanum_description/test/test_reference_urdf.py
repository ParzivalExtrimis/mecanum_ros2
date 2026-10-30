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

"""Check the xacro description against the reference URDF refs/washroom_robot.urdf."""

import math
import os
import xml.etree.ElementTree as ET

import pytest
import xacro

PKG = os.path.join(os.path.dirname(__file__), '..')
REFERENCE = os.path.join(PKG, 'refs', 'washroom_robot.urdf')
XACRO = os.path.join(PKG, 'urdf', 'mecanum.urdf.xacro')

# Frames that differ from the reference on purpose (reference name -> description name,
# offset of the description frame in the reference frame).
RENAMED = {
    # The L2 frame here is the bottom mounting face (vendor origin), not the housing centre.
    'unitree_l2_link': ('l2_mount', (0.0, 0.0, -0.0325)),
}
# The reference wheels sit 0.2 mm off centre in y; the description makes them symmetric.
WHEEL_Y_TOLERANCE = 0.00025
POSITION_TOLERANCE = 1e-4
ROTATION_TOLERANCE = 1e-6


def rpy_matrix(roll, pitch, yaw):
    """Return the rotation matrix of URDF fixed-axis roll, pitch, yaw."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def compose(a, b):
    """Compose two (rotation, translation) transforms: a then b."""
    ra, ta = a
    rb, tb = b
    r = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [ta[i] + sum(ra[i][k] * tb[k] for k in range(3)) for i in range(3)]
    return r, t


def floats(text, default='0 0 0'):
    return [float(v) for v in (text or default).split()]


def world_poses(root, base='base_footprint'):
    """Pose of every link in `base`, composed through the joint tree."""
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = floats(origin.get('xyz') if origin is not None else None)
        rpy = floats(origin.get('rpy') if origin is not None else None)
        joints[joint.find('child').get('link')] = (
            joint.find('parent').get('link'), (rpy_matrix(*rpy), xyz))
    identity = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])
    # Re-root the tree at `base` when it is a child (the reference has base_link as root).
    if base in joints:
        parent, (r, t) = joints[base]
        r_inv = [[r[j][i] for j in range(3)] for i in range(3)]
        t_inv = [-sum(r_inv[i][k] * t[k] for k in range(3)) for i in range(3)]
        start = {base: identity, parent: (r_inv, t_inv)}
    else:
        start = {base: identity}
    poses = dict(start)

    def pose(link):
        if link not in poses:
            parent, tf = joints[link]
            poses[link] = compose(pose(parent), tf)
        return poses[link]

    for link in joints:
        pose(link)
    return poses


@pytest.fixture(scope='module')
def description():
    return ET.fromstring(xacro.process_file(XACRO).toxml())


@pytest.fixture(scope='module')
def reference():
    return ET.parse(REFERENCE).getroot()


def test_frames_match_reference(description, reference):
    ours = world_poses(description)
    theirs = world_poses(reference)
    checked = 0
    for ref_name, (ref_r, ref_t) in theirs.items():
        name, offset = RENAMED.get(ref_name, (ref_name, (0.0, 0.0, 0.0)))
        assert name in ours, f'{name} missing from the description'
        expected_t = [ref_t[i] + sum(ref_r[i][k] * offset[k] for k in range(3))
                      for i in range(3)]
        r, t = ours[name]
        tol = [POSITION_TOLERANCE] * 3
        if name.endswith('_wheel_link'):
            tol[1] = WHEEL_Y_TOLERANCE
        for i in range(3):
            assert t[i] == pytest.approx(expected_t[i], abs=tol[i]), f'{name} position {i}'
            for j in range(3):
                assert r[i][j] == pytest.approx(ref_r[i][j], abs=ROTATION_TOLERANCE), \
                    f'{name} rotation {i}{j}'
        checked += 1
    assert checked == len(theirs)


def test_one_collision_per_wheel(description):
    # The Gazebo friction block merges into the single unnamed wheel collision.
    for link in description.findall('link'):
        if link.get('name').endswith('_wheel_link'):
            assert len(link.findall('collision')) == 1, link.get('name')


def test_total_mass(description, reference):
    def total(root):
        return sum(float(m.get('value')) for m in root.iter('mass'))
    # The description adds the L2's vendor mass (0.230 kg) on l2_mount.
    assert total(description) == pytest.approx(total(reference) + 0.230, abs=1e-6)
