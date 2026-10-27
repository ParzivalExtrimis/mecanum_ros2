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
Planar (SE(2)) pose math for the LIO odometry adapter and the ground-truth comparison.

A planar pose is a tuple (x, y, yaw). Quaternions are (x, y, z, w) tuples. Nothing here
depends on ROS, so it is unit-tested directly.
"""

import math


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Return the yaw (rad) of a quaternion: the Z angle of its Z-Y-X Euler decomposition."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quaternion_from_yaw(yaw):
    """Return the (x, y, z, w) quaternion of a pure rotation by ``yaw`` about Z."""
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def compose(a, b):
    """Return a ⊕ b: pose ``b`` (expressed in frame ``a``) expressed in a's parent frame."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap_angle(ayaw + byaw))


def inverse(a):
    """Return the inverse pose of ``a``."""
    ax, ay, ayaw = a
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (-c * ax - s * ay, s * ax - c * ay, wrap_angle(-ayaw))


def relative(origin, pose):
    """Return ``pose`` expressed in the frame of ``origin`` (both in the same parent frame)."""
    return compose(inverse(origin), pose)


def flatten(px, py, qx, qy, qz, qw):
    """Project a 3D pose (position and orientation quaternion) to a planar pose."""
    return (px, py, yaw_from_quaternion(qx, qy, qz, qw))


class PlanarFlattener:
    """
    Turn a stream of 3D odometry poses into planar poses that start at the origin.

    The first pose seen becomes the origin; every later pose is returned relative to it, so the
    output starts at (0, 0, 0) whatever frame the 3D odometry was initialized in.
    """

    def __init__(self):
        self.origin = None

    def reset(self):
        """Forget the origin; the next pose becomes the new origin."""
        self.origin = None

    def __call__(self, px, py, qx, qy, qz, qw):
        """Return the planar pose of a 3D pose, relative to the first pose seen."""
        planar = flatten(px, py, qx, qy, qz, qw)
        if self.origin is None:
            self.origin = planar
        return relative(self.origin, planar)
