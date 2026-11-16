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
Planar geometry shared by the booth demo's nodes. Pure functions, no ROS.

A pose is (x, y, yaw) in the map frame. The footprint is the robot's rectangle around
base_footprint: ``front`` and ``rear`` along x, ``half_width`` along y.
"""

import math

import numpy as np


def wrap(angle):
    """Return ``angle`` wrapped to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def to_body(points, pose):
    """Return Nx2 map-frame ``points`` in the body frame of ``pose``."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def to_map(points, pose):
    """Return Nx2 body-frame ``points`` in the map frame, for a body at ``pose``."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                            y + s * points[:, 0] + c * points[:, 1]])


def footprint_distance(body_points, footprint, grow=0.0):
    """
    Return each body-frame point's distance to the footprint rectangle (0 inside).

    ``grow`` moves every side outward (negative: inward).
    """
    body_points = np.asarray(body_points, dtype=float).reshape(-1, 2)
    front = footprint['front'] + grow
    rear = footprint['rear'] + grow
    half = footprint['half_width'] + grow
    centre = (front - rear) / 2.0
    half_length = (front + rear) / 2.0
    dx = np.maximum(np.abs(body_points[:, 0] - centre) - half_length, 0.0)
    dy = np.maximum(np.abs(body_points[:, 1]) - half, 0.0)
    return np.hypot(dx, dy)


def footprint_corners(pose, footprint):
    """Return the footprint's 4 corners in the map frame."""
    f, r, h = footprint['front'], footprint['rear'], footprint['half_width']
    return to_map([[f, h], [f, -h], [-r, -h], [-r, h]], pose)


def sweep_poses(start, end, step=0.02, angle_step=0.035):
    """
    Return the poses between ``start`` and ``end`` (inclusive) of a straight move.

    The heading goes linearly from start[2] to end[2] (end[2] is not wrapped: a spin gives the
    unwrapped final yaw).
    """
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    turn = abs(end[2] - start[2])
    count = max(1, int(math.ceil(max(distance / step, turn / angle_step))))
    return [(start[0] + (end[0] - start[0]) * i / count,
             start[1] + (end[1] - start[1]) * i / count,
             start[2] + (end[2] - start[2]) * i / count) for i in range(count + 1)]


def sweep_clearance(start, end, obstacles, footprint):
    """
    Return the least distance from the footprint to Nx2 ``obstacles`` along a move.

    0 means the footprint touches or covers an obstacle point somewhere on the way. Returns
    inf with no obstacles.
    """
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    if len(obstacles) == 0:
        return math.inf
    least = math.inf
    for pose in sweep_poses(start, end):
        least = min(least, float(footprint_distance(to_body(obstacles, pose), footprint).min()))
    return least
