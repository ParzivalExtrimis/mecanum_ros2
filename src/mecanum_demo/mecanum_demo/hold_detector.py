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
"Someone is close": obstacle points near the robot that the static map does not explain.

Pure functions, no ROS. demo_supervisor gives it the 2D lidar's points and the L2's filtered
cloud, flattened, in the map frame. A point counts when it is within ``radius`` of the robot's
outline and more than ``map_margin`` from every occupied cell of the static map, so that the
booth's walls and fixtures never hold the robot, even with a few centimetres of localization
error. Unknown cells count as free.

The simulator's rule was a person within 0.45 m of the outline; with the C1 seeing only the
front half (±89.5° in sim), the L2 covers the sides and the back.
"""

import math

from mecanum_demo.geometry import footprint_distance, to_body
import numpy as np
from scipy import ndimage


class StaticMap:
    """The static map's distance field: metres from each cell to the nearest occupied cell."""

    def __init__(self, occupied, resolution, origin, zones=()):
        """
        Build the field from ``occupied``: (rows, cols) bool, row 0 at the BOTTOM.

        Row 0 at the bottom is OccupancyGrid's order. ``zones`` are polygons (lists of map
        points) whose cells count as occupied too.
        """
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.shape = occupied.shape
        occupied = occupied.copy()
        if zones:
            rows, cols = np.indices(occupied.shape)
            centres = np.column_stack([
                (self.origin[0] + (cols.ravel() + 0.5) * self.resolution),
                (self.origin[1] + (rows.ravel() + 0.5) * self.resolution)])
            for polygon in zones:
                occupied |= points_in_polygon(centres, polygon).reshape(occupied.shape)
        if occupied.any():
            self.distance = ndimage.distance_transform_edt(~occupied) * self.resolution
        else:
            self.distance = np.full(occupied.shape, np.inf)

    @classmethod
    def from_occupancy_grid(cls, width, height, resolution, origin, data, threshold=65,
                            zones=()):
        # Unknown cells count as static too: near the robot they are the insides of walls and
        # furniture (a counter's top shows up in the L2 over its unknown inside), never floor a
        # person could stand on.
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls((grid >= threshold) | (grid < 0), resolution, origin, zones)

    def distance_at(self, points):
        """Return each Nx2 map point's distance to the nearest static cell (inf off the map)."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        cols = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((points[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (cols >= 0) & (cols < self.shape[1]) & (rows >= 0) & (rows < self.shape[0])
        out = np.full(len(points), np.inf)
        out[inside] = self.distance[rows[inside], cols[inside]]
        return out


def points_in_polygon(points, polygon):
    """Return a bool per Nx2 point: inside the polygon (even-odd rule)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    px, py = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        crosses = (y1 > py) != (y2 > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_at = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (px < x_at)
    return inside


def unexplained_near(points, pose, footprint, radius, static_map=None, map_margin=0.10,
                     min_points=3):
    """
    Return (someone_close, nearest distance in m or inf, number of points that count).

    ``points`` are Nx2 in the map frame. ``min_points`` keeps a few noisy returns from
    holding the robot.
    """
    mask, distance = unexplained_mask(points, pose, footprint, radius, static_map, map_margin)
    count = int(mask.sum())
    nearest = float(distance[mask].min()) if count else math.inf
    return count >= min_points, nearest, count


def unexplained_mask(points, pose, footprint, radius, static_map=None, map_margin=0.10):
    """
    Return (mask, distance) over the Nx2 map ``points``.

    mask: the points within ``radius`` of the robot's outline that the static map does not
    explain; distance: every point's distance to the outline.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0 or pose is None:
        return np.zeros(len(points), dtype=bool), np.full(len(points), math.inf)
    distance = footprint_distance(to_body(points, pose), footprint)
    near = distance <= radius
    if static_map is not None and near.any():
        near &= static_map.distance_at(points) > map_margin
    return near, distance


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, sensor_pose):
    """Return a LaserScan's valid returns as Nx2 map points, for the sensor at ``sensor_pose``."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, a = ranges[valid], angles[valid]
    x, y, yaw = sensor_pose
    return np.column_stack([x + r * np.cos(a + yaw), y + r * np.sin(a + yaw)])
