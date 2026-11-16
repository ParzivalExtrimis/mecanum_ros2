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

"""ROS glue shared by the demo nodes: the site config, TF poses, and point conversions."""

import math

import numpy as np
import yaml


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def yaw_of(q):
    """Return the yaw of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotation_of(q):
    """Return the 3x3 rotation matrix of a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose2d_of(transform):
    """Return (x, y, yaw) of a geometry_msgs TransformStamped."""
    t = transform.transform
    return (t.translation.x, t.translation.y, yaw_of(t.rotation))


def cloud_xyz(msg):
    """Return a PointCloud2's x, y, z fields as an Nx3 float array (finite points only)."""
    offsets = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= offsets.keys() or msg.point_step == 0:
        return np.zeros((0, 3))
    count = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)[:count * msg.point_step]
    raw = raw.reshape(count, msg.point_step)
    dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
    out = np.column_stack([raw[:, offsets[k]:offsets[k] + 4].copy().view(dtype).ravel()
                           for k in ('x', 'y', 'z')]).astype(float)
    return out[np.isfinite(out).all(axis=1)]


def transform_points(points, transform):
    """Apply a TransformStamped to Nx3 points."""
    t = transform.transform
    rotation = rotation_of(t.rotation)
    shift = np.array([t.translation.x, t.translation.y, t.translation.z])
    return np.asarray(points, dtype=float) @ rotation.T + shift


def occupied_points(grid_msg, threshold=65):
    """Return the centres of an OccupancyGrid's occupied cells as Nx2 map points."""
    info = grid_msg.info
    grid = np.asarray(grid_msg.data, dtype=np.int16).reshape(info.height, info.width)
    rows, cols = np.nonzero(grid >= threshold)
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    yaw = yaw_of(info.origin.orientation)
    lx, ly = (cols + 0.5) * res, (rows + 0.5) * res
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([ox + c * lx - s * ly, oy + s * lx + c * ly])
