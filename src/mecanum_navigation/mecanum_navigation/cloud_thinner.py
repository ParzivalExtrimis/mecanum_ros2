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
Thin the D435i cloud for Nav2 (docs/plan.md Phase 6).

    /d435i/depth/color/points --cloud_thinner--> /costmap/d435i/points

The camera cloud is 1 to 1.6 MB. The collision monitor and the voxel layer subscribe to it
best-effort, which cannot be set from outside, and at the default socket buffers Cyclone DDS
loses whole large best-effort messages (Phase 2). Facing a wall up close every pixel is valid,
the cloud is at its largest, and the collision monitor received none for minutes: it held the
robot stopped ("invalid source") with no way out (Phase 6). This node takes the cloud reliably
and keeps one point per ``leaf_size`` cube (the costmaps' 5 cm cells), a few thousand points,
small enough for best-effort delivery.

The point kept in each cube is the first one in it, not the mean, so every point is a real
measurement. Stamp and frame are the input's; the node needs no clock and stays off /clock.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

_OFFSET = 1 << 20          # keeps the cube indices positive (21 bits each, +-10 km at 5 cm)


def xyz_of(msg):
    """Return the Nx3 float32 x, y, z of a PointCloud2 (any point layout), finite points only."""
    offsets = {f.name: f.offset for f in msg.fields if f.datatype == PointField.FLOAT32}
    if not all(name in offsets for name in 'xyz') or msg.point_step == 0:
        return np.zeros((0, 3), np.float32)
    count = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * msg.point_step)
    raw = raw.reshape(count, msg.point_step)
    points = np.empty((count, 3), np.float32)
    for i, name in enumerate('xyz'):
        o = offsets[name]
        points[:, i] = raw[:, o:o + 4].copy().view(np.float32).ravel()
    return points[np.isfinite(points).all(axis=1)]


def thin(points, leaf_size):
    """Return one point per ``leaf_size`` cube: the first in each, in input order."""
    if len(points) == 0:
        return points
    cube = np.floor(points / leaf_size).astype(np.int64) + _OFFSET
    key = (cube[:, 0] << 42) | (cube[:, 1] << 21) | cube[:, 2]
    _, first = np.unique(key, return_index=True)
    return points[np.sort(first)]


def to_cloud(header, points):
    """Return an x, y, z PointCloud2 of ``points``."""
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate('xyz')]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return cloud


class CloudThinner(Node):
    """Republish a thinned copy of each cloud."""

    def __init__(self):
        super().__init__('cloud_thinner')
        self.declare_parameter('input_topic', '/d435i/depth/color/points')
        self.declare_parameter('output_topic', '/costmap/d435i/points')
        self.declare_parameter('leaf_size', 0.05)   # m: the costmaps' and the voxel layer's cell
        self._leaf = float(self.get_parameter('leaf_size').value)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        # Reliable out: it matches reliable and best-effort subscribers.
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, reliable)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._on_cloud, reliable)

    def _on_cloud(self, msg):
        self._pub.publish(to_cloud(msg.header, thin(xyz_of(msg), self._leaf)))


def main(args=None):
    rclpy.init(args=args)
    node = CloudThinner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
