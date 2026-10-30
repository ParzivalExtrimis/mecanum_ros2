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
Simulated depth cloud: the D435i depth image and intrinsics in, a PointCloud2 out.

    /d435i/depth/image_rect_raw (Image, 32FC1, metres)
      + /d435i/depth/camera_info -> /d435i/depth/color/points (PointCloud2)

Gazebo's rgbd_camera publishes its own cloud, but expresses it with x forward and labels it with
the optical frame, so through TF it comes out rotated. This node computes the cloud from the
depth image instead, in d435i_optical_frame (z forward, x right, y down), like the depth
pipeline of the real camera. It holds x, y, z only, without colour, and only the valid pixels.

``stride`` keeps every n-th pixel in both directions. 2 leaves a 1.3 cm spacing at 3 m, enough
for the costmaps, and a quarter of the data.

Simulation only: on the robot the RealSense driver publishes the same topic.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def depth_to_points(depth, fx, fy, cx, cy, stride=1):
    """Return the Nx3 float32 points, in the optical frame, of the valid pixels of ``depth``."""
    rows, cols = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.0)
    z = z[valid]
    x = (cols[valid] - cx) * z / fx
    y = (rows[valid] - cy) * z / fy
    return np.stack((x, y, z), axis=-1).astype(np.float32)


class SimDepthCloud(Node):
    """Publish a cloud for every depth image, once the intrinsics are known."""

    def __init__(self):
        super().__init__('sim_depth_cloud')
        self.declare_parameter('depth_topic', '/d435i/depth/image_rect_raw')
        self.declare_parameter('info_topic', '/d435i/depth/camera_info')
        self.declare_parameter('cloud_topic', '/d435i/depth/color/points')
        self.declare_parameter('stride', 2)  # STAND-IN: for the sim load

        self._stride = int(self.get_parameter('stride').value)
        self._info = None
        cloud_topic = self.get_parameter('cloud_topic').value
        self._publisher = self.create_publisher(PointCloud2, cloud_topic, 1)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value, self._on_info, 1)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, 1)
        self.get_logger().info(f'depth image -> {cloud_topic}, stride {self._stride}')

    def _on_info(self, msg):
        self._info = msg

    def _on_depth(self, msg):
        if self._info is None:
            return
        if msg.encoding != '32FC1':
            self.get_logger().error(f'expected a 32FC1 depth image, got {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        k = self._info.k
        points = depth_to_points(depth, k[0], k[4], k[2], k[5], self._stride)

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self._publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = SimDepthCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
