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
LIO odometry adapter: 3D LIO odometry in, planar /odom and odom -> base_footprint TF out.

rko_lio estimates a full 3D pose of base_footprint in its own frame (lio_odom). The base drives
on a flat floor, so this node keeps x, y and yaw only, makes the first pose the origin, and
publishes the result as the robot's odometry:

    /lio/raw_odom (rko_lio, lio_odom -> base_footprint, 3D)
      -> /odom (odom -> base_footprint, planar) and TF odom -> base_footprint

The twist is passed through as vx, vy and wz in base_footprint (the nav_msgs/Odometry
convention, which rko_lio follows). Covariances are fixed, non-zero diagonals from parameters.

``publish_tf`` and ``odom_topic`` exist for the encoder upgrade (docs/encoder_upgrade.md): with
an EKF, this node publishes on /lio/odom with publish_tf false and the EKF owns the TF.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformBroadcaster

from .planar_math import PlanarFlattener, quaternion_from_yaw


def diagonal_covariance(diagonal):
    """Return a row-major 6x6 covariance (36 floats) with ``diagonal`` on its diagonal."""
    if len(diagonal) != 6:
        raise ValueError(f'covariance diagonal needs 6 values, got {len(diagonal)}')
    cov = [0.0] * 36
    for i, value in enumerate(diagonal):
        cov[i * 7] = float(value)
    return cov


class LioOdomAdapter(Node):

    def __init__(self):
        super().__init__('lio_odom_adapter')
        self.declare_parameter('input_topic', '/lio/raw_odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        # STAND-IN: pose (x, y, z, roll, pitch, yaw) and twist variances are guesses; set them
        # from the 4H static and loop drift once LIO has run on the real robot. z, roll and
        # pitch are not estimated (planar) and get large variances.
        self.declare_parameter('pose_covariance_diagonal',
                               [0.0025, 0.0025, 1.0e6, 1.0e6, 1.0e6, 0.0012])
        self.declare_parameter('twist_covariance_diagonal',
                               [0.0025, 0.0025, 1.0e6, 1.0e6, 1.0e6, 0.0025])

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._pose_cov = diagonal_covariance(
            self.get_parameter('pose_covariance_diagonal').value)
        self._twist_cov = diagonal_covariance(
            self.get_parameter('twist_covariance_diagonal').value)
        self._flatten = PlanarFlattener()
        self._warned_frame = False

        odom_topic = self.get_parameter('odom_topic').value
        input_topic = self.get_parameter('input_topic').value
        self._pub = self.create_publisher(Odometry, odom_topic, qos_profile_system_default)
        self._tf = (TransformBroadcaster(self) if self.get_parameter('publish_tf').value
                    else None)
        self.create_subscription(Odometry, input_topic, self._on_odom, qos_profile_system_default)
        self.get_logger().info(
            f'{input_topic} -> {odom_topic} ({self._odom_frame} -> {self._base_frame}), '
            f'TF {"on" if self._tf else "off"}')

    def _on_odom(self, msg):
        if msg.child_frame_id != self._base_frame and not self._warned_frame:
            self._warned_frame = True
            self.get_logger().warn(
                f'LIO child frame is "{msg.child_frame_id}", expected "{self._base_frame}"; '
                'the planar pose and twist will describe the wrong frame')

        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y, yaw = self._flatten(p.x, p.y, q.x, q.y, q.z, q.w)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        out.pose.covariance = self._pose_cov
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        out.twist.covariance = self._twist_cov
        self._pub.publish(out)

        if self._tf is not None:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self._base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            r = t.transform.rotation
            r.x, r.y, r.z, r.w = qx, qy, qz, qw
            self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomAdapter()
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
