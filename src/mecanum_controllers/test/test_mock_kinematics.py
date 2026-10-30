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
Launch test: the drive controller on mock hardware against mecanum_base.kinematics.

Starts controllers.launch.py hardware:=mock, sends TwistStamped references on /cmd_vel and
checks that the commanded wheel velocities (echoed to /joint_states by the mock hardware)
match mecanum_base.kinematics.inverse in sign and magnitude, that /wheel/odometry returns
the commanded twist, that the speed limits in config/controllers.yaml apply, and that the
wheels stop when references stop.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from mecanum_base.kinematics import inverse, MecanumGeometry, WHEEL_NAMES
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import JointState
import yaml

SHARE = get_package_share_directory('mecanum_controllers')
TOLERANCE = 1e-3


def geometry():
    """Return the geometry in config/controllers.yaml as lx = sum, ly = 0 (only lx + ly counts)."""
    with open(os.path.join(SHARE, 'config', 'controllers.yaml')) as f:
        kin = yaml.safe_load(f)['mecanum_drive_controller']['ros__parameters']['kinematics']
    total = kin['sum_of_robot_center_projection_on_X_Y_axis']
    return MecanumGeometry(lx=total / 2, ly=total / 2, wheel_radius=kin['wheels_radius'])


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SHARE, 'launch', 'controllers.launch.py')),
            launch_arguments={'hardware': 'mock'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMockKinematics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_mock_kinematics')
        cls.joints = {}
        cls.odom = None
        cls.pub = cls.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        cls.node.create_subscription(JointState, '/joint_states', cls._on_joints, 10)
        cls.node.create_subscription(Odometry, '/wheel/odometry', cls._on_odom, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joints(cls, msg):
        cls.joints = dict(zip(msg.name, msg.velocity))

    @classmethod
    def _on_odom(cls, msg):
        cls.odom = msg

    def drive(self, vx, vy, wz, duration=1.5):
        """Publish the twist at 20 Hz for `duration` s; return the last wheel speeds and odom."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return dict(self.joints), self.odom

    def wait_for_controllers(self, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and (len(self.joints) < 4 or self.odom is None):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(len(self.joints), 4, 'no /joint_states from joint_state_broadcaster')
        self.assertIsNotNone(self.odom, 'no /wheel/odometry from mecanum_drive_controller')

    def test_wheel_speeds_match_kinematics(self):
        self.wait_for_controllers()
        geo = geometry()
        for twist in ((0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.5), (0.1, -0.15, 0.3)):
            joints, odom = self.drive(*twist)
            expected = inverse(*twist, geo)
            for wheel, speed in zip(WHEEL_NAMES, expected):
                self.assertAlmostEqual(joints[f'{wheel}_wheel_joint'], speed,
                                       delta=TOLERANCE, msg=f'{wheel} for twist {twist}')
            measured = (odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                        odom.twist.twist.angular.z)
            for axis, value, want in zip(('vx', 'vy', 'wz'), measured, twist):
                self.assertAlmostEqual(value, want, delta=TOLERANCE,
                                       msg=f'odometry {axis} for twist {twist}')

    def test_speed_limits(self):
        self.wait_for_controllers()
        with open(os.path.join(SHARE, 'config', 'controllers.yaml')) as f:
            params = yaml.safe_load(f)['mecanum_drive_controller']['ros__parameters']
        for twist, axis in (((-1.0, 0.0, 0.0), ('linear', 'x', 'min_velocity')),
                            ((0.0, 1.0, 0.0), ('linear', 'y', 'max_velocity')),
                            ((0.0, 0.0, 3.0), ('angular', 'z', 'max_velocity'))):
            _, odom = self.drive(*twist)
            limit = params[axis[0]][axis[1]][axis[2]]
            measured = {'linear': odom.twist.twist.linear,
                        'angular': odom.twist.twist.angular}[axis[0]]
            self.assertAlmostEqual(getattr(measured, axis[1]), limit, delta=TOLERANCE,
                                   msg=f'{".".join(axis)} for twist {twist}')

    def test_reference_timeout_stops_wheels(self):
        self.wait_for_controllers()
        self.drive(0.2, 0.0, 0.0)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for name, speed in self.joints.items():
            self.assertAlmostEqual(speed, 0.0, delta=TOLERANCE, msg=name)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -6, -15])
