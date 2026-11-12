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
Launch test (stage T0 of docs/hardware_drive_test.md): the real-drive stack on a fake ESP32.

Starts mecanum_hardware's fake_esp32.py on a pseudo-terminal, then controllers.launch.py
hardware:=esp32 on it, with config/controllers_hardware.yaml. Checks, in order (the methods run
in name order and share one launch):
  1. /wheel/odometry returns the commanded twist, and odom -> base_footprint is on /tf;
  2. the hardware speed limits apply;
  3. the reference timeout stops the wheels;
  4. a frozen encoder trips the following-error fault: everything stops, then the recovery of
     the test procedure (hardware active, controllers active) drives again;
  5. a mute link (as a pulled cable) stops everything, and the same recovery works.
"""

import math
import os
import signal
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ListHardwareComponents, SetHardwareComponentState, SwitchController)
from geometry_msgs.msg import TwistStamped
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from lifecycle_msgs.msg import State
from mecanum_interfaces.msg import DriveState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import yaml

SHARE = get_package_share_directory('mecanum_controllers')
FAKE = os.path.join(get_package_share_directory('mecanum_hardware'), 'test', 'fake_esp32.py')
RUN_DIR = tempfile.mkdtemp(prefix='fake_esp32_')
LINK = os.path.join(RUN_DIR, 'tty')
PID_FILE = os.path.join(RUN_DIR, 'pid')
CONTROLLERS = ['joint_state_broadcaster', 'mecanum_drive_controller']
# Mean of the measured twist over a steady second. The counts are exact on the fake, so this
# only allows for the count quantization and for STATE and read() running out of phase.
TOLERANCE_LINEAR = 0.003
TOLERANCE_ANGULAR = 0.01


def hardware_limits():
    with open(os.path.join(SHARE, 'config', 'controllers_hardware.yaml')) as f:
        return yaml.safe_load(f)['mecanum_drive_controller']['ros__parameters']


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['python3', FAKE, '--link', LINK, '--pid-file', PID_FILE], output='screen'),
        # The plugin opens the port on configure: give the fake a moment to make it.
        launch.actions.TimerAction(period=1.5, actions=[
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(SHARE, 'launch', 'controllers.launch.py')),
                launch_arguments={'hardware': 'esp32', 'port': LINK,
                                  'reference_topic': '/cmd_vel'}.items()),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestEsp32Fake(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_esp32_fake')
        cls.joints_time = 0.0
        cls.joints = {}
        cls.odom = []
        cls.drive_state = None
        cls.odom_tf = False
        cls.pub = cls.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        cls.node.create_subscription(JointState, '/joint_states', cls._on_joints, 10)
        cls.node.create_subscription(Odometry, '/wheel/odometry', cls._on_odom, 10)
        cls.node.create_subscription(DriveState, '/drive/state', cls._on_drive, 10)
        cls.node.create_subscription(TFMessage, '/tf', cls._on_tf, 10)
        cls.list_hw = cls.node.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')
        cls.set_hw = cls.node.create_client(
            SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
        cls.switch = cls.node.create_client(
            SwitchController, '/controller_manager/switch_controller')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joints(cls, msg):
        cls.joints = dict(zip(msg.name, msg.velocity))
        cls.joints_time = time.monotonic()

    @classmethod
    def _on_odom(cls, msg):
        cls.odom.append((time.monotonic(), msg))

    @classmethod
    def _on_drive(cls, msg):
        cls.drive_state = msg

    @classmethod
    def _on_tf(cls, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                cls.odom_tf = True

    # Helpers.

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def drive(self, vx, vy, wz, duration=2.5, settle=1.2):
        """Publish at 20 Hz; return the mean odometry twist after `settle` s."""
        start = time.monotonic()
        del self.odom[:]
        while time.monotonic() < start + duration:
            self.publish(vx, vy, wz)
            self.spin_for(0.05)
        window = [m.twist.twist for t, m in self.odom if t > start + settle]
        self.assertGreater(len(window), 10, 'too few /wheel/odometry messages')
        mean = [sum(getattr(getattr(w, a), b) for w in window) / len(window)
                for a, b in (('linear', 'x'), ('linear', 'y'), ('angular', 'z'))]
        return mean

    def assert_twist(self, measured, want, label):
        for axis, value, target, tol in zip(
                ('vx', 'vy', 'wz'), measured, want,
                (TOLERANCE_LINEAR, TOLERANCE_LINEAR, TOLERANCE_ANGULAR)):
            self.assertAlmostEqual(value, target, delta=tol, msg=f'{axis} for {label}')

    def wait_for_controllers(self, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and (len(self.joints) < 4 or not self.odom):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(len(self.joints), 4, 'no /joint_states from joint_state_broadcaster')
        self.assertTrue(self.odom, 'no /wheel/odometry from mecanum_drive_controller')

    def call(self, client, request, timeout=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), f'{client.srv_name} did not answer')
        return future.result()

    def hardware_state(self):
        result = self.call(self.list_hw, ListHardwareComponents.Request())
        return {c.name: c.state.label for c in result.component}['mecanum_base']

    def signal_fake(self, sig):
        with open(PID_FILE) as f:
            os.kill(int(f.read()), sig)

    def expect_stop(self, vx, timeout=5.0):
        """Keep commanding vx; return once /joint_states has stopped for 0.5 s."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.publish(vx, 0.0, 0.0)
            self.spin_for(0.05)
            if time.monotonic() - self.joints_time > 0.5:
                return
        self.fail('/joint_states still flows: the drive did not stop')

    def recover(self):
        """Recover as docs/hardware_drive_test.md says: hardware active, then the controllers."""
        request = SetHardwareComponentState.Request()
        request.name = 'mecanum_base'
        request.target_state = State(id=State.PRIMARY_STATE_ACTIVE, label='active')
        self.assertTrue(self.call(self.set_hw, request, timeout=15.0).ok,
                        'the hardware did not come back to active')
        request = SwitchController.Request()
        request.activate_controllers = CONTROLLERS
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(sec=5)
        self.assertTrue(self.call(self.switch, request).ok, 'the controllers did not reactivate')
        type(self).joints_time = 0.0   # the callback sets the class attribute
        end = time.monotonic() + 5.0
        while time.monotonic() < end and time.monotonic() - self.joints_time > 0.2:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertLess(time.monotonic() - self.joints_time, 0.2,
                        'no /joint_states after recovery')

    # Tests, in name order.

    def test_1_odometry_follows_twist(self):
        self.wait_for_controllers()
        self.assertEqual(self.hardware_state(), 'active')
        for twist in ((0.08, 0.0, 0.0), (0.0, -0.08, 0.0), (0.0, 0.0, 0.25),
                      (0.05, 0.05, -0.15)):
            self.assert_twist(self.drive(*twist), twist, f'twist {twist}')
        self.assertTrue(self.odom_tf, 'no odom -> base_footprint on /tf (enable_odom_tf)')
        self.assertIsNotNone(self.drive_state, 'no /drive/state from the plugin')
        self.assertEqual(self.drive_state.faults & 0x80, 0, 'fault latched on the fake')

    def test_2_speed_limits(self):
        self.wait_for_controllers()
        limits = hardware_limits()
        measured = self.drive(0.5, 0.0, 0.0)
        self.assertAlmostEqual(measured[0], limits['linear']['x']['max_velocity'],
                               delta=TOLERANCE_LINEAR, msg='linear.x.max_velocity')
        measured = self.drive(0.0, 0.0, -1.0)
        self.assertAlmostEqual(measured[2], limits['angular']['z']['min_velocity'],
                               delta=TOLERANCE_ANGULAR, msg='angular.z.min_velocity')

    def test_3_reference_timeout_stops_wheels(self):
        self.wait_for_controllers()
        self.drive(0.08, 0.0, 0.0)
        self.spin_for(1.5)
        for name, speed in self.joints.items():
            self.assertAlmostEqual(speed, 0.0, delta=1e-6, msg=name)
        for value in self.drive_state.command:
            self.assertEqual(value, 0.0)

    def test_4_following_error_stops_and_recovers(self):
        self.wait_for_controllers()
        self.drive(0.08, 0.0, 0.0, duration=1.0, settle=0.5)
        self.signal_fake(signal.SIGUSR1)   # freeze the front-left encoder
        try:
            self.expect_stop(0.08)
        finally:
            self.signal_fake(signal.SIGUSR1)
        self.assertNotEqual(self.hardware_state(), 'active')
        self.spin_for(1.0)
        self.recover()
        self.assert_twist(self.drive(0.08, 0.0, 0.0), (0.08, 0.0, 0.0), 'after the fault')
        self.assertEqual(self.drive_state.faults & 0x8F, 0, 'the fault was not cleared')

    def test_5_link_loss_stops_and_recovers(self):
        self.wait_for_controllers()
        self.drive(0.0, 0.08, 0.0, duration=1.0, settle=0.5)
        self.signal_fake(signal.SIGUSR2)   # mute the link
        try:
            self.expect_stop(0.08)
        finally:
            self.signal_fake(signal.SIGUSR2)
        self.assertNotEqual(self.hardware_state(), 'active')
        self.spin_for(1.0)
        self.recover()
        measured = self.drive(0.0, 0.08, 0.0)
        self.assert_twist(measured, (0.0, 0.08, 0.0), 'after the link loss')
        self.assertTrue(all(math.isfinite(v) for v in measured))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -6, -15])
