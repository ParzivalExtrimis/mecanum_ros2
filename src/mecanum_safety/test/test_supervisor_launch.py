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
Launch test of safety_supervisor with fake sensors, on the wall clock.

Every watched topic is published by this test. Each one is silenced in turn, and the gate must
report it lost and block the motion its rule names; the cliff, bumper and e-stop inputs are
then driven through their values.
"""

import math
import os
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from mecanum_interfaces.msg import SafetyState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, Range
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')
FULL_STOP = {'/scan': LaserScan, '/l2/points_filtered': PointCloud2,
             '/d435i/depth/color/points': PointCloud2, '/joint_states': JointState,
             '/odom': Odometry}
FORWARD, BACK = (0.2, 0.0, 0.0), (-0.2, 0.0, 0.0)
TIMEOUT = 3.0


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('mecanum_safety')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'safety.launch.py'))),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSupervisor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_safety_supervisor')
        cls.lock = threading.Lock()
        cls.muted = set()
        cls.cliff = {c: 0.119 for c in CORNERS}
        cls.bumper = {'front': False, 'rear': False}
        cls.cmd = FORWARD
        cls.out = None
        cls.state = None
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        cls.pubs = {t: cls.node.create_publisher(m, t, reliable) for t, m in FULL_STOP.items()}
        for c in CORNERS:
            cls.pubs[f'/ultrasonic/{c}'] = cls.node.create_publisher(Range, f'/ultrasonic/{c}', 10)
            cls.pubs[f'/cliff/{c}'] = cls.node.create_publisher(Range, f'/cliff/{c}', 10)
        for b in cls.bumper:
            cls.pubs[f'/bumper/{b}'] = cls.node.create_publisher(Bool, f'/bumper/{b}', 10)
        cls.estop_pub = cls.node.create_publisher(Bool, '/estop', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.cmd_pub = cls.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        cls.node.create_subscription(TwistStamped, '/cmd_vel_safe', cls._on_out, 10)
        cls.node.create_subscription(SafetyState, '/safety/state', cls._on_state, 10)
        cls.reset = cls.node.create_client(Trigger, '/safety_supervisor/reset')
        cls.node.create_timer(0.05, cls._publish_all)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_out(cls, msg):
        cls.out = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    @classmethod
    def _on_state(cls, msg):
        cls.state = msg

    @classmethod
    def _publish_all(cls):
        with cls.lock:
            for topic, pub in cls.pubs.items():
                if topic in cls.muted:
                    continue
                if topic.startswith('/cliff/'):
                    pub.publish(Range(range=cls.cliff[topic.split('/')[-1]]))
                elif topic.startswith('/bumper/'):
                    pub.publish(Bool(data=cls.bumper[topic.split('/')[-1]]))
                else:
                    pub.publish(pub.msg_type())
            msg = TwistStamped()
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = cls.cmd
            cls.cmd_pub.publish(msg)

    def wait_for(self, predicate, what):
        deadline = time.monotonic() + TIMEOUT
        while time.monotonic() < deadline:
            if self.state is not None and predicate(self.state):
                time.sleep(0.15)  # let the gated commands follow the state
                return
            time.sleep(0.02)
        self.fail(f'timed out waiting for {what}; last state {self.state}')

    def expect_clear(self):
        self.wait_for(lambda s: not s.reasons and not s.estop_latched, 'a clear state')

    def expect_output(self, cmd, expected):
        with self.lock:
            type(self).cmd = cmd
        time.sleep(0.2)
        self.assertEqual(self.out, expected, f'command {cmd}')

    def test_1_clear_passes(self):
        self.expect_clear()
        self.expect_output(FORWARD, FORWARD)

    def test_2_each_lost_topic(self):
        self.expect_clear()
        for topic in [*FULL_STOP, *(f'/{k}/{c}' for k in ('ultrasonic', 'cliff') for c in CORNERS),
                      '/bumper/front', '/bumper/rear']:
            with self.subTest(topic=topic):
                with self.lock:
                    self.muted.add(topic)
                self.wait_for(lambda s, t=topic: f'lost {t}' in s.reasons, f'lost {topic}')
                toward = BACK if 'rear' in topic else FORWARD
                away = FORWARD if 'rear' in topic else BACK
                self.expect_output(toward, (0.0, 0.0, 0.0))
                if topic in FULL_STOP:
                    self.assertTrue(self.state.stopped)
                    self.expect_output(away, (0.0, 0.0, 0.0))
                else:
                    self.assertFalse(self.state.stopped)
                    self.expect_output(away, away)
                with self.lock:
                    self.muted.discard(topic)
                self.expect_clear()

    def test_3_cliff_and_bumper(self):
        self.expect_clear()
        with self.lock:
            self.cliff['front_left'] = math.inf
        self.wait_for(lambda s: 'cliff front_left' in s.reasons, 'the cliff')
        self.expect_output(FORWARD, (0.0, 0.0, 0.0))
        self.expect_output(BACK, BACK)
        with self.lock:
            self.cliff['front_left'] = 0.119
            self.bumper['rear'] = True
        self.wait_for(lambda s: list(s.reasons) == ['bumper rear'], 'the rear bumper')
        self.expect_output(BACK, (0.0, 0.0, 0.0))
        self.expect_output(FORWARD, FORWARD)
        with self.lock:
            self.bumper['rear'] = False
        self.expect_clear()

    def test_4_estop_latches(self):
        self.expect_clear()
        self.assertTrue(self.reset.wait_for_service(timeout_sec=5.0))
        self.estop_pub.publish(Bool(data=True))
        self.wait_for(lambda s: s.estop_latched and s.stopped, 'the e-stop')
        self.expect_output(BACK, (0.0, 0.0, 0.0))
        self.assertFalse(self.call_reset().success, 'reset while pressed')
        self.estop_pub.publish(Bool(data=False))
        time.sleep(0.3)
        self.assertTrue(self.state.estop_latched, 'still latched after release')
        self.assertTrue(self.call_reset().success)
        self.expect_clear()
        self.expect_output(FORWARD, FORWARD)

    def call_reset(self):
        future = self.reset.call_async(Trigger.Request())
        deadline = time.monotonic() + TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done(), 'reset service')
        return future.result()
