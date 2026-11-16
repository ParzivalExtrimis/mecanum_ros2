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
Launch test of the demo supervisor, on the wall clock, with a fake base and localization.

The test stands in for the segment driver (a /demo/move_segment server that "arrives" after
0.2 s by moving TF map -> base_footprint) and for the panel (commands, /panel/drive,
/panel/heartbeat). It runs on ROS_DOMAIN_ID 93; run it with no simulation up.
"""

import math
import os
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '93'

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped  # noqa: E402
import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing  # noqa: E402, F401
import launch_testing.actions  # noqa: E402
from mecanum_interfaces.action import MoveSegment  # noqa: E402
from mecanum_interfaces.msg import SupervisorState  # noqa: E402
from mecanum_interfaces.srv import SupervisorCommand  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, Header  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402
import yaml  # noqa: E402

TIMEOUT = 5.0
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='mecanum_demo', executable='demo_supervisor',
                                output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDemoSupervisor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(get_package_share_directory('mecanum_demo'), 'config',
                               'booth_demo.yaml')) as f:
            cfg = yaml.safe_load(f)
        x, y = cfg['poses']['home']
        cls.pose = [x, y, cfg['headings']['W']]
        cls.state = None
        cls.goals = []
        cls.cancels = 0
        cls.cmds = []
        cls.heartbeat = True
        cls.drive = None
        rclpy.init()
        cls.node = rclpy.create_node('test_demo_supervisor')
        group = ReentrantCallbackGroup()
        cls.tf = TransformBroadcaster(cls.node)
        cls.node.create_subscription(SupervisorState, '/supervisor/state', cls._on_state,
                                     LATCHED)
        cls.node.create_subscription(TwistStamped, '/cmd_vel', cls._on_cmd, 10)
        cls.client = cls.node.create_client(SupervisorCommand, '/supervisor/command')
        cls.estop = cls.node.create_publisher(Bool, '/estop', LATCHED)
        cls.drive_pub = cls.node.create_publisher(Twist, '/panel/drive', 10)
        cls.beat_pub = cls.node.create_publisher(Header, '/panel/heartbeat', 10)
        cls.external_pub = cls.node.create_publisher(TwistStamped, '/manual/cmd_vel', 10)
        cls.bypass = []
        cls.node.create_subscription(Bool, '/safety/bypass',
                                     lambda m: cls.bypass.append((time.monotonic(), m.data)), 10)
        ActionServer(cls.node, MoveSegment, '/demo/move_segment', cls._execute,
                     cancel_callback=cls._on_cancel, callback_group=group)
        cls.node.create_timer(0.05, cls._tick)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_state(cls, msg):
        cls.state = msg

    @classmethod
    def _on_cmd(cls, msg):
        cls.cmds.append((time.monotonic(), msg.twist.linear.x))

    @classmethod
    def _on_cancel(cls, _):
        cls.cancels += 1
        return CancelResponse.ACCEPT

    @classmethod
    def _execute(cls, handle):
        goal = handle.request
        cls.goals.append(goal.label)
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            if handle.is_cancel_requested:
                handle.canceled()
                return MoveSegment.Result(success=False, error='canceled')
            time.sleep(0.02)
        if goal.kind == 'move':
            cls.pose[0], cls.pose[1] = goal.target.x, goal.target.y
            if not goal.keep_heading:
                cls.pose[2] = goal.target.theta
        elif goal.kind == 'turn':
            cls.pose[2] = goal.target.theta
        else:
            cls.pose[2] += goal.delta_yaw
        # As the real driver, report arrival only once TF shows it.
        cls._tick()
        time.sleep(0.1)
        handle.succeed()
        return MoveSegment.Result(success=True)

    @classmethod
    def _tick(cls):
        t = TransformStamped()
        t.header.stamp = cls.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = cls.pose[0], cls.pose[1]
        t.transform.rotation.z = math.sin(cls.pose[2] / 2.0)
        t.transform.rotation.w = math.cos(cls.pose[2] / 2.0)
        cls.tf.sendTransform(t)
        if cls.heartbeat:
            cls.beat_pub.publish(Header())
        if cls.drive is not None:
            cls.drive_pub.publish(cls.drive)

    def wait_for(self, predicate, timeout=TIMEOUT):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def command(self, action, *args):
        self.assertTrue(self.client.wait_for_service(timeout_sec=TIMEOUT))
        future = self.client.call_async(SupervisorCommand.Request(
            action=action, args=list(args), request_id='test'))
        self.assertTrue(self.wait_for(future.done))
        return future.result()

    def in_state(self, name, timeout=TIMEOUT):
        return self.wait_for(lambda: self.state is not None and self.state.state == name,
                             timeout)

    def test_1_idle_at_home(self):
        self.assertTrue(self.wait_for(lambda: self.state is not None
                                      and self.state.location == 'home'))
        self.assertEqual(self.state.state, 'idle')
        self.assertEqual(list(self.state.allowed_actions),
                         ['start_mode', 'run_feature', 'hold_check'])
        self.assertEqual(self.state.hold_check, 'on')
        self.assertEqual(len(self.state.features), 8)

    def test_2_refused_command_says_why(self):
        reply = self.command('start_mode', '2')
        self.assertFalse(reply.accepted)
        self.assertEqual(reply.reason, 'Already at home')
        self.assertEqual(reply.request_id, 'test')

    def test_3_mode_1_runs_moves_and_stop_cancels(self):
        self.assertTrue(self.command('start_mode', '1').accepted)
        self.assertTrue(self.wait_for(lambda: 'Spin flourish' in self.goals))
        self.assertEqual(self.goals[:3], ['Sidestep out of home', 'Glide across the booth',
                                          'Sidestep to centre stage'])
        self.assertTrue(self.wait_for(lambda: self.state.feature == 'dock'))
        self.assertTrue(self.command('stop').accepted)
        self.assertTrue(self.in_state('stopped'))
        self.assertTrue(self.wait_for(lambda: self.cancels >= 1))
        self.assertEqual(self.state.stop_kind, 'staff')
        self.assertTrue(self.state.can_resume)

    def test_4_manual_drive_expires_and_needs_the_heartbeat(self):
        self.assertTrue(self.command('start_mode', '3').accepted)
        self.assertTrue(self.in_state('manual'))
        type(self).cmds = []
        type(self).drive = Twist()
        type(self).drive.linear.x = 0.1
        self.assertTrue(self.wait_for(lambda: any(v > 0.05 for _, v in self.cmds)))
        type(self).drive = None
        stopped_at = time.monotonic()
        time.sleep(0.6)
        moving = [t for t, v in self.cmds if v > 0.05 and t > stopped_at + 0.3]
        self.assertEqual(moving, [], 'drive commands outlived the panel by more than 0.3 s')
        self.assertEqual(self.cmds[-1][1], 0.0)
        # Heartbeat lost while the D-pad is still held: the drive stops.
        type(self).drive = Twist()
        type(self).drive.linear.x = 0.1
        self.assertTrue(self.wait_for(lambda: self.cmds[-1][1] > 0.05))
        type(self).heartbeat = False
        lost_at = time.monotonic()
        time.sleep(1.6)
        late = [t for t, v in self.cmds if v > 0.05 and t > lost_at + 1.3]
        self.assertEqual(late, [], 'drive went on after the heartbeat stopped')
        type(self).drive = None
        type(self).heartbeat = True
        self.assertTrue(self.command('exit_manual').accepted)
        self.assertTrue(self.in_state('idle'))

    def test_4b_external_controller_needs_manual_but_no_heartbeat(self):
        external = TwistStamped()
        external.twist.linear.x = 0.1
        # Outside Manual it is ignored.
        type(self).cmds = []
        for _ in range(5):
            self.external_pub.publish(external)
            time.sleep(0.05)
        self.assertFalse(any(v > 0.05 for _, v in self.cmds), 'external drive outside Manual')
        self.assertTrue(self.command('start_mode', '3').accepted)
        self.assertTrue(self.in_state('manual'))
        self.assertFalse(self.state.manual_checks)
        # The safety gate's bypass is refreshed while the checks are off.
        type(self).bypass = []
        self.assertTrue(self.wait_for(lambda: sum(1 for _, b in self.bypass if b) >= 3))
        # No panel at all: the external controller still drives.
        type(self).heartbeat = False
        time.sleep(1.2)
        type(self).cmds = []
        for _ in range(5):
            self.external_pub.publish(external)
            time.sleep(0.05)
        self.assertTrue(self.wait_for(lambda: any(v > 0.05 for _, v in self.cmds)))
        type(self).heartbeat = True
        # Checks on: the bypass ends at once.
        self.assertTrue(self.command('manual_checks', 'on').accepted)
        self.assertTrue(self.wait_for(lambda: self.bypass and self.bypass[-1][1] is False))
        self.assertTrue(self.command('exit_manual').accepted)
        self.assertTrue(self.in_state('idle'))
        type(self).bypass = []
        time.sleep(0.6)
        self.assertFalse(any(b for _, b in self.bypass), 'bypass refreshed outside Manual')

    def test_5_estop_then_release_lands_on_stopped(self):
        self.estop.publish(Bool(data=True))
        self.assertTrue(self.in_state('estop'))
        self.assertEqual(list(self.state.allowed_actions), [])
        self.estop.publish(Bool(data=False))
        self.assertTrue(self.in_state('stopped'))
        self.assertEqual(self.state.stop_kind, 'estop')
        self.assertTrue(self.command('go_home').accepted)
