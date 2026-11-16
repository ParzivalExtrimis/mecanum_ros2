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
The booth demo's base executor: the /demo/move_segment action server.

    ros2 run mecanum_demo segment_driver --ros-args -p config:=<booth_demo.yaml>

One prewritten move at a time (mecanum_interfaces/action/MoveSegment): a straight holonomic
move, a turn or a spin, closed-loop on TF map -> base_footprint (AMCL in the booth), with the
control law of segment_control.py. Commands go out as TwistStamped on /cmd_vel at 30 Hz, only
while a goal runs; the safety gate passes them on to the drive.

Refusals and aborts (MoveSegment result ``error``):
    blocked      before it starts, the move's footprint sweep comes closer than
                 ``min_clearance`` to an occupied cell of /map
    no_pose      no map -> base_footprint transform for 1 s
    no_progress  the robot made no progress for ``no_progress_s``, for example while the
                 safety gate blocks it
    canceled     the client canceled it (STOP, hold, a new goal)

Clock: the node's clock, sim time in sim, so that the command stamps match the drive
controller's clock (it drops commands older than 0.25 s).
"""

import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from mecanum_demo.geometry import footprint_distance, sweep_poses, to_body, wrap
from mecanum_demo.ros_util import load_config, occupied_points, pose2d_of
from mecanum_demo.segment_control import SegmentController
from mecanum_interfaces.action import MoveSegment
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def default_config():
    return os.path.join(get_package_share_directory('mecanum_demo'), 'config',
                        'booth_demo.yaml')


class SegmentDriver(Node):

    def __init__(self):
        super().__init__('segment_driver')
        p = self.declare_parameter
        self.cfg = load_config(p('config', default_config()).value)
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.rate = p('rate', 30.0).value
        self.min_clearance = p('min_clearance', 0.01).value
        self.no_progress_s = p('no_progress_s', 3.0).value
        speeds = dict(self.cfg['speeds'])
        for key in speeds:
            speeds[key] = p(f'speeds.{key}', float(speeds[key])).value
        self.speeds = speeds
        self.footprint = self.cfg['footprint']

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.obstacles = np.zeros((0, 2))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value, self._on_map,
                                 latched)
        self.cmd_pub = self.create_publisher(TwistStamped, p('cmd_topic', '/cmd_vel').value, 10)
        self.lock = threading.Lock()
        self.active = None          # the goal handle being executed
        ActionServer(self, MoveSegment, '/demo/move_segment', self._execute,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     handle_accepted_callback=self._accepted,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Ready: /demo/move_segment -> /cmd_vel')

    def _on_map(self, msg):
        self.obstacles = occupied_points(msg)
        self.get_logger().info(f'Map: {len(self.obstacles)} occupied cells')

    def _accepted(self, goal_handle):
        # A new goal replaces the running one.
        with self.lock:
            if self.active is not None and self.active.is_active:
                self.active.abort()
            self.active = goal_handle
        goal_handle.execute()

    def pose(self):
        try:
            return pose2d_of(self.tf.lookup_transform(self.map_frame, self.base_frame, Time()))
        except TransformException:
            return None

    def wait_pose(self, timeout=1.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            pose = self.pose()
            if pose is not None:
                return pose
            time.sleep(0.05)
        return None

    def clearance(self, start, end):
        """Least footprint distance to the map's occupied cells along the move."""
        if len(self.obstacles) == 0:
            return math.inf
        reach = math.hypot(self.footprint['front'] + 0.5, self.footprint['half_width'] + 0.5)
        lo = np.minimum(start[:2], end[:2]) - reach
        hi = np.maximum(start[:2], end[:2]) + reach
        near = self.obstacles[np.all((self.obstacles >= lo) & (self.obstacles <= hi), axis=1)]
        if len(near) == 0:
            return math.inf
        # Cells the robot already covers at the start are not obstacles of this move (it may
        # sit a little into a wall's cells after a bump or with localization error).
        start_d = footprint_distance(to_body(near, start), self.footprint)
        near = near[start_d > 0.0]
        if len(near) == 0:
            return math.inf
        least = math.inf
        for pose in sweep_poses(start, end, step=0.02, angle_step=0.05)[1:]:
            least = min(least, float(
                footprint_distance(to_body(near, pose), self.footprint).min()))
        return least

    def publish(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    def _result(self, goal_handle, success, error='', message=''):
        result = MoveSegment.Result()
        result.success, result.error, result.message = success, error, message
        if goal_handle.is_active:
            if success:
                goal_handle.succeed()
            elif error == 'canceled' and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        return result

    def _execute(self, goal_handle):
        goal = goal_handle.request
        label = goal.label or goal.kind
        start = self.wait_pose()
        if start is None:
            return self._result(goal_handle, False, 'no_pose', 'No map -> base_footprint')
        target = (goal.target.x, goal.target.y, goal.target.theta)
        if goal.kind == 'move':
            yaw = start[2] if goal.keep_heading else start[2] + wrap(target[2] - start[2])
            end = (target[0], target[1], yaw)
        elif goal.kind == 'turn':
            end = (start[0], start[1], start[2] + wrap(target[2] - start[2]))
        elif goal.kind == 'spin':
            end = (start[0], start[1], start[2] + goal.delta_yaw)
        else:
            return self._result(goal_handle, False, 'invalid', f'Unknown kind "{goal.kind}"')
        least = self.clearance(start, end)
        if least < self.min_clearance:
            self.get_logger().warn(f'"{label}" refused: {least * 100:.1f} cm to the map')
            return self._result(goal_handle, False, 'blocked',
                                f'{least * 100:.1f} cm from a wall or fixture')
        ctl = SegmentController(goal.kind, start, target=target,
                                keep_heading=goal.keep_heading, delta_yaw=goal.delta_yaw,
                                slow=goal.slow, speeds=self.speeds)
        self.get_logger().info(f'"{label}": {goal.kind} from ({start[0]:.2f}, {start[1]:.2f}, '
                               f'{start[2]:.2f}), clearance {least * 100:.1f} cm')
        period = 1.0 / self.rate
        clock = self.get_clock()
        last_t = clock.now()
        best_left = math.inf
        best_at = time.monotonic()
        lost_since = None
        feedback = MoveSegment.Feedback()
        while rclpy.ok():
            time.sleep(period)
            if not goal_handle.is_active:
                self.publish()
                return MoveSegment.Result(success=False, error='canceled',
                                          message='Replaced by a new goal')
            if goal_handle.is_cancel_requested:
                self.publish()
                return self._result(goal_handle, False, 'canceled', 'Canceled')
            pose = self.pose()
            if pose is None:
                lost_since = lost_since or time.monotonic()
                self.publish()
                if time.monotonic() - lost_since > 1.0:
                    return self._result(goal_handle, False, 'no_pose',
                                        'Lost map -> base_footprint')
                continue
            lost_since = None
            now = clock.now()
            dt = (now - last_t).nanoseconds * 1e-9
            last_t = now
            if dt <= 0.0:       # sim clock paused or not advanced: hold the last command
                continue
            (vx, vy, wz), arrived = ctl.step(pose, dt)
            self.publish(vx, vy, wz)
            if arrived:
                self.publish()
                return self._result(goal_handle, True)
            fraction, metres, radians = ctl.progress(pose)
            left = metres + 0.3 * radians
            if left < best_left - 0.005:
                best_left = left
                best_at = time.monotonic()
            elif time.monotonic() - best_at > self.no_progress_s:
                self.publish()
                return self._result(goal_handle, False, 'no_progress',
                                    f'No progress for {self.no_progress_s:.0f} s')
            feedback.progress = float(fraction)
            feedback.remaining_s = float(ctl.remaining_s(pose))
            goal_handle.publish_feedback(feedback)
        self.publish()
        return self._result(goal_handle, False, 'canceled', 'Shutting down')


def main(args=None):
    rclpy.init(args=args)
    node = SegmentDriver()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
