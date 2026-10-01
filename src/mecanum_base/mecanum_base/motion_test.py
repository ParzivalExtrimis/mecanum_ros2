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
Motion test: drive forward, strafe left and rotate, one at a time, to check wheel mapping.

Each step publishes a constant velocity on cmd_vel for ``move_duration`` seconds, then zero
for ``pause_duration`` seconds. Watch which way the robot moves: forward is +x, strafe left
is +y, rotate is counter-clockwise seen from above. With ``return_to_start`` (default) each
step is followed by the opposite motion, so the robot ends roughly where it started and the
test can be repeated; it needs about ``linear_speed * move_duration`` of free space in front
and to the left, plus room to turn on the spot.

If ``measure_odom_topic`` is set (``/sim/ground_truth/odom`` in simulation, ``/odom`` once
LIO runs on hardware), each step's displacement is measured in the robot's frame at the start
of the step and compared with the commanded motion. Yaw is unwrapped, so turns beyond 180
degrees are measured correctly.
"""

import math

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_from_quaternion(q):
    """Return the yaw (rad) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_yaw(previous_unwrapped, previous_raw, raw):
    """Return the continuous yaw after a new raw (wrapped) sample, given the previous one."""
    return previous_unwrapped + wrap_angle(raw - previous_raw)


def displacement_in_start_frame(start, end):
    """
    Return (dx, dy, dyaw) of pose ``end`` relative to pose ``start``, both (x, y, yaw).

    dx and dy are expressed in the robot frame at ``start``. Yaws must be unwrapped
    (continuous); dyaw is their plain difference, so it can exceed 180 degrees.
    """
    x0, y0, yaw0 = start
    x1, y1, yaw1 = end
    wx, wy = x1 - x0, y1 - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    return (c * wx + s * wy, -s * wx + c * wy, yaw1 - yaw0)


def judge_step(name, expected, measured, drift_tolerance, rotate_translation_tolerance):
    """
    Judge one measured step. Returns (passed, text).

    Linear steps pass when the robot moved in the commanded direction and the cross-axis
    drift is at most ``drift_tolerance`` of the travel. The rotation step passes when it turned
    counter-clockwise and translated at most ``rotate_translation_tolerance`` metres. The
    travel and rotation ratios are reported but not judged: they depend on calibration.
    """
    dx, dy, dyaw = measured
    if name == 'rotate_ccw':
        translation = math.hypot(dx, dy)
        ok = dyaw > 0.0 and translation <= rotate_translation_tolerance
        ratio = dyaw / expected[2] if expected[2] else float('nan')
        return ok, (f'{"PASS" if ok else "FAIL"} (rotation {"CCW" if dyaw > 0 else "CW"}, '
                    f'{ratio:.0%} of commanded, translation {translation:.3f} m, '
                    f'limit {rotate_translation_tolerance:.3f} m)')
    if name == 'forward':
        travel, cross, commanded = dx, dy, expected[0]
    else:
        travel, cross, commanded = dy, dx, expected[1]
    drift = abs(cross) / max(abs(travel), 1e-6)
    ok = travel * commanded > 0.0 and drift <= drift_tolerance
    ratio = travel / commanded if commanded else float('nan')
    return ok, (f'{"PASS" if ok else "FAIL"} (travel {ratio:.0%} of commanded, '
                f'cross-axis drift {drift:.1%}, limit {drift_tolerance:.0%})')


class MotionTest(Node):

    def __init__(self):
        super().__init__('motion_test')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_vel_stamped', False)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('move_duration', 3.0)
        self.declare_parameter('pause_duration', 1.5)
        self.declare_parameter('start_delay', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('measure_odom_topic', '')
        self.declare_parameter('drift_tolerance', 0.05)
        self.declare_parameter('rotate_translation_tolerance', 0.05)

        v = float(self.get_parameter('linear_speed').value)
        w = float(self.get_parameter('angular_speed').value)
        self._move = float(self.get_parameter('move_duration').value)
        self._pause = float(self.get_parameter('pause_duration').value)
        # (name, twist, judged)
        tests = [('forward', (v, 0.0, 0.0)), ('strafe_left', (0.0, v, 0.0)),
                 ('rotate_ccw', (0.0, 0.0, w))]
        self._steps = []
        for name, (vx, vy, wz) in tests:
            self._steps.append((name, (vx, vy, wz), True))
            if self.get_parameter('return_to_start').value:
                self._steps.append((name + '_return', (-vx, -vy, -wz), False))

        self._stamped = bool(self.get_parameter('cmd_vel_stamped').value)
        topic = self.get_parameter('cmd_vel_topic').value
        msg_type = TwistStamped if self._stamped else Twist
        self._pub = self.create_publisher(msg_type, topic, 10)

        self._pose = None
        self._raw_yaw = None
        odom_topic = self.get_parameter('measure_odom_topic').value
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._results = []
        self._step = -1
        self._phase = 'delay'
        self._phase_start = None
        self._step_start_pose = None
        self.done = False
        self.passed = True
        rate = float(self.get_parameter('rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f'motion test: {len(self._steps)} steps of {self._move:.1f} s at '
            f'{v:.2f} m/s / {w:.2f} rad/s on {topic}'
            + (f', measuring with {odom_topic}' if odom_topic else ''))

    def _on_odom(self, msg):
        p = msg.pose.pose
        raw = yaw_from_quaternion(p.orientation)
        yaw = raw if self._pose is None else unwrap_yaw(self._pose[2], self._raw_yaw, raw)
        self._raw_yaw = raw
        self._pose = (p.position.x, p.position.y, yaw)

    def _publish(self, vx, vy, wz):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        if self._stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
            msg.twist = twist
            self._pub.publish(msg)
        else:
            self._pub.publish(twist)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_timer(self):
        if self.done:
            return
        now = self._now()
        if self._phase_start is None:
            self._phase_start = now
            return
        elapsed = now - self._phase_start

        if self._phase == 'delay':
            self._publish(0.0, 0.0, 0.0)
            if elapsed >= float(self.get_parameter('start_delay').value):
                self._start_step(now)
        elif self._phase == 'move':
            name, (vx, vy, wz), _ = self._steps[self._step]
            self._publish(vx, vy, wz)
            if elapsed >= self._move:
                self._phase, self._phase_start = 'pause', now
        elif self._phase == 'pause':
            self._publish(0.0, 0.0, 0.0)
            if elapsed >= self._pause:
                self._finish_step()
                if self._step + 1 < len(self._steps):
                    self._start_step(now)
                else:
                    self._report()

    def _start_step(self, now):
        self._step += 1
        self._phase, self._phase_start = 'move', now
        self._step_start_pose = self._pose
        name, twist, _ = self._steps[self._step]
        self.get_logger().info(
            f'[{name}] vx={twist[0]:+.2f} vy={twist[1]:+.2f} wz={twist[2]:+.2f} '
            f'for {self._move:.1f} s')

    def _finish_step(self):
        name, (vx, vy, wz), judged = self._steps[self._step]
        if not judged:
            return
        expected = (vx * self._move, vy * self._move, wz * self._move)
        if self._step_start_pose is None or self._pose is None:
            self._results.append((name, expected, None))
            return
        measured = displacement_in_start_frame(self._step_start_pose, self._pose)
        self._results.append((name, expected, measured))

    def _report(self):
        self.done = True
        self._timer.cancel()
        self._publish(0.0, 0.0, 0.0)
        tol = float(self.get_parameter('drift_tolerance').value)
        rot_tol = float(self.get_parameter('rotate_translation_tolerance').value)
        log = self.get_logger()
        log.info('motion test finished')
        for name, expected, measured in self._results:
            exp = (f'expected dx={expected[0]:+.3f} dy={expected[1]:+.3f} '
                   f'dyaw={math.degrees(expected[2]):+.1f} deg')
            if measured is None:
                log.info(f'[{name}] {exp} (no odometry, check the motion by eye)')
                continue
            ok, verdict = judge_step(name, expected, measured, tol, rot_tol)
            self.passed = self.passed and ok
            dx, dy, dyaw = measured
            log.info(f'[{name}] {exp} | measured dx={dx:+.3f} dy={dy:+.3f} '
                     f'dyaw={math.degrees(dyaw):+.1f} deg | {verdict}')


def main(args=None):
    rclpy.init(args=args)
    node = MotionTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish(0.0, 0.0, 0.0)
        passed = node.passed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
