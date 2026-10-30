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
Motion test: scripted drive patterns, to check wheel mapping and to exercise odometry.

``pattern`` selects the script:

axes      (default) Drive forward, strafe left and rotate, one at a time, to check wheel
          mapping. Described below; only this pattern is judged PASS or FAIL per step.
loop      Laps of a closed rectangle: forward ``loop_length``, rotate 180 degrees, forward
          ``loop_length``, strafe right ``loop_width``, rotate 180 degrees, strafe right
          ``loop_width``. From the restroom world's spawn (0, -1) facing +x, it runs along the
          corridor and up into the lobby. Used to measure LIO drift (phase 4).
corridor  Laps of forward ``loop_length``, then reverse ``loop_length``.

In ``loop`` and ``corridor``, each segment has a target distance or angle. With
``measure_odom_topic`` set, a segment ends when the measured progress reaches its target, so
calibration errors (the simulation rotates at 110% of the command) do not accumulate into the
walls; a segment that takes more than twice its nominal time stops the test with a failure.
Without odometry, each segment runs for its nominal time (target / speed). Every step is
followed by ``pause_duration`` seconds of zero velocity.

In ``axes``, each step publishes a constant velocity on cmd_vel for ``move_duration``
seconds, then zero for ``pause_duration`` seconds. Watch which way the robot moves: forward is
+x, strafe left is +y, rotate is counter-clockwise seen from above. With ``return_to_start``
(default) each step is followed by the opposite motion, so the robot ends roughly where it
started and the test can be repeated; it needs about ``linear_speed * move_duration`` of free
space in front and to the left, plus room to turn on the spot.

If ``measure_odom_topic`` is set (``/sim/ground_truth/odom`` in simulation, ``/odom`` once
LIO runs on hardware), each ``axes`` step's displacement is measured in the robot's frame at
the start of the step and compared with the commanded motion. Yaw is unwrapped, so turns
beyond 180 degrees are measured correctly.
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


def segment_progress(twist, displacement):
    """
    Return how far a segment has progressed toward its target.

    Linear segments: the displacement (in the start frame) along the commanded direction.
    Rotations: the yaw change in the commanded sense.
    """
    vx, vy, wz = twist
    dx, dy, dyaw = displacement
    if vx == 0.0 and vy == 0.0:
        return dyaw if wz > 0.0 else -dyaw if wz < 0.0 else 0.0
    return (dx * vx + dy * vy) / math.hypot(vx, vy)


def build_pattern(pattern, linear_speed, angular_speed, loop_length=4.0, loop_width=2.0,
                  laps=1, return_to_start=True):
    """
    Return the steps of a pattern as (name, (vx, vy, wz), judged, target) tuples.

    ``target`` is None for timed steps (``axes``) and a distance (m) or angle (rad) otherwise.
    """
    v, w = linear_speed, angular_speed
    if pattern == 'axes':
        steps = []
        for name, twist in (('forward', (v, 0.0, 0.0)), ('strafe_left', (0.0, v, 0.0)),
                            ('rotate_ccw', (0.0, 0.0, w))):
            steps.append((name, twist, True, None))
            if return_to_start:
                steps.append((name + '_return', tuple(-c for c in twist), False, None))
        return steps
    if pattern == 'loop':
        lap = [('forward', (v, 0.0, 0.0), loop_length),
               ('rotate_ccw', (0.0, 0.0, w), math.pi),
               ('forward', (v, 0.0, 0.0), loop_length),
               ('strafe_right', (0.0, -v, 0.0), loop_width),
               ('rotate_ccw', (0.0, 0.0, w), math.pi),
               ('strafe_right', (0.0, -v, 0.0), loop_width)]
    elif pattern == 'corridor':
        lap = [('forward', (v, 0.0, 0.0), loop_length),
               ('reverse', (-v, 0.0, 0.0), loop_length)]
    else:
        raise ValueError(f'unknown pattern "{pattern}" (axes, loop or corridor)')
    return [(f'lap{n + 1}_{i + 1}_{name}', twist, False, target)
            for n in range(laps) for i, (name, twist, target) in enumerate(lap)]


def nominal_duration(twist, target):
    """Return the time (s) a segment with ``target`` takes at the commanded speed."""
    vx, vy, wz = twist
    speed = math.hypot(vx, vy) or abs(wz)
    return target / speed


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
        self.declare_parameter('pattern', 'axes')
        self.declare_parameter('loop_length', 4.0)
        self.declare_parameter('loop_width', 2.0)
        self.declare_parameter('laps', 1)

        v = float(self.get_parameter('linear_speed').value)
        w = float(self.get_parameter('angular_speed').value)
        self._move = float(self.get_parameter('move_duration').value)
        self._pause = float(self.get_parameter('pause_duration').value)
        pattern = self.get_parameter('pattern').value
        self._steps = build_pattern(
            pattern, v, w,
            loop_length=float(self.get_parameter('loop_length').value),
            loop_width=float(self.get_parameter('loop_width').value),
            laps=int(self.get_parameter('laps').value),
            return_to_start=bool(self.get_parameter('return_to_start').value))

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
            f'motion test ({pattern}): {len(self._steps)} steps at '
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
            name, (vx, vy, wz), _, target = self._steps[self._step]
            if target is None:
                finished = elapsed >= self._move
            elif self._step_start_pose is None or self._pose is None:
                finished = elapsed >= nominal_duration((vx, vy, wz), target)
            else:
                progress = segment_progress(
                    (vx, vy, wz), displacement_in_start_frame(self._step_start_pose, self._pose))
                finished = progress >= target
                if not finished and elapsed > 2.0 * nominal_duration((vx, vy, wz), target):
                    self.get_logger().error(
                        f'[{name}] reached {progress:.3f} of {target:.3f} in {elapsed:.1f} s, '
                        'twice the nominal time: stopping')
                    self.passed = False
                    self._report()
                    return
            if finished:
                self._publish(0.0, 0.0, 0.0)
                self._phase, self._phase_start = 'pause', now
            else:
                self._publish(vx, vy, wz)
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
        name, twist, _, target = self._steps[self._step]
        until = (f'for {self._move:.1f} s' if target is None
                 else f'for {target:.3f} {"rad" if twist[0] == twist[1] == 0.0 else "m"}')
        self.get_logger().info(
            f'[{name}] vx={twist[0]:+.2f} vy={twist[1]:+.2f} wz={twist[2]:+.2f} {until}')

    def _finish_step(self):
        name, (vx, vy, wz), judged, target = self._steps[self._step]
        if target is not None and self._step_start_pose is not None and self._pose is not None:
            progress = segment_progress(
                (vx, vy, wz), displacement_in_start_frame(self._step_start_pose, self._pose))
            self.get_logger().info(f'[{name}] done: {progress:.3f} of {target:.3f}')
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
