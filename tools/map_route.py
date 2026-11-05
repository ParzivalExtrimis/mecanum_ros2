#!/usr/bin/env python3
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
Drive a fixed mapping route through the restroom world, for repeatable runs (Phase 5).

    ros2 launch mecanum_bringup mapping.launch.py gui:=false rviz:=false
    tools/map_route.py
    tools/save_map.py restroom

The route follows the README's advice for a lidar that sees only the front 180-200 degrees:
forward passes, turns in place at the ends, never reversing into the unseen. It covers the
corridor up to the threshold, the restroom through the doorway (the sink counter, the bin, the
stall's opening and the toilet), the lobby around the column, and ends at home, closing a loop.

Closed loop on the EKF's /odom, which is measured odometry; no ground truth. Commands go to
/cmd_vel as TwistStamped, so the safety gate sees them like teleop's. The waypoints are world
coordinates, moved into odom with --start (the spawn pose).
"""

import argparse
import math
import sys
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

EAST, NORTH, WEST, SOUTH = 0.0, math.pi / 2, math.pi, -math.pi / 2
# Waypoints (x, y, yaw) in restroom.sdf's world frame. A waypoint changes either the position
# or the heading, so turns happen in place. Clearances are for the footprint
# x -0.405..0.421, y +-0.258 (mecanum_description).
ROUTE = [
    (4.5, -1.0, EAST),     # along the corridor; the threshold at x 5.35 stays 0.43 m ahead
    (3.45, -1.0, EAST),    # back to the doorway (straight back along a pass just seen)
    (3.45, -1.0, NORTH),
    (3.45, 1.4, NORTH),    # through the doorway, x 3.0..3.9: 0.19 m each side
    (3.45, 1.4, EAST),
    (4.9, 1.4, EAST),      # past the bin
    (4.9, 1.4, NORTH),
    (4.9, 2.9, NORTH),     # beside the stall's opening; the post at x 5.4 is 0.24 m away
    (4.9, 2.9, EAST),      # look into the stall at the toilet
    (4.9, 2.9, WEST),
    (3.3, 2.9, WEST),      # toward the sink counter, 0.23 m short
    (3.3, 2.9, SOUTH),
    (3.45, 1.4, SOUTH),
    (3.45, -1.0, SOUTH),   # out through the doorway
    (3.45, -1.0, WEST),
    (-0.35, -1.0, WEST),   # the corridor's west end
    (-0.35, -1.0, NORTH),
    (-0.35, 2.0, NORTH),   # up the lobby's west side; the glass at y 2.79 stays 0.37 m ahead
    (-0.35, 2.0, EAST),
    (-0.35, 2.1, EAST),
    (1.3, 2.1, EAST),      # north of the column (0.19 m), south of the glass
    (1.3, 2.1, SOUTH),
    (1.3, -0.9, SOUTH),    # down the lobby's east side, 0.13 m short of the bench
    (1.3, -0.9, WEST),
    (0.0, -1.0, WEST),
    (0.0, -1.0, EAST),     # home
]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def command(error_x, error_y, error_yaw, yaw, gain, max_linear, max_angular):
    """Return the body-frame (vx, vy, wz) toward a pose error given in the odom frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    ex, ey = c * error_x + s * error_y, -s * error_x + c * error_y
    vx, vy = gain * ex, gain * ey
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        # Scale both axes together: the drive controller clamps each axis on its own, which
        # would bend the path.
        vx, vy = vx * max_linear / speed, vy * max_linear / speed
    wz = max(-max_angular, min(max_angular, 2.0 * gain * error_yaw))
    return vx, vy, wz


class MapRoute(Node):

    def __init__(self, waypoints, args):
        super().__init__('map_route')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.waypoints = waypoints
        self.index = 0
        self.pose = None
        self.started = None
        self.waypoint_started = None
        self.travelled = 0.0
        self.turned = 0.0
        self.last = None
        self.done = False
        self.failed = None
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(0.05, self._on_timer)

    def _on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        if self.last is not None:
            self.travelled += math.hypot(p.position.x - self.last[0], p.position.y - self.last[1])
            self.turned += abs(wrap(yaw - self.last[2]))
        self.pose = (p.position.x, p.position.y, yaw)
        self.last = self.pose

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def _on_timer(self):
        if self.done or self.pose is None or self.now() == 0.0:
            return
        now = self.now()
        if self.started is None:
            self.started = self.waypoint_started = now
        x, y, yaw = self.pose
        tx, ty, tyaw = self.waypoints[self.index]
        ex, ey, eyaw = tx - x, ty - y, wrap(tyaw - yaw)
        if math.hypot(ex, ey) < self.args.tolerance and abs(eyaw) < self.args.yaw_tolerance:
            self.get_logger().info(
                f'waypoint {self.index + 1}/{len(self.waypoints)} reached '
                f'({tx:.2f}, {ty:.2f}, {math.degrees(tyaw):.0f} deg odom) '
                f'after {now - self.waypoint_started:.1f} s')
            self.index += 1
            self.waypoint_started = now
            if self.index == len(self.waypoints):
                self.publish(0.0, 0.0, 0.0)
                self.done = True
                return
            tx, ty, tyaw = self.waypoints[self.index]
            ex, ey, eyaw = tx - x, ty - y, wrap(tyaw - yaw)
        if now - self.waypoint_started > self.args.waypoint_timeout:
            self.publish(0.0, 0.0, 0.0)
            self.failed = (f'waypoint {self.index + 1} ({tx:.2f}, {ty:.2f}) not reached in '
                           f'{self.args.waypoint_timeout:.0f} s; at ({x:.2f}, {y:.2f}, '
                           f'{math.degrees(yaw):.0f} deg)')
            self.done = True
            return
        self.publish(*command(ex, ey, eyaw, yaw, self.args.gain, self.args.speed,
                              self.args.turn_speed))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--start', type=float, nargs=3, default=(0.0, -1.0, 0.0),
                        metavar=('X', 'Y', 'YAW'),
                        help='World pose where odom starts (default: the spawn pose)')
    parser.add_argument('--speed', type=float, default=0.2, help='Top linear speed, m/s')
    parser.add_argument('--turn-speed', type=float, default=0.5, help='Top turn rate, rad/s')
    parser.add_argument('--gain', type=float, default=1.0, help='Position gain, 1/s')
    parser.add_argument('--tolerance', type=float, default=0.03, help='Position tolerance, m')
    parser.add_argument('--yaw-tolerance', type=float, default=0.03,
                        help='Heading tolerance, rad')
    parser.add_argument('--waypoint-timeout', type=float, default=60.0,
                        help='Simulation seconds allowed per waypoint')
    args = parser.parse_args(argv)

    sx, sy, syaw = args.start
    c, s = math.cos(syaw), math.sin(syaw)
    waypoints = [(c * (x - sx) + s * (y - sy), -s * (x - sx) + c * (y - sy), wrap(yaw - syaw))
                 for x, y, yaw in ROUTE]

    rclpy.init()
    node = MapRoute(waypoints, args)
    wall_start = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose is None and time.monotonic() - wall_start > 30.0:
                node.failed = 'no /odom: is the simulation running?'
                break
    except KeyboardInterrupt:
        node.failed = 'interrupted'
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0, 0.0)
    elapsed = node.now() - node.started if node.started else 0.0
    print(f'route: {node.index}/{len(waypoints)} waypoints, {node.travelled:.1f} m and '
          f'{math.degrees(node.turned):.0f} deg turned in {elapsed:.0f} s (simulation)')
    node.destroy_node()
    rclpy.try_shutdown()
    if node.failed:
        sys.exit(f'FAIL: {node.failed}')


if __name__ == '__main__':
    main()
