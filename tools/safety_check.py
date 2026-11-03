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

r"""
Check the safety gate in the sensor test world (docs/plan.md Phase 4).

Start the simulation first:
    ros2 launch mecanum_bringup robot.launch.py gui:=false lio:=false \
      world:=$(ros2 pkg prefix mecanum_simulation)/share/mecanum_simulation/worlds/sensor_test.sdf
    tools/safety_check.py

Checks, each with a PASS or FAIL. Every command goes to /cmd_vel, as teleop's do, and the
robot is judged on Gazebo ground truth (shortcut S1):

1. Chain: the supervisor is the only publisher of /cmd_vel_safe, and the drive controller does
   not listen to /cmd_vel.
2. Cliffs: the robot drives at the floor hole forward, in reverse, and sideways both ways, at
   each --speeds. PASS if no wheel contact point crosses the edge. The stopping distance is
   counted from the first cliff block. Then the motion away must be allowed.
3. Bumpers: driven into the bumper wall, front and rear. The robot must stop, the command
   toward the wall must stay blocked, and backing off must be allowed and clear the block.
4. E-stop: true stops the robot; false keeps it stopped; reset while true is refused; reset
   after release lets it move.
5. Sensor loss (--only loss, last, as it ends the simulation's sensors): the cliff, ultrasonic,
   depth cloud and bumper relays, then the bridge, are stopped one at a time. Each must give its
   block within its timeout.

Stations move the robot with Gazebo's set_pose service, so run this on a test world only.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time

from geometry_msgs.msg import TwistStamped
from mecanum_interfaces.msg import SafetyState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

WORLD = 'sensor_test'
# Wheel contact points in base_footprint: the wheel centres (mecanum_params.xacro).
WHEELS = {'front_left': (0.236, 0.215), 'front_right': (0.236, -0.215),
          'rear_left': (-0.236, 0.215), 'rear_right': (-0.236, -0.215)}
HOLE = ((1.0, 2.2), (5.4, 6.6))  # sensor_test.sdf, x and y
GAP = 0.3  # m from the leading wheel contact to the edge at the start
# Approach: station (x, y, yaw) with the leading wheels GAP from the edge, and the unit command.
APPROACHES = {
    'forward': ((HOLE[0][0] - 0.236 - GAP, 6.0, 0.0), (1.0, 0.0)),
    'reverse': ((HOLE[0][1] + 0.236 + GAP, 6.0, 0.0), (-1.0, 0.0)),
    'left': ((1.6, HOLE[1][0] - 0.215 - GAP, 0.0), (0.0, 1.0)),
    'right': ((1.6, HOLE[1][1] + 0.215 + GAP, 0.0), (0.0, -1.0)),
}
BUMPER_STATIONS = {'front': ((0.5, 9.0, 0.0), 1.0), 'rear': ((1.5, 9.0, 0.0), -1.0)}
BUMPER_SPEED = 0.1
OPEN_STATION = (-5.0, -1.0, 0.0)
STILL = 0.005  # m: no motion
AWAY = 0.03  # m: moved away
# Process to stop (a unique command line part), and the reasons that must appear.
LOSSES = [
    ('__node:=sim_cliff_tof', [f'lost /cliff/{p}' for p in WHEELS]),
    ('__node:=sim_ultrasonics', [f'lost /ultrasonic/{p}' for p in WHEELS]),
    ('lib/mecanum_simulation/sim_bumper', ['lost /bumper/front', 'lost /bumper/rear']),
    ('lib/mecanum_simulation/sim_depth_cloud', ['lost /d435i/depth/color/points']),
    ('lib/ros_gz_bridge/parameter_bridge', ['lost /scan', 'lost /l2/points_filtered']),
]
LOSS_LIMIT = 1.5  # s: the longest timeout (1.0 s) plus the state period and margin


class SafetyCheck(Node):

    def __init__(self):
        super().__init__('safety_check')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.ground_truth = None
        self.state = None
        self.state_log = []  # (monotonic, reasons, ground-truth x, y)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_gt,
                                 qos_profile_sensor_data)
        self.create_subscription(SafetyState, '/safety/state', self._on_state, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.reset_client = self.create_client(Trigger, '/safety_supervisor/reset')

    def _on_gt(self, msg):
        self.ground_truth = msg

    def _on_state(self, msg):
        self.state = msg
        p = self.ground_truth.pose.pose.position if self.ground_truth else None
        self.state_log.append((time.monotonic(), tuple(msg.reasons),
                               p.x if p else math.nan, p.y if p else math.nan))

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, seconds):
        deadline = self.sim_now() + seconds
        limit = time.monotonic() + 10.0 * seconds + 10.0
        while self.sim_now() < deadline and time.monotonic() < limit:
            time.sleep(0.01)

    def pose(self):
        """Return the ground-truth (x, y, yaw, z) of base_footprint."""
        p = self.ground_truth.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        return p.position.x, p.position.y, yaw, p.position.z

    def wheels_in_world(self):
        x, y, yaw, _ = self.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        return {name: (x + c * wx - s * wy, y + s * wx + c * wy)
                for name, (wx, wy) in WHEELS.items()}

    def move_to(self, station, settle=1.5):
        x, y, yaw = station
        request = (f'name: "mecanum", position: {{x: {x}, y: {y}, z: 0.01}}, '
                   f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', request],
                       check=True, capture_output=True)
        self.wait_sim(settle)

    def drive(self, vx, vy, seconds, until=None):
        """Command a twist for a span of sim time (or until ``until()``), then stop."""
        deadline = self.sim_now() + seconds
        while self.sim_now() < deadline and not (until and until()):
            self.send(vx, vy)
            time.sleep(0.02)
        self.send(0.0, 0.0)

    def send(self, vx, vy, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    def reasons(self):
        return set(self.state.reasons) if self.state else set()

    def wait_reasons(self, predicate, limit):
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            if predicate(self.reasons()):
                return True
            time.sleep(0.02)
        return False

    def reset(self):
        future = self.reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None


class Report:

    def __init__(self):
        self.failures = 0

    def check(self, passed, text):
        self.failures += 0 if passed else 1
        print(f'  {"PASS" if passed else "FAIL"}  {text}', flush=True)


def depth_into_hole(point):
    """Return how far a point is past the nearest hole edge (negative: still on the deck)."""
    (x0, x1), (y0, y1) = HOLE
    x, y = point
    inside = min(x - x0, x1 - x, y - y0, y1 - y)
    if inside > 0:
        return inside
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return -math.hypot(dx, dy)


def check_chain(node, report):
    print('\n1. Chain')
    pubs = [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel_safe')]
    report.check(pubs == ['safety_supervisor'], f'/cmd_vel_safe publishers: {pubs}')
    subs = [i.node_name for i in node.get_subscriptions_info_by_topic('/cmd_vel_safe')]
    report.check(subs == ['mecanum_drive_controller'], f'/cmd_vel_safe subscribers: {subs}')
    subs = sorted(i.node_name for i in node.get_subscriptions_info_by_topic('/cmd_vel'))
    report.check('mecanum_drive_controller' not in subs, f'/cmd_vel subscribers: {subs}')


def check_cliffs(node, report, speeds):
    print('\n2. Cliffs, at the floor hole')
    for speed in speeds:
        for name, (station, (ux, uy)) in APPROACHES.items():
            node.move_to(station)
            if node.reasons():
                report.check(False, f'{name} {speed} m/s: blocked before the run: '
                             f'{sorted(node.reasons())}')
                continue
            node.state_log.clear()
            run_time = (GAP + 0.3) / speed + 1.0
            node.drive(ux * speed, uy * speed, run_time)
            node.wait_sim(0.5)
            blocks = [(x, y) for _, r, x, y in node.state_log
                      if any(s.startswith('cliff') for s in r)]
            x, y, _, z = node.pose()
            worst = max(depth_into_hole(p) for p in node.wheels_in_world().values())
            if blocks:
                stop = math.hypot(x - blocks[0][0], y - blocks[0][1])
                detail = f'stopped {stop * 100:.1f} cm after the first cliff block'
            else:
                detail = 'no cliff block'
            fell = z < -0.03
            report.check(bool(blocks) and worst <= 0.0 and not fell,
                         f'{name:7s} {speed:.1f} m/s: {detail}; leading wheel contact '
                         f'{-worst * 100:+.1f} cm from the edge'
                         f'{" (wheel over the hole)" if worst > 0 else ""}'
                         f'{", FELL" if fell else ""}')
            if fell or not blocks:
                continue
            # Holding the command toward the hole must not move the robot.
            before = node.pose()
            node.drive(ux * speed, uy * speed, 1.0)
            moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
            report.check(moved < STILL, f'{name:7s} {speed:.1f} m/s: held toward the hole, '
                         f'moved {moved * 1000:.1f} mm')
            node.drive(-ux * 0.1, -uy * 0.1, 1.0)
            moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
            report.check(moved > AWAY, f'{name:7s} {speed:.1f} m/s: backing away allowed, '
                         f'moved {moved * 100:.1f} cm')


def check_bumpers(node, report):
    print('\n3. Bumpers, driven into the wall')
    for plank, (station, direction) in BUMPER_STATIONS.items():
        node.move_to(station)
        node.state_log.clear()
        node.drive(direction * BUMPER_SPEED, 0.0, 2.0)
        pressed = any(f'bumper {plank}' in r for _, r, _, _ in node.state_log)
        report.check(pressed, f'{plank}: blocked by the plank')
        node.state_log.clear()
        before = node.pose()
        node.drive(direction * BUMPER_SPEED, 0.0, 1.0)
        moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
        presses = sum(1 for a, b in zip(node.state_log, node.state_log[1:])
                      if f'bumper {plank}' in b[1] and f'bumper {plank}' not in a[1])
        report.check(moved < STILL, f'{plank}: held toward the wall, moved {moved * 1000:.1f} '
                     f'mm, {presses} new press(es)')
        node.drive(-direction * BUMPER_SPEED, 0.0, 1.0)
        moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
        node.wait_sim(0.3)
        report.check(moved > AWAY and f'bumper {plank}' not in node.reasons(),
                     f'{plank}: backed off {moved * 100:.1f} cm, block cleared')


def check_estop(node, report):
    print('\n4. E-stop')
    node.move_to(OPEN_STATION)
    node.estop_pub.publish(Bool(data=True))
    report.check(node.wait_reasons(lambda r: 'estop' in r, 1.0), 'latched on true')
    before = node.pose()
    node.drive(0.2, 0.0, 1.0)
    moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
    report.check(moved < STILL, f'no motion while latched: {moved * 1000:.1f} mm')
    result = node.reset()
    report.check(result is not None and not result.success,
                 f'reset refused while true: {result.message if result else "no reply"}')
    node.estop_pub.publish(Bool(data=False))
    node.wait_sim(0.5)
    node.drive(0.2, 0.0, 1.0)
    moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
    report.check('estop' in node.reasons() and moved < STILL,
                 f'still latched after release: {moved * 1000:.1f} mm')
    result = node.reset()
    report.check(result is not None and result.success, 'reset accepted after release')
    node.wait_sim(0.3)
    node.drive(0.2, 0.0, 1.0)
    moved = math.hypot(node.pose()[0] - before[0], node.pose()[1] - before[1])
    report.check(moved > AWAY, f'moves after reset: {moved * 100:.1f} cm')


def check_loss(node, report):
    print('\n5. Sensor loss (the relays and the bridge are stopped for good)')
    for pattern, expected in LOSSES:
        pids = subprocess.run(['pgrep', '-f', pattern], capture_output=True,
                              text=True).stdout.split()
        pids = [int(p) for p in pids if int(p) != os.getpid()]
        if not pids:
            report.check(False, f'{pattern}: no process')
            continue
        start = time.monotonic()
        for pid in pids:
            os.kill(pid, signal.SIGINT)
        ok = node.wait_reasons(lambda r: set(expected) <= r, LOSS_LIMIT + 1.0)
        elapsed = time.monotonic() - start
        report.check(ok and elapsed <= LOSS_LIMIT,
                     f'{pattern.split("/")[-1]}: {", ".join(expected)} after {elapsed:.2f} s')
    report.check(node.state is not None and node.state.stopped, 'stopped after the losses')


def spin(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--speeds', nargs='+', type=float, default=[0.3, 0.2, 0.1])
    parser.add_argument('--only', nargs='+', default=['chain', 'cliffs', 'bumpers', 'estop'],
                        choices=['chain', 'cliffs', 'bumpers', 'estop', 'loss'])
    args = parser.parse_args(argv)

    rclpy.init()
    node = SafetyCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=spin, args=(executor,), daemon=True)
    spinner.start()
    report = Report()
    try:
        limit = time.monotonic() + 30.0
        while time.monotonic() < limit and (node.ground_truth is None or node.state is None):
            time.sleep(0.05)
        if node.ground_truth is None or node.state is None:
            sys.exit('No ground truth or /safety/state: is the simulation running?')
        node.wait_sim(2.0)
        if 'chain' in args.only:
            check_chain(node, report)
        if 'cliffs' in args.only:
            check_cliffs(node, report, args.speeds)
        if 'bumpers' in args.only:
            check_bumpers(node, report)
        if 'estop' in args.only:
            check_estop(node, report)
        if 'loss' in args.only:
            check_loss(node, report)
    finally:
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n{report.failures} failure(s)')
    sys.exit(1 if report.failures else 0)


if __name__ == '__main__':
    main()
