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
Acceptance check of the booth demo in simulation (docs/demo_app.md).

    ros2 launch mecanum_bringup demo_sim.launch.py gui:=false browser:=false   # terminal 1
    tools/demo_check.py                                                        # terminal 2

Drives the supervisor through /supervisor/command as the panel would, and judges the robot on
Gazebo's ground truth (shortcut S1: evaluation only). The booth world's frame is the map
frame, so ground truth and named poses compare directly.

Stages
  1  Mode 1, --loops full loops (default 3) with nobody near: no hold at all, and every
     feature ends on its spot (owner's target, proposed: <= 5 cm and 3 deg, judged on the
     worst loop; the dock pose is reported only). AMCL's error is reported.
  2  Hold: the visitor leans in during the spin; the state must turn to hold within 1.2 s of
     the visitor's move (Gazebo's move, the sensor frame, the 0.5 s hold_confirm_s and the
     10 Hz loop) and the base must stop; it resumes after the visitor steps back.
  3  STOP while moving: the safety gate's output is zero within 0.2 s, the base stops.
     Resume carries on.
  4  E-stop: pressed while moving, the base stops; released, the state is stopped; Go home.
  5  Manual: the panel's drive moves the base; without the heartbeat it stops within 1.3 s.
     An external controller (/manual/cmd_vel) is ignored outside Manual and drives in
     Manual without a heartbeat. At centre stage with the visitor close: checks off, it
     turns the base and the safety gate says bypassed; checks on, it is refused.
  6  Hold bypass: the visitor holds the spin; "False alarm" (hold_skip) resumes with the
     visitor still there and no hold follows for the rest of the loop; the next loop holds
     again. Then hold_check off resumes and stays clear, and hold_check on holds again.
Exit code 0 when every judged item passes.
"""

import argparse
import math
import sys
import threading
import time

from geometry_msgs.msg import Twist, TwistStamped
from mecanum_interfaces.msg import SafetyState, SupervisorState
from mecanum_interfaces.srv import SupervisorCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool
import yaml

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Where each feature of Mode 1 ends: (spot, heading).
ENDS = {'leave_home': ('stage', 'W'), 'spin': ('stage', 'E'), 'dock': ('dock', 'E'),
        'replay': ('dock', 'E'), 'back_out': ('stage', 'E'), 'greet': ('stage', 'W'),
        'go_home': ('home', 'W')}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Checker(Node):

    def __init__(self, cfg):
        super().__init__('demo_check')
        self.cfg = cfg
        self.state = None
        self.gt = None
        self.gt_speed = 0.0
        self.safe = []          # (monotonic, |cmd|) on /cmd_vel_safe
        self.lock = threading.Lock()
        self.events = []        # (monotonic, previous feature, state message)
        self.create_subscription(SupervisorState, '/supervisor/state', self._on_state, LATCHED)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_gt, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_safe', self._on_safe, 10)
        self.client = self.create_client(SupervisorCommand, '/supervisor/command')
        self.visitor = self.create_client(SetBool, '/sim/visitor')
        self.estop = self.create_publisher(Bool, '/estop', LATCHED)
        self.drive = self.create_publisher(Twist, '/panel/drive', 10)
        self.beat = self.create_publisher(Header, '/panel/heartbeat', 10)
        self.external = self.create_publisher(TwistStamped, '/manual/cmd_vel', 10)
        self.safety = None
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 10)
        self.ext_driving = None
        self.driving = None
        self.beating = False
        self.create_timer(0.05, self._panel)

    def _on_state(self, msg):
        with self.lock:
            prev = self.state
            if prev is not None and prev.feature != msg.feature and prev.feature:
                self.events.append((time.monotonic(), prev.feature, msg, self.gt))
            self.state = msg

    def _on_safety(self, msg):
        self.safety = msg

    def _on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
        t = msg.twist.twist
        self.gt_speed = math.hypot(t.linear.x, t.linear.y) + 0.3 * abs(t.angular.z)

    def _on_safe(self, msg):
        t = msg.twist
        self.safe.append((time.monotonic(),
                          abs(t.linear.x) + abs(t.linear.y) + abs(t.angular.z)))

    def _panel(self):
        if self.beating:
            self.beat.publish(Header(frame_id='demo_check'))
        if self.driving is not None:
            self.drive.publish(self.driving)
        if self.ext_driving is not None:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist = self.ext_driving
            self.external.publish(msg)

    # ----- helpers -------------------------------------------------------------------------

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def command(self, action, *args):
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('no /supervisor/command')
        future = self.client.call_async(SupervisorCommand.Request(
            action=action, args=list(args), request_id='demo_check'))
        self.wait(future.done, 5.0)
        reply = future.result()
        print(f'  > {action} {" ".join(args)}: '
              + ('accepted' if reply.accepted else 'REFUSED: ' + reply.reason))
        return reply.accepted

    def in_state(self, *names, timeout=10.0):
        return self.wait(lambda: self.state is not None and self.state.state in names, timeout)

    def set_visitor(self, inside):
        self.visitor.wait_for_service(timeout_sec=5.0)
        future = self.visitor.call_async(SetBool.Request(data=inside))
        self.wait(future.done, 8.0)
        return future.result() is not None and future.result().success

    def pose_error(self, spot, heading, pose):
        x, y = self.cfg['poses'][spot]
        return (math.hypot(pose[0] - x, pose[1] - y),
                abs(wrap(pose[2] - self.cfg['headings'][heading])))

    def settled(self, timeout=5.0):
        return self.wait(lambda: self.gt_speed < 0.01, timeout)


class Report:

    def __init__(self):
        self.failures = 0

    def item(self, ok, text, judged=True):
        tag = ('PASS' if ok else 'FAIL') if judged else 'INFO'
        if judged and not ok:
            self.failures += 1
        print(f'  [{tag}] {text}')


def stage_mode1(c, r, args):
    print(f'Stage 1: Mode 1, {args.loops} loop(s) with nobody near')
    c.events.clear()
    if not c.command('start_mode', '1'):
        r.item(False, 'Mode 1 refused')
        return
    holds = {'n': 0, 'since': None, 'was': False}

    def done():
        # No visitor in this stage, so any hold is a false one; give up on one that lasts.
        now = time.monotonic()
        held = c.state.state == 'hold'
        if held and not holds['was']:
            holds['n'] += 1
            holds['since'] = now
        holds['was'] = held
        if held and now - holds['since'] > 20.0:
            return True
        return c.state.loop_count > args.loops or c.state.state == 'stopped'

    ok = c.wait(done, 400.0 * args.loops)
    if not ok or c.state.state in ('stopped', 'hold'):
        r.item(False, f'loop did not complete: {c.state.state} {c.state.message}')
    r.item(holds['n'] == 0, f'false holds with nobody near: {holds["n"]} '
                            f'in {min(c.state.loop_count - 1, args.loops)} loop(s)')
    worst = {}
    for _, feature, msg, gt in list(c.events):
        if feature not in ENDS or gt is None:
            continue
        spot, heading = ENDS[feature]
        d, a = c.pose_error(spot, heading, gt)
        amcl = math.hypot(msg.pose.x - gt[0], msg.pose.y - gt[1])
        amcl_a = abs(wrap(msg.pose.theta - gt[2]))
        worst.setdefault(feature, []).append((d, a, amcl, amcl_a))
    for feature, ends in worst.items():
        spot = ENDS[feature][0]
        d = max(e[0] for e in ends)
        a = max(e[1] for e in ends)
        amcl = max(e[2] for e in ends)
        amcl_a = max(e[3] for e in ends)
        each = ', '.join(f'{e[0] * 100:.1f}' for e in ends)
        r.item(d <= args.tolerance and math.degrees(a) <= args.angle,
               f'{feature:10s} ends at {spot:5s}: worst {d * 100:4.1f} cm, '
               f'{math.degrees(a):4.1f} deg (each loop: {each} cm; AMCL off by up to '
               f'{amcl * 100:.1f} cm, {math.degrees(amcl_a):.1f} deg)', spot != 'dock')
    c.command('stop')
    c.in_state('stopped')
    c.command('go_home')
    c.in_state('idle', timeout=120.0)


def stage_hold(c, r):
    print('Stage 2: hold for a visitor')
    c.command('start_mode', '1')
    if not c.wait(lambda: c.state.feature == 'spin' and c.state.state == 'running', 120.0):
        r.item(False, 'never reached the spin')
        return
    time.sleep(1.5)
    t0 = time.monotonic()
    moved = c.set_visitor(True)
    r.item(moved, 'visitor moved into the booth front')
    held = c.in_state('hold', timeout=5.0)
    dt = time.monotonic() - t0
    r.item(held and dt <= 1.2, f'hold after {dt:.2f} s (from the visitor request)')
    r.item(c.settled(3.0), 'base stopped during the hold')
    time.sleep(2.0)
    r.item(c.state.state == 'hold', 'still holding while the visitor stays')
    c.set_visitor(False)
    t1 = time.monotonic()
    resumed = c.in_state('running', timeout=10.0)
    r.item(resumed, f'resumed {time.monotonic() - t1:.1f} s after the visitor left '
                    f'(needs {c.cfg["hold_clear_s"]:.0f} s of clear sim time)')


def stage_stop(c, r):
    print('Stage 3: STOP while moving')
    if not c.wait(lambda: c.state.state == 'running' and c.gt_speed > 0.05, 60.0):
        r.item(False, 'never saw the base moving')
        return
    t0 = time.monotonic()
    c.command('stop')
    ok = c.wait(lambda: any(t > t0 and v < 1e-6 for t, v in list(c.safe)), 2.0)
    first_zero = min((t for t, v in list(c.safe) if t > t0 and v < 1e-6), default=math.inf)
    r.item(ok and first_zero - t0 <= 0.2,
           f'zero on /cmd_vel_safe {first_zero - t0:.3f} s after STOP')
    r.item(c.settled(3.0), 'base stopped')
    r.item(c.in_state('stopped', timeout=3.0), 'state stopped')
    r.item(c.command('resume'), 'resume accepted')
    r.item(c.in_state('running', timeout=3.0), 'running again')


def stage_estop(c, r):
    print('Stage 4: e-stop')
    if not c.wait(lambda: c.state.state == 'running' and c.gt_speed > 0.05, 60.0):
        r.item(False, 'never saw the base moving')
        return
    c.estop.publish(Bool(data=True))
    r.item(c.in_state('estop', timeout=2.0), 'state estop')
    r.item(c.settled(3.0), 'base stopped')
    c.estop.publish(Bool(data=False))
    r.item(c.in_state('stopped', timeout=2.0) and c.state.stop_kind == 'estop',
           'released: state stopped, kind estop')
    r.item(c.command('go_home'), 'go home accepted (clears the gate latch)')
    r.item(c.in_state('idle', timeout=120.0), 'back home and idle')


def turned(c, seconds):
    """How far the base turns (rad) while the external controller asks 0.3 rad/s."""
    yaws = [c.gt[2]]
    turn = Twist()
    turn.angular.z = 0.3
    c.ext_driving = turn
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        time.sleep(0.05)
        yaws.append(c.gt[2])
    c.ext_driving = None
    c.settled(3.0)
    return sum(abs(wrap(b - a)) for a, b in zip(yaws, yaws[1:]))


def stage_manual(c, r):
    print('Stage 5: Manual driving and the heartbeat')
    c.beating = True
    if not c.command('start_mode', '3'):
        r.item(False, 'Manual refused')
        return
    start = c.gt
    drive = Twist()
    drive.linear.x = -0.1          # backwards, away from the left wall, into the booth
    c.driving = drive
    time.sleep(2.0)
    moved = math.hypot(c.gt[0] - start[0], c.gt[1] - start[1])
    r.item(moved > 0.05, f'D-pad drive moved the base {moved * 100:.1f} cm in 2 s')
    c.beating = False
    t0 = time.monotonic()
    stopped = c.wait(lambda: c.gt_speed < 0.01, 3.0)
    r.item(stopped and time.monotonic() - t0 <= 1.3,
           f'base stopped {time.monotonic() - t0:.2f} s after the heartbeat stopped')
    c.driving = None
    c.beating = True
    c.command('exit_manual')
    c.in_state('idle')
    c.beating = False

    # External controller, from centre stage (the one place the robot can turn).
    c.command('start_mode', '1')
    if not c.wait(lambda: c.state.feature == 'spin' and c.state.state == 'running', 120.0):
        r.item(False, 'never reached the spin')
        return
    c.command('stop')
    c.in_state('stopped')
    c.settled(3.0)
    r.item(turned(c, 1.5) < 0.02, 'external controller ignored while stopped (not Manual)')
    c.command('start_mode', '3')
    c.in_state('manual')
    time.sleep(1.0)
    r.item(not c.state.manual_checks and c.safety is not None and c.safety.bypassed,
           'Manual starts with the checks off; the safety gate says bypassed')
    a = turned(c, 2.0)
    r.item(a > 0.2, f'external controller turned the base {math.degrees(a):.0f} deg in 2 s '
                    'with no heartbeat')
    r.item(c.set_visitor(True), 'visitor moved into the booth front')
    time.sleep(2.0)
    r.item(c.state.person_close, 'supervisor sees the visitor close')
    a = turned(c, 2.0)
    r.item(a > 0.2, f'checks off, visitor close: turned {math.degrees(a):.0f} deg')
    c.command('manual_checks', 'on')
    time.sleep(1.0)
    r.item(c.state.manual_blocked and c.safety is not None and not c.safety.bypassed,
           'checks on: Manual drive blocked, the safety gate no longer bypassed')
    a = turned(c, 2.0)
    r.item(a < 0.02, f'checks on, visitor close: turned {math.degrees(a):.1f} deg')
    c.set_visitor(False)
    c.command('manual_checks', 'off')
    time.sleep(1.0)
    c.beating = True
    c.command('exit_manual')
    c.in_state('idle')
    c.beating = False
    time.sleep(1.0)
    r.item(c.safety is not None and not c.safety.bypassed, 'out of Manual: not bypassed')
    c.command('start_mode', '2')
    c.in_state('idle', timeout=120.0)


def stage_bypass(c, r):
    print('Stage 6: the hold bypass and the hold setting')
    c.command('start_mode', '1')
    if not c.wait(lambda: c.state.feature == 'spin' and c.state.state == 'running', 120.0):
        r.item(False, 'never reached the spin')
        return
    time.sleep(1.5)
    c.set_visitor(True)
    r.item(c.in_state('hold', timeout=5.0), 'hold for the visitor')
    loop = c.state.loop_count
    t0 = time.monotonic()
    c.command('hold_skip')
    r.item(c.in_state('running', timeout=1.0) and c.state.hold_check == 'off_loop',
           f'False alarm: running again {time.monotonic() - t0:.2f} s later, hold off for '
           'this loop')
    holds = {'n': 0, 'was': False}

    def wrapped():
        held = c.state.state == 'hold'
        if held and not holds['was']:
            holds['n'] += 1
        holds['was'] = held
        return c.state.loop_count > loop or c.state.state == 'stopped'

    r.item(c.wait(wrapped, 300.0), 'the loop ended with the visitor still there')
    r.item(holds['n'] == 0, f'holds for the rest of that loop: {holds["n"]}')
    r.item(c.state.hold_check == 'on', 'hold check back on for the next loop')
    r.item(c.in_state('hold', timeout=120.0), f'next loop holds again ({c.state.feature})')
    c.command('hold_check', 'off')
    r.item(c.in_state('running', timeout=1.0) and c.state.hold_check == 'off',
           'hold_check off: running again')
    r.item(not c.wait(lambda: c.state.state == 'hold', 6.0), 'no hold for 6 s with it off')
    near = c.state.person_close
    c.command('hold_check', 'on')
    if near:
        r.item(c.in_state('hold', timeout=2.0), 'hold_check on: holds again')
    else:
        r.item(True, f'hold_check on (visitor no longer close at {c.state.feature}: not judged)',
               judged=False)
    c.set_visitor(False)
    c.command('stop')
    c.in_state('stopped')
    c.command('go_home')
    c.in_state('idle', timeout=120.0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--config', default=None)
    parser.add_argument('--tolerance', type=float, default=0.05)
    parser.add_argument('--angle', type=float, default=3.0)
    parser.add_argument('--stages', default='123456')
    parser.add_argument('--loops', type=int, default=3)
    args = parser.parse_args(argv)
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        import os
        args.config = os.path.join(get_package_share_directory('mecanum_demo'), 'config',
                                   'booth_demo.yaml')
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    rclpy.init()
    c = Checker(cfg)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(c)
    threading.Thread(target=executor.spin, daemon=True).start()
    r = Report()
    try:
        if not c.wait(lambda: c.state is not None and c.state.pose_valid and c.gt is not None,
                      60.0):
            print('No supervisor state or ground truth: is demo_sim.launch.py running?')
            return 2
        if c.state.state != 'idle':
            c.command('stop')
            c.command('go_home')
            c.in_state('idle', timeout=120.0)
        stages = {'1': lambda: stage_mode1(c, r, args), '2': lambda: stage_hold(c, r),
                  '3': lambda: stage_stop(c, r), '4': lambda: stage_estop(c, r),
                  '5': lambda: stage_manual(c, r), '6': lambda: stage_bypass(c, r)}
        for key in args.stages:
            stages[key]()
    finally:
        c.estop.publish(Bool(data=False))
        print(f'\n{r.failures} failure(s)')
        executor.shutdown()
        c.destroy_node()
        rclpy.try_shutdown()
    return 1 if r.failures else 0


if __name__ == '__main__':
    sys.exit(main())
