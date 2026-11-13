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
Following-error soak on blocks: the T2 check 4 sequence, scripted, to choose the limit.

Stage T2 of docs/hardware_drive_test.md, wheels in the air. Run it beside
drive_test.launch.py (following_error_limit:=0 recommended, so that the run is not cut short).
It drives, at the teleop speeds (0.1 m/s, 0.3 rad/s by default):
  1. still for --still s (drift at rest);
  2. --cycles hard cycles per axis: +1 s, stop 1 s, -1 s, stop 1 s, for forward/back,
     strafe left/right and rotate left/right;
  3. --reversals straight reversals (+1 s then -1 s, no stop between), then a stop;
  4. still for --still s again.
A stop is zero commands, so every start and stop is the ESP32's accel_backstop ramp, as with
a teleop key release.

It records every /drive/state report to <out-dir>/fe_soak.csv with the phase, and prints per
phase the largest following error per wheel, and the settled error after each stop, so that a
build-up (counts lost or gained on reversals, the encoder tap) shows apart from the lag. The
suggested limit follows the test document: 2 to 3 times the largest seen (2.5 here), never
below 0.05 rad; no suggestion when the settled error drifts.

Aborts at once (zero command, exit 2) on a latched ESP32 fault or a silent drive; Ctrl-C or
SIGTERM sends zero and exits.

Usage:
    tools/fe_soak.py                       # asks for Enter before driving
    tools/fe_soak.py --cycles 5 --still 10 # a shorter run
"""

import argparse
import csv
import datetime
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drive_tune import FAULT_LATCHED, MODE_FAULT, SILENT_DRIVE_S, hardware_params  # noqa: E402

TWO_PI = 2.0 * math.pi
RATE = 50.0
WHEELS = ('FL', 'FR', 'RL', 'RR')
MIN_LIMIT_RAD = 0.05
LIMIT_FACTOR = 2.5
DRIFT_COUNTS = 4        # settled error moving more than this over a phase is a build-up


def _interrupt(*_):
    raise KeyboardInterrupt


# ------------------------------------------------------------------ pure functions (testable)

def build_sequence(speed, turn, cycles, reversals, still, press=1.0, pause=1.0):
    """Return [(phase, label, (vx, vy, wz), seconds, settled)]; settled marks a stop's end."""
    seq = [('still_start', 'still', (0.0, 0.0, 0.0), still, True)]
    axes = (('forward_back', (speed, 0.0, 0.0)), ('strafe', (0.0, speed, 0.0)),
            ('rotate', (0.0, 0.0, turn)))
    for phase, (vx, vy, wz) in axes:
        for c in range(cycles):
            seq.append((phase, f'{c + 1}+', (vx, vy, wz), press, False))
            seq.append((phase, f'{c + 1}+ stop', (0.0, 0.0, 0.0), pause, True))
            seq.append((phase, f'{c + 1}-', (-vx, -vy, -wz), press, False))
            seq.append((phase, f'{c + 1}- stop', (0.0, 0.0, 0.0), pause, True))
    for c in range(reversals):
        seq.append(('reversals', f'{c + 1}+', (speed, 0.0, 0.0), press, False))
        seq.append(('reversals', f'{c + 1}-', (-speed, 0.0, 0.0), press, False))
    if reversals:
        seq.append(('reversals', 'stop', (0.0, 0.0, 0.0), pause, True))
    seq.append(('still_end', 'still', (0.0, 0.0, 0.0), still, True))
    return seq


def summarize(rows, settled, counts_per_rev):
    """
    Summarize a run.

    rows: [(phase, fe[4])] for every report; settled: [(phase, fe[4])] at the end of each stop.
    Returns (phases, peak_counts, drift, suggestion_rad) where phases maps a phase to
    {'max': [4], 'settled_first': [4], 'settled_last': [4]}, peak_counts is the largest |fe|
    over the run, drift is True when a settled error moved by more than DRIFT_COUNTS within a
    phase or over the run, and suggestion_rad is the limit (None on drift or no data).
    """
    phases = {}
    for phase, fe in rows:
        p = phases.setdefault(phase, {'max': [0] * 4, 'settled_first': None,
                                      'settled_last': None})
        p['max'] = [max(m, abs(v)) for m, v in zip(p['max'], fe)]
    for phase, fe in settled:
        p = phases.setdefault(phase, {'max': [0] * 4, 'settled_first': None,
                                      'settled_last': None})
        if p['settled_first'] is None:
            p['settled_first'] = list(fe)
        p['settled_last'] = list(fe)
    if not rows:
        return phases, 0, False, None
    peak = max(max(p['max']) for p in phases.values())
    drift = False
    for p in phases.values():
        if p['settled_first'] is not None and any(
                abs(b - a) > DRIFT_COUNTS
                for a, b in zip(p['settled_first'], p['settled_last'])):
            drift = True
    if settled and any(abs(b - a) > DRIFT_COUNTS
                       for a, b in zip(settled[0][1], settled[-1][1])):
        drift = True
    suggestion = None
    if not drift:
        suggestion = max(MIN_LIMIT_RAD, LIMIT_FACTOR * peak * TWO_PI / counts_per_rev)
    return phases, peak, drift, suggestion


# ------------------------------------------------------------------ ROS side

class Soak:

    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from mecanum_interfaces.msg import DriveState
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        self.rclpy = rclpy
        self.TwistStamped = TwistStamped
        self.args = args
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGTERM, _interrupt)
        self.node = rclpy.create_node('fe_soak')
        self.pub = self.node.create_publisher(TwistStamped, args.cmd_topic, 10)
        self.drive = None
        self.joints_time = 0.0
        self.urdf = None
        self.phase = 'wait'
        self.label = ''
        self.cmd = (0.0, 0.0, 0.0)
        self.rows = []
        self.writer = None
        self.node.create_subscription(DriveState, '/drive/state', self._on_drive, 200)
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/robot_description', self._on_urdf, latched)

    def _on_drive(self, msg):
        self.drive = msg
        if self.writer is None or self.phase == 'wait':
            return
        fe = list(msg.following_error)
        self.rows.append((self.phase, fe))
        self.writer.writerow([f'{time.monotonic():.4f}', self.phase, self.label,
                              *(f'{v:.3f}' for v in self.cmd), *fe, *msg.counts,
                              *msg.pulses, *(f'{v:.4f}' for v in msg.command),
                              msg.faults, msg.mode, msg.mcu_rx_errors, msg.host_rx_errors])

    def _on_joints(self, _):
        self.joints_time = time.monotonic()

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)

    def publish(self, vx, vy, wz):
        msg = self.TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.pub.publish(msg)
        self.cmd = (vx, vy, wz)

    def stop(self):
        for _ in range(5):
            self.publish(0.0, 0.0, 0.0)
            self.spin(0.02)

    def fault(self):
        if time.monotonic() - self.joints_time > SILENT_DRIVE_S:
            return 'no /joint_states (the drive stopped reporting)'
        if self.drive is not None and (self.drive.faults & FAULT_LATCHED
                                       or self.drive.mode == MODE_FAULT):
            return f'ESP32 fault 0x{self.drive.faults:04x}'
        return None

    def wait_ready(self, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and (self.urdf is None or self.drive is None
                                          or time.monotonic() - self.joints_time > 1.0):
            self.spin(0.1)
        if self.urdf is None or self.drive is None:
            sys.exit('No /robot_description or /drive/state: is drive_test.launch.py running?')
        self.hw = hardware_params(self.urdf)
        self.cpr = float(self.hw.get('counts_per_wheel_rev', 4000))

    def run(self):
        a = self.args
        self.wait_ready()
        reason = self.fault()
        if reason:
            print(f'Not starting: {reason}. Recover first (test document, section 7).')
            return 2
        limit = float(self.hw.get('following_error_limit', 0))
        seq = build_sequence(a.speed, a.turn, a.cycles, a.reversals, a.still, a.press,
                             a.pause)
        total = sum(s[3] for s in seq)
        print(f'Following-error limit in use: {limit:.3f} rad'
              + (' (off)' if limit == 0 else ' (a trip ends the run)'))
        print(f'Sequence: still {a.still:.0f} s, {a.cycles} cycles each of forward/back, '
              f'strafe, rotate at {a.speed} m/s / {a.turn} rad/s, {a.reversals} reversals, '
              f'still {a.still:.0f} s: about {total / 60:.1f} min.')
        if not a.yes:
            try:
                input('Robot on blocks, wheels clear, hands away, someone at the power lead. '
                      'Enter to start, Ctrl-C to quit: ')
            except EOFError:
                return 1
        os.makedirs(a.out_dir, exist_ok=True)
        path = os.path.join(a.out_dir, 'fe_soak.csv')
        settled = []
        with open(path, 'w', newline='') as f:
            self.writer = csv.writer(f)
            self.writer.writerow(
                ['t', 'phase', 'step', 'vx', 'vy', 'wz',
                 *(f'fe_{w}' for w in WHEELS), *(f'counts_{w}' for w in WHEELS),
                 *(f'pulses_{w}' for w in WHEELS), *(f'cmd_{w}' for w in WHEELS),
                 'faults', 'mode', 'mcu_rx_errors', 'host_rx_errors'])
            current = None
            for phase, label, (vx, vy, wz), seconds, is_stop in seq:
                if phase != current:
                    print(f'  {phase}')
                    current = phase
                self.phase, self.label = phase, label
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    self.publish(vx, vy, wz)
                    self.spin(1.0 / RATE)
                    reason = self.fault()
                    if reason:
                        self.stop()
                        print(f'ABORT in {phase} {label}: {reason}. Recorded to {path}.')
                        self.report(settled, aborted=True)
                        return 2
                if is_stop and self.drive is not None:
                    settled.append((phase, list(self.drive.following_error)))
            self.stop()
            self.phase = 'wait'
        print(f'Recorded {len(self.rows)} reports to {path}.')
        self.report(settled)
        return 0

    def report(self, settled, aborted=False):
        phases, peak, drift, suggestion = summarize(self.rows, settled, self.cpr)

        def fmt(v):
            return ' '.join(f'{x:5d}' for x in v) if v is not None else '    -'
        print(f'\n{"phase":14s} {"max |FE| FL FR RL RR":>24s}   '
              f'{"settled first":>23s}   {"settled last":>23s}   (counts)')
        for phase, p in phases.items():
            print(f'{phase:14s} {fmt(p["max"]):>24s}   {fmt(p["settled_first"]):>23s}   '
                  f'{fmt(p["settled_last"]):>23s}')
        print(f'\nLargest |FE|: {peak} counts = {peak * TWO_PI / self.cpr:.4f} rad.')
        if aborted:
            print('The run was aborted: no limit suggested (the peak includes the fault). '
                  'Recover (test document, section 7) and run it again.')
        elif drift:
            print(f'The settled error moved by more than {DRIFT_COUNTS} counts: a build-up, '
                  'not lag. No limit suggested: look at the encoder tap first (test '
                  'document, section 9), and send the CSV.')
        elif suggestion is not None:
            print(f'No build-up. Suggested following_error_limit:={suggestion:.3f} '
                  f'({LIMIT_FACTOR}x the largest, at least {MIN_LIMIT_RAD} rad).')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument('--speed', type=float, default=0.1, help='m/s, x and y')
    parser.add_argument('--turn', type=float, default=0.3, help='rad/s')
    parser.add_argument('--cycles', type=int, default=20, help='Cycles per axis')
    parser.add_argument('--reversals', type=int, default=10)
    parser.add_argument('--still', type=float, default=30.0, help='Still phases (s)')
    parser.add_argument('--press', type=float, default=1.0, help='Drive time per press (s)')
    parser.add_argument('--pause', type=float, default=1.0, help='Stop time (s)')
    parser.add_argument('--yes', action='store_true',
                        help='Do not ask (the fake ESP32 only: no one confirms the wheels '
                        'are clear)')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--out-dir', default=os.path.expanduser(
        datetime.datetime.now().strftime('~/drive_tune/fe_soak_%Y%m%d_%H%M%S')))
    args = parser.parse_args(argv)
    if args.speed <= 0 or args.turn <= 0 or args.cycles < 0 or args.reversals < 0:
        parser.error('speeds must be positive, counts not negative')

    from rclpy.executors import ExternalShutdownException
    soak = Soak(args)
    code = 1
    try:
        code = soak.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nInterrupted: stopping.')
        code = 130
    finally:
        if soak.rclpy.ok():
            soak.stop()
        soak.node.destroy_node()
        soak.rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
