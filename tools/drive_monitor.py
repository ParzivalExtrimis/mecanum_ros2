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
Watch the ESP32 drive per wheel, and find the motor order and signs on blocks.

Stages T1 and T2 of docs/hardware_drive_test.md. Run it beside drive_test.launch.py.

watch (default): twice a second, per wheel (FL FR RL RR, the protocol's order), the command,
    the speed from the emitted pulses and from the encoder counts, the pulses and counts since
    the last reset (Enter resets), and the following error; then the fault bits and the mode.
    For the pulses and counts per revolution: mark a wheel, press Enter, turn it 10 times
    under a slow command, and divide the two totals by 10.

identify: on blocks only. Drives three slow patterns (forward, strafe left, rotate
    counter-clockwise), one at a time, while the owner looks at each physical wheel and types
    which way it rolls. From the answers, and from the counts' signs against the pulses', it
    prints the motor_for_wheel, dir_invert and enc_invert launch arguments that make every
    wheel right. Run it with following_error_limit:=0 (the check is off): with a wrong sign the
    counts run against the pulses and the fault would trip at once.

    tools/drive_monitor.py
    tools/drive_monitor.py identify
"""

import argparse
import math
import re
import select
import signal
import sys
import time

TWO_PI = 2.0 * math.pi
WHEELS = ('FL', 'FR', 'RL', 'RR')
FAULT_BITS = {0x01: 'FE FL', 0x02: 'FE FR', 0x04: 'FE RL', 0x08: 'FE RR', 0x10: 'watchdog',
              0x20: 'no config', 0x40: 'speed clamped', 0x80: 'LATCHED',
              0x100: 'step/encoder init'}
MODES = {0: 'no config', 1: 'idle', 2: 'running', 3: 'FAULT'}

# Wheel speed signs of the X-roller inverse kinematics (mecanum_drive_controller), per pattern:
# forward (+vx), strafe left (+vy), rotate counter-clockwise (+wz).
PATTERNS = (('forward', (1.0, 0.0, 0.0)), ('strafe left', (0.0, 1.0, 0.0)),
            ('rotate counter-clockwise', (0.0, 0.0, 1.0)))
SIGNS = {'FL': (1, -1, -1), 'FR': (1, 1, 1), 'RL': (1, 1, -1), 'RR': (1, -1, 1)}


def _interrupt(*_):
    raise KeyboardInterrupt


def decode_faults(bits):
    names = [name for bit, name in FAULT_BITS.items() if bits & bit]
    return ', '.join(names) if names else 'none'


def hardware_params(urdf_xml):
    """Return the <param> values of the ros2_control hardware block, as strings."""
    hardware = re.search(r'<hardware>(.*?)</hardware>', urdf_xml, re.S)
    if not hardware:
        return {}
    return dict(re.findall(r'<param name="([^"]+)">\s*([^<]*?)\s*</param>', hardware.group(1)))


def solve_mapping(order, dir_invert, enc_invert, observed, count_sign):
    """
    Return the new (motor_for_wheel, dir_invert, enc_invert) from the block observations.

    order[w]: the motor index now driving logical wheel w (FL FR RL RR). dir_invert and
    enc_invert: per motor 0..3. observed[p]: for physical wheel p, the roll directions (+1
    forward, -1 backward) seen in the three PATTERNS. count_sign[w]: +1 when logical wheel w's
    counts moved with its pulses in the forward pattern, -1 against.
    Raises ValueError when the observations fit no wheel, or two physical wheels the same one.
    """
    new_order = [None] * 4
    new_dir = list(dir_invert)
    new_enc = list(enc_invert)
    used = set()
    for p, wheel in enumerate(WHEELS):
        triple = tuple(observed[p])
        match = None
        for w, name in enumerate(WHEELS):
            for s in (1, -1):
                if tuple(s * v for v in SIGNS[name]) == triple:
                    match = (w, s)
        if match is None:
            raise ValueError(f'{wheel}: {triple} fits no wheel; check the answers')
        w, s = match
        if w in used:
            raise ValueError(f'{wheel} turns like another wheel ({WHEELS[w]}); check the answers')
        used.add(w)
        motor = order[w]
        new_order[p] = motor
        if s < 0:
            new_dir[motor] ^= 1
    # The reported counts are raw, so flipping a motor's direction also turns its counts
    # against its pulses: the encoder flips when exactly one of the two disagrees.
    for w in range(4):
        motor = order[w]
        if (count_sign[w] < 0) != (new_dir[motor] != dir_invert[motor]):
            new_enc[motor] ^= 1
    return new_order, new_dir, new_enc


class Monitor:

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from mecanum_interfaces.msg import DriveState
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        self.rclpy = rclpy
        self.TwistStamped = TwistStamped
        # Ctrl-C and SIGTERM raise KeyboardInterrupt with the context still valid, so the
        # exit path can still send zero (and restore the limits).
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGTERM, _interrupt)
        self.node = rclpy.create_node('drive_monitor')
        self.state = None
        self.prev = None
        self.urdf = None
        self.node.create_subscription(DriveState, '/drive/state', self._on_state, 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/robot_description',
                                      lambda m: setattr(self, 'urdf', m.data), latched)
        self.pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def _on_state(self, msg):
        self.state = msg

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_ready(self):
        end = time.monotonic() + 10.0
        while time.monotonic() < end and (self.state is None or self.urdf is None):
            self.spin(0.1)
        if self.state is None or self.urdf is None:
            sys.exit('No /drive/state or /robot_description: is drive_test.launch.py running?')
        self.hw = hardware_params(self.urdf)
        self.ppr = float(self.hw.get('pulses_per_rev', 1600))
        self.cpr = float(self.hw.get('counts_per_wheel_rev', 4000))

    def publish(self, vx, vy, wz):
        msg = self.TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def watch(self):
        self.wait_ready()
        print(f'pulses_per_rev {self.ppr:.0f}, counts_per_wheel_rev {self.cpr:.0f} (from the '
              'URDF). Enter resets the totals; Ctrl-C quits.')
        base = self.state
        last, last_t = self.state, time.monotonic()
        while True:
            self.spin(0.5)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                base = self.state
                print('-- totals reset --')
            s, now = self.state, time.monotonic()
            dt = max(now - last_t, 1e-3)
            print('wheel    cmd  pulse  count  (rad/s)    pulses    counts  (since Enter)'
                  '      FE (counts)')
            for w, name in enumerate(WHEELS):
                v_p = (s.pulses[w] - last.pulses[w]) * TWO_PI / self.ppr / dt
                v_c = (s.counts[w] - last.counts[w]) * TWO_PI / self.cpr / dt
                print(f'{name:5} {s.command[w]:6.3f} {v_p:6.3f} {v_c:6.3f}           '
                      f'{s.pulses[w] - base.pulses[w]:9d} {s.counts[w] - base.counts[w]:9d}'
                      f'                {s.following_error[w]:6d}')
            print(f'mode {MODES.get(s.mode, s.mode)}, faults {decode_faults(s.faults)}, '
                  f'rx errors MCU {s.mcu_rx_errors} host {s.host_rx_errors}\n')
            last, last_t = s, now

    def ask_while_driving(self, twist, prompt):
        """Publish `twist` at 20 Hz until the owner answers `prompt`; then stop."""
        print(prompt, end='', flush=True)
        while True:
            self.publish(*twist)
            self.spin(0.05)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                answer = sys.stdin.readline().strip()
                for _ in range(10):
                    self.publish(0.0, 0.0, 0.0)
                    self.spin(0.05)
                return answer

    def identify(self, speed, turn):
        self.wait_ready()
        if float(self.hw.get('following_error_limit', 0)) != 0.0:
            sys.exit('Relaunch with following_error_limit:=0 first: wrong signs would trip '
                     'the following-error fault at once.')
        order = [int(v) for v in self.hw.get('motor_for_wheel', '0 1 2 3').split()]
        dirs = [int(v) for v in self.hw.get('dir_invert', '0 0 0 0').split()]
        encs = [int(v) for v in self.hw.get('enc_invert', '0 0 0 0').split()]
        print(f'Now: motor_for_wheel "{" ".join(map(str, order))}", dir_invert '
              f'"{" ".join(map(str, dirs))}", enc_invert "{" ".join(map(str, encs))}".\n'
              'Wheels in the air. For each pattern, look at the physical wheels in the order '
              'FL FR RL RR (front: the end with the D435i camera) and type, for each, f if '
              'its top moves forward (it would roll the robot forward), b if backward, '
              '0 if it does not turn. Example: f b f b')
        observed = [[0, 0, 0] for _ in WHEELS]
        count_sign = [1] * 4
        for k, (name, unit) in enumerate(PATTERNS):
            twist = (unit[0] * speed, unit[1] * speed, unit[2] * turn)
            while True:
                start = self.state
                answer = self.ask_while_driving(
                    twist, f'\n{name} {twist}: FL FR RL RR roll (f/b/0)? ')
                end = self.state
                letters = answer.lower().split()
                if len(letters) == 4 and all(c in 'fb0' for c in letters):
                    break
                print('Four answers, each f, b or 0, please.')
            if '0' in letters:
                sys.exit(f'A wheel did not turn in "{name}": check its driver power, PUL/DIR '
                         'wiring and pin map before mapping.')
            for p, c in enumerate(letters):
                observed[p][k] = 1 if c == 'f' else -1
            if k == 0:
                for w in range(4):
                    dp = end.pulses[w] - start.pulses[w]
                    dc = end.counts[w] - start.counts[w]
                    if dc == 0:
                        print(f'{WHEELS[w]}: no counts while its pulses moved {dp}: the '
                              'encoder or the tap of that motor (decision 0006, "Gaps").')
                    count_sign[w] = 1 if dp * dc >= 0 else -1
        try:
            new_order, new_dir, new_enc = solve_mapping(order, dirs, encs, observed, count_sign)
        except ValueError as e:
            sys.exit(str(e))
        print('\nRelaunch drive_test.launch.py with:\n'
              f'  motor_for_wheel:="{" ".join(map(str, new_order))}" '
              f'dir_invert:="{" ".join(map(str, new_dir))}" '
              f'enc_invert:="{" ".join(map(str, new_enc))}"\n'
              'then run identify again: every pattern must answer as the robot should '
              '(forward: f f f f; strafe left: b f f b; rotate counter-clockwise: b f b f).')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument('mode', nargs='?', choices=('watch', 'identify'), default='watch')
    parser.add_argument('--speed', type=float, default=0.05, help='identify: m/s')
    parser.add_argument('--turn', type=float, default=0.15, help='identify: rad/s')
    args = parser.parse_args(argv)
    from rclpy.executors import ExternalShutdownException
    monitor = Monitor()
    try:
        if args.mode == 'watch':
            monitor.watch()
        else:
            monitor.identify(args.speed, args.turn)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if args.mode == 'identify' and monitor.rclpy.ok():
            for _ in range(5):
                monitor.publish(0.0, 0.0, 0.0)
                monitor.spin(0.02)
        monitor.node.destroy_node()
        monitor.rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
