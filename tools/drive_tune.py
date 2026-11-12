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
Drive tuning ladders on the real base: one speed step at a time, judged by the limit rule.

Stages T4 and T5 of docs/hardware_drive_test.md. Run it beside drive_test.launch.py (and the
L2 driver for a wall or gyro reference). For each step it:
  1. asks the owner to confirm (Enter; s skips, q quits): there is no physical E-stop;
  2. raises the drive controller's speed limit for that axis to the step, if needed (restored
     on exit), and refuses a step the ESP32's max_wheel_speed would clamp;
  3. measures the reference while still: the wall distance (tools/l2_wall_range.py) for x
     and y, or the L2 gyro's bias for rotation;
  4. drives the leg (distance or angle at the step's speed), sends zero for 0.5 s (or nothing,
     with --stop timeout: the reference timeout stops it), and waits until the wheels are
     still; the stop is the odometry travel from the end of the leg to rest;
  5. measures the reference again, and asks for the tape (--tape) and for what the owner saw;
  6. drives the return leg the same way, so the robot ends near where it started.

Each leg is judged against the limit rule (owner, 2026-11-11). It fails on any of:
  - a following error over 50% of the stack's limit (following_error_limit of the xacro);
  - slip over 3% forward, 8% strafe or 3% rotation against the reference (the tape when given,
    else the L2 wall or gyro);
  - a fault, or a stall, tipping or judder seen by the owner.
It aborts at once (zero command, exit 2) on a latched ESP32 fault or a silent drive.

Ramps: the controller has no acceleration limits on the hardware (controllers_hardware.yaml
says why); the ESP32's accel_backstop sets every start and stop. An acceleration ladder is one
run of this tool per backstop value, each after relaunching drive_test.launch.py with
accel_backstop:=<value>; the tool reads the value in use from /robot_description.

Writes <out-dir>/samples.csv (50 Hz) and appends to <out-dir>/summary.csv (one row per leg).

Usage, for instance:
    tools/drive_tune.py --axis x --speeds 0.1,0.15,0.2 --distance 1.5 --reference wall \
        --wall front --tape
    tools/drive_tune.py --axis wz --speeds 0.3,0.5 --angle 720 --reference gyro
    tools/drive_tune.py --axis combined --twist 0.1,0.1,0 --twist 0.1,0,0.3 --duration 4
"""

import argparse
import csv
import datetime
import math
import os
import re
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

TWO_PI = 2.0 * math.pi
RATE = 50.0
SLIP_LIMIT_PCT = {'x': 3.0, 'y': 8.0, 'wz': 3.0}
FE_SHARE = 0.5          # of following_error_limit
SPEED_MARGIN = 0.9      # of max_wheel_speed
FAULT_LATCHED = 0x80
MODE_FAULT = 3
STILL_LINEAR = 0.002    # m/s
STILL_ANGULAR = 0.005   # rad/s
SILENT_DRIVE_S = 0.3


def _interrupt(*_):
    raise KeyboardInterrupt


# ------------------------------------------------------------------ pure functions (testable)

def wheel_speed_needed(vx, vy, wz, radius, lxly):
    """Largest wheel speed (rad/s) of the X-roller inverse kinematics."""
    return (abs(vx) + abs(vy) + lxly * abs(wz)) / radius


def hardware_params(urdf_xml):
    """Return the <param> values of the ros2_control hardware block, as strings."""
    hardware = re.search(r'<hardware>(.*?)</hardware>', urdf_xml, re.S)
    if not hardware:
        return {}
    return dict(re.findall(r'<param name="([^"]+)">\s*([^<]*?)\s*</param>', hardware.group(1)))


def body_displacement(start, end):
    """(dx, dy, dyaw) of pose `end` in the frame of pose `start`; poses are (x, y, yaw)."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, end[2] - start[2]


def slip_pct(measured, reference):
    """Odometry over the reference, in % (positive: the wheels report more than happened)."""
    if reference is None or abs(reference) < 1e-6:
        return None
    return 100.0 * (measured - reference) / abs(reference)


def judge(axis, slip, max_fe, fe_limit, observed_problem, fault):
    """Return (passed, reasons) under the limit rule."""
    reasons = []
    if fault:
        reasons.append(f'fault: {fault}')
    if fe_limit > 0 and max_fe > FE_SHARE * fe_limit:
        reasons.append(f'following error {max_fe:.3f} rad > {FE_SHARE:.0%} of {fe_limit:.3f}')
    if slip is not None and axis in SLIP_LIMIT_PCT and abs(slip) > SLIP_LIMIT_PCT[axis]:
        reasons.append(f'slip {slip:+.2f}% > {SLIP_LIMIT_PCT[axis]:.0f}%')
    if observed_problem:
        reasons.append(f'observed: {observed_problem}')
    return not reasons, reasons


def ramp_time(samples, t0, target, index):
    """Seconds from 10% to 90% of `target` in samples[(t, value...)] after t0, or None."""
    if abs(target) < 1e-6:
        return None
    t10 = t90 = None
    for row in samples:
        if row[0] < t0:
            continue
        frac = row[index] / target
        if t10 is None and frac >= 0.1:
            t10 = row[0]
        if t10 is not None and frac >= 0.9:
            t90 = row[0]
            break
    return None if t10 is None or t90 is None else t90 - t10


def unwrap(prev, angle):
    """Angle continued from prev (both rad), without the jump at +-pi."""
    return prev + math.atan2(math.sin(angle - prev), math.cos(angle - prev))


# ------------------------------------------------------------------ ROS side

class Tuner:

    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from mecanum_interfaces.msg import DriveState
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.TwistStamped = TwistStamped
        self.args = args
        # Ctrl-C and SIGTERM raise KeyboardInterrupt with the context still valid, so the
        # exit path can still send zero (and restore the limits).
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGTERM, _interrupt)
        self.node = rclpy.create_node('drive_tune')
        self.pub = self.node.create_publisher(TwistStamped, args.cmd_topic, 10)
        self.pose = None            # (x, y, yaw unwrapped)
        self.twist = (0.0, 0.0, 0.0)
        self.drive = None
        self.joints_time = 0.0
        self.gyro = None            # base-frame z rate
        self.gyro_time = 0.0
        self.urdf = None
        self.imu_rot = None
        self.cmd = (0.0, 0.0, 0.0)
        self.samples = []
        self.leg = ''
        self.node.create_subscription(Odometry, '/wheel/odometry', self._on_odom, 20)
        self.node.create_subscription(DriveState, '/drive/state', self._on_drive, 50)
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self.node.create_subscription(Imu, '/l2/imu', self._on_imu, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/robot_description', self._on_urdf, latched)
        self.wall = None
        if args.reference == 'wall':
            from l2_wall_range import WallRange
            self.wall = WallRange(self.node, args.wall)
        if args.reference == 'gyro':
            import tf2_ros
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.saved_limits = {}

    # Callbacks.

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.pose is not None:
            yaw = unwrap(self.pose[2], yaw)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        t = msg.twist.twist
        self.twist = (t.linear.x, t.linear.y, t.angular.z)

    def _on_drive(self, msg):
        self.drive = msg

    def _on_joints(self, _):
        self.joints_time = time.monotonic()

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_imu(self, msg):
        if self.args.reference != 'gyro':
            return
        if self.imu_rot is None:
            try:
                import numpy as np
                from l2_mount_check import quat_to_matrix
                import rclpy.time
                tf = self.tf_buffer.lookup_transform(
                    'base_footprint', msg.header.frame_id, rclpy.time.Time())
                r = tf.transform.rotation
                self.imu_rot = np.asarray(quat_to_matrix(r.x, r.y, r.z, r.w))
            except Exception:
                return
        w = msg.angular_velocity
        self.gyro = float(self.imu_rot[2] @ [w.x, w.y, w.z])
        self.gyro_time = time.monotonic()

    # Helpers.

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

    def fe_rad(self):
        cpr = float(self.hw.get('counts_per_wheel_rev', 4000))
        return [abs(v) * TWO_PI / cpr for v in self.drive.following_error]

    def fault(self):
        """Return a reason to abort now, or None."""
        if time.monotonic() - self.joints_time > SILENT_DRIVE_S:
            return 'no /joint_states (the drive stopped reporting)'
        if self.drive is not None and (self.drive.faults & FAULT_LATCHED
                                       or self.drive.mode == MODE_FAULT):
            return f'ESP32 fault 0x{self.drive.faults:04x}'
        return None

    def record(self):
        pose = self.pose or (math.nan,) * 3
        fe = self.fe_rad() if self.drive else [math.nan] * 4
        row = [time.monotonic(), self.leg, *self.cmd, *pose, *self.twist, *fe,
               self.drive.faults if self.drive else -1, self.drive.mode if self.drive else -1,
               self.gyro if self.gyro is not None else math.nan]
        self.samples.append(row)
        self.writer.writerow([f'{v:.6f}' if isinstance(v, float) else v for v in row])

    def wait_ready(self, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and (self.urdf is None or self.pose is None
                                          or self.drive is None):
            self.spin(0.1)
        if self.urdf is None or self.pose is None or self.drive is None:
            sys.exit('No /robot_description, /wheel/odometry or /drive/state: is '
                     'drive_test.launch.py running?')
        self.hw = hardware_params(self.urdf)
        self.radius = float(re.search(r'wheels_radius:\s*([0-9.]+)', open(
            self.controllers_yaml()).read()).group(1))
        self.lxly = float(re.search(
            r'sum_of_robot_center_projection_on_X_Y_axis:\s*([0-9.]+)',
            open(self.controllers_yaml()).read()).group(1))

    @staticmethod
    def controllers_yaml():
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('mecanum_controllers'), 'config',
                            'controllers.yaml')

    # Controller limits.

    def _param_call(self, srv_type, name, request):
        client = self.node.create_client(srv_type, f'/mecanum_drive_controller/{name}')
        if not client.wait_for_service(timeout_sec=3.0):
            sys.exit(f'/mecanum_drive_controller/{name} is not available')
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def get_limit(self, name):
        from rcl_interfaces.srv import GetParameters
        result = self._param_call(GetParameters, 'get_parameters',
                                  GetParameters.Request(names=[name]))
        return result.values[0].double_value

    def set_limit(self, name, value):
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        p = Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
        result = self._param_call(SetParameters, 'set_parameters',
                                  SetParameters.Request(parameters=[p]))
        if not result.results[0].successful:
            sys.exit(f'Cannot set {name}: {result.results[0].reason}')

    def ensure_limit(self, axis, speed):
        prefix = {'x': 'linear.x', 'y': 'linear.y', 'wz': 'angular.z'}[axis]
        for bound, value in (('max_velocity', abs(speed)), ('min_velocity', -abs(speed))):
            name = f'{prefix}.{bound}'
            current = self.get_limit(name)
            if name not in self.saved_limits:
                self.saved_limits[name] = current
            if abs(value) > abs(current) + 1e-9:
                self.set_limit(name, value)
                print(f'  raised {name} to {value:+.3f}')

    def restore_limits(self):
        for name, value in self.saved_limits.items():
            try:
                self.set_limit(name, value)
            except SystemExit:
                print(f'Could not restore {name} to {value}')

    # References.

    def measure_reference(self):
        """Still reference: wall distance (m), or the gyro bias (rad/s); None without one."""
        if self.args.reference == 'wall':
            result = self.wall.measure(self.spin, 1.0)
            if result is None:
                print('  no wall found')
                return None
            print(f'  wall {result[0]:.3f} m (yaw {result[1]:+.2f} deg, {result[3]} points)')
            return result[0]
        if self.args.reference == 'gyro':
            values = []
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                self.spin(0.01)
                if self.gyro is not None and time.monotonic() - self.gyro_time < 0.05:
                    values.append(self.gyro)
            return sum(values) / len(values) if values else None
        return None

    # A leg.

    def drive_leg(self, twist, duration):
        """
        Drive `twist` for `duration` s, then zero, then wait until still.

        Returns (abort reason or None, t_go, t_zero, gyro integral, max FE).
        """
        dt = 1.0 / RATE
        gyro_sum = 0.0
        max_fe = 0.0
        t_go = time.monotonic()
        t_zero = t_go + duration
        # --stop zero: half a second of zero commands, then silence. --stop timeout: silence at
        # once, so the stop goes through the controller's reference timeout (0.25 s).
        t_quiet = t_zero + (0.5 if self.args.stop == 'zero' else 0.0)
        still_since = None
        next_tick = t_go
        last = t_go
        while True:
            now = time.monotonic()
            if now < t_zero:
                self.publish(*twist)
            elif now < t_quiet:
                self.publish(0.0, 0.0, 0.0)
            else:
                self.cmd = (0.0, 0.0, 0.0)   # silent: the reference timeout path
                vx, vy, wz = self.twist
                still = (math.hypot(vx, vy) < STILL_LINEAR and abs(wz) < STILL_ANGULAR)
                still_since = (still_since or now) if still else None
                if still_since and now - still_since > 0.3:
                    break
                if now > t_quiet + 5.0:
                    return 'the wheels did not come to rest', t_go, t_zero, gyro_sum, max_fe
            next_tick += dt
            self.spin(max(0.0, next_tick - time.monotonic()))
            now = time.monotonic()
            if self.gyro is not None:
                gyro_sum += self.gyro * (now - last)
            last = now
            if self.drive is not None:
                max_fe = max(max_fe, *self.fe_rad())
            self.record()
            reason = self.fault()
            if reason:
                self.stop()
                return reason, t_go, t_zero, gyro_sum, max_fe
        return None, t_go, t_zero, gyro_sum, max_fe

    def ask(self, prompt):
        if self.args.yes:
            return ''
        try:
            return input(prompt).strip()
        except EOFError:
            return 'q'

    def run_leg(self, axis, twist, duration, label, step):
        """One leg: reference, drive, reference, judgement. Returns False to abort."""
        self.leg = label
        ref_start = self.measure_reference()
        pose_start = self.pose
        abort, t_go, t_zero, gyro_sum, max_fe = self.drive_leg(twist, duration)
        pose_end = self.pose
        dx, dy, dyaw = body_displacement(pose_start, pose_end)
        odom = {'x': dx, 'y': dy, 'wz': dyaw}.get(axis)

        # Stopping: odometry travel from the zero command to rest.
        stop_rows = [r for r in self.samples if r[1] == label and r[0] >= t_zero]
        stop_dist = None
        if stop_rows:
            s0, s1 = stop_rows[0], stop_rows[-1]
            sd = body_displacement((s0[5], s0[6], s0[7]), (s1[5], s1[6], s1[7]))
            stop_dist = math.hypot(sd[0], sd[1]) if axis != 'wz' else abs(sd[2])
        index = {'x': 8, 'y': 9, 'wz': 10}.get(axis)
        target = {'x': twist[0], 'y': twist[1], 'wz': twist[2]}.get(axis)
        ramp = ramp_time([r for r in self.samples if r[1] == label], t_go, target, index) \
            if index else None

        reference = None
        if abort is None and axis in ('x', 'y') and ref_start is not None:
            ref_end = self.measure_reference()
            if ref_end is not None:
                toward = {'front': 1, 'left': 1, 'back': -1, 'right': -1}[self.args.wall]
                reference = (ref_start - ref_end) * toward
        if abort is None and axis == 'wz' and ref_start is not None:
            reference = gyro_sum - ref_start * (self.samples[-1][0] - t_go)

        tape = None
        if abort is None and self.args.tape and axis != 'combined':
            unit = 'deg' if axis == 'wz' else 'm'
            text = self.ask(f'  tape: {label} travel in {unit}, signed (Enter: none): ')
            try:
                tape = float(text)
                if axis == 'wz':
                    tape = math.radians(tape)
            except ValueError:
                tape = None
        observed = '' if abort else self.ask(
            '  stall noise, tipping or judder? (Enter: none, else describe): ')
        slip = slip_pct(odom, tape if tape is not None else reference) \
            if odom is not None else None

        fe_limit = float(self.hw.get('following_error_limit', 0.0))
        passed, reasons = judge(axis, slip, max_fe, fe_limit, observed, abort)
        verdict = 'PASS' if passed else 'FAIL'
        unit = 'rad' if axis == 'wz' else 'm'
        print(f'  {label}: {verdict}  odom {odom if odom is not None else math.nan:+.4f} {unit}'
              f'  ref {reference if reference is not None else math.nan:+.4f}'
              f'  tape {tape if tape is not None else math.nan:+.4f}'
              f'  slip {slip if slip is not None else math.nan:+.2f}%'
              f'  max FE {max_fe:.4f} rad (limit {fe_limit:.3f})'
              f'  stop {stop_dist if stop_dist is not None else math.nan:.4f} {unit}'
              f'  ramp {ramp if ramp is not None else math.nan:.2f} s')
        for reason in reasons:
            print(f'    - {reason}')
        self.summary.writerow([
            datetime.datetime.now().isoformat(timespec='seconds'), label, axis, step,
            *twist, f'{duration:.3f}', self.hw.get('accel_backstop', ''),
            self.hw.get('max_wheel_speed', ''), fe_limit,
            *(f'{v:.5f}' if v is not None else '' for v in
              (odom, reference, tape, slip, max_fe, stop_dist, ramp)),
            f'{dx:.5f}', f'{dy:.5f}', f'{dyaw:.5f}', verdict, '; '.join(reasons)])
        self.summary_file.flush()
        if abort:
            print(f'ABORT: {abort}')
            return False
        return True

    def steps(self):
        a = self.args
        if a.axis == 'combined':
            for twist in a.twist:
                yield twist, a.duration
            return
        for speed in a.speeds:
            if a.axis == 'wz':
                yield (0.0, 0.0, speed), math.radians(a.angle) / abs(speed)
            else:
                twist = (speed, 0.0, 0.0) if a.axis == 'x' else (0.0, speed, 0.0)
                yield twist, a.distance / abs(speed)

    def run(self):
        a = self.args
        os.makedirs(a.out_dir, exist_ok=True)
        samples_path = os.path.join(a.out_dir, 'samples.csv')
        summary_path = os.path.join(a.out_dir, 'summary.csv')
        new_summary = not os.path.exists(summary_path)
        with open(samples_path, 'a', newline='') as samples_file, \
                open(summary_path, 'a', newline='') as self.summary_file:
            self.writer = csv.writer(samples_file)
            self.writer.writerow(['t', 'leg', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'odom_x', 'odom_y',
                                  'odom_yaw', 'odom_vx', 'odom_vy', 'odom_wz', 'fe_fl', 'fe_fr',
                                  'fe_rl', 'fe_rr', 'faults', 'mode', 'gyro_wz'])
            self.summary = csv.writer(self.summary_file)
            if new_summary:
                self.summary.writerow([
                    'time', 'leg', 'axis', 'step', 'vx', 'vy', 'wz', 'duration_s',
                    'accel_backstop', 'max_wheel_speed', 'fe_limit_rad', 'odom', 'reference',
                    'tape', 'slip_pct', 'max_fe_rad', 'stop', 'ramp_s', 'dx', 'dy', 'dyaw',
                    'result', 'reasons'])
            self.wait_ready()
            print(f'accel_backstop {self.hw.get("accel_backstop")} rad/s^2, max_wheel_speed '
                  f'{self.hw.get("max_wheel_speed")} rad/s, following_error_limit '
                  f'{self.hw.get("following_error_limit")} rad; writing {a.out_dir}')
            max_wheel = float(self.hw.get('max_wheel_speed', 0.0))
            for k, (twist, duration) in enumerate(self.steps(), start=1):
                need = wheel_speed_needed(*twist, self.radius, self.lxly)
                if need > SPEED_MARGIN * max_wheel:
                    print(f'Step {k} {twist} needs {need:.2f} rad/s at a wheel, over '
                          f'{SPEED_MARGIN:.0%} of max_wheel_speed {max_wheel}: relaunch '
                          f'drive_test.launch.py with max_wheel_speed:={need / 0.8:.1f} first.')
                    break
                answer = self.ask(
                    f'\nStep {k}: twist {twist} for {duration:.1f} s and back. Path clear, '
                    'someone at the power lead? Enter runs, s skips, q quits: ')
                if answer.lower().startswith('q'):
                    break
                if answer.lower().startswith('s'):
                    continue
                if a.axis != 'combined':
                    self.ensure_limit(a.axis, max(abs(v) for v in twist))
                else:
                    for axis, v in zip(('x', 'y', 'wz'), twist):
                        if v:
                            self.ensure_limit(axis, abs(v))
                axis = a.axis
                if not self.run_leg(axis, twist, duration, f'{k}out', k):
                    return 2
                if a.no_return:
                    continue
                back = tuple(-v for v in twist)
                if not self.run_leg(axis, back, duration, f'{k}back', k):
                    return 2
        return 0


def parse_floats(text):
    return [float(v) for v in text.split(',') if v.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument('--axis', choices=('x', 'y', 'wz', 'combined'), required=True)
    parser.add_argument('--speeds', type=parse_floats, default=[],
                        help='Comma list of step speeds (m/s or rad/s; sign = direction)')
    parser.add_argument('--twist', type=lambda t: tuple(parse_floats(t)), action='append',
                        default=[], help='vx,vy,wz for --axis combined; repeat per step')
    parser.add_argument('--distance', type=float, default=1.0, help='Leg length (m), x and y')
    parser.add_argument('--angle', type=float, default=360.0, help='Leg angle (deg), wz')
    parser.add_argument('--duration', type=float, default=3.0, help='Leg time (s), combined')
    parser.add_argument('--reference', choices=('wall', 'gyro', 'none'), default='none')
    parser.add_argument('--wall', choices=('front', 'back', 'left', 'right'), default='front',
                        help='The wall the L2 measures (x: front/back, y: left/right)')
    parser.add_argument('--tape', action='store_true',
                        help='Ask for the tape-measured travel after each leg')
    parser.add_argument('--no-return', action='store_true', help='Skip the return legs')
    parser.add_argument('--stop', choices=('zero', 'timeout'), default='zero',
                        help='End a leg with zero commands, or with silence (the reference '
                        'timeout path, as a teleop key release)')
    parser.add_argument('--yes', action='store_true',
                        help='Do not ask (the fake ESP32 only: no one confirms the path)')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--out-dir', default=os.path.expanduser(
        datetime.datetime.now().strftime('~/drive_tune/%Y%m%d_%H%M%S')))
    args = parser.parse_args(argv)
    if args.axis == 'combined' and (not args.twist or any(len(t) != 3 for t in args.twist)):
        parser.error('--axis combined needs --twist vx,vy,wz (one per step)')
    if args.axis != 'combined' and not args.speeds:
        parser.error('--speeds is required')
    if args.axis == 'wz' and args.reference == 'wall':
        parser.error('rotation uses --reference gyro (or none with --tape)')

    from rclpy.executors import ExternalShutdownException
    tuner = Tuner(args)
    code = 1
    try:
        code = tuner.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nInterrupted: stopping.')
        code = 130
    finally:
        if tuner.rclpy.ok():
            tuner.stop()
            tuner.restore_limits()
        else:
            print('ROS shut down first: the reference timeout stops the drive; the speed '
                  f'limits raised at run time stay until relaunch: {tuner.saved_limits}')
        tuner.node.destroy_node()
        tuner.rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
