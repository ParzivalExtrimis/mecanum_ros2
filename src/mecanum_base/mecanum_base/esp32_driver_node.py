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
ESP32 drive node: cmd_vel -> mecanum inverse kinematics -> per-motor ESP32 commands.

Open-loop by design: the motors have no encoders, so this node publishes no odometry and
no joint states. Odometry comes from LiDAR-inertial odometry only. See
docs/decisions/0001-open-loop-drive-without-ros2-control.md.

Safety: commands are sent at a fixed rate; zeros are sent when no cmd_vel arrives within
``cmd_timeout``, after any exception in the control loop, and on shutdown (SIGINT/SIGTERM).
"""

import signal
import threading

from geometry_msgs.msg import Twist, TwistStamped
from mecanum_base.drive_core import DriveCore, DriveLimits, DriveOutput
from mecanum_base.kinematics import MecanumGeometry, WHEEL_NAMES
from mecanum_base.transport import make_transport, TRANSPORTS
from mecanum_base.wheel_calibration import WheelCalibration
from mecanum_interfaces.msg import WheelCommands
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Defaults derived from mecanum_description/urdf/mecanum_params.xacro. config/esp32_driver.yaml
# repeats them; test/test_urdf_defaults.py checks both against the URDF.
DEFAULT_WHEEL_RADIUS = 0.05   # wheel_radius
DEFAULT_LX = 0.2255           # (front_axle_x - rear_axle_x) / 2 = (0.216 + 0.235) / 2
DEFAULT_LY = 0.207            # track_width / 2 = 0.414 / 2


class Esp32DriverNode(Node):

    def __init__(self):
        super().__init__('esp32_driver')

        self.declare_parameter('transport', 'mock')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_vel_stamped', False)
        self.declare_parameter('motor_commands_topic', '/esp32/motor_commands')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('cmd_timeout', 0.25)
        self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS)
        self.declare_parameter('lx', DEFAULT_LX)
        self.declare_parameter('ly', DEFAULT_LY)
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_wz', 0.8)

        calibrations = {}
        for index, wheel in enumerate(WHEEL_NAMES):
            prefix = f'wheels.{wheel}.'
            self.declare_parameter(prefix + 'motor_id', index)
            self.declare_parameter(prefix + 'sign', 1)
            self.declare_parameter(prefix + 'scale', 1.0 / 15.0)
            self.declare_parameter(prefix + 'deadband', 0.0)
            self.declare_parameter(prefix + 'max_value', 1.0)
            calibrations[wheel] = WheelCalibration(
                motor_id=self._int(prefix + 'motor_id'),
                sign=self._int(prefix + 'sign'),
                scale=self._float(prefix + 'scale'),
                deadband=self._float(prefix + 'deadband'),
                max_value=self._float(prefix + 'max_value'))

        geometry = MecanumGeometry(
            lx=self._float('lx'), ly=self._float('ly'),
            wheel_radius=self._float('wheel_radius'))
        limits = DriveLimits(self._float('max_vx'), self._float('max_vy'),
                             self._float('max_wz'))
        self._core = DriveCore(geometry, calibrations, limits, self._float('cmd_timeout'))

        transport_name = self.get_parameter('transport').value
        if transport_name not in TRANSPORTS:
            raise ValueError(f'transport must be one of {TRANSPORTS}, got {transport_name!r}')
        self._transport = make_transport(
            transport_name, node=self, log=self.get_logger().info,
            sim_topic=self.get_parameter('motor_commands_topic').value)
        self._transport.connect()
        self._stopped = False

        self._debug_pub = self.create_publisher(WheelCommands, '~/wheel_commands', 10)

        stamped = self.get_parameter('cmd_vel_stamped').value
        topic = self.get_parameter('cmd_vel_topic').value
        if stamped:
            self.create_subscription(TwistStamped, topic, self._on_twist_stamped, 10)
        else:
            self.create_subscription(Twist, topic, self._on_twist, 10)

        rate_hz = self._float('rate_hz')
        if not rate_hz > 0.0:
            raise ValueError('rate_hz must be > 0')
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self._was_timed_out = True

        self.get_logger().info(
            f'esp32_driver ready: transport={transport_name}, cmd_vel={topic} '
            f'({"TwistStamped" if stamped else "Twist"}), rate={rate_hz:.1f} Hz, '
            f'cmd_timeout={self._core.cmd_timeout:.2f} s, lx={geometry.lx:.4f} '
            f'ly={geometry.ly:.4f} r={geometry.wheel_radius:.4f}, limits=({limits.max_vx}, '
            f'{limits.max_vy}, {limits.max_wz})')

    def _float(self, name):
        return float(self.get_parameter(name).value)

    def _int(self, name):
        return int(self.get_parameter(name).value)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _store(self, twist):
        ok = self._core.set_command(twist.linear.x, twist.linear.y, twist.angular.z,
                                    self._now())
        if not ok:
            self.get_logger().warn('ignoring cmd_vel with non-finite values',
                                   throttle_duration_sec=1.0)

    def _on_twist(self, msg):
        self._store(msg)

    def _on_twist_stamped(self, msg):
        self._store(msg.twist)

    def _on_timer(self):
        try:
            output = self._core.compute(self._now())
            self._send(output)
            if output.timed_out and not self._was_timed_out:
                self.get_logger().info('cmd_vel timed out, sending zeros')
            self._was_timed_out = output.timed_out
        except Exception as exc:  # noqa: BLE001 - any failure must stop the motors
            self.get_logger().error(f'control loop error, sending zeros: {exc!r}')
            self._send_zeros()

    def _send(self, output: DriveOutput):
        self._transport.send(output.commands)
        self._publish_debug(output)

    def _send_zeros(self):
        try:
            self._send(self._core.zero_output())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'failed to send zeros: {exc!r}')

    def _publish_debug(self, output: DriveOutput):
        msg = WheelCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wheel_names = list(WHEEL_NAMES)
        msg.vx, msg.vy, msg.wz = (float(v) for v in output.twist)
        msg.omega_requested = [float(v) for v in output.omega_requested]
        msg.omega_sent = [float(v) for v in output.omega_sent]
        msg.saturation_factor = float(output.saturation_factor)
        msg.motor_id = [int(c.motor_id) for c in output.commands]
        msg.direction = [int(c.direction) for c in output.commands]
        msg.value = [float(c.value) for c in output.commands]
        msg.timed_out = bool(output.timed_out)
        self._debug_pub.publish(msg)

    def stop(self):
        """Send zeros and close the transport. Safe to call more than once."""
        if self._stopped:
            return
        self._stopped = True
        self._timer.cancel()
        self.get_logger().info('stopping: sending zeros')
        self._send_zeros()
        try:
            self._transport.close()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'failed to close transport: {exc!r}')


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves: the handlers only set a flag, the loop exits between
    # spin_once calls, and the ROS context is still valid while the final zero command is
    # published. (Raising from a signal handler can interrupt rclpy mid-call and hang node
    # destruction.) Short spin timeouts keep the loop responsive even when no timer fires,
    # e.g. on sim time before /clock starts.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    signal.signal(signal.SIGTERM, lambda signum, frame: stop.set())
    node = None
    try:
        node = Esp32DriverNode()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except NotImplementedError as exc:
        print(f'[esp32_driver] FATAL: {exc}', flush=True)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
