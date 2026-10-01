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
Simulated ESP32 + motors.

Subscribes to the driver's sim transport (mecanum_interfaces/MotorCommands), maps each
motor ID to a wheel joint through a configurable wiring table, applies the per-wheel plant
model and publishes the wheel speed (std_msgs/Float64, rad/s) on /sim/wheel_cmd/<wheel>,
which ros_gz_bridge forwards to the Gazebo JointController of that wheel.

Like the real firmware (as far as is known), it holds the last command until a new one
arrives. An optional watchdog (``watchdog_timeout`` > 0) stops all wheels when commands
stop; it is disabled by default so that the driver's own timeout is what gets tested.
"""

import signal
import threading

from mecanum_interfaces.msg import MotorCommands
from mecanum_simulation.plant_model import MotorPlant
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class SimMotorPlant(Node):

    def __init__(self):
        super().__init__('sim_motor_plant')
        self.declare_parameter('motor_commands_topic', '/esp32/motor_commands')
        self.declare_parameter('wheel_command_prefix', '/sim/wheel_cmd/')
        self.declare_parameter('watchdog_timeout', 0.0)
        # wiring.<wheel> = ESP32 motor ID that drives this wheel.
        self._wiring = {}
        self._plants = {}
        for index, wheel in enumerate(WHEELS):
            self.declare_parameter(f'wiring.{wheel}', index)
            self.declare_parameter(f'plant.{wheel}.sign', 1)
            self.declare_parameter(f'plant.{wheel}.scale', 1.0 / 15.0)
            self.declare_parameter(f'plant.{wheel}.deadband', 0.0)
            self.declare_parameter(f'plant.{wheel}.max_omega', 15.0)
            self._wiring[int(self.get_parameter(f'wiring.{wheel}').value)] = wheel
            self._plants[wheel] = MotorPlant(
                sign=int(self.get_parameter(f'plant.{wheel}.sign').value),
                scale=float(self.get_parameter(f'plant.{wheel}.scale').value),
                deadband=float(self.get_parameter(f'plant.{wheel}.deadband').value),
                max_omega=float(self.get_parameter(f'plant.{wheel}.max_omega').value))
        if len(self._wiring) != len(WHEELS):
            raise ValueError(f'wiring must map four distinct motor IDs, got {self._wiring}')

        prefix = self.get_parameter('wheel_command_prefix').value
        self._pubs = {wheel: self.create_publisher(Float64, prefix + wheel, 10)
                      for wheel in WHEELS}
        self.create_subscription(
            MotorCommands, self.get_parameter('motor_commands_topic').value,
            self._on_commands, 10)

        self._last_rx = None
        self._watchdog = float(self.get_parameter('watchdog_timeout').value)
        if self._watchdog > 0.0:
            self.create_timer(self._watchdog / 2.0, self._on_watchdog)
        self.get_logger().info(
            f'sim_motor_plant ready: wiring {self._wiring}, '
            f'watchdog {"off" if self._watchdog <= 0.0 else f"{self._watchdog:.2f} s"}')

    def _on_commands(self, msg):
        if not (len(msg.motor_id) == len(msg.direction) == len(msg.value)):
            self.get_logger().error('malformed MotorCommands (array lengths differ), ignored')
            return
        self._last_rx = self.get_clock().now()
        for motor_id, direction, value in zip(msg.motor_id, msg.direction, msg.value):
            wheel = self._wiring.get(int(motor_id))
            if wheel is None:
                self.get_logger().warn(f'no wheel wired to motor ID {motor_id}',
                                       throttle_duration_sec=2.0)
                continue
            self._publish(wheel, self._plants[wheel].omega(int(direction), float(value)))

    def _on_watchdog(self):
        if self._last_rx is None:
            return
        age = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if age > self._watchdog:
            for wheel in WHEELS:
                self._publish(wheel, 0.0)

    def _publish(self, wheel, omega):
        self._pubs[wheel].publish(Float64(data=float(omega)))


def main(args=None):
    # SIGINT/SIGTERM only set a flag; the loop below exits between spin_once calls. Raising
    # from the signal handler can interrupt rclpy mid-call and hang node destruction.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    signal.signal(signal.SIGTERM, lambda signum, frame: stop.set())
    node = None
    try:
        node = SimMotorPlant()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
