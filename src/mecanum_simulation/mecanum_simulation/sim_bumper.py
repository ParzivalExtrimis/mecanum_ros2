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
Simulated bumper switches: Gazebo plank contacts in, std_msgs/Bool out.

Gazebo's contact sensor publishes only while a plank touches something, so "not pressed" is the
absence of messages. This node turns that into the level signal of a microswitch:

    /sim/bumper/<pos> (ros_gz_interfaces/Contacts) -> /bumper/<pos> (Bool, 20 Hz)

A plank counts as pressed until ``hold_time`` after its last contact, which also rides over the
contact flicker of a plank pushed against a wall.

The node runs on the wall clock: Bool has no stamp, and following /clock (500 Hz) would take half
a core in Python. The hold time and the rate are therefore wall time, the same thing at the
real-time factor of 1 the simulation runs at.

Simulation only: on the robot the sensor hub reports the switches on the same topics.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool


def pressed(last_contact, now, hold_time):
    """Return whether a contact at time ``last_contact`` (None: never) still holds at ``now``."""
    return last_contact is not None and now - last_contact <= hold_time


class SimBumper(Node):
    """Publish the state of each bumper plank at a fixed rate."""

    def __init__(self):
        super().__init__('sim_bumper')
        self.declare_parameter('planks', ['front', 'rear'])
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('hold_time', 0.1)  # STAND-IN: the switch's release time

        self._hold_time = float(self.get_parameter('hold_time').value)
        planks = self.get_parameter('planks').value
        self._last_contact = {plank: None for plank in planks}
        self._publishers_by_plank = {}
        # Kept so the subscriptions stay alive for the life of the node.
        self._subscriptions_by_plank = {}
        for plank in planks:
            self._publishers_by_plank[plank] = self.create_publisher(
                Bool, f'/bumper/{plank}', 10)
            self._subscriptions_by_plank[plank] = self.create_subscription(
                Contacts, f'/sim/bumper/{plank}',
                lambda msg, name=plank: self._on_contacts(msg, name),
                qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._publish)
        self.get_logger().info(f'/sim/bumper/* -> /bumper/* for {planks}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_contacts(self, msg, plank):
        if msg.contacts:
            self._last_contact[plank] = self._now()

    def _publish(self):
        now = self._now()
        for plank, publisher in self._publishers_by_plank.items():
            publisher.publish(Bool(data=pressed(self._last_contact[plank], now, self._hold_time)))


def main(args=None):
    rclpy.init(args=args)
    node = SimBumper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
