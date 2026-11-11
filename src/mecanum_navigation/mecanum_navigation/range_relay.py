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
Relay the ultrasonic ranges to the local costmap's range layer (decision D6-a, 0005).

    /ultrasonic/<pos> --range_relay--> /costmap/ultrasonic/<pos>

Nav2's RangeSensorLayer drops a reading outside [min_range, max_range], so a cone with no echo
(+inf, REP 117) would never clear what the layer marked before. The relay turns +inf into a
reading at max_range; with clear_on_max_reading the layer then clears the cone.

``mark_max_range`` (m, 0 = off) caps what the layer may mark: a reading beyond it becomes a
max-range reading at the cap, which clears the cone up to the cap and marks nothing. It is the
D6-a fallback if the wide cones put phantom obstacles in doorways.

The stamp and the frame are the sensor's; the node needs no clock and stays off /clock.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


def relay_range(msg, mark_max_range=0.0):
    """Return the Range for the costmap: no echo becomes max range, optionally capped."""
    out = Range()
    out.header = msg.header
    out.radiation_type = msg.radiation_type
    out.field_of_view = msg.field_of_view
    out.min_range = msg.min_range
    out.max_range = msg.max_range
    reading = msg.range
    if math.isnan(reading):
        return None
    if mark_max_range > 0.0 and mark_max_range < out.max_range:
        out.max_range = float(mark_max_range)
    if math.isinf(reading) and reading > 0.0:
        reading = out.max_range
    elif math.isinf(reading):
        # -inf: an object closer than the sensor can measure (REP 117).
        reading = out.min_range
    out.range = float(min(max(reading, out.min_range), out.max_range))
    return out


class RangeRelay(Node):
    """Relay each ultrasonic topic to its costmap topic."""

    def __init__(self):
        super().__init__('range_relay')
        self.declare_parameter('sensors', ['front_left', 'front_right', 'rear_left',
                                           'rear_right'])
        self.declare_parameter('input_prefix', '/ultrasonic')
        self.declare_parameter('output_prefix', '/costmap/ultrasonic')
        self.declare_parameter('mark_max_range', 0.0)
        self._cap = float(self.get_parameter('mark_max_range').value)
        inp = self.get_parameter('input_prefix').value
        outp = self.get_parameter('output_prefix').value
        self._subs = []
        for name in self.get_parameter('sensors').value:
            # Reliable out: it matches a reliable or a best-effort subscriber.
            pub = self.create_publisher(Range, f'{outp}/{name}', 10)
            self._subs.append(self.create_subscription(
                Range, f'{inp}/{name}', lambda msg, pub=pub: self._on_range(msg, pub),
                qos_profile_sensor_data))

    def _on_range(self, msg, pub):
        out = relay_range(msg, self._cap)
        if out is not None:
            pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RangeRelay()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
