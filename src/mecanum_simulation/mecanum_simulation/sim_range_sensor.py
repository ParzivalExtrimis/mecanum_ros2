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
Simulated range sensors: one gpu_lidar cone in, one sensor_msgs/Range out, per sensor.

Gazebo Harmonic has no sonar sensor and ros_gz_bridge has no Range message. Each ultrasonic and
cliff ToF is a small gpu_lidar cone (urdf/mecanum.gazebo.xacro), bridged as its cloud: the
bridge's LaserScan would keep only the middle row of the cone. This node reports the nearest
return of each cone, like the real sensors do:

    /sim/ultrasonic/<pos> (PointCloud2) -> /ultrasonic/<pos> (Range, ULTRASOUND)
    /sim/cliff/<pos> (PointCloud2) -> /cliff/<pos> (Range, INFRARED)

One instance runs per sensor group (config/sim_range_sensors.yaml). The stamp and frame come
from the cloud. A cone without a return gives +inf, the REP 117 value for no detection.

Simulation only: on the robot the sensor drivers publish the same Range topics.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2


RADIATION_TYPES = {'ultrasound': Range.ULTRASOUND, 'infrared': Range.INFRARED}


def cloud_ranges(cloud):
    """Return the distance of every point of a cloud from the sensor; a ray with no hit is inf."""
    points = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'], skip_nans=False)
    return np.linalg.norm(points, axis=1)


def cone_range(ranges, min_range, max_range):
    """Return the nearest finite return clamped to the limits, or +inf if there is none."""
    ranges = np.asarray(ranges, dtype=float)
    hits = ranges[np.isfinite(ranges)]
    if not hits.size:
        return math.inf
    return float(np.clip(hits.min(), min_range, max_range))


class SimRangeSensor(Node):
    """Publish a Range for every cloud of one group of simulated range sensors."""

    def __init__(self):
        super().__init__('sim_range_sensor')
        self.declare_parameter('cloud_prefix', '/sim/ultrasonic')
        self.declare_parameter('range_prefix', '/ultrasonic')
        self.declare_parameter('positions', ['front_left', 'front_right',
                                             'rear_left', 'rear_right'])
        self.declare_parameter('radiation_type', 'ultrasound')
        self.declare_parameter('field_of_view', math.radians(60.0))
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 4.5)

        radiation = self.get_parameter('radiation_type').value
        if radiation not in RADIATION_TYPES:
            raise ValueError(f'radiation_type must be one of {sorted(RADIATION_TYPES)}, '
                             f'got {radiation!r}')
        self._radiation_type = RADIATION_TYPES[radiation]
        self._field_of_view = float(self.get_parameter('field_of_view').value)
        self._min_range = float(self.get_parameter('min_range').value)
        self._max_range = float(self.get_parameter('max_range').value)

        cloud_prefix = self.get_parameter('cloud_prefix').value
        range_prefix = self.get_parameter('range_prefix').value
        positions = self.get_parameter('positions').value
        # Kept so the subscriptions stay alive for the life of the node.
        self._subscriptions_by_position = {}
        for position in positions:
            publisher = self.create_publisher(Range, f'{range_prefix}/{position}', 10)
            self._subscriptions_by_position[position] = self.create_subscription(
                PointCloud2, f'{cloud_prefix}/{position}',
                lambda cloud, pub=publisher: self._on_cloud(cloud, pub),
                qos_profile_sensor_data)
        self.get_logger().info(
            f'{cloud_prefix}/* -> {range_prefix}/* for {len(positions)} sensors')

    def _on_cloud(self, cloud, publisher):
        msg = Range()
        msg.header = cloud.header
        msg.radiation_type = self._radiation_type
        msg.field_of_view = self._field_of_view
        msg.min_range = self._min_range
        msg.max_range = self._max_range
        msg.range = cone_range(cloud_ranges(cloud), self._min_range, self._max_range)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimRangeSensor()
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
