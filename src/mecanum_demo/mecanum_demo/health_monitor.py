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
The panel's Health screen: one diagnostic status per part of the robot, on /robot/health.

    ros2 run mecanum_demo health_monitor

Publishes diagnostic_msgs/DiagnosticArray at 1 Hz. Each status is OK, WARN (slow) or ERROR
(silent), judged by message rates on the steady clock; the messages are taken raw, so even
the L2 cloud costs no deserialization.

    2D lidar       /scan                  expected 5 Hz or more
    L2 lidar       /l2/points_filtered    expected 4 Hz or more
    Drive          /joint_states          expected 20 Hz or more; /drive/state faults, when
                                          the real ESP32 drive runs
    Localization   /amcl_pose             AMCL's position spread (it publishes on updates)
    Safety gate    /safety/state          the gate's reasons
    Battery        STAND-IN: not measured; the robot has no battery monitor yet
    Arm CAN links  not integrated (the arm is outside this workspace)
"""

from collections import deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from mecanum_interfaces.msg import DriveState, SafetyState
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan, PointCloud2

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


class Rate:
    """Arrival times of one topic over the last few seconds."""

    def __init__(self, window=3.0):
        self.window = window
        self.times = deque()

    def feed(self):
        self.times.append(time.monotonic())

    def hz(self):
        now = time.monotonic()
        while self.times and now - self.times[0] > self.window:
            self.times.popleft()
        return len(self.times) / self.window


def status(name, level, message, **values):
    return DiagnosticStatus(level=level, name=name, message=message, hardware_id='mecanum',
                            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])


class HealthMonitor(Node):

    def __init__(self):
        super().__init__('health_monitor')
        self.rates = {}
        self.topics = {
            'scan': ('/scan', LaserScan, 5.0, qos_profile_sensor_data),
            'l2': ('/l2/points_filtered', PointCloud2, 4.0, 10),
            'joints': ('/joint_states', JointState, 20.0, 10),
        }
        for key, (topic, kind, _, qos) in self.topics.items():
            self.rates[key] = Rate()
            self.create_subscription(kind, topic, lambda _, k=key: self.rates[k].feed(), qos,
                                     raw=True)
        self.drive_faults = None
        self.rates['drive'] = Rate()
        self.create_subscription(DriveState, '/drive/state', self._on_drive, 10)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.safety = None
        self.safety_at = -math.inf
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 10)
        self.pub = self.create_publisher(DiagnosticArray, '/robot/health', 10)
        self.create_timer(1.0, self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_drive(self, msg):
        self.rates['drive'].feed()
        self.drive_faults = msg.faults

    def _on_amcl(self, msg):
        c = msg.pose.covariance
        self.amcl = (math.sqrt(max(c[0], 0.0) + max(c[7], 0.0)), math.sqrt(max(c[35], 0.0)))

    def _on_safety(self, msg):
        self.safety = msg
        self.safety_at = time.monotonic()

    def _rate_status(self, name, key):
        topic, _, expected, _ = self.topics[key]
        hz = self.rates[key].hz()
        if hz == 0.0:
            return status(name, ERROR, f'No data on {topic}', topic=topic, rate_hz=0)
        level = OK if hz >= expected else WARN
        return status(name, level, f'{hz:.1f} Hz' + ('' if level == OK else ' (slow)'),
                      topic=topic, rate_hz=f'{hz:.1f}')

    def _publish(self):
        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.status.append(status('Battery', WARN, 'Not measured', note='STAND-IN'))
        out.status.append(self._rate_status('2D lidar', 'scan'))
        out.status.append(self._rate_status('L2 lidar', 'l2'))
        drive = self._rate_status('Drive', 'joints')
        if self.rates['drive'].hz() > 0.0 and self.drive_faults:
            drive.level = ERROR
            drive.message = f'Drive faults 0x{self.drive_faults:04x}'
        out.status.append(drive)
        if self.amcl is None:
            out.status.append(status('Localization', WARN, 'No AMCL pose yet'))
        else:
            spread, yaw = self.amcl
            level = OK if spread < 0.10 else WARN
            out.status.append(status('Localization', level,
                                     f'±{spread * 100:.0f} cm, ±{math.degrees(yaw):.0f}°',
                                     spread_m=f'{spread:.3f}', yaw_rad=f'{yaw:.3f}'))
        if self.safety is None or time.monotonic() - self.safety_at > 2.0:
            out.status.append(status('Safety gate', ERROR, 'No /safety/state'))
        elif self.safety.reasons:
            level = ERROR if self.safety.stopped else WARN
            out.status.append(status('Safety gate', level, ', '.join(self.safety.reasons)))
        else:
            out.status.append(status('Safety gate', OK, 'Clear'))
        out.status.append(status('Arm CAN links', WARN, 'Arm not integrated'))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
