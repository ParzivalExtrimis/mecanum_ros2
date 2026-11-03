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
The safety gate between every velocity source and the drive controller (docs/plan.md Phase 4).

    /cmd_vel (TwistStamped, every source) -> gate -> /cmd_vel_safe (TwistStamped, controller)

Each command is forwarded with its stamp, or replaced with zero when a block forbids it
(mecanum_safety/safety_logic.py has the rules). When a new block appears, a zero command goes
out at once. When the sources go quiet the gate publishes nothing, so the controller's
reference timeout still applies.

Inputs: /cliff/<pos>, /bumper/<front|rear>, /estop (transient local) and, for the watchdogs
only, /scan, /l2/points_filtered, /d435i/depth/color/points, /ultrasonic/<pos>,
/joint_states and /odom. Each watch_* parameter turns off the watchdog of a sensor that is not
fitted (robot.launch.py's use_* arguments); a sensor that is not watched gives no protection.

The node runs on the wall clock and its watchdogs on the steady clock, never on sim time: the
simulation's one bridge also carries /clock, and a watchdog on a stopped clock would never fire.
Following /clock (500 Hz) would also cost most of a core in Python. A zero command that
replaces a source's command keeps that command's stamp, which the controller checks against
its own clock; one sent on a new block carries the last command's stamp.

Outputs: /cmd_vel_safe, /safety/state (mecanum_interfaces/SafetyState, 10 Hz and on change).
Service ~/reset (std_srvs/Trigger) clears a latched e-stop once /estop is false.
"""

import time

from geometry_msgs.msg import TwistStamped
from mecanum_interfaces.msg import SafetyState
from mecanum_safety.safety_logic import decide, EStop, Layout, Twist2D, Watchdog
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, Range
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')
PLANKS = ('front', 'rear')

# Watched topics that stop all motion: parameter, topic, type, reliable QoS (the big clouds are
# published reliable, and lose fragments as best effort).
FULL_STOP = (
    ('watch_lidar_2d', '/scan', LaserScan, False),
    ('watch_l2', '/l2/points_filtered', PointCloud2, True),
    ('watch_d435i', '/d435i/depth/color/points', PointCloud2, True),
    ('watch_joint_states', '/joint_states', JointState, False),
    ('watch_odom', '/odom', Odometry, False),
)


def topic_of(kind, name):
    return f'/{kind}/{name}'


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        p = self.declare_parameter
        self.cmd_out = self.create_publisher(
            TwistStamped, p('cmd_out_topic', '/cmd_vel_safe').value, 10)
        self.state_pub = self.create_publisher(SafetyState, '/safety/state', 10)
        self.floor_distance = p('floor_distance', 0.119).value
        self.cliff_margin = p('cliff_margin', 0.05).value
        cliff_x, cliff_y = p('cliff_x', 0.3535).value, p('cliff_y', 0.2078).value
        bumper_x = p('bumper_x', 0.3915).value
        front_x = p('ultrasonic_front_x', 0.408).value
        rear_x = p('ultrasonic_rear_x', -0.401).value
        half_width = p('half_width', 0.255).value
        watch = {name: p(name, True).value for name in (
            'watch_lidar_2d', 'watch_l2', 'watch_d435i', 'watch_joint_states', 'watch_odom',
            'watch_ultrasonics', 'watch_cliff_tof', 'watch_bumpers')}
        timeout = {name: p(f'timeout_{name}', value).value for name, value in (
            ('lidar_2d', 0.5), ('l2', 1.0), ('d435i', 0.5), ('joint_states', 0.2),
            ('odom', 0.2), ('ultrasonics', 0.5), ('cliff_tof', 0.25), ('bumpers', 0.25))}

        sign = {'front': 1.0, 'rear': -1.0}
        side = {'left': 1.0, 'right': -1.0}
        self.layout = Layout(
            cliffs={c: (sign[c.split('_')[0]] * cliff_x, side[c.split('_')[1]] * cliff_y)
                    for c in CORNERS} if watch['watch_cliff_tof'] else {},
            bumpers={b: sign[b] * bumper_x for b in PLANKS} if watch['watch_bumpers'] else {},
            ultrasonics={u: front_x if u.startswith('front') else rear_x
                         for u in CORNERS} if watch['watch_ultrasonics'] else {},
            half_width=half_width)

        timeouts = {}
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for param, topic, msg_type, big in FULL_STOP:
            if watch[param]:
                timeouts[topic] = timeout[param[len('watch_'):]]
                # Raw: only the arrival counts, so the clouds are never deserialized.
                self.create_subscription(
                    msg_type, topic, lambda _, t=topic: self._feed(t),
                    reliable if big else qos_profile_sensor_data, raw=True)
        self.full_stop_topics = tuple(timeouts)
        for name in self.layout.ultrasonics:
            topic = topic_of('ultrasonic', name)
            timeouts[topic] = timeout['ultrasonics']
            self.create_subscription(Range, topic, lambda _, t=topic: self._feed(t),
                                     qos_profile_sensor_data, raw=True)
        self.cliff_ranges = {}
        for name in self.layout.cliffs:
            topic = topic_of('cliff', name)
            timeouts[topic] = timeout['cliff_tof']
            self.create_subscription(
                Range, topic, lambda msg, n=name, t=topic: self._on_cliff(n, t, msg),
                qos_profile_sensor_data)
        self.bumpers = {}
        for name in self.layout.bumpers:
            topic = topic_of('bumper', name)
            timeouts[topic] = timeout['bumpers']
            self.create_subscription(
                Bool, topic, lambda msg, n=name, t=topic: self._on_bumper(n, t, msg),
                qos_profile_sensor_data)
        self.watchdog = Watchdog(timeouts)
        self.estop = EStop()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/estop', self._on_estop, latched)
        self.create_subscription(TwistStamped, p('cmd_in_topic', '/cmd_vel').value,
                                 self._on_cmd, 10)
        self.create_service(Trigger, '~/reset', self._on_reset)

        self.decision = None
        self.last_cmd = None
        self.last_stamp = None
        self.blocking = False
        # Steady: keeps checking the watchdogs when /clock stops.
        self.create_timer(1.0 / p('state_rate', 10.0).value, self._tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._update()
        self.get_logger().info(f'Gating /cmd_vel -> {self.cmd_out.topic_name}; watching '
                               f'{len(timeouts)} topics')

    def _feed(self, topic):
        self.watchdog.feed(topic, time.monotonic())

    def _on_cliff(self, name, topic, msg):
        self.cliff_ranges[name] = msg.range
        self._feed(topic)
        self._update()

    def _on_bumper(self, name, topic, msg):
        self.bumpers[name] = msg.data
        self._feed(topic)
        self._update()

    def _on_estop(self, msg):
        self.estop.update(msg.data)
        self._update()

    def _on_reset(self, _, response):
        response.success, response.message = self.estop.reset()
        self.get_logger().info(response.message)
        self._update()
        return response

    def _tick(self):
        self._update(publish_state=True)

    def _update(self, publish_state=False):
        """Recompute the decision; on any change, publish the state and stop if now blocked."""
        decision = decide(
            self.layout, estop_latched=self.estop.latched,
            lost=self.watchdog.lost(time.monotonic()), full_stop_topics=self.full_stop_topics,
            cliff_ranges=self.cliff_ranges, floor_distance=self.floor_distance,
            cliff_margin=self.cliff_margin, bumpers_pressed=self.bumpers, topic_of=topic_of)
        changed = self.decision is None or decision.reasons != self.decision.reasons
        self.decision = decision
        if changed:
            if decision.reasons:
                self.get_logger().warn(f'Blocked: {", ".join(decision.reasons)}')
            else:
                self.get_logger().info('Clear')
            # A new block stops a command already in force, without waiting for the next one.
            if self.last_cmd is not None and decision.gate(self.last_cmd) != self.last_cmd:
                self._publish(Twist2D(), self.last_stamp)
        if changed or publish_state:
            state = SafetyState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.estop_latched = self.estop.latched
            state.stopped = decision.stopped
            state.reasons = list(decision.reasons)
            self.state_pub.publish(state)

    def _on_cmd(self, msg):
        t = msg.twist
        cmd = Twist2D(t.linear.x, t.linear.y, t.angular.z)
        self.last_cmd = cmd
        self.last_stamp = msg.header.stamp
        out = self.decision.gate(cmd)
        if out == cmd:
            self.cmd_out.publish(msg)
        else:
            self._publish(out, msg.header.stamp, msg.header.frame_id)

    def _publish(self, cmd, stamp, frame_id='base_footprint'):
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = cmd.vx, cmd.vy, cmd.wz
        self.cmd_out.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    # The events executor costs about a tenth of the default one per message. With some 300
    # messages a second, the default executor took a third of a core.
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
