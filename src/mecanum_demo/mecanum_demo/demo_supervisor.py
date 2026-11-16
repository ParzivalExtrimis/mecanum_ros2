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
The booth demo's supervisor: decides everything the panel, the TV and the controller show.

    ros2 run mecanum_demo demo_supervisor --ros-args -p config:=<booth_demo.yaml>

The decisions are in demo_logic.py; this node feeds it and carries them out.

Publishes
    /supervisor/state  mecanum_interfaces/SupervisorState, 10 Hz and on every change,
                       reliable and transient local
    /demo/plan         nav_msgs/Path: the base moves still ahead, for the TV
    /cmd_vel           TwistStamped, Manual driving only (the segment driver drives the rest)
    /safety/bypass     std_msgs/Bool, 5 Hz while in Manual with the safety checks off (the
                       default): the safety gate then stops only for the e-stop. It expires in
                       the gate when no longer refreshed; false is sent at once on leaving
    /demo/hold_points  sensor_msgs/PointCloud2 (map frame): the points the hold detector counts
                       now, empty when none, to see what a hold is about in RViz
    /initialpose       once at start: the robot is at home (initial_pose_at_home), with a tight
                       spread, for AMCL
Serves
    /supervisor/command                 mecanum_interfaces/SupervisorCommand
    /supervisor/dev/arm_checks_pass     std_srvs/SetBool, a development switch: the teleop
                                        gate's arm checks pass, so that the live and ended
                                        screens can be shown without an arm
Uses
    /demo/move_segment  the segment driver's action, one goal per base move
    TF map -> base_footprint and the sensors' frames
    /map                the static map, for the hold detector
    /scan, /l2/points_filtered   the hold detector's points
    /odom               whether the base moves (the gate's "base parked" check)
    /estop              std_msgs/Bool, transient local: the hardware e-stop
    /safety/state       the safety gate; its e-stop latch is cleared through
                        /safety_supervisor/reset when staff choose what to do after a release
    /panel/drive        geometry_msgs/Twist from the panel's D-pad (20 Hz while held). It is
                        restamped and passed on to /cmd_vel only in Manual driving, while the
                        panel's heartbeat is fresh, and stops 200 ms after the last message
    /panel/heartbeat    std_msgs/Header, 2 Hz from the panel; missing for heartbeat_timeout
                        stops the D-pad's Manual driving
    /manual/cmd_vel     TwistStamped from an external controller (teleop_twist_joy,
                        teleop_twist_keyboard): restamped and passed on to /cmd_vel only in
                        Manual driving, with the same speed caps but without the panel's
                        heartbeat or expiry; the drive controller's reference timeout still
                        stops the base when the stream stops. Ignored outside Manual
    /joy                sensor_msgs/Joy from a controller: button joy_stop_button sends STOP,
                        straight to this node, so it works when the panel or the bridge is down

Clocks: the logic's timers (hold clearance, gate fallback, live idle) and the command stamps
run on the node's clock, sim time in sim. The panel's heartbeat and drive expiry are on the
steady clock, because the panel lives on wall time.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist,
                               TwistStamped)
from mecanum_demo.demo_logic import DemoLogic
from mecanum_demo.hold_detector import scan_points, StaticMap, unexplained_mask
from mecanum_demo.ros_util import cloud_xyz, load_config, pose2d_of, transform_points
from mecanum_interfaces.action import MoveSegment
from mecanum_interfaces.msg import FeatureStatus, GateCheck, SafetyState, SupervisorState
from mecanum_interfaces.srv import SupervisorCommand
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Joy, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def default_config():
    return os.path.join(get_package_share_directory('mecanum_demo'), 'config',
                        'booth_demo.yaml')


class DemoSupervisor(Node):

    def __init__(self):
        super().__init__('demo_supervisor')
        p = self.declare_parameter
        self.cfg = load_config(p('config', default_config()).value)
        self.logic = DemoLogic(self.cfg)
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.hold_cloud_band = (p('hold_cloud_min_z', 0.10).value,
                                p('hold_cloud_max_z', 1.80).value)
        self.hold_map_margin = float(self.cfg.get('hold_map_margin', 0.10))
        self.hold_min_points = int(self.cfg.get('hold_min_points', 3))
        self.hold_confirm_s = float(self.cfg.get('hold_confirm_s', 0.0))
        self.hold_max_age = p('hold_max_age', 1.0).value
        self.heartbeat_timeout = p('heartbeat_timeout', 1.0).value
        self.drive_timeout = p('drive_timeout', 0.2).value
        # STAND-IN: the Manual speed caps and the controller's STOP button (index 1 is B on an
        # Xbox-layout pad); set them for the real controller.
        self.drive_caps = (p('drive_max_vx', 0.2).value, p('drive_max_vy', 0.2).value,
                           p('drive_max_wz', 0.6).value)
        self.joy_stop_button = p('joy_stop_button', 1).value
        self.moving_speed = p('moving_speed', 0.02).value
        # The robot starts every demo day on the home tape: tell AMCL so, with a tight spread,
        # instead of its default 0.5 m one, which in the 2 x 2 m booth can settle a few cm off.
        self.start_at_home = p('initial_pose_at_home', True).value
        self.initial_sigma = (p('initial_sigma_xy', 0.02).value,
                              p('initial_sigma_yaw', 0.035).value)
        self.initial_sent = False

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.static_map = None
        self.scan_pts = (None, 0.0)      # (Nx2 map points, steady time)
        self.cloud_pts = (None, 0.0)
        self.drive = (Twist(), -math.inf)
        self.heartbeat_at = -math.inf
        self.driving = False
        self.detect_since = None         # ROS time the current detection started
        self.evidence = None             # what the detector last counted, for the log
        self.bypass_sent = False
        self.external_refused_at = -math.inf
        self.joy_pressed = False
        self.safety = None

        self.state_pub = self.create_publisher(SupervisorState, '/supervisor/state', LATCHED)
        self.plan_pub = self.create_publisher(Path, '/demo/plan', LATCHED)
        self.cmd_pub = self.create_publisher(TwistStamped, p('cmd_topic', '/cmd_vel').value, 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.bypass_pub = self.create_publisher(Bool, '/safety/bypass', 10)
        self.hold_points_pub = self.create_publisher(PointCloud2, '/demo/hold_points', 10)
        self.create_service(SupervisorCommand, '/supervisor/command', self._on_command)
        self.create_service(SetBool, '/supervisor/dev/arm_checks_pass', self._on_arm_switch)
        self.safety_reset = self.create_client(Trigger, '/safety_supervisor/reset')
        self.mover = ActionClient(self, MoveSegment, '/demo/move_segment')
        self.goal_handle = None
        self.goal_id = None

        self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED)
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self._on_scan,
                                 qos_profile_sensor_data)
        cloud = p('hold_cloud_topic', '/l2/points_filtered').value
        if cloud:
            self.create_subscription(PointCloud2, cloud, self._on_cloud, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom').value, self._on_odom, 10)
        self.create_subscription(Bool, '/estop', self._on_estop, LATCHED)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 10)
        self.create_subscription(Twist, '/panel/drive', self._on_drive, 10)
        self.create_subscription(Header, '/panel/heartbeat', self._on_heartbeat, 10)
        self.create_subscription(TwistStamped, '/manual/cmd_vel', self._on_manual_cmd, 10)
        self.create_subscription(Joy, p('joy_topic', '/joy').value, self._on_joy, 10)

        self.last_tick = None
        self.last_published = None
        self.create_timer(1.0 / p('state_rate', 10.0).value, self._tick)
        self.create_timer(1.0 / p('drive_rate', 20.0).value, self._drive_tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.2, self._bypass_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Demo supervisor ready')

    # ----- inputs -------------------------------------------------------------------------

    def _lookup(self, frame, stamp=None):
        # map <- frame at stamp (a sensor's capture time), else the latest. A moving robot's
        # sensor data must be placed with the pose it was taken from: at 0.25 m/s, 0.2 s of
        # difference puts a wall 5 cm off, close enough to look like a person.
        if stamp is not None:
            try:
                return self.tf.lookup_transform(self.map_frame, frame, Time.from_msg(stamp))
            except TransformException:
                pass
        try:
            return self.tf.lookup_transform(self.map_frame, frame, Time())
        except TransformException:
            return None

    def _on_map(self, msg):
        info = msg.info
        self.static_map = StaticMap.from_occupancy_grid(
            info.width, info.height, info.resolution,
            (info.origin.position.x, info.origin.position.y), msg.data,
            zones=list(self.cfg.get('static_zones', {}).values()))
        self.get_logger().info(f'Map {info.width} x {info.height} for the hold detector')

    def _on_scan(self, msg):
        t = self._lookup(msg.header.frame_id, msg.header.stamp)
        if t is None:
            return
        self.scan_pts = (scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max, pose2d_of(t)),
                         time.monotonic())

    def _on_cloud(self, msg):
        t = self._lookup(msg.header.frame_id, msg.header.stamp)
        if t is None:
            return
        points = transform_points(cloud_xyz(msg)[::2], t)
        low, high = self.hold_cloud_band
        points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
        self.cloud_pts = (points[:, :2], time.monotonic())

    def _on_odom(self, msg):
        t = msg.twist.twist
        self.logic.set_base_moving(math.hypot(t.linear.x, t.linear.y) > self.moving_speed
                                   or abs(t.angular.z) > 2.5 * self.moving_speed)

    def _on_estop(self, msg):
        before = self.logic.state
        self.logic.set_estop(msg.data)
        if self.logic.state != before:
            self._sync_motion()
            self._publish_state()

    def _on_safety(self, msg):
        self.safety = msg

    def _on_drive(self, msg):
        self.drive = (msg, time.monotonic())

    def _on_heartbeat(self, msg):
        self.heartbeat_at = time.monotonic()

    def _on_manual_cmd(self, msg):
        """Pass an external controller's command on, in Manual driving only."""
        if not self.logic.manual_drive_allowed():
            now = time.monotonic()
            if now - self.external_refused_at > 5.0:
                self.external_refused_at = now
                self.get_logger().info('External drive ignored: not in Manual driving')
            return
        t = msg.twist
        vx, vy, wz = self.drive_caps
        self.publish_cmd(max(-vx, min(vx, t.linear.x)), max(-vy, min(vy, t.linear.y)),
                         max(-wz, min(wz, t.angular.z)))

    def _on_joy(self, msg):
        index = self.joy_stop_button
        pressed = 0 <= index < len(msg.buttons) and msg.buttons[index] == 1
        if pressed and not self.joy_pressed:
            self.get_logger().info('STOP from the controller')
            self._command('stop', [])
        self.joy_pressed = pressed

    def _on_arm_switch(self, request, response):
        self.logic.set_arm_checks_pass(request.data)
        response.success = True
        response.message = f'Arm checks {"pass" if request.data else "fail"} (development)'
        self._publish_state()
        return response

    def _on_command(self, request, response):
        response.request_id = request.request_id
        response.accepted, response.reason = self._command(request.action, list(request.args))
        return response

    def _command(self, action, args):
        was = self.logic.state
        self._update_pose()
        accepted, reason = self.logic.command(action, args)
        text = f'{action} {" ".join(args)}'.strip()
        if accepted:
            self.get_logger().info(f'{text}: accepted')
        else:
            self.get_logger().warn(f'{text}: refused: {reason}')
        if accepted and was == 'stopped':
            self._clear_safety_latch()
        if accepted and (action == 'stop'
                         or (was == 'manual' and self.logic.state != 'manual')):
            self.publish_cmd(0.0, 0.0, 0.0)
        self._bypass_tick()
        self._sync_motion()
        self._flush_log()
        self._publish_state()
        return accepted, reason

    def _clear_safety_latch(self):
        """After an e-stop release, staff chose what to do: clear the gate's latch."""
        if self.safety is not None and self.safety.estop_latched and \
                self.safety_reset.service_is_ready():
            self.safety_reset.call_async(Trigger.Request())

    # ----- the loop -----------------------------------------------------------------------

    def _tick(self):
        now = self.get_clock().now()
        dt = 0.0 if self.last_tick is None else (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        pose = self._update_pose()
        if pose is not None and not self.initial_sent and self.start_at_home:
            self._send_initial_pose()
        was = self.logic.state
        self.logic.set_person_close(self._someone_close(pose, now))
        self.logic.tick(dt)
        if self.logic.state == 'hold' and was != 'hold':
            self.get_logger().info('Hold evidence: ' + self._evidence_text())
        self._sync_motion()
        self._flush_log()
        self._publish_state()
        self._publish_plan(pose)

    def _send_initial_pose(self):
        """Once localization runs (map -> base_footprint exists): start AMCL at home."""
        if self.initial_pub.get_subscription_count() == 0:
            return
        x, y = self.cfg['poses']['home']
        yaw = self.cfg['headings'][self.cfg['features']['go_home']['home_heading']]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        sxy, syaw = self.initial_sigma
        msg.pose.covariance[0] = msg.pose.covariance[7] = sxy * sxy
        msg.pose.covariance[35] = syaw * syaw
        self.initial_pub.publish(msg)
        self.initial_sent = True
        self.get_logger().info(f'Told localization the robot is at home ({x:.2f}, {y:.2f})')

    def _update_pose(self):
        t = self._lookup(self.base_frame)
        pose = pose2d_of(t) if t is not None else None
        self.logic.set_pose(pose)
        return pose

    def _someone_close(self, pose, now):
        """Return whether someone has been close for hold_confirm_s; publish what counts."""
        seen = self._detect(pose)
        if not seen:
            self.detect_since = None
            return False
        if self.detect_since is None:
            self.detect_since = now
        return (now - self.detect_since).nanoseconds * 1e-9 >= self.hold_confirm_s

    def _detect(self, pose):
        fresh = time.monotonic() - self.hold_max_age
        counted, sources = [], []
        nearest = math.inf
        if pose is not None:
            for name, (pts, at) in (('C1', self.scan_pts), ('L2', self.cloud_pts)):
                if pts is None or at < fresh or not len(pts):
                    continue
                mask, distance = unexplained_mask(pts, pose, self.cfg['footprint'],
                                                  self.cfg['hold_radius'], self.static_map,
                                                  self.hold_map_margin)
                if mask.any():
                    counted.append(pts[mask])
                    sources.append(f'{name} {int(mask.sum())}')
                    nearest = min(nearest, float(distance[mask].min()))
        points = np.vstack(counted) if counted else np.zeros((0, 2))
        self._publish_hold_points(points)
        self.evidence = (len(points), nearest, sources,
                         points.mean(axis=0) if len(points) else None)
        return len(points) >= self.hold_min_points

    def _evidence_text(self):
        if self.evidence is None or not self.evidence[0]:
            return 'none'
        count, nearest, sources, centre = self.evidence
        return (f'{count} points ({", ".join(sources)}), nearest {nearest:.2f} m from the '
                f'outline, around ({centre[0]:.2f}, {centre[1]:.2f})')

    def _publish_hold_points(self, points):
        if not len(points) and self.evidence is not None and not self.evidence[0]:
            return      # still empty: nothing new to show
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        xyz = np.column_stack([points, np.full(len(points), 0.2)]) if len(points) else []
        self.hold_points_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz))

    def _bypass_tick(self):
        """Refresh the safety gate's bypass while Manual runs with its checks off."""
        if self.logic.safety_bypass():
            self.bypass_pub.publish(Bool(data=True))
            self.bypass_sent = True
        elif self.bypass_sent:
            self.bypass_pub.publish(Bool(data=False))
            self.bypass_sent = False

    def _sync_motion(self):
        """Make the segment driver run exactly the base move the logic wants."""
        wanted = self.logic.motion
        wanted_id = wanted['id'] if wanted else None
        if wanted_id == self.goal_id:
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_id = wanted_id
        if wanted is None:
            return
        if not self.mover.server_is_ready():
            self.logic.segment_result(wanted_id, False, 'unavailable',
                                      'The segment driver is not running')
            self.goal_id = None
            self._sync_motion()
            return
        goal = MoveSegment.Goal()
        goal.kind = wanted['kind']
        goal.target = Pose2D(x=float(wanted['x']), y=float(wanted['y']),
                             theta=float(wanted['yaw']))
        goal.keep_heading = bool(wanted['keep_heading'])
        goal.delta_yaw = float(wanted['delta_yaw'])
        goal.slow = bool(wanted['slow'])
        goal.label = wanted['label']
        future = self.mover.send_goal_async(
            goal, feedback_callback=lambda fb, i=wanted_id: self.logic.set_segment_remaining(
                i, fb.feedback.remaining_s))
        future.add_done_callback(lambda f, i=wanted_id: self._on_goal_response(i, f))

    def _on_goal_response(self, motion_id, future):
        handle = future.result()
        if motion_id != self.goal_id:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.logic.segment_result(motion_id, False, 'rejected', 'Goal rejected')
            self._after_result()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, i=motion_id: self._on_result(i, f))

    def _on_result(self, motion_id, future):
        if motion_id != self.goal_id:
            return
        self.goal_handle = None
        self.goal_id = None
        result = future.result().result
        # The next step may check where the robot is: use the pose it arrived at.
        self._update_pose()
        self.logic.segment_result(motion_id, result.success, result.error, result.message)
        self._after_result()

    def _after_result(self):
        self._sync_motion()
        self._flush_log()
        self._publish_state()

    # ----- Manual driving -----------------------------------------------------------------

    def _drive_tick(self):
        twist, at = self.drive
        now = time.monotonic()
        allowed = (self.logic.manual_drive_allowed()
                   and now - self.heartbeat_at <= self.heartbeat_timeout
                   and now - at <= self.drive_timeout)
        if allowed:
            vx, vy, wz = self.drive_caps
            self.publish_cmd(max(-vx, min(vx, twist.linear.x)),
                             max(-vy, min(vy, twist.linear.y)),
                             max(-wz, min(wz, twist.angular.z)))
            self.driving = True
        elif self.driving:
            self.publish_cmd(0.0, 0.0, 0.0)
            self.driving = False

    def publish_cmd(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    # ----- outputs ------------------------------------------------------------------------

    def _flush_log(self):
        for line in self.logic.log:
            self.get_logger().info(line)
        self.logic.log.clear()

    def _publish_state(self):
        snap = self.logic.snapshot()
        msg = SupervisorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for key in ('state', 'mode', 'mode_label', 'playlist', 'playlist_labels', 'index',
                    'feature', 'step_label', 'step_is_arm', 'eta_s', 'eta_plus_teleop', 'loop',
                    'loop_count', 'location', 'location_label', 'base_moving', 'arm_stowed',
                    'arm_integrated', 'manual_sub', 'manual_blocked', 'manual_checks',
                    'hold_check', 'hold_clear_s',
                    'hold_clear_needed_s', 'person_close', 'gate_remaining_s',
                    'live_elapsed_s', 'live_idle_s', 'live_idle_limit_s', 'ended_reason',
                    'ended_length_s', 'demo_count', 'last_demo', 'message', 'stop_kind',
                    'can_resume', 'notice', 'allowed_actions'):
            value = snap[key]
            field = type(getattr(msg, key))
            setattr(msg, key, field(value) if field in (int, float, bool, str) else value)
        if snap['pose'] is not None:
            msg.pose = Pose2D(x=float(snap['pose'][0]), y=float(snap['pose'][1]),
                              theta=float(snap['pose'][2]))
            msg.pose_valid = True
        msg.checks = [GateCheck(name=c['name'], ok=c['ok'], detail=c['detail'])
                      for c in snap['checks']]
        msg.features = [FeatureStatus(**f) for f in snap['features']]
        self.state_pub.publish(msg)

    def _publish_plan(self, pose):
        waypoints = self.logic.remaining_waypoints()
        key = (tuple(waypoints), pose is None)
        if not waypoints and self.last_published == key:
            return
        self.last_published = key
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        points = ([pose[:2]] if pose is not None and waypoints else []) + list(waypoints)
        for x, y in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = DemoSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
