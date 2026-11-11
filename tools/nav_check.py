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

"""
Check navigation in the restroom world (docs/plan.md Phase 6).

    ros2 launch mecanum_bringup navigation.launch.py gui:=false rviz:=false
    tools/nav_check.py                       # every stage, from the robot at home
    tools/nav_check.py --only suite gaps     # some stages

Judged on Gazebo ground truth (shortcut S1). The map frame is placed in the world by a best fit
of the map onto the world file (tools/map_check.py's), so the map's few-cm offset is removed
from every error. Stages, in order:

  suite     Goals across the restroom: the corridor, through the doorway, a pure strafe, into
            the stall (precise checker), the sink, the lobby, home (precise). PASS per goal:
            Nav2 succeeds, the ground-truth error is within the goal checker's tolerance + 5 cm
            (owner, 2026-11-05; the precise goals are reported only), and the robot touches
            nothing. The strafe goal must also be driven sideways (heading change < 15 deg).
  gaps      Goals whose straight line crosses what the 2D map misses: behind the glass, into
            the stall across its partition (both must be reached without contact) and past the
            threshold (must be refused: Nav2 fails and the robot never crosses it).
  obstacles Objects put on the corridor route at run time (Gazebo's create service): a box (go
            around), a low box under the 2D lidar's plane (the D435i), an overhang (the D435i
            and the L2), a box dropped just ahead of the moving robot, a wall across the
            corridor (stop and give up) and unknown glass (decision D6-a: stop before it).
            The ones to go around block the straight route and leave a 1.10 m gap on the north
            side, wider than the 0.9 m doorway. A centred box leaves 0.75 m each side, which
            the 5 cm costmap sees as about 0.65 m for the 0.52 m robot: passing or refusing
            both pass there. PASS: no contact; the passable ones are also reached.
  reverse   A box behind the robot: Nav2's BackUp behaviour, then reverse commands straight
            into the collision monitor (/cmd_vel_smoothed). PASS: no contact.
  relocalize  AMCL is given a start 0.4 m and 25 deg off; PASS if within 10 cm / 3 deg after at
            most 3 m of travel (owner, 2026-11-05).

Throughout: the localization error (map -> base_footprint against ground truth) is sampled at
5 Hz; PASS if at most 10 cm / 3 deg outside the relocalize stage. CPU per process (psutil) and
the real-time factor (ground-truth stamps against the wall clock) are reported only.
Contact is judged in 2D: the footprint at the ground-truth pose against every world shape
between the floor and the robot's top.
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from mecanum_localization import map_compare as mc
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import numpy as np
import psutil
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy.time
from std_msgs.msg import String
import tf2_ros

E, N, W, S = 0.0, math.pi / 2, math.pi, -math.pi / 2
FOOTPRINT = (-0.405, 0.421, -0.258, 0.258)  # x min, x max, y min, y max (mecanum_description)
ROBOT_TOP = 0.90
SCAN_HEIGHT = 0.13
TOLERANCE = {'staging': (0.10, 0.10), 'precise': (0.05, 0.05)}  # nav2.yaml goal checkers
ALLOWANCE = 0.05            # m, owner 2026-11-05: map offset + localization
LOC_LIMIT = (0.10, math.radians(3.0))
STRAFE_YAW_LIMIT = math.radians(15.0)

# (name, world x, y, yaw, goal checker, judged)
SUITE = [
    ('corridor_east', 4.5, -1.0, E, 'staging', True),
    ('doorway_in', 3.45, 1.5, N, 'staging', True),
    ('strafe_right', 4.8, 1.5, N, 'staging', True),      # 1.35 m sideways, same heading
    ('stall', 5.6, 2.95, E, 'precise', False),
    ('sink', 3.3, 2.9, W, 'staging', True),
    ('lobby', -0.3, 1.5, N, 'staging', True),
    ('home', 0.0, -1.0, E, 'precise', False),
]
STRAFE_GOALS = {'strafe_right'}

# Monitored processes for the CPU report: a label and a substring of the command line.
PROCESSES = [
    ('gazebo', 'gz sim'), ('bridge', 'parameter_bridge'),
    ('controller_server', 'controller_server'),
    ('planner_server', 'planner_server'), ('bt_navigator', 'bt_navigator'),
    ('behavior_server', 'behavior_server'), ('velocity_smoother', 'velocity_smoother'),
    ('collision_monitor', 'collision_monitor'), ('amcl', 'nav2_amcl'),
    ('slam_toolbox', 'localization_slam_toolbox'), ('map_server', 'map_server'),
    ('ekf', 'ekf_node'), ('safety_supervisor', 'safety_supervisor'),
    ('range_relay', 'range_relay'), ('sim_depth_cloud', 'sim_depth_cloud'),
    ('sim_range_sensor', 'sim_range_sensor'), ('self_filter', 'self_filter'),
    ('robot_state_publisher', 'robot_state_publisher'), ('nav_check', 'nav_check.py'),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    """Planar pose a * b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def footprint_points(step=0.01):
    """Return the footprint's outline and interior as Nx2 points, base frame."""
    x0, x1, y0, y1 = FOOTPRINT
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    outline = np.vstack([np.column_stack([xs, np.full_like(xs, y0)]),
                         np.column_stack([xs, np.full_like(xs, y1)]),
                         np.column_stack([np.full_like(ys, x0), ys]),
                         np.column_stack([np.full_like(ys, x1), ys])])
    gx, gy = np.meshgrid(np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05))
    return np.vstack([outline, np.column_stack([gx.ravel(), gy.ravel()])])


class Box:
    """An inserted test object, as a world Shape plus its Gazebo SDF."""

    def __init__(self, name, x, y, size, z_low=0.0, hidden=False):
        self.name = name
        self.x, self.y, self.size, self.z_low, self.hidden = x, y, size, z_low, hidden
        self.shape = mc.Shape(name, 'box', x, y, z_low + size[2] / 2.0, 0.0, tuple(size),
                              hidden)

    def sdf(self):
        sx, sy, sz = self.size
        flags = '<visibility_flags>268435456</visibility_flags>' if self.hidden else ''
        transparency = '<transparency>0.7</transparency>' if self.hidden else ''
        geometry = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        return (f'<sdf version="1.9"><model name="{self.name}"><static>true</static>'
                f'<pose>{self.x} {self.y} {self.z_low + sz / 2.0} 0 0 0</pose>'
                f'<link name="link"><collision name="collision">{geometry}</collision>'
                f'<visual name="visual">{geometry}{transparency}{flags}'
                '<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse>'
                '</material></visual></link></model></sdf>')


class Monitor(Node):
    """Ground truth, the localization estimate, contact and the CPU samples."""

    def __init__(self, shapes, world_from_map):
        super().__init__('nav_check_monitor')
        self.shapes = list(shapes)
        self.world_from_map = world_from_map
        self.map_from_world = inverse(world_from_map)
        self.lock = threading.Lock()
        self.gt = None
        self.gt_stamp = None
        self.stage = 'start'
        self.loc_samples = []      # (stage, travelled, position error, yaw error)
        self.loc_poses = []        # (stage, ground truth, estimate in the world)
        self.clearance = {}        # stage or goal -> (min clearance, shape name)
        self.label = 'start'
        self.travelled = 0.0
        self._last_xy = None
        self._points = footprint_points()
        self.rtf_marks = []
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_gt, 10)
        self.create_timer(0.2, self._sample)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_smoothed', 10)
        self.selector_pub = self.create_publisher(String, '/goal_checker_selector', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE))
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def _on_gt(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        with self.lock:
            self.gt = pose
            self.gt_stamp = stamp
            if self._last_xy is not None:
                self.travelled += math.dist(self._last_xy, pose[:2])
            self._last_xy = pose[:2]
            self._contact(pose)

    def _contact(self, pose):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pts = self._points
        world = np.column_stack([pose[0] + c * pts[:, 0] - s * pts[:, 1],
                                 pose[1] + s * pts[:, 0] + c * pts[:, 1]])
        best = self.clearance.get(self.label, (math.inf, ''))
        for shape in self.shapes:
            low, high = shape.z_range
            if high <= 0.001 or low >= ROBOT_TOP:
                continue
            d = float(shape.signed_distance(world).min())
            if d < best[0]:
                best = (d, shape.name)
        self.clearance[self.label] = best

    def estimate(self):
        """Return the map -> base_footprint estimate, or None."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))

    def _sample(self):
        est = self.estimate()
        with self.lock:
            gt = self.gt
            travelled = self.travelled
            if gt is not None:
                self.rtf_marks.append((time.monotonic(), self.gt_stamp))
        if est is None or gt is None:
            return
        est_world = compose(self.world_from_map, est)
        error = math.dist(est_world[:2], gt[:2])
        yaw_error = abs(wrap(est_world[2] - gt[2]))
        with self.lock:
            self.loc_samples.append((self.stage, travelled, error, yaw_error))
            self.loc_poses.append((self.stage, gt, est_world))

    def truth(self):
        with self.lock:
            return self.gt

    def sim_stamp(self):
        """Return the latest simulation time as a message stamp (the node is on wall time)."""
        with self.lock:
            t = self.gt_stamp or 0.0
        return rclpy.time.Time(nanoseconds=int(t * 1e9)).to_msg()

    def truth_in_map(self):
        gt = self.truth()
        return compose(self.map_from_world, gt) if gt else None

    def set_label(self, label, stage=None):
        with self.lock:
            self.label = label
            if stage is not None:
                self.stage = stage

    def select_checker(self, name):
        self.selector_pub.publish(String(data=name))


class CpuSampler(threading.Thread):
    """Sample the CPU use of the monitored processes every few seconds."""

    def __init__(self, period=5.0):
        super().__init__(daemon=True)
        self.period = period
        self.samples = {label: [] for label, _ in PROCESSES}
        self.running = True

    def _find(self):
        found = {}
        for proc in psutil.process_iter(['cmdline']):
            cmd = ' '.join(proc.info['cmdline'] or [])
            for label, pattern in PROCESSES:
                if pattern in cmd and 'ros2 launch' not in cmd and 'bash -c' not in cmd:
                    found.setdefault(label, []).append(proc)
        return found

    def run(self):
        procs = self._find()
        for plist in procs.values():
            for p in plist:
                try:
                    p.cpu_percent(None)
                except psutil.Error:
                    pass
        while self.running:
            time.sleep(self.period)
            for label, plist in procs.items():
                total = 0.0
                for p in plist:
                    try:
                        total += p.cpu_percent(None)
                    except psutil.Error:
                        pass
                self.samples[label].append(total)


class Report:
    def __init__(self):
        self.failures = 0
        self.lines = []

    def check(self, passed, text, judged=True):
        tag = ('PASS' if passed else 'FAIL') if judged else 'INFO'
        if judged and not passed:
            self.failures += 1
        line = f'[{tag}] {text}'
        self.lines.append(line)
        print(line, flush=True)

    def info(self, text):
        self.check(True, text, judged=False)


class Checker:
    def __init__(self, args, monitor, nav, report):
        self.args = args
        self.monitor = monitor
        self.nav = nav
        self.report = report
        self.world = os.path.splitext(os.path.basename(args.world_file))[0]

    # --- helpers ---------------------------------------------------------------------------

    def map_pose(self, x, y, yaw):
        """World -> the map frame, by the best fit."""
        return compose(self.monitor.map_from_world, (x, y, yaw))

    def pose_stamped(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        # Time 0: the goal is in the global frame already, no transform is needed.
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.orientation.w = math.cos(pose[2] / 2.0)
        return msg

    def go(self, label, world_goal, checker='staging', timeout=180.0, watch=None):
        """Send a NavigateToPose goal; return (result, seconds, ground truth at the end)."""
        self.monitor.select_checker(checker)
        time.sleep(0.3)
        self.monitor.set_label(label)
        goal = self.pose_stamped(self.map_pose(*world_goal))
        start = time.monotonic()
        self.nav.goToPose(goal)
        timed_out = False
        while not self.nav.isTaskComplete():
            if watch is not None:
                watch()
            if time.monotonic() - start > timeout:
                timed_out = True
                self.nav.cancelTask()
                end = time.monotonic() + 10.0
                while not self.nav.isTaskComplete() and time.monotonic() < end:
                    time.sleep(0.1)
                break
            time.sleep(0.1)
        seconds = time.monotonic() - start
        # After a cancel, BasicNavigator can still hold the previous goal's status.
        result = TaskResult.CANCELED if timed_out else self.nav.getResult()
        return result, seconds, self.monitor.truth()

    def clearance(self, label):
        d, name = self.monitor.clearance.get(label, (math.inf, ''))
        return d, name

    def contact_ok(self, label):
        d, name = self.clearance(label)
        return d > 0.0, f'closest {d * 100:.1f} cm ({name})'

    def gz(self, service, reqtype, req):
        subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/{service}', '--reqtype',
                        reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req',
                        req], check=True, capture_output=True)

    def insert(self, box):
        sdf = box.sdf().replace('"', '\\"')
        self.gz('create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')
        self.monitor.shapes.append(box.shape)

    def remove(self, box):
        self.gz('remove', 'gz.msgs.Entity', f'name: "{box.name}" type: MODEL')
        self.monitor.shapes = [s for s in self.monitor.shapes if s.name != box.name]

    def ensure_at(self, world_goal, label):
        """Drive to a pose for the next test; report only."""
        result, seconds, _ = self.go(label, world_goal)
        if result != TaskResult.SUCCEEDED:
            self.report.info(f'{label}: could not get there ({result})')

    # --- stages ----------------------------------------------------------------------------

    def suite(self):
        self.monitor.set_label('suite', stage='suite')
        for name, x, y, yaw, checker, judged in SUITE:
            if self.args.goals and name not in self.args.goals:
                continue
            start_truth = self.monitor.truth()
            yaw_track = []

            def watch():
                gt = self.monitor.truth()
                if gt is not None:
                    yaw_track.append(gt[2])

            result, seconds, gt = self.go(name, (x, y, yaw), checker, watch=watch)
            goal_world = (x, y, yaw)
            err = math.dist(gt[:2], goal_world[:2])
            yaw_err = abs(wrap(gt[2] - yaw))
            xy_tol, yaw_tol = TOLERANCE[checker]
            ok_contact, contact = self.contact_ok(name)
            reached = (result == TaskResult.SUCCEEDED and err <= xy_tol + ALLOWANCE
                       and yaw_err <= yaw_tol + math.radians(3.0))
            outcome = result.name if result else None
            text = (f'goal {name} ({checker}): {outcome} in {seconds:.0f} s,'
                    f' ground truth {err * 100:.1f} cm / {math.degrees(yaw_err):.1f} deg from the'
                    f' goal (tolerance {xy_tol * 100:.0f}+{ALLOWANCE * 100:.0f} cm); {contact}')
            if judged:
                self.report.check(reached and ok_contact, text)
            else:
                self.report.check(ok_contact, text + '; accuracy reported only')
            if name in STRAFE_GOALS and start_truth is not None and yaw_track:
                swing = max(abs(wrap(a - start_truth[2])) for a in yaw_track)
                self.report.check(swing < STRAFE_YAW_LIMIT,
                                  f'goal {name} driven sideways: heading changed at most '
                                  f'{math.degrees(swing):.1f} deg (limit 15)')

    def gaps(self):
        self.monitor.set_label('gaps', stage='gaps')
        for name, goal in (('behind_glass', (0.2, 3.4, E)),
                           ('across_partition_start', (6.2, 1.2, N)),
                           ('into_stall_across_partition', (5.8, 2.9, N))):
            if name == 'across_partition_start':
                self.ensure_at(goal, name)
                continue
            result, seconds, gt = self.go(name, goal)
            ok_contact, contact = self.contact_ok(name)
            err = math.dist(gt[:2], goal[:2])
            reached = err <= TOLERANCE['staging'][0] + ALLOWANCE
            self.report.check(result == TaskResult.SUCCEEDED and reached and ok_contact,
                              f'gap {name}: {result.name if result else None} in {seconds:.0f} s,'
                              f' {err * 100:.1f} cm from the goal; {contact}')
        # Past the threshold: the corridor's east end is unreachable.
        self.ensure_at((4.5, -1.0, E), 'threshold_start')
        crossed = []

        def watch():
            gt = self.monitor.truth()
            if gt is not None:
                c, s = math.cos(gt[2]), math.sin(gt[2])
                front = max(gt[0] + c * x - s * y for x in FOOTPRINT[:2]
                            for y in FOOTPRINT[2:])
                crossed.append(front)

        result, seconds, gt = self.go('past_threshold', (6.3, -1.0, E), timeout=120.0,
                                      watch=watch)
        ok_contact, contact = self.contact_ok('past_threshold')
        furthest = max(crossed) if crossed else float('nan')
        self.report.check(result != TaskResult.SUCCEEDED and furthest < 5.35 and ok_contact,
                          f'gap past_threshold: refused ({result.name if result else None} '
                          f'after {seconds:.0f} s); furthest robot point x {furthest:.2f} m '
                          f'(threshold from 5.35); {contact}')

    def obstacles(self):
        self.monitor.set_label('obstacles', stage='obstacles')
        start, goal = (0.0, -1.0, E), (4.5, -1.0, E)
        # North edge of the boxes to go around: the corridor's north wall is at y -0.05.
        edge = -0.05 - 1.10

        def south_of(size):
            return edge - size[1] / 2.0

        cases = [
            ('box', Box('nc_box', 2.5, south_of((0.4, 0.4)), (0.4, 0.4, 0.5)), True),
            ('low_box', Box('nc_low_box', 2.5, south_of((0.4, 0.6)), (0.4, 0.6, 0.10)), True),
            ('overhang', Box('nc_overhang', 2.5, south_of((0.5, 0.8)), (0.5, 0.8, 0.25),
                             z_low=0.60), True),
            ('box_centred', Box('nc_box_c', 2.5, -1.0, (0.4, 0.4, 0.5)), None),
        ]
        for name, box, passable in cases:
            self.ensure_at(start, f'{name}_start')
            self.insert(box)
            time.sleep(1.0)
            result, seconds, gt = self.go(name, goal, timeout=180.0 if passable else 120.0)
            ok_contact, contact = self.contact_ok(name)
            reached = result == TaskResult.SUCCEEDED
            self.report.check((reached or passable is None) and ok_contact,
                              f'obstacle {name}: {result.name if result else None} in '
                              f'{seconds:.0f} s; {contact}')
            self.remove(box)
        # Dropped ahead of the moving robot, then a wall across the corridor, then glass.
        for name, make, passable in (
                ('dropped_ahead', lambda x: Box('nc_dropped', x + 0.9, south_of((0.4, 0.4)),
                                                (0.4, 0.4, 0.5)), True),
                ('corridor_wall', lambda x: Box('nc_wall', 2.6, -1.0, (0.1, 1.9, 1.0)), False),
                ('unknown_glass', lambda x: Box('nc_glass', 2.6, -1.0, (0.02, 1.9, 1.0),
                                                hidden=True), False)):
            self.ensure_at(start, f'{name}_start')
            inserted = []

            def watch():
                gt = self.monitor.truth()
                if not inserted and gt is not None and gt[0] > 1.3:
                    box = make(gt[0])
                    self.insert(box)
                    inserted.append(box)

            result, seconds, gt = self.go(name, goal, timeout=90.0 if not passable else 180.0,
                                          watch=watch)
            ok_contact, contact = self.contact_ok(name)
            gap = ''
            if inserted:
                box = inserted[0]
                front = gt[0] + FOOTPRINT[1]
                gap = f', stopped {100 * (box.x - box.size[0] / 2 - front):.0f} cm short' \
                    if not passable else ''
            if passable:
                ok = result == TaskResult.SUCCEEDED and ok_contact
            else:
                ok = ok_contact
            self.report.check(ok, f'obstacle {name}: {result.name if result else None} after '
                                  f'{seconds:.0f} s{gap}; {contact}')
            for box in inserted:
                self.remove(box)

    def reverse(self):
        self.monitor.set_label('reverse_start', stage='reverse')
        self.ensure_at((0.6, -1.0, E), 'reverse_start')
        gt = self.monitor.truth()
        # A box 0.25 m behind the robot's rear, tall enough for the rear ultrasonics (0.30 m).
        rear = gt[0] - 0.405
        box = Box('nc_rear', rear - 0.25 - 0.1, gt[1], (0.2, 0.7, 0.6))
        self.insert(box)
        time.sleep(1.5)
        self.monitor.set_label('reverse_backup')
        self.nav.backup(backup_dist=0.6, backup_speed=0.1, time_allowance=20)
        while not self.nav.isTaskComplete():
            time.sleep(0.1)
        result = self.nav.getResult()
        gt = self.monitor.truth()
        gap = (gt[0] - 0.405) - (box.x + 0.1)
        ok_contact, contact = self.contact_ok('reverse_backup')
        self.report.check(ok_contact, f'reverse BackUp 0.6 m toward a box 0.25 m behind: '
                                      f'{result.name if result else None}, {gap * 100:.1f} cm '
                                      f'left; {contact}')
        # Straight into the collision monitor, as the smoother would send them.
        self.monitor.set_label('reverse_direct')
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
            msg = TwistStamped()
            msg.header.stamp = self.monitor.sim_stamp()
            msg.header.frame_id = 'base_footprint'
            msg.twist.linear.x = -0.12
            self.monitor.cmd_pub.publish(msg)
            time.sleep(0.05)
        stop = TwistStamped()
        stop.header.stamp = self.monitor.sim_stamp()
        self.monitor.cmd_pub.publish(stop)
        gt = self.monitor.truth()
        gap = (gt[0] - 0.405) - (box.x + 0.1)
        ok_contact, contact = self.contact_ok('reverse_direct')
        self.report.check(ok_contact, f'reverse -0.12 m/s for 6 s into the collision monitor: '
                                      f'{gap * 100:.1f} cm left; {contact}')
        self.remove(box)

    def relocalize(self):
        self.monitor.set_label('relocalize_start', stage='relocalize_start')
        self.ensure_at((0.0, -1.0, E), 'relocalize_start')
        time.sleep(2.0)
        truth = self.monitor.truth_in_map()
        offset = (truth[0] + 0.4 * math.cos(math.radians(30)),
                  truth[1] + 0.4 * math.sin(math.radians(30)), wrap(truth[2] +
                                                                    math.radians(25)))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.monitor.sim_stamp()
        msg.pose.pose.position.x, msg.pose.pose.position.y = offset[0], offset[1]
        msg.pose.pose.orientation.z = math.sin(offset[2] / 2)
        msg.pose.pose.orientation.w = math.cos(offset[2] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2
        cov[35] = math.radians(15) ** 2
        msg.pose.covariance = cov
        self.monitor.initial_pub.publish(msg)
        time.sleep(1.0)
        with self.monitor.lock:
            base = self.monitor.travelled
            self.monitor.stage = 'relocalize'
        self.monitor.set_label('relocalize')
        result, seconds, _ = self.go('relocalize', (4.5, -1.0, E))
        with self.monitor.lock:
            samples = [(t - base, e, y) for st, t, e, y in self.monitor.loc_samples
                       if st == 'relocalize']
        lines = []
        converged_at = None
        for metre in range(0, 6):
            near = [s for s in samples if metre <= s[0] < metre + 1]
            if near:
                e = max(s[1] for s in near)
                y = max(s[2] for s in near)
                lines.append(f'{metre}-{metre + 1} m: {e * 100:.0f} cm / {math.degrees(y):.1f}')
        for i, (t, e, y) in enumerate(samples):
            if all(s[1] <= LOC_LIMIT[0] and s[2] <= LOC_LIMIT[1] for s in samples[i:]):
                converged_at = t
                break
        self.report.check(converged_at is not None and converged_at <= 3.0,
                          'relocalize from 0.4 m / 25 deg off: within 10 cm / 3 deg for good '
                          f'after {converged_at if converged_at is not None else float("nan"):.2f}'
                          f' m of travel (limit 3 m); worst per metre: ' + '; '.join(lines))
        self.report.info(f'relocalize goal: {result.name if result else None} in {seconds:.0f} s')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    share = get_package_share_directory('mecanum_simulation')
    parser.add_argument('--world-file', default=os.path.join(share, 'worlds', 'restroom.sdf'))
    parser.add_argument('--map', default=os.path.join(
        get_package_share_directory('mecanum_navigation'), 'maps', 'restroom.yaml'))
    parser.add_argument('--map-pose', type=float, nargs=3, default=(0.0, -1.0, 0.0),
                        metavar=('X', 'Y', 'YAW'), help='Nominal map frame in the world')
    parser.add_argument('--localizer', default='amcl', choices=['amcl', 'slam_toolbox'])
    parser.add_argument('--only', nargs='+',
                        default=['suite', 'gaps', 'obstacles', 'reverse', 'relocalize'],
                        choices=['suite', 'gaps', 'obstacles', 'reverse', 'relocalize'])
    parser.add_argument('--poses-csv', help='Write the localization samples to this CSV')
    parser.add_argument('--goals', nargs='+', choices=[g[0] for g in SUITE],
                        help='Only these goals of the suite, in suite order')
    args = parser.parse_args(argv)

    shapes = mc.load_world_shapes(args.world_file)
    cells, _ = mc.load_occupied_cells(args.map)
    expected = mc.expected_shapes(shapes, SCAN_HEIGHT)
    world_from_map, residual = mc.best_fit(cells, expected, initial=tuple(args.map_pose))
    contact_shapes = [s for s in shapes if s.z_range[1] > 0.001 and s.z_range[0] < ROBOT_TOP]

    rclpy.init()
    monitor = Monitor(contact_shapes, world_from_map)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    nav = BasicNavigator(node_name='nav_check_navigator')
    report = Report()
    cpu = CpuSampler()
    try:
        report.info(f'map frame in the world (best fit): x {world_from_map[0]:.3f} y '
                    f'{world_from_map[1]:.3f} yaw {math.degrees(world_from_map[2]):.2f} deg, '
                    f'residual median {np.median(residual) * 100:.1f} cm')
        localizer = 'amcl' if args.localizer == 'amcl' else 'bt_navigator'
        nav.waitUntilNav2Active(localizer=localizer)
        limit = time.monotonic() + 30.0
        while monitor.truth() is None and time.monotonic() < limit:
            time.sleep(0.1)
        if monitor.truth() is None:
            sys.exit('No ground truth: is the simulation running?')
        cpu.start()
        checker = Checker(args, monitor, nav, report)
        for stage in args.only:
            if stage == 'relocalize' and args.localizer != 'amcl':
                report.info('relocalize: AMCL only, skipped')
                continue
            getattr(checker, stage)()
        cpu.running = False
        with monitor.lock:
            samples = [s for s in monitor.loc_samples
                       if s[0] not in ('relocalize', 'relocalize_start')]
        if samples:
            errors = np.array([s[2] for s in samples])
            yaws = np.degrees([s[3] for s in samples])
            report.check(errors.max() <= LOC_LIMIT[0] and yaws.max() <= math.degrees(
                LOC_LIMIT[1]), f'localization ({args.localizer}) against ground truth, '
                f'{len(samples)} samples: max {errors.max() * 100:.1f} cm / {yaws.max():.2f} '
                f'deg (limit 10 cm / 3 deg); median {np.median(errors) * 100:.1f} cm / '
                f'{np.median(yaws):.2f} deg')
        if args.poses_csv:
            with open(args.poses_csv, 'w') as f:
                f.write('stage,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw\n')
                for stage, gt, est in monitor.loc_poses:
                    f.write(f'{stage},{gt[0]:.4f},{gt[1]:.4f},{gt[2]:.4f},'
                            f'{est[0]:.4f},{est[1]:.4f},{est[2]:.4f}\n')
        marks = monitor.rtf_marks
        if len(marks) > 10:
            rtf = (marks[-1][1] - marks[0][1]) / (marks[-1][0] - marks[0][0])
            report.info(f'real-time factor over the run: {rtf:.2f}')
        cpu_lines = []
        for label, values in cpu.samples.items():
            if values:
                cpu_lines.append(f'{label} {np.median(values):.0f}% (max {max(values):.0f})')
        report.info('CPU, median of 5 s samples, % of a core: ' + ', '.join(cpu_lines))
    finally:
        cpu.running = False
        executor.shutdown()
        nav.destroy_node()
        monitor.destroy_node()
        rclpy.try_shutdown()
    print(f'\n{report.failures} failure(s)')
    return 1 if report.failures else 0


if __name__ == '__main__':
    sys.exit(main())
