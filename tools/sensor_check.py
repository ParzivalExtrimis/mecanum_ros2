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
Check the simulated sensor suite against the sensor contract and the sensor test objects.

Start the simulation in the sensor test world first (LIO is not needed):
    ros2 launch mecanum_bringup robot.launch.py gui:=false lio:=false \
      world:=$(ros2 pkg prefix mecanum_simulation)/share/mecanum_simulation/worlds/sensor_test.sdf
    tools/sensor_check.py

Checks, each with a PASS or FAIL, in docs/plan.md Phase 2 order:

1. Contract: every topic of the plan's sensor contract, its frame and its rate on sim time.
2. Self view: in the open, no sensor sees the robot, and the L2 cloud has nothing near it.
3. Objects: the robot is put at a station in front of each object in worlds/sensor_test.sdf.
   Every sensor's returns are brought to the world frame, through TF and Gazebo ground truth
   (shortcut S1). A sensor sees an object if enough returns lie on its box, within --tolerance.
   The range sensors are compared with the same cone cast at the box.
4. Cliffs: over the edge of the floor hole, forward and in reverse.
5. Bumpers: driven into the bumper wall, forward and in reverse, and back off.

Stations move the robot with Gazebo's set_pose service, so run this on a test world only. The
final pose is the last station. --only picks some checks: --only contract self.
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
import tf2_ros
from transforms3d.quaternions import quat2mat

POSITIONS = ('front_left', 'front_right', 'rear_left', 'rear_right')

# Topic, type, frame (None: no header), rate in Hz.
CONTRACT = [
    ('/l2/points', PointCloud2, 'l2_lidar', 5.55),
    ('/l2/imu', Imu, 'l2_imu', 250.0),
    ('/l2/points_filtered', PointCloud2, 'l2_lidar', 5.55),
    ('/scan', LaserScan, 'lidar_2d_link', 10.0),
    ('/d435i/color/image_raw', Image, 'd435i_optical_frame', 15.0),
    ('/d435i/color/camera_info', CameraInfo, 'd435i_optical_frame', 15.0),
    ('/d435i/depth/image_rect_raw', Image, 'd435i_optical_frame', 15.0),
    ('/d435i/depth/camera_info', CameraInfo, 'd435i_optical_frame', 15.0),
    ('/d435i/depth/color/points', PointCloud2, 'd435i_optical_frame', 15.0),
    ('/d435i/imu', Imu, 'd435i_link', 200.0),
    *[(f'/ultrasonic/{p}', Range, f'ultrasonic_{p}_link', 10.0) for p in POSITIONS],
    *[(f'/cliff/{p}', Range, f'cliff_tof_{p}_link', 20.0) for p in POSITIONS],
    ('/bumper/front', Bool, None, 20.0),
    ('/bumper/rear', Bool, None, 20.0),
]
RATE_TOLERANCE = 0.1
WALL_CLOCK_TOPICS = ('/bumper/front', '/bumper/rear')  # sim_bumper's timer is on wall time

# The cones of urdf/mecanum.gazebo.xacro: half widths (deg) and ray counts.
ULTRASONIC_CONE = (30.0, 9, 3.0, 3)

CLOUD_TOPICS = {'lidar_2d': '/scan', 'l2': '/l2/points_filtered',
                'd435i': '/d435i/depth/color/points'}
FRONT_ULTRASONICS = ('front_left', 'front_right')

# Station (x, y, yaw), then whether each sensor must see the object.
STATIONS = {
    'low_threshold': ((0.0, 0.0, 0.0), {
        'd435i': True, 'lidar_2d': False, 'l2': False, 'ultrasonic': False}),
    'stall_partition': ((0.0, 3.0, 0.0), {
        'd435i': True, 'l2': True, 'lidar_2d': False, 'ultrasonic': True}),
    'counter_overhang': ((0.0, -3.0, 0.0), {
        'd435i': True, 'l2': True, 'lidar_2d': False, 'ultrasonic': False}),
    'glass_panel': ((0.0, -6.0, 0.0), {
        'ultrasonic': True, 'lidar_2d': False, 'd435i': False, 'l2': False}),
}
OPEN_STATION = (-5.0, -1.0, 0.0)
# The floor hole's west edge is x = 1.0; 0.29 m ahead of the robot lies between the wheel contact
# (0.236 m) and the cliff sensor's cone at the floor (0.325 m).
HOLE_STATIONS = {'front': (0.71, 6.0, 0.0), 'rear': (0.71, 6.0, math.pi)}
FLOOR_DISTANCE = 0.119  # m, cliff_tof_z above the floor
CLIFF_MARGIN = 0.05  # STAND-IN of plan Phase 4
# 4.6 cm off the wall, which is at x 0.95..1.05, y 9
BUMPER_STATIONS = {'front': ((0.5, 9.0, 0.0), 1.0), 'rear': ((1.5, 9.0, 0.0), -1.0)}
BUMPER_SPEED = 0.1  # m/s
ROBOT_BOX = ((-0.43, 0.45), (-0.29, 0.29), (-0.05, 0.86))  # self_filter.yaml, base_footprint
MIN_RETURNS = 5  # returns on a box for a sensor to see it
NEAR_BOX = 0.2  # m; wide, so that a misplaced return shows as an offset
SELF_RADIUS = 2.0  # m; the open station has nothing this close
FLOOR_BAND = 0.03  # m; returns below this height are the floor (depth noise included)


def transform_matrix(translation, quaternion_xyzw):
    """Return the 4x4 transform of a translation and an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion_xyzw
    matrix = np.eye(4)
    matrix[:3, :3] = quat2mat((w, x, y, z))
    matrix[:3, 3] = translation
    return matrix


def apply(matrix, points):
    """Return Nx3 ``points`` through a 4x4 transform."""
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def box_distance(points, centre, size):
    """Return each point's distance to an axis-aligned box, 0 inside it."""
    outside = np.maximum(np.abs(points - np.asarray(centre)) - np.asarray(size) / 2.0, 0.0)
    return np.linalg.norm(outside, axis=1)


def cone_directions(h_half, h_samples, v_half, v_samples):
    """Return the unit ray directions of a gpu_lidar cone along +x."""
    azimuth, elevation = np.meshgrid(
        np.radians(np.linspace(-h_half, h_half, h_samples)),
        np.radians(np.linspace(-v_half, v_half, v_samples)))
    return np.stack((np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth),
                     np.sin(elevation)), axis=-1).reshape(-1, 3)


def ray_box_distance(origin, directions, centre, size):
    """Return the nearest hit of rays on an axis-aligned box, or inf (the slab method)."""
    low = np.asarray(centre) - np.asarray(size) / 2.0 - origin
    high = np.asarray(centre) + np.asarray(size) / 2.0 - origin
    with np.errstate(divide='ignore', invalid='ignore'):
        near = low / directions
        far = high / directions
    near, far = np.minimum(near, far), np.maximum(near, far)
    # A ray parallel to a slab gives nan there: it only matters whether it lies inside.
    near = np.where(np.isnan(near), -np.inf, near)
    far = np.where(np.isnan(far), np.inf, far)
    enter, leave = near.max(axis=1), far.min(axis=1)
    hits = enter[(enter <= leave) & (enter > 0.0)]
    return float(hits.min()) if hits.size else math.inf


def load_boxes(world):
    """Return {link: (centre, size)} of the boxes of the model test_objects in a world file."""
    boxes = {}
    for model in ET.parse(world).getroot().iter('model'):
        if model.get('name') != 'test_objects':
            continue
        for link in model.iter('link'):
            pose = [float(v) for v in link.find('pose').text.split()]
            size = [float(v) for v in link.find('.//box/size').text.split()]
            boxes[link.get('name')] = (pose[:3], size)
    return boxes


class SensorCheck(Node):
    """Collect the contract topics and keep the latest message of each."""

    def __init__(self):
        super().__init__('sensor_check')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.latest = {}
        self.counts = {}
        self.frames = {}
        self.ground_truth = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        # The clouds and images are too big for best effort over the default socket buffers.
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for topic, msg_type, _, _ in CONTRACT:
            big = msg_type in (PointCloud2, Image)
            self.create_subscription(
                msg_type, topic, lambda msg, name=topic: self._on_message(name, msg),
                reliable if big else qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self._on_ground_truth,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def _on_message(self, topic, msg):
        self.latest[topic] = msg
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if hasattr(msg, 'header'):
            self.frames[topic] = msg.header.frame_id

    def _on_ground_truth(self, msg):
        self.ground_truth = msg

    def sim_now(self):
        """Return the simulation time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, seconds):
        """Wait for a span of simulation time."""
        deadline = self.sim_now() + seconds
        limit = time.monotonic() + 10.0 * seconds + 10.0
        while self.sim_now() < deadline and time.monotonic() < limit:
            time.sleep(0.01)

    def fresh(self, topics, seconds=0.6):
        """Forget ``topics``, wait, and return the new message of each (None if none came)."""
        for topic in topics:
            self.latest.pop(topic, None)
        self.wait_sim(seconds)
        return {topic: self.latest.get(topic) for topic in topics}

    def base_in_world(self):
        """Return the ground-truth pose of base_footprint as a 4x4 transform."""
        pose = self.ground_truth.pose.pose
        return transform_matrix(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    def frame_in_base(self, frame):
        """Return a robot frame in base_footprint as a 4x4 transform."""
        t = self.tf_buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
        v, q = t.transform.translation, t.transform.rotation
        return transform_matrix((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))

    def points_in_base(self, topic, msg):
        """Return the finite returns of a scan or a cloud in base_footprint."""
        if isinstance(msg, LaserScan):
            ranges = np.asarray(msg.ranges)
            angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
            keep = np.isfinite(ranges)
            points = np.stack((ranges[keep] * np.cos(angles[keep]),
                               ranges[keep] * np.sin(angles[keep]),
                               np.zeros(int(keep.sum()))), axis=-1)
        else:
            points = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'],
                                                    skip_nans=True).astype(np.float64)
            points = points[np.isfinite(points).all(axis=1)]
        return apply(self.frame_in_base(msg.header.frame_id), points)

    def move_to(self, world, station, settle=1.5):
        """Put the robot at (x, y, yaw) with Gazebo's set_pose service and let it settle."""
        x, y, yaw = station
        request = (f'name: "mecanum", position: {{x: {x}, y: {y}, z: 0.01}}, '
                   f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', request],
                       check=True, capture_output=True)
        self.wait_sim(settle)

    def drive(self, vx, seconds):
        """Command a forward speed for a span of simulation time, then stop."""
        deadline = self.sim_now() + seconds
        while self.sim_now() < deadline:
            self._send(vx)
            time.sleep(0.02)
        self._send(0.0)

    def _send(self, vx):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = vx
        self.cmd_pub.publish(msg)


class Report:
    """Print checks as they run and keep count of the failures."""

    def __init__(self):
        self.failures = 0

    def check(self, passed, text):
        """Print one PASS or FAIL line."""
        self.failures += 0 if passed else 1
        print(f'  {"PASS" if passed else "FAIL"}  {text}', flush=True)


def check_contract(node, report, window):
    print(f'\n1. Contract, {window:.0f} s of simulation time')
    node.counts.clear()
    wall_start, sim_start = time.monotonic(), node.sim_now()
    node.wait_sim(window)
    elapsed, wall = node.sim_now() - sim_start, time.monotonic() - wall_start
    counts = dict(node.counts)
    for topic, _, frame, rate in CONTRACT:
        measured = counts.get(topic, 0) / (wall if topic in WALL_CLOCK_TOPICS else elapsed)
        rate_ok = abs(measured - rate) <= RATE_TOLERANCE * rate
        seen_frame = node.frames.get(topic)
        frame_ok = frame is None or seen_frame == frame
        report.check(rate_ok and frame_ok,
                     f'{topic:32s} {measured:7.2f} Hz (contract {rate:g})  {seen_frame or "-"}')
    print(f'  real-time factor over the window: {elapsed / wall:.2f}')
    publishers = node.get_publishers_info_by_topic('/scan')
    report.check(len(publishers) == 1, f'/scan has {len(publishers)} publisher(s)')


def check_self_view(node, report, world):
    print(f'\n2. Self view, in the open at {OPEN_STATION}')
    node.move_to(world, OPEN_STATION)
    topics = ['/scan', '/l2/points', '/l2/points_filtered', '/d435i/depth/color/points',
              *[f'/ultrasonic/{p}' for p in POSITIONS], *[f'/cliff/{p}' for p in POSITIONS]]
    msgs = node.fresh(topics, 1.0)
    if any(msg is None for msg in msgs.values()):
        report.check(False, f'no data on {[t for t, m in msgs.items() if m is None]}')
        return
    pose = node.ground_truth.pose.pose
    matrix = node.base_in_world()
    print(f'        robot tilt {math.degrees(math.asin(min(1.0, -matrix[2, 0]))):+.3f} deg pitch, '
          f'{math.degrees(math.atan2(matrix[2, 1], matrix[2, 2])):+.3f} deg roll, '
          f'height {pose.position.z:+.4f} m')
    scan = msgs['/scan']
    ranges = np.asarray(scan.ranges)
    near = np.flatnonzero(ranges < SELF_RADIUS)
    report.check(near.size == 0, f'/scan: {near.size} returns within {SELF_RADIUS} m '
                 f'(of {np.isfinite(ranges).sum()} in all)')
    for i in near[:10]:
        print(f'        {math.degrees(scan.angle_min + i * scan.angle_increment):+.1f} deg: '
              f'{scan.ranges[i]:.3f} m')
    centre = [sum(limits) / 2.0 for limits in ROBOT_BOX]
    size = [limits[1] - limits[0] for limits in ROBOT_BOX]
    for topic in ('/l2/points', '/l2/points_filtered'):
        distance = box_distance(node.points_in_base(topic, msgs[topic]), centre, size)
        print(f'        {topic}: {int((distance == 0.0).sum())} points in the self-filter box, '
              f'{int(((distance > 0.0) & (distance < 0.3)).sum())} within 0.3 m of it')
    distance = box_distance(node.points_in_base('', msgs['/l2/points_filtered']), centre, size)
    report.check(not (distance < 0.3).any(), 'no filtered L2 point within 0.3 m of the box')
    distance = box_distance(node.points_in_base('', msgs['/l2/points']), centre, size)
    report.check(not ((distance > 0.0) & (distance < 0.3)).any(),
                 'no raw L2 point in the 0.3 m around the box (the box holds all self hits)')
    depth = node.points_in_base('', msgs['/d435i/depth/color/points'])
    report.check(not (box_distance(depth, centre, size) == 0.0).any(),
                 f'no D435i point in the box ({len(depth)} points on the floor, '
                 f'z {depth[:, 2].min():+.3f}..{depth[:, 2].max():+.3f} m)')
    for p in POSITIONS:
        value = msgs[f'/ultrasonic/{p}'].range
        report.check(math.isinf(value), f'/ultrasonic/{p}: {value:.3f} (no floor echo)')
    for p in POSITIONS:
        value = msgs[f'/cliff/{p}'].range
        report.check(abs(value - FLOOR_DISTANCE) < 0.02, f'/cliff/{p}: {value:.3f} m (the floor)')


def sees_points(node, topic, msg, base_in_world, box):
    """Return (returns near the box, 95th percentile of their distance from it)."""
    centre, size = box
    points = apply(base_in_world, node.points_in_base(topic, msg))
    distance = box_distance(points[points[:, 2] > FLOOR_BAND], centre, size)
    on_box = distance[distance <= NEAR_BOX]
    return len(on_box), float(np.percentile(on_box, 95)) if len(on_box) else 0.0


def check_objects(node, report, world, boxes, tolerance):
    print('\n3. Objects')
    cone = cone_directions(*ULTRASONIC_CONE)
    for name, (station, expected) in STATIONS.items():
        print(f'  {name}, station {station}')
        node.move_to(world, station)
        topics = list(CLOUD_TOPICS.values()) + [f'/ultrasonic/{p}' for p in FRONT_ULTRASONICS]
        msgs = node.fresh(topics, 1.0)
        base_in_world = node.base_in_world()
        for sensor, topic in CLOUD_TOPICS.items():
            if sensor not in expected:
                continue
            if msgs[topic] is None:
                report.check(False, f'{sensor}: no data')
                continue
            count, offset = sees_points(node, topic, msgs[topic], base_in_world, boxes[name])
            seen = count >= MIN_RETURNS
            good = seen == expected[sensor] and (not seen or offset <= tolerance)
            report.check(good, f'{sensor:10s} {count:6d} returns, 95% within {offset:.3f} m '
                         f'(must {"" if expected[sensor] else "not "}see it)')
        if 'ultrasonic' in expected:
            for p in FRONT_ULTRASONICS:
                topic = f'/ultrasonic/{p}'
                link = base_in_world @ node.frame_in_base(f'ultrasonic_{p}_link')
                predicted = ray_box_distance(link[:3, 3], cone @ link[:3, :3].T, *boxes[name])
                value = msgs[topic].range if msgs[topic] else math.nan
                seen = math.isfinite(value) and abs(value - predicted) <= tolerance
                report.check(seen == expected['ultrasonic'],
                             f'ultrasonic {p}: {value:.3f} m, the box at {predicted:.3f} m '
                             f'(must {"" if expected["ultrasonic"] else "not "}see it)')


def check_cliffs(node, report, world):
    print('\n4. Cliffs, at the edge of the floor hole')
    threshold = FLOOR_DISTANCE + CLIFF_MARGIN
    for leading, station in HOLE_STATIONS.items():
        node.move_to(world, station)
        msgs = node.fresh([f'/cliff/{p}' for p in POSITIONS])
        for p in POSITIONS:
            value = msgs[f'/cliff/{p}'].range
            over_hole = p.startswith(leading)
            good = value > threshold if over_hole else abs(value - FLOOR_DISTANCE) < 0.02
            report.check(good, f'{leading} over the hole: /cliff/{p}: {value:.3f} m')


def check_bumpers(node, report, world):
    print('\n5. Bumpers, driven into the wall')
    for plank, (station, direction) in BUMPER_STATIONS.items():
        other = 'rear' if plank == 'front' else 'front'
        node.move_to(world, station)
        before = node.fresh([f'/bumper/{plank}'])[f'/bumper/{plank}']
        report.check(before is not None and not before.data, f'/bumper/{plank} false before')
        node.drive(direction * BUMPER_SPEED, 1.5)
        msgs = node.fresh([f'/bumper/{plank}', f'/bumper/{other}'], 0.1)
        report.check(msgs[f'/bumper/{plank}'] is not None and msgs[f'/bumper/{plank}'].data,
                     f'/bumper/{plank} true on contact')
        report.check(msgs[f'/bumper/{other}'] is not None and not msgs[f'/bumper/{other}'].data,
                     f'/bumper/{other} stays false')
        node.drive(-direction * BUMPER_SPEED, 1.5)
        after = node.fresh([f'/bumper/{plank}'], 0.3)[f'/bumper/{plank}']
        report.check(after is not None and not after.data, f'/bumper/{plank} false after backing')


def spin(executor):
    """Run the executor until it is shut down."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world-file', default=os.path.join(
        get_package_share_directory('mecanum_simulation'), 'worlds', 'sensor_test.sdf'))
    parser.add_argument('--window', type=float, default=10.0,
                        help='Simulation seconds for the rate measurement')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='How far (m) a return may lie from an object')
    parser.add_argument('--only', nargs='+', default=['contract', 'self', 'objects', 'cliffs',
                                                      'bumpers'],
                        choices=['contract', 'self', 'objects', 'cliffs', 'bumpers'])
    args = parser.parse_args(argv)
    world = os.path.splitext(os.path.basename(args.world_file))[0]

    rclpy.init()
    node = SensorCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=spin, args=(executor,), daemon=True)
    spinner.start()
    report = Report()
    try:
        limit = time.monotonic() + 30.0
        while time.monotonic() < limit and (node.sim_now() == 0.0 or node.ground_truth is None):
            time.sleep(0.05)
        if node.ground_truth is None:
            sys.exit('No ground truth: is the simulation running?')
        node.wait_sim(2.0)  # TF and the sensors
        if 'contract' in args.only:
            check_contract(node, report, args.window)
        if 'self' in args.only:
            check_self_view(node, report, world)
        if 'objects' in args.only:
            check_objects(node, report, world, load_boxes(args.world_file), args.tolerance)
        if 'cliffs' in args.only:
            check_cliffs(node, report, world)
        if 'bumpers' in args.only:
            check_bumpers(node, report, world)
    finally:
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n{report.failures} failure(s)')
    sys.exit(1 if report.failures else 0)


if __name__ == '__main__':
    main()
