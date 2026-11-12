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
Distance from the robot to a flat wall, from the Unitree L2: a reference for the drive tests.

The wall is the dominant line in a strip of the cloud on one side of the robot (front, back,
left or right of base_footprint), found with l2_mount_check.fit_line_xy. The distance is from
base_footprint to that line, along the line's normal; the yaw is the wall's angle against the
robot's axis (0 when square to it). The robot must stand still while it measures: the L2 sweeps
take about 0.1 s, so a moving robot smears the wall.

Used as a module by tools/drive_tune.py (WallRange), and on its own to set the robot up for a
run (square to the wall, at a known distance):
    tools/l2_wall_range.py --side front --duration 2

Needs /l2/points (ros2 launch mecanum_lidar l2.launch.py robot_model:=false rviz:=false) and
the TF from base_footprint to the L2 (robot_state_publisher, from drive_test.launch.py).
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from l2_mount_check import cloud_xyz, fit_line_xy, quat_to_matrix  # noqa: E402, I100
import numpy as np  # noqa: E402, I100

# Rotation that brings each side's wall normal onto +y in the fit frame, so that the wall is
# close to y = m x + b there (fit_line_xy fits y over x).
SIDE_ANGLE = {'left': 0.0, 'front': math.pi / 2, 'right': math.pi, 'back': -math.pi / 2}


def strip(points, side, half_width=0.4, z_min=0.15, z_max=1.8, near=0.35, far=8.0):
    """Points (Nx3, base frame) in the strip toward `side`, as Nx2 in the fit frame."""
    a = SIDE_ANGLE[side]
    c, s = math.cos(a), math.sin(a)
    x = c * points[:, 0] - s * points[:, 1]
    y = s * points[:, 0] + c * points[:, 1]
    keep = ((np.abs(x) < half_width) & (y > near) & (y < far)
            & (points[:, 2] > z_min) & (points[:, 2] < z_max))
    return np.stack([x[keep], y[keep]], axis=1)


def wall_distance(points, side, **kwargs):
    """
    Distance (m) from base_footprint to the wall on `side`, from Nx3 points in base_footprint.

    Returns (distance, yaw deg, residual std, inliers), or None with too few points.
    """
    xy = strip(points, side, **kwargs)
    if len(xy) < 30:
        return None
    yaw, b, resid, n = fit_line_xy(xy, tolerance=0.02)
    if n < 30:
        return None
    return abs(b) * math.cos(math.radians(yaw)), yaw, resid, n


class WallRange:
    """Collects /l2/points in base_footprint; measure() gives the wall distance while still."""

    def __init__(self, node, side, base_frame='base_footprint', topic='/l2/points'):
        import tf2_ros
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        self.node = node
        self.side = side
        self.base_frame = base_frame
        self.clouds = []
        self.collect = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        node.create_subscription(PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.last_cloud = 0.0

    def _on_cloud(self, msg):
        self.last_cloud = time.monotonic()
        if self.collect:
            self.clouds.append(msg)

    def alive(self, within=1.0):
        return time.monotonic() - self.last_cloud < within

    def _to_base(self, msg):
        import rclpy.time
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, msg.header.frame_id, rclpy.time.Time())
        r = tf.transform.rotation
        t = tf.transform.translation
        rot = quat_to_matrix(r.x, r.y, r.z, r.w)
        xyz, _, _ = cloud_xyz(msg)
        return xyz @ rot.T + np.array([t.x, t.y, t.z])

    def measure(self, spin, duration=1.0):
        """
        Collect for `duration` s (spin(seconds) runs the node) and fit the wall.

        Returns (distance, yaw deg, residual std, inliers, clouds) or None.
        """
        self.clouds = []
        self.collect = True
        spin(duration)
        self.collect = False
        if not self.clouds:
            return None
        points = np.concatenate([self._to_base(m) for m in self.clouds])
        result = wall_distance(points, self.side)
        if result is None:
            return None
        return (*result, len(self.clouds))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--side', choices=sorted(SIDE_ANGLE), default='front')
    parser.add_argument('--duration', type=float, default=2.0, help='Seconds per measurement')
    parser.add_argument('--repeat', type=int, default=0,
                        help='Measurements to take (0 = until Ctrl-C)')
    args = parser.parse_args(argv)

    import rclpy
    rclpy.init()
    node = rclpy.create_node('l2_wall_range')
    wall = WallRange(node, args.side)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    count = 0
    try:
        while args.repeat == 0 or count < args.repeat:
            count += 1
            result = wall.measure(spin, args.duration)
            if result is None:
                print(f'{args.side}: no wall found (clouds arriving: {wall.alive()})')
                continue
            distance, yaw, resid, n, clouds = result
            print(f'{args.side} wall: {distance:.3f} m, yaw {yaw:+.2f} deg, '
                  f'residual {resid * 1000:.1f} mm, {n} points, {clouds} clouds')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
