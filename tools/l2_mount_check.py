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
Check the Unitree L2 mount against the URDF, and the L2's output against the sim assumptions.

Run it with the robot standing still on a flat, level floor under a flat ceiling, the L2
driver (or the simulation) publishing /l2/points and /l2/imu, and robot_state_publisher
publishing the URDF TF. It collects data for --duration seconds and reports:

1. Plane fit (--plane ceiling, the default, or floor): a horizontal plane seen through the
   URDF transform should be level in base_footprint. A tilted plane means the real mount
   differs from the URDF; the tool reports the L2's tilt and its roll/pitch error. The robot's
   L2 stands upright at 0.80 m, so it sees the ceiling, but the floor only from about 7 m out
   (wide mode). The ceiling also gives the L2's height when --ceiling-height is tape-measured;
   the floor gives it directly.
2. IMU gravity: the mean accelerometer reading in l2_imu, against the URDF prediction. It gives
   the L2's tilt from vertical, and, on a tilted mount only, its rotation about its own axis
   (the l2_mount -> l2_lidar yaw).
3. Wall (optional, --wall-side left|right): with the robot parked parallel to a straight wall,
   the wall's direction gives the heading error of the whole mount, and so the l2_lidar yaw on
   an upright mount. Its offset is the distance to the wall (compare with a tape measure).
4. Cloud stream: rate, points per cloud, azimuth coverage per cloud (one full sweep = 360 deg
   in the simulation), the per-point "time" field, and the share of points on the robot.
5. Noise: plane residual (range noise) and IMU standard deviations, for STANDINS.md.

Usage (hardware, after ros2 launch mecanum_lidar l2.launch.py robot_model:=true rviz:=false):
    tools/l2_mount_check.py --duration 20 --wall-side right --ceiling-height 2.70
Simulation (restroom world, ceiling 2.4 m):
    tools/l2_mount_check.py --use-sim-time --wall-side right --ceiling-height 2.4
"""

import argparse
import math
import os
import sys
import time

import numpy as np

G = 9.80665


# ---------------------------------------------------------------- pure math (unit-testable)

def quat_to_matrix(x, y, z, w):
    """Return the 3x3 rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def fit_plane(points, iterations=3, reject_sigma=3.0):
    """
    Least-squares plane through Nx3 points with outlier rejection.

    Returns (unit normal pointing to +z, offset d with n.p = d, residual std, inlier count).
    """
    pts = np.asarray(points, dtype=float)
    keep = np.ones(len(pts), dtype=bool)
    for _ in range(iterations):
        p = pts[keep]
        centroid = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - centroid, full_matrices=False)
        n = vt[2]
        if n[2] < 0:
            n = -n
        d = float(n @ centroid)
        resid = pts @ n - d
        sigma = float(np.std(resid[keep]))
        new_keep = np.abs(resid) <= max(reject_sigma * sigma, 1e-4)
        if new_keep.sum() < 3 or np.array_equal(new_keep, keep):
            keep = new_keep if new_keep.sum() >= 3 else keep
            break
        keep = new_keep
    resid = pts[keep] @ n - d
    return n, d, float(np.std(resid)), int(keep.sum())


def rotation_between(a, b):
    """Return the smallest rotation matrix that turns unit vector a onto unit vector b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(a @ b)
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.eye(3) if c > 0 else -np.eye(3)
    k = v / s
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * kx + (1 - c) * (kx @ kx)


def plane_correction(normal, d, sensor_pos, sensor_rot):
    """
    Infer the real sensor pose from a horizontal plane measured through the URDF transform.

    normal, d: plane n.p = d in base_footprint (normal toward +z), computed with the URDF sensor
    pose. sensor_pos, sensor_rot: URDF position and rotation (columns = sensor axes) in
    base_footprint. Returns (signed distance of the sensor above the plane: positive above a
    floor, negative below a ceiling; real rotation; correction rotation).
    """
    corr = rotation_between(np.asarray(normal, float), np.array([0.0, 0.0, 1.0]))
    offset = float(np.asarray(normal) @ np.asarray(sensor_pos) - d)
    return offset, corr @ sensor_rot, corr


def tilt_from_vertical(rot):
    """Angle (deg) between the sensor's Z axis and the vertical (up or down), column axes."""
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(rot[2, 2])))))


def roll_pitch(corr):
    """Roll and pitch (deg) of a small correction rotation, as seen in base_footprint."""
    return (math.degrees(math.atan2(corr[2, 1], corr[2, 2])),
            math.degrees(-math.asin(max(-1.0, min(1.0, corr[2, 0])))))


def gravity_angles(acc):
    """
    Tilt (deg, sensor Z from the vertical, up or down) and spin (deg) of the sensor from gravity.

    acc is the mean accelerometer reading (specific force, pointing up) in the sensor frame.
    The spin is the rotation of the sensor about its own Z axis, measured from the
    orientation whose +X points uphill. It is defined only on a tilted mount (it is noise on an
    upright or inverted one); there it equals the l2_mount -> l2_lidar yaw.
    """
    a = np.asarray(acc, float) / np.linalg.norm(acc)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(a[2])))))
    spin = math.degrees(math.atan2(-a[1], a[0]))
    return tilt, spin


def fit_line_xy(points, tolerance=0.02, trials=300, seed=0):
    """
    Robust line through Nx2 points: RANSAC for the dominant line, then least squares.

    Returns (yaw deg of the line, its offset b in y = m x + b, residual std, inliers).
    """
    p = np.asarray(points, float)
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(trials):
        i, j = rng.choice(len(p), 2, replace=False)
        d = p[j] - p[i]
        length = np.hypot(*d)
        if length < 0.2:
            continue
        normal = np.array([-d[1], d[0]]) / length
        inliers = np.abs((p - p[i]) @ normal) < tolerance
        if best is None or inliers.sum() > best.sum():
            best = inliers
    q = p[best] if best is not None else p
    a = np.vstack([q[:, 0], np.ones(len(q))]).T
    (m, b), *_ = np.linalg.lstsq(a, q[:, 1], rcond=None)
    resid = q[:, 1] - (m * q[:, 0] + b)
    return math.degrees(math.atan(m)), float(b), float(np.std(resid)), int(len(q))


def azimuth_coverage(points_xy, bin_deg=5.0):
    """Degrees of azimuth (in bins of bin_deg) that contain at least one point."""
    az = np.degrees(np.arctan2(points_xy[:, 1], points_xy[:, 0]))
    bins = np.floor((az + 180.0) / bin_deg).astype(int)
    return len(np.unique(bins)) * bin_deg


def cloud_xyz(msg):
    """Return Nx3 float array of finite x, y, z from a PointCloud2, and the field names."""
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [np.frombuffer(raw[:, off[c]:off[c] + 4].tobytes(), dtype=np.float32) for c in 'xyz']
    xyz = np.stack(cols, axis=1).astype(float)
    time_span = None
    if 'time' in off:
        t = np.frombuffer(raw[:, off['time']:off['time'] + 4].tobytes(), dtype=np.float32)
        if len(t):
            time_span = float(t.max() - t.min())
    return xyz[np.isfinite(xyz).all(axis=1)], list(off), time_span


# ---------------------------------------------------------------- ROS collection and report

def load_crop_box():
    """Self-filter box from mecanum_localization's installed config (or None)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        path = os.path.join(get_package_share_directory('mecanum_localization'), 'config',
                            'self_filter.yaml')
        p = yaml.safe_load(open(path))['self_filter']['ros__parameters']
        return [p[k] for k in ('min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z')]
    except Exception:  # noqa: BLE001 - the box is optional
        return None


def in_box(p, box, grow=0.0):
    """Mask of points inside the box, grown by `grow` except at the floor."""
    return ((p[:, 0] > box[0] - grow) & (p[:, 0] < box[1] + grow)
            & (p[:, 1] > box[2] - grow) & (p[:, 1] < box[3] + grow)
            & (p[:, 2] > box[4]) & (p[:, 2] < box[5] + grow))


def verdict(ok):
    """Return the report tag for a check."""
    return 'OK  ' if ok else 'WARN'


def main(argv=None):
    """Collect data, compare it with the URDF and the sim, and print the report."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=15.0, help='seconds to collect')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--cloud-topic', default='/l2/points')
    parser.add_argument('--imu-topic', default='/l2/imu')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--wall-side', choices=['none', 'left', 'right'], default='none',
                        help='a straight wall parallel to the robot on this side')
    parser.add_argument('--wall-distance', type=float, default=None,
                        help='tape-measured distance (m) from the robot centreline to the wall')
    parser.add_argument('--plane', choices=['ceiling', 'floor'], default='ceiling',
                        help='horizontal plane to fit (the upright L2 hardly sees the floor)')
    parser.add_argument('--ceiling-height', type=float, default=None,
                        help='tape-measured ceiling height (m) above the floor at the robot')
    parser.add_argument('--plane-radius', type=float, nargs=2, default=None,
                        help='horizontal range (m) of plane points used for the fit; default '
                        '0..3 for the ceiling, 0.5..10 for the floor')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    import tf2_ros

    rclpy.init()
    node = Node('l2_mount_check')
    if args.use_sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    clouds, cloud_times, imu_acc, imu_gyro, imu_times = [], [], [], [], []
    node.create_subscription(
        PointCloud2, args.cloud_topic,
        lambda m: (clouds.append(m), cloud_times.append(time.monotonic())),
        qos_profile_sensor_data)

    def on_imu(m):
        imu_acc.append((m.linear_acceleration.x, m.linear_acceleration.y,
                        m.linear_acceleration.z))
        imu_gyro.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        imu_times.append(time.monotonic())
        if len(imu_acc) == 1:
            on_imu.frame = m.header.frame_id

    on_imu.frame = None
    node.create_subscription(Imu, args.imu_topic, on_imu, qos_profile_sensor_data)

    print(f'Collecting {args.duration:.0f} s from {args.cloud_topic} and {args.imu_topic} '
          '- keep the robot still...', flush=True)
    t_end = time.monotonic() + args.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    if len(clouds) < 2 or len(imu_acc) < 10:
        print(f'Not enough data: {len(clouds)} clouds, {len(imu_acc)} IMU messages. Is the '
              'driver (or simulation) running?')
        return 2

    def lookup(frame):
        tf = tf_buffer.lookup_transform(args.base_frame, frame, rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(q.x, q.y, q.z, q.w)

    lidar_frame = clouds[-1].header.frame_id
    try:
        s_pos, s_rot = lookup(lidar_frame)
        _, imu_rot = lookup(on_imu.frame)
    except Exception as exc:  # noqa: BLE001
        print(f'No TF from {args.base_frame} to the sensor frames: {exc}. Is '
              'robot_state_publisher running (robot_model:=true)?')
        return 2

    # Stack the last clouds (up to ~3 s) in base_footprint, through the URDF transform.
    recent = clouds[-max(2, min(len(clouds), 20)):]
    per_cloud = [cloud_xyz(m) for m in recent]
    pts_l = np.vstack([p for p, _, _ in per_cloud])
    pts_b = pts_l @ s_rot.T + s_pos
    fields = per_cloud[-1][1]
    spans = [s for _, _, s in per_cloud if s is not None]

    rows = []

    def row(name, measured, expected, ok, note=''):
        rows.append((verdict(ok), name, measured, expected, note))

    # 1. Horizontal plane: the ceiling (above the L2) or the floor
    horiz = np.hypot(pts_b[:, 0], pts_b[:, 1])
    box = load_crop_box()
    radius = args.plane_radius or ((0.0, 3.0) if args.plane == 'ceiling' else (0.5, 10.0))
    near = (horiz > radius[0]) & (horiz < radius[1])
    if args.plane == 'floor':
        plane_z = 0.0
    elif args.ceiling_height is not None:
        plane_z = args.ceiling_height
    else:
        # The ceiling is the highest dense layer above the L2.
        high = near & (pts_b[:, 2] > s_pos[2] + 0.5)
        plane_z = float(np.percentile(pts_b[high, 2], 90)) if high.sum() else None
    cand = near & (np.abs(pts_b[:, 2] - plane_z) < 0.10) if plane_z is not None \
        else np.zeros(len(pts_b), dtype=bool)
    if box is not None:
        cand &= ~in_box(pts_b, box)
    if cand.sum() < 100:
        print(f'Only {int(cand.sum())} {args.plane} points: the mount may be far off, or the '
              f'{args.plane} is not visible. Check in RViz.')
        plane = None
    else:
        n, d, sigma, used = fit_plane(pts_b[cand])
        offset, real_rot, corr = plane_correction(n, d, s_pos, s_rot)
        roll_e, pitch_e = roll_pitch(corr)
        urdf_tilt, real_tilt = tilt_from_vertical(s_rot), tilt_from_vertical(real_rot)
        plane = (sigma, used)
        if args.plane == 'floor':
            row('L2 height above floor [m]', f'{offset:.3f}', f'{s_pos[2]:.3f}',
                abs(offset - s_pos[2]) <= 0.01, 'tol 1 cm')
        elif args.ceiling_height is not None:
            height = args.ceiling_height + offset
            row('L2 height above floor [m]', f'{height:.3f}', f'{s_pos[2]:.3f}',
                abs(height - s_pos[2]) <= 0.02,
                f'tol 2 cm; ceiling {-offset:.3f} m above the L2')
        else:
            row('L2 distance to ceiling [m]', f'{-offset:.3f}', '(tape)', True,
                'pass --ceiling-height to check the L2 height')
        row('L2 tilt from vertical [deg]', f'{real_tilt:.2f}', f'{urdf_tilt:.2f}',
            abs(real_tilt - urdf_tilt) <= 0.5, f'tol 0.5 deg; from the {args.plane}')
        row('mount roll error [deg]', f'{roll_e:+.2f}', '0.00', abs(roll_e) <= 0.5,
            f'+ = {args.plane} seen rising to the right')
        row('mount pitch error [deg]', f'{pitch_e:+.2f}', '0.00', abs(pitch_e) <= 0.5,
            f'+ = {args.plane} seen rising ahead')

    # 2. IMU gravity
    acc = np.mean(imu_acc, axis=0)
    expected_acc = imu_rot.T @ np.array([0.0, 0.0, G])
    m_tilt, m_spin = gravity_angles(acc)
    e_tilt, e_spin = gravity_angles(expected_acc)
    spin_err = (m_spin - e_spin + 180.0) % 360.0 - 180.0
    row('IMU |accel| [m/s^2]', f'{np.linalg.norm(acc):.3f}', f'{G:.3f}',
        abs(np.linalg.norm(acc) - G) < 0.3, '~1.0 would mean units of g')
    row('IMU tilt from vertical [deg]', f'{m_tilt:.2f}', f'{e_tilt:.2f}',
        abs(m_tilt - e_tilt) <= 1.0, 'tol 1 deg; includes floor slope')
    if e_tilt > 5.0:
        row('L2 spin about own axis [deg]', f'{m_spin:+.2f}', f'{e_spin:+.2f}',
            abs(spin_err) <= 2.0, f'= l2_lidar yaw on the mount; error {spin_err:+.2f}')
    else:
        row('L2 spin about own axis [deg]', 'n/a', 'n/a', True,
            'mount not tilted: read the yaw from the wall check')

    # 3. Wall
    if args.wall_side != 'none':
        sign = 1.0 if args.wall_side == 'left' else -1.0
        # The upright L2 sees side walls from its own height (0.80 m) up; 0.9-2.0 m avoids
        # most furniture and stays below the ceiling. |y| > 0.30 m clears the robot.
        band = (pts_b[:, 2] > 0.9) & (pts_b[:, 2] < 2.0) & (pts_b[:, 0] > -1.0) \
            & (pts_b[:, 0] < 2.0) & (sign * pts_b[:, 1] > 0.30) & (sign * pts_b[:, 1] < 4.0)
        if band.sum() < 30:
            row('wall heading [deg]', 'no wall points', '0.00', False,
                f'nothing found on the {args.wall_side}')
        else:
            yaw, offset, wsig, wn = fit_line_xy(pts_b[band][:, :2])
            row('wall heading [deg]', f'{yaw:+.2f}', '0.00', abs(yaw) <= 0.5,
                f'{wn} points; + = wall turning left; robot must be parallel')
            if args.wall_distance is not None:
                row('wall distance [m]', f'{abs(offset):.3f}', f'{args.wall_distance:.3f}',
                    abs(abs(offset) - args.wall_distance) <= 0.02, 'tol 2 cm')
            else:
                row('wall distance [m]', f'{abs(offset):.3f}', '(tape)', True,
                    'compare with a tape measure')

    # 4. Cloud stream
    dt = np.diff(cloud_times)
    rate = 1.0 / float(np.median(dt)) if len(dt) else 0.0
    sizes = [len(p) for p, _, _ in per_cloud]
    coverage = float(np.median([azimuth_coverage(p[:, :2]) for p, _, _ in per_cloud]))
    row('cloud rate [Hz]', f'{rate:.2f}', '5.55 (sim: 1 sweep)', abs(rate - 5.55) < 0.3,
        'set cloud_scan_num for full sweeps')
    row('azimuth per cloud [deg]', f'{coverage:.0f}', '360', coverage >= 350,
        'from points per 5-deg bin')
    row('points per cloud', f'{int(np.median(sizes))}', '11520 (sim)', True, '')
    row('per-point "time" field', 'yes' if 'time' in fields else 'no', 'yes (hardware)',
        True, f'span {np.median(spans) * 1000:.0f} ms' if spans else 'sim has none')
    if box is not None:
        on_robot = float(in_box(pts_b, box).mean())
        near = float((in_box(pts_b, box, grow=0.10) & ~in_box(pts_b, box)
                      & (pts_b[:, 2] > 0.05)).mean())
        row('points on the robot', f'{on_robot:.1%}', '~0.6% (sim)', True,
            'removed by the self-filter')
        row('points just outside the box', f'{near:.2%}', '~0', near < 0.005,
            'self-hits the box misses, or real objects within 10 cm')

    # 5. Noise
    gyro = np.asarray(imu_gyro)
    imu_rate = (len(imu_times) - 1) / (imu_times[-1] - imu_times[0])
    row('IMU rate [Hz]', f'{imu_rate:.0f}', '250 (sim)', abs(imu_rate - 250) < 25, '')
    row('gyro std [rad/s]', ' '.join(f'{v:.4f}' for v in gyro.std(axis=0)), '0.002 (sim)',
        True, f'bias {np.linalg.norm(gyro.mean(axis=0)):.4f} rad/s')
    row('accel std [m/s^2]', ' '.join(f'{v:.3f}' for v in np.std(imu_acc, axis=0)),
        '0.02 (sim)', True, '')
    if plane is not None:
        row(f'{args.plane} residual [m]', f'{plane[0]:.4f}', '0.01 (sim noise)', True,
            f'{plane[1]} {args.plane} points')

    print(f'\nL2 frame {lidar_frame}, IMU frame {on_imu.frame}, {len(clouds)} clouds, '
          f'{len(imu_acc)} IMU samples\n')
    w = max(len(r[1]) for r in rows)
    print(f'      {"check":{w}}  {"measured":>22}  {"URDF / sim":>20}  note')
    for v, name, meas, exp, note in rows:
        print(f'{v}  {name:{w}}  {meas:>22}  {exp:>20}  {note}')
    warns = sum(1 for r in rows if r[0] == 'WARN')
    print(f'\n{warns} warning(s). See docs/hardware_l2_test.md for what each one means.')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if warns else 0


if __name__ == '__main__':
    sys.exit(main())
