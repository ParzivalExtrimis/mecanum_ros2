# mecanum_localization

This package holds the perception and localization for the mecanum base:
- **Phase 3:** the L2 self-filter and the cloud-to-scan conversion.
- **Phase 4:** LIO (rko_lio) and its planar odometry adapter, plus `gt_compare` to measure
  drift in simulation.
- **Later:** slam_toolbox mapping in phase 6, and slam_toolbox or AMCL localization in phase 7.

It uses upstream nodes where they exist. Topics are identical with the real L2 driver,
`mecanum_lidar`, and the simulated L2 in `mecanum_simulation`.

## Data flow

```
/l2/points            PointCloud2, l2_lidar       real driver or Gazebo
  -> self_filter      pcl_ros CropBox (negative): drop points on the robot
/l2/points_filtered   PointCloud2, l2_lidar       -> LIO (phase 4), costmaps (phase 7)
  -> cloud_to_scan    pointcloud_to_laserscan: fold a height band into a 2D scan
/scan                 LaserScan, base_footprint   -> slam_toolbox / AMCL (phases 6, 7)

/l2/points_filtered + /l2/imu
  -> rko_lio          lidar-inertial odometry, 3D, frame lio_odom; its TF goes to /lio/tf_unused
/lio/raw_odom         Odometry, lio_odom -> base_footprint
  -> lio_odom_adapter keep x, y, yaw; start at the origin
/odom                 Odometry, odom -> base_footprint, plus the TF odom -> base_footprint
```

TF ownership: the adapter is the only publisher of `odom -> base_footprint`. rko_lio has no
switch to turn its TF off, so `lio.launch.py` remaps its `/tf` to `/lio/tf_unused`; `lio_odom`
never appears in the TF tree. There are no encoders, so wheel commands are never odometry.

## Self-filter: `config/self_filter.yaml`

The L2 hangs inverted under its bracket at the rear, 0.93 m up, tilted 30° forward. It
therefore sees the chassis plate, the wheels, the mast and the bracket. The filter drops every
point inside a box in `base_footprint`:

| Axis | Range | Basis |
|---|---|---|
| x | -0.33 to 0.33 m | Chassis length 0.605 m, plus a stand-in margin |
| y | -0.27 to 0.27 m | Wheels at y ±0.247 m, plus a stand-in margin |
| z | -0.05 to 1.00 m | Bracket top at 0.954 m |

The filter transforms the cloud into `base_footprint`, crops it, and publishes it back in
`l2_lidar`. LIO therefore sees sensor-frame points.

This one node is the place to swap in a URDF-based self-filter once the arm is mounted.

## Cloud to scan: `config/cloud_to_scan.yaml`

- **Band.** Points between 0.10 and 0.50 m above the floor, a stand-in, become a 360° scan in
  `base_footprint` at 0.5° resolution.
- **Rear coverage is limited.** In its own frame the L2 sees the hemisphere on its dome side.
  Tilted 30° forward, a point at horizontal distance r and azimuth φ from the L2 (φ = 0 ahead)
  is in view only if its height z satisfies:
  ```
  z ≤ 0.93 + r · tan(30°) · cos φ
  ```
  Ahead, it sees up to 30° above the horizontal. To the sides, it sees everything up to its own
  height of 0.93 m. Toward the rear, the limit drops with distance, so **low, near objects are
  visible and tall or far ones are not**. Straight back, the floor leaves the view about 1.6 m
  from the L2; at 45° rear-left or rear-right, about 2.3 m.
- **Pole shadow.** The pole stands 66 mm behind the L2. It blocks a sector about **60° wide
  straight back**, roughly 150° to 210° azimuth around the L2. Nothing there is ever visible.
- **Measured in sim with a 0.3 m box at 45° rear-right:**

  | Distance from the L2 | Result |
  |---|---|
  | 1.0 m | Fully visible and in `/scan` |
  | 2.0 m | Lowest 0.21 m visible; in `/scan` |
  | 2.6 m | Invisible |
  | 1.0 m straight back | Invisible, because of the pole shadow |

  Phase 7 accounts for this in its rear safety design.

## LIO: `config/rko_lio.yaml`, `launch/lio.launch.py`

rko_lio 0.4.0 from apt, `online_node`, started directly: its own launch file takes a flat
config and no remappings.

- **Inputs.** `/l2/points_filtered` (frame `l2_lidar`) and `/l2/imu` (frame `l2_imu`). The
  extrinsics come from `/tf_static`: rko_lio reads the frames from the message headers and looks
  them up against `base_frame: base_footprint`. All of these joints are fixed.
- **Output.** `/lio/raw_odom` in `lio_odom`. The pose is 3D; the twist is in `base_footprint`.
- **Simulation settings.**
  - `deskew: false`. The simulated cloud has no per-point time field and no motion distortion.
  - `initialization_phase: false`. The initialization estimates IMU bias and gravity from the
    first two scans and assumes the robot is still. LIO starts with Gazebo while the robot is
    still settling after the spawn, which gave an accelerometer bias of 4.3 m/s² (0.007 when
    started at rest). The simulated IMU has no bias, so starting from identity is right.
  - `min_range` 0.3 m, `max_range` 30 m, `voxel_size` 0.25 m: stand-ins for indoors. The
    upstream defaults (1.0, 100, 1.0) are outdoor values.
- **Hardware: `config/rko_lio_hardware.yaml`.** Loaded by `mecanum_bringup
  hardware_sensors.launch.py lio:=true`. It differs only in `deskew: true` with the driver's
  `time` field (float32 seconds after the header stamp, which the SDK sets to the cloud's
  start; rko_lio recognises `t`, `time`, `timestamp`, `timestamps` and `stamps`),
  `lidar_timestamps.force_relative: true`, and `initialization_phase: true`, with the robot
  standing still at start-up. Keep the other values of the two files in step. A sim dry run
  with a synthetic `time` field loaded it with deskewing on, and the at-rest initialization
  found an accelerometer bias of 0.01 m/s². The procedure is `docs/hardware_lio_test.md`.
- rko_lio sometimes crashes on shutdown (exit code -11). It is harmless.
- At start-up rko_lio logs a few "TF lookup error ... from_frame" warnings until the first
  messages arrive. They are harmless.

## Odometry adapter: `lio_odom_adapter`

Flattens rko_lio's 3D pose to x, y and yaw, makes the first pose the origin, and publishes
`nav_msgs/Odometry` plus the TF. The math is in `planar_math.py`, with unit tests.

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/lio/raw_odom` | rko_lio odometry |
| `odom_topic` | `/odom` | Output; `/lio/odom` once an EKF fuses it (`docs/encoder_upgrade.md`) |
| `publish_tf` | `true` | Publish `odom -> base_footprint`; `false` hands the TF to the EKF |
| `odom_frame`, `base_frame` | `odom`, `base_footprint` | Output frames |
| `pose_covariance_diagonal` | x, y 0.0025; yaw 0.0012; z, roll, pitch 1e6 | Stand-in |
| `twist_covariance_diagonal` | vx, vy 0.0025; wz 0.0025; others 1e6 | Stand-in |

The twist is passed through as vx, vy and wz. If rko_lio ever reset, the pose would jump;
`reset_on_registration_error` is off.

## Drift against ground truth: `gt_compare` (simulation only)

Pairs `/odom` with `/sim/ground_truth/odom`, interpolated at each estimate's stamp, both taken
relative to the first estimate. It reports the path length, final and maximum position error
(also as a percentage of the path), yaw error, RMS error, mean twist error and jumps, every
`report_period` seconds and on exit (`duration`, or Ctrl-C / SIGTERM). The run passes when the
final and the maximum position error are under `max_drift_percent` (1%) of the path, the yaw
error is under `max_yaw_error_deg` (2°), and there are no jumps. Both targets are stand-ins.
`csv_path` writes the paired trajectory.

```bash
ros2 launch mecanum_bringup robot.launch.py gui:=false
ros2 run mecanum_localization gt_compare --ros-args -p use_sim_time:=true
ros2 launch mecanum_base motion_test.launch.py use_sim_time:=true pattern:=loop laps:=2 \
  measure_odom_topic:=/sim/ground_truth/odom
```

## Phase 4 results in simulation (2026-10-15): targets not met

| Run | Path | Result |
|---|---|---|
| `loop`, 2 laps, restroom world | 24.2 m, 720° | Final error 4.48 m (18.5%), max yaw error 9.3°. FAIL. |
| `corridor`, 4 m out and back, facing +x | 8.0 m | LIO saw almost no motion: max error 4.0 m (50%). FAIL. |
| `corridor`, 3 m out and back, facing +x | 6.0 m | 0.03 m read at the 3 m point, 0.05 m at 2 m (the 4H stage 2 reference). FAIL. |

- **Where it fails.** Driving along the corridor toward +x, forward or backward, LIO reports
  almost no motion. About 3.9 m of the loop's error comes from the first 4 m segment, and most
  of the rest from the same segment in lap 2. Facing −x, where the end wall is close, while
  strafing in the lobby and while rotating, it tracks.
- **Likely cause: the simulated scan pattern.** The simulated L2 is a fixed 360 × 32 grid,
  identical in every sweep. On flat surfaces, such as the floor (about 55% of the filtered
  points), the walls and the ceiling, its scan lines stay at the same place relative to the
  sensor while the robot moves. rko_lio's point-to-point matching then aligns them with the
  lines already in its map, which reads as "no motion". The only true constraint along the
  corridor is the far end wall, about 150 points per scan.
- **Evidence.** On the out-and-back run, removing the floor points before LIO recovered about
  80% of the motion (max error 0.89 m, 11%). Changing `voxel_size` did not help: 0.1 m gave
  42%, 0.5 m 50%.
- **Not yet known.** Whether the real L2 behaves the same. Its pattern shifts between
  revolutions, so it should suffer less. Stage 2 of `docs/hardware_lio_test.md` tests exactly
  this.
- **What works.** TF ownership as designed (`/tf` publishers: robot_state_publisher and
  lio_odom_adapter; rko_lio only on `/lio/tf_unused`). `publish_tf:=false` publishes no TF.
  When LIO tracks, its twist matches ground truth (0.149 against 0.150 m/s).
- **CPU.** rko_lio used 9–15% of one laptop core and about 29 MB.

## Usage

```bash
# Normally started by the bringup:
ros2 launch mecanum_bringup robot.launch.py rviz:=true

# On their own:
ros2 launch mecanum_localization perception.launch.py use_sim_time:=true
ros2 launch mecanum_localization lio.launch.py use_sim_time:=true
```

| `perception.launch.py` argument | Default |
|---|---|
| `use_sim_time` | `false` |
| `cloud_topic` | `/l2/points` |
| `filtered_cloud_topic` | `/l2/points_filtered` |
| `scan_topic` | `/scan` |

| `lio.launch.py` argument | Default |
|---|---|
| `use_sim_time` | `false` |
| `config` | `config/rko_lio.yaml` |
| `odom_topic` | `/odom` |
| `publish_tf` | `true` |
