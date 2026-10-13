# mecanum_localization

This package holds the perception and localization for the mecanum base:
- **Now, from phase 3:** the L2 self-filter and the cloud-to-scan conversion.
- **Later:** LIO and its odometry adapter in phase 4, slam_toolbox mapping in phase 6, and
  slam_toolbox or AMCL localization in phase 7.

It uses upstream nodes and configs only. Topics are identical with the real L2 driver,
`mecanum_lidar`, and the simulated L2 in `mecanum_simulation`.

## Data flow

```
/l2/points            PointCloud2, l2_lidar       real driver or Gazebo
  -> self_filter      pcl_ros CropBox (negative): drop points on the robot
/l2/points_filtered   PointCloud2, l2_lidar       -> LIO (phase 4), costmaps (phase 7)
  -> cloud_to_scan    pointcloud_to_laserscan: fold a height band into a 2D scan
/scan                 LaserScan, base_footprint   -> slam_toolbox / AMCL (phases 6, 7)
```

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
  Tilted 30° forward, it sees well ahead and to the sides. Behind the robot it only sees below
  about 30° under the horizontal, or 24° in wide mode. Rear obstacles therefore enter the scan
  band only between about 0.75 and 1.4 m behind the L2. Phase 7 accounts for this in the rear
  safety design.

## Usage

```bash
# Normally started by the bringup:
ros2 launch mecanum_bringup robot.launch.py rviz:=true

# On its own:
ros2 launch mecanum_localization perception.launch.py use_sim_time:=true
```

| Argument | Default |
|---|---|
| `use_sim_time` | `false` |
| `cloud_topic` | `/l2/points` |
| `filtered_cloud_topic` | `/l2/points_filtered` |
| `scan_topic` | `/scan` |
