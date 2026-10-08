# mecanum_ros2

A ROS 2 navigation stack for a 4-wheel mecanum base. The robot navigates from a start pose to a
named staging pose, performs a precise final approach (dock), runs a task, and returns home.
A toilet-cleaning arm will be mounted later. It is not part of the robot yet, but the design
leaves room for it.

The build is **simulation-first**. The motor firmware is being rewritten and the LiDAR is not
mounted yet. Everything is developed and tested in Gazebo. Hardware then replaces two pieces:
the motor transport and the LiDAR driver.

## Hardware facts

- **Base:** four mecanum wheels in an X-roller layout. The motors have **no encoders**, so
  there is no wheel feedback.
- **Motor control:** an ESP32 on the network, with each motor individually addressable. The
  protocol is pending the firmware rewrite.
- **LiDAR:** Unitree L2 4D LiDAR with a 360°×96° field of view, a 5.55 Hz sweep and a
  built-in 6-axis IMU.
- **Onboard computer:** a Jetson; the model and OS are to be confirmed.
- **Odometry:** LiDAR-inertial odometry (LIO) is the **only** source. Wheel commands are never
  published as odometry.

Unknown values are marked as stand-ins. See [STANDINS.md](STANDINS.md).

## Platform

| Item | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8), with ros_gz |

## Packages

| Package | Status | Responsibility |
|---|---|---|
| `mecanum_description` | done: base; phase 1: L2 sensor frames added | URDF/xacro, meshes, test display launch |
| `mecanum_simulation` | done, phase 2 | Gazebo world, sim-only xacro (mecanum friction, wheel controllers, ground truth), bridge, simulated ESP32 and motors |
| `mecanum_base` | done, phase 2; calibration tool in phase 5 | ESP32 driver node, mecanum kinematics, motion test |
| `mecanum_interfaces` | messages done, phase 2; actions in phase 8 | Motor command and debug messages, plus task and mission actions later |
| `mecanum_localization` | planned, phases 3, 4, 6 | Self-filter, cloud-to-scan, LIO and its odom adapter, slam_toolbox, AMCL |
| `mecanum_navigation` | planned, phase 7 | Nav2 parameters, collision monitor, docking, keepout mask, maps |
| `mecanum_mission` | planned, phase 8 | Mission executive and placeholder task server |
| `mecanum_bringup` | `robot.launch.py` done, phase 2 | Top-level launch files and RViz configs |

## Design decisions

- **[0001: Open-loop drive without ros2_control](docs/decisions/0001-open-loop-drive-without-ros2-control.md).**
  The motors have no encoders. A Python driver node with pluggable transports therefore owns
  the drive, and odometry comes from LIO only.
- **[Encoder upgrade guide](docs/encoder_upgrade.md).** These are the steps to move to
  ros2_control once the ESP32 reports measured wheel state: `mecanum_controllers`,
  `mecanum_hardware`, stamped `cmd_vel` and an EKF.

## TF tree

Each link has exactly one publisher.

```
map -> odom -> base_footprint -> base_link -> {wheels, l2_mast, l2_mount -> {l2_lidar, l2_imu}}
```

| Transform | Publisher |
|---|---|
| `map -> odom` | slam_toolbox or AMCL, never both (phase 6 onward) |
| `odom -> base_footprint` | LIO odom adapter only (phase 4 onward) |
| `base_footprint` and below | robot_state_publisher from the URDF |

The wheel joints are continuous but have no encoders, so joint_state_publisher publishes zero
angles for them. The Unitree driver and LIO packages broadcast their own TF, which is remapped
away in launch files.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dev/mecanum_ros2
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

These apt packages are needed from phase 3 onward:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
  ros-jazzy-opennav-docking ros-jazzy-opennav-docking-bt ros-jazzy-spatio-temporal-voxel-layer \
  ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan ros-jazzy-rko-lio
```

## Run

### View the robot description

```bash
ros2 launch mecanum_description display.launch.py
```

The L2 and its mast are included by default. The mast is a 725 mm pole at the rear edge of
the chassis plate. A bracket on top, tilted 20°, holds the L2 inverted, so it looks forward
and down at the floor ahead. The meshes come from the CAD; see
`src/mecanum_description/refs/mecanum_ros2_lidar_placement.jpeg`. The bracket top is about
0.85 m above the floor, which is the robot's height for the costmaps.

The URDF can be generated directly with xacro:

```bash
xacro $(ros2 pkg prefix mecanum_description)/share/mecanum_description/urdf/mecanum.urdf.xacro
xacro .../mecanum.urdf.xacro use_l2:=false                              # bare base
xacro .../mecanum.urdf.xacro l2_mast_x:=0.2875 l2_mast_yaw:=3.14159     # pole at the front
xacro .../mecanum.urdf.xacro l2_bracket_yaw:=3.14159                    # L2 cable toward the tip
```

`l2_bracket_yaw` is provisional; see `STANDINS.md`.

**Re-exporting the mast from CAD.** The CAD exports are in millimetres, in the CAD assembly
frame. Convert them in place to metres in the `l2_mast` frame:

```bash
tools/cad_stl_to_ros.py --origin=-5,-2115,-21 --axes=y,-x,z --scale 0.001 --report-l2 \
  src/mecanum_description/meshes/visual/725mmpole.stl \
  src/mecanum_description/meshes/visual/lidarsetup01.stl
```

The script refuses files it has already converted. If the bracket changes, copy the printed L2
face centre into `l2_face_x/y/z` in `unitree_l2.xacro`.

### Simulation: drive the base (phase 2)

```bash
ros2 launch mecanum_bringup robot.launch.py              # Gazebo GUI + robot + driver
ros2 launch mecanum_bringup robot.launch.py gui:=false   # headless
```

In a second terminal, drive the robot with the keyboard. Hold shift for strafing:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or run the motion test. It drives forward, strafes left and rotates, returning to the start
after each step, and judges each step against Gazebo ground truth:

```bash
ros2 launch mecanum_base motion_test.launch.py use_sim_time:=true \
  measure_odom_topic:=/sim/ground_truth/odom
```

To dry-run the driver without any simulator, using the mock transport:

```bash
ros2 launch mecanum_base base.launch.py transport:=mock
```

### Mapping, navigation and mission runs

The exact launch sequences will be added here as phases 6 to 8 land.

## Development phases

Each phase ends with `colcon build`, `colcon test`, and a checkpoint before the next phase
starts.

| Phase | Content | Status |
|---|---|---|
| 0 | Explore workspace, verify upstream packages | done |
| 1 | L2 sensor frames in the description, `STANDINS.md`, this README | done |
| 2 | Gazebo world, simulated drive plant, ESP32 driver with mock and sim transports, motion test | done |
| 3 | Simulated L2 cloud and IMU, self-filter, cloud-to-scan | next |
| 4 | LIO and odom adapter, evaluated against Gazebo ground truth | |
| 5 | Motor calibration tool, validated against injected plant errors | |
| 6 | Mapping with slam_toolbox, map saving | |
| 7 | Localization and Nav2 | |
| 8 | Docking and mission executive, end to end | |
| H | Hardware: ESP32 transport, Unitree driver, real mount pose, real calibration | when hardware is ready |
