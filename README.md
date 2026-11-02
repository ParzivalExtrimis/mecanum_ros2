# mecanum_ros2

A ROS 2 navigation stack for a washroom robot on a 4-wheel mecanum base. The robot navigates
from a start pose to a named staging pose, performs a precise final approach (dock) at the
toilet, runs a task, and returns home. A toilet-cleaning arm will be mounted later. It is not
part of the robot yet, but the design leaves room for it.

The build is **simulation only** for now: everything is developed and tested in Gazebo, end to
end. The plan is in [docs/plan.md](docs/plan.md).

## Robot facts

The robot model is in `mecanum_description`, built from the reference URDF
`src/mecanum_description/refs/washroom_robot.urdf` and the sensor placement and BOM document
[docs/Washroom_Robot_Prototype_Sensors_and_BOM.pdf](docs/Washroom_Robot_Prototype_Sensors_and_BOM.pdf).

- **Base:** a 0.79 × 0.51 m body, 0.74 m tall, on four 0.205 m mecanum wheels in an X-roller
  layout (wheelbase 0.472 m, track 0.430 m). The motors have **position and velocity
  encoders**. In simulation the drive runs on ros2_control; the hardware path keeps the
  open-loop driver until the ESP32 hardware plugin exists.
- **Motor control:** an ESP32 on the network, with each motor individually addressable. The
  protocol is pending the firmware rewrite.
- **Sensors:**
  - **Unitree L2 4D LiDAR,** upright on the top of the robot at 0.80 m, with a built-in 6-axis
    IMU;
  - **RealSense D435i** depth camera, at the front, pitched 8° down;
  - **RPLIDAR C1** 2D lidar, at 0.13 m in a 200° front slot;
  - **four ultrasonic rangers,** two facing forward and two facing back;
  - **four downward cliff ToF sensors;**
  - **front and rear bumper planks;**
  - **an E-stop.**
- **Onboard computer:** a Jetson Orin Nano Super.
- **Odometry:** wheel odometry fused with the IMUs in an EKF (plan Phase 3). Wheel commands are
  never published as odometry.

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
| `mecanum_description` | done: model v2 with all sensor frames (plan Phase 0) | URDF/xacro, reference URDF, test display launch |
| `mecanum_simulation` | done: ros2_control drive and the sensor suite (plan Phase 2) | Gazebo worlds, sim-only xacro (mecanum friction, sensors, ground truth), bridge, sim-only sensor nodes |
| `mecanum_base` | done (open-loop driver, hardware path only) | ESP32 driver node, mecanum kinematics, motion test |
| `mecanum_controllers` | done, Phase 1 | ros2_control xacro (mock or Gazebo hardware), `mecanum_drive_controller` and `joint_state_broadcaster` config, mock kinematics test |
| `mecanum_interfaces` | messages done; safety and mission interfaces in Phases 4 and 7 | Motor command and debug messages, plus safety, task and mission interfaces later |
| `mecanum_lidar` | driver done, hardware test paused | Unitree L2 driver (`/l2/points`, `/l2/imu`, no TF) on the vendored SDK, bench-test launch, RViz config |
| `mecanum_localization` | self-filter, cloud-to-scan, LIO pipeline done; EKF and SLAM in Phases 3 and 5 | Self-filter, cloud-to-scan, LIO and its odom adapter, EKF, slam_toolbox, AMCL |
| `mecanum_safety` | planned, Phase 4 | Safety supervisor: cliffs, bumpers, E-stop, sensor loss |
| `mecanum_navigation` | planned, Phase 6 | Nav2 parameters, collision monitor, docking, keepout mask, maps |
| `mecanum_mission` | planned, Phase 7 | Mission executive and placeholder task server |
| `mecanum_bringup` | `robot.launch.py` done | Top-level launch files and RViz configs |

## Design decisions

- **[0001: Open-loop drive without ros2_control](docs/decisions/0001-open-loop-drive-without-ros2-control.md).**
  Superseded by 0002: it was made when the motors had no encoders.
- **[0002: ros2_control with measured wheel state](docs/decisions/0002-ros2-control-with-measured-wheel-state.md).**
  The simulation drives through `mecanum_drive_controller` on gz_ros2_control. Every velocity
  source publishes `TwistStamped` on `/cmd_vel`; wheel odometry owns `odom -> base_footprint`
  until the EKF.
- **[Encoder upgrade guide](docs/encoder_upgrade.md).** The steps to move to ros2_control with
  measured wheel state: `mecanum_controllers`, `mecanum_hardware`, stamped `cmd_vel` and an
  EKF. Plan Phase 1 follows its simulation steps.

## TF tree

Each link has exactly one publisher.

```
map -> odom -> base_footprint -> base_link -> {chassis_link, wheels, l2_mount -> l2_lidar -> l2_imu,
                                               d435i_link -> d435i_optical_frame, lidar_2d_link,
                                               ultrasonic_*, cliff_tof_*, bumper_*}
```

| Transform | Publisher |
|---|---|
| `map -> odom` | AMCL or slam_toolbox, never both (plan Phase 5 onward) |
| `odom -> base_footprint` | sim: `mecanum_drive_controller` from the measured wheel states; hardware: the LIO odom adapter; the EKF from plan Phase 3 |
| `base_footprint` and below | robot_state_publisher from the URDF |

In simulation `joint_state_broadcaster` publishes the measured wheel angles. On hardware,
until the ESP32 plugin exists, joint_state_publisher publishes zero angles.
The L2 driver (`mecanum_lidar`) publishes no TF. LIO packages broadcast their own TF, which is
remapped away in launch files.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dev/mecanum_ros2
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

These apt packages are needed for perception, LIO, mapping and navigation:

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

All sensor frames are included by default; each has a `use_<sensor>` xacro arg (see
`src/mecanum_description/README.md`). The URDF can be generated directly with xacro:

```bash
xacro $(ros2 pkg prefix mecanum_description)/share/mecanum_description/urdf/mecanum.urdf.xacro
xacro .../mecanum.urdf.xacro use_l2:=false use_d435i:=false   # without some sensors
```

[docs/cad_vs_urdf.png](docs/cad_vs_urdf.png) compares the CAD model with the URDF.

**Converting CAD exports.** The model is built from primitives and has no meshes. If meshes are
added, `tools/cad_stl_to_ros.py` converts STL exports in place from millimetres in the CAD
frame to metres in a link frame, given the CAD point that becomes the link origin (`--origin`)
and the axis mapping (`--axes`). The script refuses files it has already converted.

### Simulation: drive the base

```bash
ros2 launch mecanum_bringup robot.launch.py              # Gazebo GUI + robot + controllers
ros2 launch mecanum_bringup robot.launch.py gui:=false   # headless
```

In a second terminal, drive the robot with the keyboard. The drive controller takes
`TwistStamped` on `/cmd_vel`. Hold shift for strafing:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Or run the motion test. It drives forward, strafes left and rotates, returning to the start
after each step, and judges each step against Gazebo ground truth:

```bash
ros2 launch mecanum_base motion_test.launch.py use_sim_time:=true \
  measure_odom_topic:=/sim/ground_truth/odom
```

### Simulation: sensors and perception

`robot.launch.py` also starts the simulated sensors and the L2 perception pipeline. The topics
are those of the plan's sensor contract:

| Topic | Content |
|---|---|
| `/l2/points` | Raw cloud, one full sweep at 5.55 Hz |
| `/l2/imu` | IMU at 250 Hz |
| `/l2/points_filtered` | Cloud without the robot body |
| `/scan` | RPLIDAR C1 at 10 Hz |
| `/d435i/color/*`, `/d435i/depth/*`, `/d435i/imu` | D435i images, cloud and IMU |
| `/ultrasonic/*`, `/cliff/*` | The four ultrasonics and the four cliff ToF sensors, `Range` |
| `/bumper/front`, `/bumper/rear` | Bumper switches, `Bool` |

```bash
ros2 launch mecanum_bringup robot.launch.py rviz:=true   # RViz shows the sensors
```

`tools/sensor_check.py` checks the whole suite in the sensor test world: topics, frames, rates,
that no sensor sees the robot, and which sensors see each test object. See
`src/mecanum_simulation/README.md`.

The L2 stands upright on the top of the robot, at 0.80 m, in wide mode (−6° to 90° elevation).
It sees all around, but upward: the walls above the robot and the ceiling. It reaches low
objects only from about 3 m out, so it is not a near-field obstacle sensor; the 2D lidar, the
D435i and the ultrasonics take that role (plan, "Sensor roles"). It sees almost none of the
robot itself. See `src/mecanum_localization/README.md`.

To dry-run the controllers without Gazebo, on mock hardware:

```bash
ros2 launch mecanum_controllers controllers.launch.py hardware:=mock
```

### Hardware: test the Unitree L2

Hardware work is paused; see [docs/hardware_transition.md](docs/hardware_transition.md). The
procedures below are kept ready for when it resumes.

The full procedure is in [docs/hardware_l2_test.md](docs/hardware_l2_test.md). It covers
configuring the L2 to match the simulation, verifying the mount with `tools/l2_mount_check.py`,
and comparing sim with real. The quick bench test:

Connect the L2 to the computer's Ethernet port. The L2 ships in Ethernet mode, at
192.168.1.62. The computer needs 192.168.1.2 on that port; see "Ethernet setup" in the
[mecanum_lidar README](src/mecanum_lidar/README.md), which also covers USB-UART
(`connection:=serial`).

```bash
ros2 launch mecanum_lidar l2.launch.py                       # driver + RViz, L2 on a desk
ros2 launch mecanum_lidar l2.launch.py robot_model:=true fixed_frame:=base_footprint  # mounted
```

### Hardware: LIO with the robot pushed by hand

The procedure is in [docs/hardware_lio_test.md](docs/hardware_lio_test.md). Start with the
robot standing still:

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py lio:=true
tools/lio_marks.py      # press Enter at each tape mark to log /odom
```

### Mapping, navigation and mission runs

The exact launch sequences will be added here as plan Phases 5 to 8 land.

## Development phases

Development runs in simulation only; the plan is in [docs/plan.md](docs/plan.md). Each phase
ends with `colcon build`, `colcon test`, and a checkpoint before the next phase starts. The
first plan (phases 1 to 4, for the previous robot model) built the drive, the L2 simulation,
the self-filter and the LIO pipeline; its paused hardware work is kept in
[docs/hardware_transition.md](docs/hardware_transition.md).

| Phase | Content | Status |
|---|---|---|
| 0 | Model v2 description from the reference URDF | done 2026-10-27 |
| 1 | Drive on encoders: ros2_control in simulation | done 2026-10-28 |
| 2 | Simulated sensor suite: 2D lidar, D435i, ultrasonics, cliff ToF, bumpers | done 2026-10-30 |
| 3 | State estimation: EKF on wheel odometry and IMU; LIO re-evaluated | |
| 4 | Safety layer: cliffs, bumpers, E-stop, sensor loss | |
| 5 | Mapping with slam_toolbox on the 2D lidar | |
| 6 | Localization and Nav2 | |
| 7 | Docking and mission executive | |
| 8 | End-to-end acceptance in simulation | |
