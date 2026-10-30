# mecanum_bringup

Top-level launch files and RViz configurations for the mecanum base.

| Launch file | Phase | Starts |
|---|---|---|
| `robot.launch.py` | 1 | Simulation (ros2_control drive) or hardware base (open-loop driver), L2 perception and LIO odometry |
| `hardware_sensors.launch.py` | paused hardware stages 3H, 4H | Real L2: driver, URDF TF, self-filter, scan and the sensors RViz view; LIO with `lio:=true`. No drive node. |
| `mapping.launch.py` | 5, planned | Robot, EKF and slam_toolbox mapping |
| `navigation.launch.py` | 6, planned | Robot, EKF, localization and Nav2 |
| `mission.launch.py` | 7, planned | Navigation plus docking and the mission executive |

## `robot.launch.py`

```bash
ros2 launch mecanum_bringup robot.launch.py              # Gazebo + robot + controllers (sim)
ros2 launch mecanum_bringup robot.launch.py gui:=false   # headless
ros2 launch mecanum_bringup robot.launch.py rviz:=true   # also RViz
```

| Argument | Default | Meaning |
|---|---|---|
| `sim` | `true` | `true` for the Gazebo simulation, `false` for real hardware |
| `rviz` | `false` | Start RViz |
| `rviz_config` | `rviz/sensors.rviz` | Robot model, L2 clouds, `/scan`, D435i cloud and image, the ultrasonic and cliff ranges. `rviz/robot.rviz` shows only the model and TF. The bumpers have no RViz display: `ros2 topic echo /bumper/front`. |
| `gui` | `true` | Gazebo GUI; sim only |
| `world` | restroom | Gazebo world; sim only |
| `use_l2` | `true` | Include the Unitree L2 in the model |
| `use_d435i`, `use_lidar_2d`, `use_ultrasonics`, `use_cliff_tof`, `use_bumpers` | `true` | Include that sensor in the model; sim only |
| `use_l2_scan` | `false` | Also make a 2D scan from the L2 cloud, on `/l2/scan`, for comparison |
| `lio` | `true` | Run LIO odometry; needs `use_l2` |

With `sim:=true`, the launch starts:
- the simulation from `mecanum_simulation/sim.launch.py`, including the controllers:
  `mecanum_drive_controller` takes `TwistStamped` on `/cmd_vel` and publishes
  `/wheel/odometry` and the `odom -> base_footprint` TF; `joint_state_broadcaster` publishes
  `/joint_states`;
- the simulated sensors, on the plan's contract topics; `/scan` is the 2D lidar's;
- the L2 perception from `mecanum_localization/perception.launch.py`: the self-filter, from
  `/l2/points` to `/l2/points_filtered`;
- LIO odometry from `mecanum_localization/lio.launch.py`: rko_lio and the odometry adapter,
  which publish `/lio/odom` only, with no TF;
- everything on `use_sim_time:=true`.

With `sim:=false`, it starts robot_state_publisher, joint_state_publisher and the open-loop
`esp32_driver` with `transport:=esp32`, and LIO publishes `/odom` and the TF. The real
transport does not exist until the ESP32 firmware rewrite defines its protocol, so the driver
exits with an explanation.

Drive the robot with the keyboard, stamped. Hold shift for holonomic, strafing keys:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

The wheel odometry provides the `odom` frame. RViz starts with `base_footprint` as its fixed
frame, so that the same view also works on hardware without odometry. To watch the robot move
in RViz, set Global Options > Fixed Frame to `odom`.

The LIO configuration here is the simulation one: deskewing and the start-up initialization
are off. On hardware, use `hardware_sensors.launch.py lio:=true`, which loads
`rko_lio_hardware.yaml`.

## `hardware_sensors.launch.py`

This launch shows the real L2 the way `robot.launch.py rviz:=true` shows the simulated one. It
starts:
- the L2 driver from `mecanum_lidar`;
- robot_state_publisher and joint_state_publisher, for the URDF TF including the L2 mount;
- the self-filter and the cloud-to-scan conversion;
- RViz with `rviz/sensors.rviz`, showing the robot model, the raw and filtered clouds and `/scan`.

It does not start the drive node, so the robot does not move.

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py                                 # Ethernet (default)
ros2 launch mecanum_bringup hardware_sensors.launch.py connection:=serial serial_port:=/dev/ttyACM0  # USB-UART
```

Arguments: `connection`, `serial_port`, `lidar_ip`, `local_ip`, `params_file` (driver config,
default `mecanum_lidar/config/l2.yaml`), `rviz` and `rviz_config`, plus:

| Argument | Default | Meaning |
|---|---|---|
| `lio` | `false` | Also run LIO with the hardware config, for `/odom` and `odom -> base_footprint`. Start it with the robot standing still. Used by `docs/hardware_lio_test.md`. |
| `lio_config` | `mecanum_localization/config/rko_lio_hardware.yaml` | rko_lio parameters |
| `replay` | `false` | Leave out the driver and run on the bag's clock, to replay a recording: `ros2 bag play <bag> --clock --topics /l2/points /l2/imu` |

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py lio:=true     # push the robot by hand
```

Only one L2 driver can own the serial port. Stop any other `l2.launch.py` first.
