# mecanum_bringup

Top-level launch files and RViz configurations for the mecanum base.

| Launch file | Phase | Starts |
|---|---|---|
| `robot.launch.py` | 2, 3 | Simulation or hardware base, the drive node and L2 perception |
| `hardware_sensors.launch.py` | 3, H | Real L2: driver, URDF TF, self-filter, scan and the sensors RViz view. No drive node. |
| `mapping.launch.py` | 6, planned | Robot, LIO and slam_toolbox mapping |
| `navigation.launch.py` | 7, planned | Robot, LIO, localization and Nav2 |
| `mission.launch.py` | 8, planned | Navigation plus docking and the mission executive |

## `robot.launch.py`

```bash
ros2 launch mecanum_bringup robot.launch.py              # Gazebo + robot + driver (sim)
ros2 launch mecanum_bringup robot.launch.py gui:=false   # headless
ros2 launch mecanum_bringup robot.launch.py rviz:=true   # also RViz
```

| Argument | Default | Meaning |
|---|---|---|
| `sim` | `true` | `true` for the Gazebo simulation, `false` for real hardware |
| `rviz` | `false` | Start RViz |
| `rviz_config` | `rviz/sensors.rviz` | Robot model, raw and filtered L2 clouds, and `/scan`. `rviz/robot.rviz` shows only the model and TF. |
| `gui` | `true` | Gazebo GUI; sim only |
| `world` | restroom | Gazebo world; sim only |
| `use_l2` | `true` | Include the Unitree L2 in the model |
| `plant_file` | ideal plant | Simulated motor plant; sim only |

With `sim:=true`, the launch starts:
- the simulation from `mecanum_simulation/sim.launch.py`;
- `esp32_driver` with `transport:=sim`;
- the L2 perception from `mecanum_localization/perception.launch.py`: the self-filter, from `/l2/points` to `/l2/points_filtered`, then the scan conversion to `/scan`;
- everything on `use_sim_time:=true`.

With `sim:=false`, it starts robot_state_publisher, joint_state_publisher and `esp32_driver`
with `transport:=esp32`. The real transport does not exist until the ESP32 firmware rewrite
defines its protocol, so the driver exits with an explanation. The L2 driver and the hardware
launch arrive in the hardware phase.

Drive the robot with the keyboard. Hold shift for holonomic, strafing keys:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Until LIO runs in phase 4, there is no `odom` frame. RViz uses `base_footprint` as its fixed
frame, so the robot's motion is only visible in Gazebo.

## `hardware_sensors.launch.py`

This launch shows the real L2 the way `robot.launch.py rviz:=true` shows the simulated one. It
starts:
- the L2 driver from `mecanum_lidar`;
- robot_state_publisher and joint_state_publisher, for the URDF TF including the mast;
- the self-filter and the cloud-to-scan conversion;
- RViz with `rviz/sensors.rviz`, showing the robot model, the raw and filtered clouds and `/scan`.

It does not start the drive node, so the robot does not move.

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py                                 # Ethernet (default)
ros2 launch mecanum_bringup hardware_sensors.launch.py connection:=serial serial_port:=/dev/ttyACM0  # USB-UART
```

Arguments: `connection`, `serial_port`, `lidar_ip`, `local_ip`, `params_file` (driver config,
default `mecanum_lidar/config/l2.yaml`), `rviz` and `rviz_config`.

Only one L2 driver can own the serial port. Stop any other `l2.launch.py` first.
