# mecanum_bringup

Top-level launch files and RViz configurations for the mecanum base.

| Launch file | Phase | Starts |
|---|---|---|
| `robot.launch.py` | 2 | Simulation or hardware base, plus the drive node |
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
| `rviz` | `false` | Start RViz with `rviz/robot.rviz` |
| `gui` | `true` | Gazebo GUI; sim only |
| `world` | restroom | Gazebo world; sim only |
| `use_l2` | `true` | Include the Unitree L2 in the model |
| `plant_file` | ideal plant | Simulated motor plant; sim only |

With `sim:=true`, the launch starts:
- the simulation from `mecanum_simulation/sim.launch.py`;
- `esp32_driver` with `transport:=sim`;
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
