# mecanum_controllers

ros2_control for the mecanum base: the ros2_control xacro, the configuration of
`mecanum_drive_controller` and `joint_state_broadcaster`, and the launch file that starts them.
See [decision 0002](../../docs/decisions/0002-ros2-control-with-measured-wheel-state.md).
ros2_control files live here, never in `mecanum_description`.

## Files

| Path | Content |
|---|---|
| `urdf/mecanum.ros2_control.xacro` | Macro `mecanum_ros2_control(prefix, hardware)`: one system with the four wheel joints, each with a velocity command interface and position and velocity state interfaces. `hardware` is `mock` (`mock_components/GenericSystem`) or `sim` (`gz_ros2_control/GazeboSimSystem`, plus the gz_ros2_control plugin). |
| `urdf/mecanum_mock.urdf.xacro` | The description plus the macro on mock hardware, for dry runs |
| `config/controllers.yaml` | Controller manager at 50 Hz, the drive controller (geometry, limits, frames, covariances) and the broadcaster |
| `launch/controllers.launch.py` | The spawners; with `hardware:=mock` also robot_state_publisher and `ros2_control_node` |
| `test/test_mock_kinematics.py` | Launch test on mock hardware |

The simulation includes the macro from `mecanum_simulation/urdf/mecanum_sim.urdf.xacro` and
starts `controllers.launch.py` once the robot is spawned. The ESP32 hardware plugin
(`mecanum_hardware`) does not exist yet; see `docs/encoder_upgrade.md`, section 2.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | in | The drive controller's `~/reference`. All velocity sources publish here; from plan Phase 4 the safety gate sits in between. |
| `/wheel/odometry` | `nav_msgs/Odometry` | out | From the measured wheel states, `odom` → `base_footprint` |
| `/tf` | `odom → base_footprint` | out | `enable_odom_tf: true` until the EKF takes it over in plan Phase 3 |
| `/joint_states` | `sensor_msgs/JointState` | out | From `joint_state_broadcaster`, 50 Hz |

The controller applies the speed limits in `controllers.yaml` (0.3 m/s, 0.3 m/s, 0.8 rad/s,
stand-ins) and stops the wheels when no reference arrives for 0.25 s
(`reference_timeout`). Its inverse kinematics are those of `mecanum_base/kinematics.py`
(X-roller layout), and the test checks that.

## Run

```bash
# Dry run without Gazebo (wall clock)
ros2 launch mecanum_controllers controllers.launch.py hardware:=mock
ros2 control list_controllers
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
ros2 topic echo /joint_states

# In simulation this is started by mecanum_simulation/sim.launch.py
ros2 launch mecanum_bringup robot.launch.py gui:=false
```

## Tests

`colcon test --packages-select mecanum_controllers` runs the ament lint suite and
`test_mock_kinematics.py`, in its own DDS domain (`ROS_DOMAIN_ID=91`) so that another
controller manager on the machine cannot interfere. On mock hardware it checks:
- the wheel speeds for pure forward, strafe, rotation and a combined twist against
  `mecanum_base.kinematics.inverse`, in sign and magnitude;
- that `/wheel/odometry` returns the commanded twist;
- that the speed limits apply;
- that the wheels stop when the references stop.
