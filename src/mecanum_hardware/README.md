# mecanum_hardware

ros2_control hardware for the mecanum base, built around the wheel state of the **tapped motor
encoders** ([decision 0006](../../docs/decisions/0006-wheel-state-from-tapped-encoders.md)):
each motor's A and B encoder outputs reach the ESP32 through an optocoupler and a level
converter, and the MCU counts them.

| Part | Status | Content |
|---|---|---|
| `WheelEncoder` (`include/mecanum_hardware/wheel_encoder.hpp`) | done | One wheel's encoder rules, no ROS: a 64-bit count to radians, the velocity from the count change over the read period, an optional report delay, and the following-error check (integrated velocity command against the count). Unit tested. |
| `mecanum_hardware/SimEncoderSystem` | done | The Gazebo hardware plugin. It wraps gz_ros2_control's `GazeboSimSystem` and reports each wheel's state through a `WheelEncoder` fed with the count Gazebo's exact wheel angle gives. Built only where gz_ros2_control is installed. |
| `mecanum_hardware/Esp32System` | planned: hardware stage 5H | The same `WheelEncoder`, fed with the MCU's counts ([`docs/hardware_transition.md`](../../docs/hardware_transition.md), [`docs/encoder_upgrade.md`](../../docs/encoder_upgrade.md) sections 0 and 2). |

## Why the velocity is a count difference

`mecanum_drive_controller` integrates each wheel's velocity state times its own update period.
With the velocity set to the count change since the previous `read()` divided by that period,
the integration lands exactly on the count, so the wheel odometry carries the encoder's
resolution and nothing else, even if reports and cycles do not line up one to one.

## SimEncoderSystem

Selected by `hardware:=sim` in `mecanum_controllers/urdf/mecanum.ros2_control.xacro`, with these
hardware parameters (STAND-INs, see `STANDINS.md`):

| Parameter | Default in the xacro | Meaning |
|---|---|---|
| `counts_per_wheel_rev` | 4000 | Quadrature counts per wheel revolution: a 1000-line encoder, x4, direct drive |
| `report_delay_cycles` | 1 | Control cycles (20 ms each) between a count and its report |
| `following_error_limit` | see the xacro | Commanded minus counted wheel position (rad) that faults; 0 turns it off |

On a fault it stops all wheels, logs the wheel and the error, and returns `ERROR` from
`read()`: ros2_control deactivates the controllers, `/joint_states` and `/wheel/odometry` stop,
and the safety gate's `joint_states` watchdog blocks all motion. The simulation must be
restarted to clear it, as the robot would need its fault reset.

Topics (the same names as the ESP32 plugin will use):

| Topic | Type | Content |
|---|---|---|
| `/drive/following_error` | `std_msgs/Float64MultiArray` | Once per second of sim time: the current following error of each wheel joint, then the largest \|error\| of each so far (rad) |
| `/drive/inject_wheel_fault` | `std_msgs/String` | Test only: a joint name; that wheel's count freezes, as a dead encoder channel would |

gz_ros2_control calls `write()` on every physics step but `read()` once per control period, so
the commanded position is integrated in `read()`, from the command the controller wrote at the
previous update.

`GazeboSimSystem` is loaded through pluginlib and held, not subclassed: its private data type is
not installed, so a subclass could not be built or destroyed outside gz_ros2_control's library.

## Try it

```bash
ros2 launch mecanum_bringup robot.launch.py
ros2 topic echo /joint_states --field position     # multiples of 2*pi/4000
ros2 topic echo /drive/following_error
ros2 topic pub --once /drive/inject_wheel_fault std_msgs/msg/String \
  "{data: front_left_wheel_joint}"                 # while driving: the drive stops
```
