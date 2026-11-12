# mecanum_hardware

ros2_control hardware for the mecanum base, built around the wheel state of the **tapped motor
encoders** ([decision 0006](../../docs/decisions/0006-wheel-state-from-tapped-encoders.md)):
each motor's A and B encoder outputs reach the ESP32 through an optocoupler and a level
converter, and the MCU counts them.

| Part | Status | Content |
|---|---|---|
| `WheelEncoder` (`include/mecanum_hardware/wheel_encoder.hpp`) | done | One wheel's encoder rules, no ROS: a 64-bit count to radians, the velocity from the count change over the read period, an optional report delay, and the following-error check (integrated velocity command against the count). Unit tested. |
| `mecanum_hardware/SimEncoderSystem` | done | The Gazebo hardware plugin. It wraps gz_ros2_control's `GazeboSimSystem` and reports each wheel's state through a `WheelEncoder` fed with the count Gazebo's exact wheel angle gives. Built only where gz_ros2_control is installed. |
| `mecanum_hardware/Esp32System` | written, tested on the fake ESP32; hardware stages T1 to T5 pending | The real drive: the serial link to the ESP32 (`firmware/mecanum_drive_esp32`), the same `WheelEncoder` fed with the MCU's counts. Procedure and protocol: [`docs/hardware_drive_test.md`](../../docs/hardware_drive_test.md). |
| `drive_protocol` (`include/mecanum_hardware/drive_protocol.hpp`) | done | The serial protocol: COBS frames with a CRC-16, the six messages. Unit tested against byte vectors from the protocol table. |
| `test/fake_esp32.py`, `test/drive_protocol.py` | done | A fake ESP32 on a pseudo-terminal with the firmware's behaviour, and the Python protocol reference it uses. Installed to `share/mecanum_hardware/test`. |

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

Topics (the same names as the ESP32 plugin uses):

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

## Esp32System

Selected by `hardware:=esp32` (`mecanum_controllers/urdf/mecanum_esp32.urdf.xacro`, through
`controllers.launch.py hardware:=esp32` or `mecanum_bringup drive_test.launch.py`). The
ESP32 emits the step pulses, counts the encoders and checks one against the other; this plugin
configures it, sends the wheel commands and turns the counts into the wheel state.

| Parameter | Default in the xacro | Meaning |
|---|---|---|
| `port`, `baud` | `/dev/ttyUSB0`, 921600 | The DevKitC's USB serial port (use `/dev/serial/by-id/...`) |
| `pulses_per_rev`, `counts_per_wheel_rev` | 1600, 4000 (STAND-INs) | Step pulses and encoder counts per wheel revolution |
| `motor_for_wheel`, `dir_invert`, `enc_invert` | `0 1 2 3`, `0 0 0 0`, `0 0 0 0` (STAND-INs) | Which motor drives FL FR RL RR; per motor, inverted steps or counts |
| `watchdog_ms` | 200 | The ESP32 ramps to zero without a command for this long |
| `accel_backstop` | 2.5 rad/s² (STAND-IN) | The ESP32's ramp per wheel: for now every start and stop (`controllers_hardware.yaml` says why) |
| `max_wheel_speed` | 3.5 rad/s (STAND-IN) | The ESP32 clamps wheel commands to this |
| `following_error_limit` | 0.5 rad (STAND-IN); 0 = off | Pulses against counts, per wheel, that latch a fault |
| `state_timeout_ms`, `connect_timeout_s` | 100, 3.0 | A STATE older than this is an error; HELLO must arrive within this |

`read()` returns `ERROR` on a stale STATE, a lost link or a latched fault: the controllers
deactivate, `/joint_states` stops, and the safety gate blocks. The component goes back to
unconfigured; `ros2 control set_hardware_component_state mecanum_base active` and reactivating
the controllers resume (`docs/hardware_drive_test.md`, section 7).

Topics: `/drive/state` (`mecanum_interfaces/DriveState`, every STATE: counts, pulses, following
errors, faults, mode, the command sent, bad-frame counts) and `/drive/following_error` (as in
the simulation).

Try it without the board:

```bash
python3 src/mecanum_hardware/test/fake_esp32.py --link /tmp/fake_esp32
ros2 launch mecanum_bringup drive_test.launch.py port:=/tmp/fake_esp32 rviz:=true
tools/drive_monitor.py
```
