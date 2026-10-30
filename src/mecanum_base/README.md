# mecanum_base

This package holds the drive node, mecanum kinematics and the motion test for the mecanum
base.

**The simulation no longer uses the drive node.** Since plan Phase 1 it drives through
ros2_control (`mecanum_controllers`,
[decision 0002](../../docs/decisions/0002-ros2-control-with-measured-wheel-state.md)). The
open-loop `esp32_driver` stays for the hardware path (`robot.launch.py sim:=false`), with the
`mock` and `esp32` transports, until the ESP32 hardware plugin replaces it
([encoder_upgrade.md](../../docs/encoder_upgrade.md)). It publishes no odometry and no joint
states; it was designed when the motors had no encoders
([decision 0001](../../docs/decisions/0001-open-loop-drive-without-ros2-control.md)).
`kinematics.py` is the reference the drive controller is tested against, and `motion_test`
drives both.

## Data flow

```
/cmd_vel (geometry_msgs/Twist, or TwistStamped with cmd_vel_stamped:=true)
  -> esp32_driver
       clamp twist to max_vx / max_vy / max_wz (uniform scaling keeps direction)
       -> inverse kinematics (X-roller layout)
       -> proportional saturation: if any wheel exceeds its reachable speed, all four
          are scaled by one factor, so the motion direction is preserved
       -> per-wheel calibration: rad/s -> (motor_id, direction, value)
       -> Transport, called at a fixed rate_hz (20 Hz), not only when commands arrive
  -> mock  : logs commands
     esp32 : real ESP32 link (not implemented until the firmware protocol exists)
```

The driver sends **zeros** in three cases:
- when no command arrives within `cmd_timeout`, which defaults to 0.25 s;
- after any exception in the control loop;
- on shutdown, including SIGINT and SIGTERM.

The commands actually sent are published on `/esp32_driver/wheel_commands` as
`mecanum_interfaces/WheelCommands`. These are commands, not feedback.

## Kinematics

lx is the half wheelbase, ly is the half track and r is the wheel radius:

```
FL = (vx - vy - (lx+ly) wz) / r      FR = (vx + vy + (lx+ly) wz) / r
RL = (vx + vy - (lx+ly) wz) / r      RR = (vx - vy + (lx+ly) wz) / r
```

`mecanum_base/kinematics.py` is pure Python and has unit tests.

The geometry (`wheel_radius`, `lx`, `ly`) comes from `mecanum_description` and is set in
`config/esp32_driver.yaml`. The node has no built-in defaults for it. `test/test_urdf_defaults.py`
checks the YAML against the URDF. The current values are `wheel_radius` 0.1025 m, `lx`
0.2360 m and `ly` 0.2150 m.

The formulas rotate about the centre of the wheel group. If the wheel centres are offset from
`base_link`, the offset is ignored for open-loop control.

## Wheel calibration model

```
value     = deadband + scale * |omega|    (capped at max_value)
direction = sign * sign(omega)
```

The calibration is set per wheel in `config/wheel_calibration.yaml`. Each wheel has a
motor ID, sign, scale, deadband and max value. All of these are stand-ins until the firmware
and motors exist; see `STANDINS.md`. The open-loop calibration tool of the previous plan is
not planned any more: with encoders, wheel speed is closed-loop (ideally on the ESP32), and
these values move into the ros2_control hardware plugin (`docs/encoder_upgrade.md`).

## Nodes

### `esp32_driver`

| Parameter | Default | Meaning |
|---|---|---|
| `transport` | `mock` | `mock` or `esp32` |
| `cmd_vel_topic` | `cmd_vel` | Input topic |
| `cmd_vel_stamped` | `false` | `true` subscribes to TwistStamped. Jazzy Nav2 publishes Twist by default. |
| `rate_hz` | `20.0` | Fixed send rate |
| `cmd_timeout` | `0.25` | Seconds without cmd_vel before zeros are sent |
| `wheel_radius`, `lx`, `ly` | none, required | Geometry, from the URDF |
| `max_vx`, `max_vy`, `max_wz` | `0.3`, `0.3`, `0.8` | Stand-in velocity limits |
| `wheels.<wheel>.{motor_id, sign, scale, deadband, max_value}` | see YAML | Calibration |

### `motion_test`

The motion test drives forward, then strafes left, then rotates counter-clockwise. Each step
runs for `move_duration` seconds and is followed by a stop. It is used to verify wheel
mapping and signs.

With `return_to_start:=true`, the default, each step is followed by the opposite motion, so
the test can be repeated in place. It needs about `linear_speed x move_duration` of free
space in front and to the left, plus room to turn on the spot. Yaw is unwrapped, so turns
beyond 180° are judged correctly. The travel and rotation percentages are reported but not
judged, because they depend on calibration; see the measured figures in
`mecanum_simulation/README.md`.

If `measure_odom_topic` is set, the test measures each step and prints PASS or FAIL for
direction and cross-axis drift. Use `/sim/ground_truth/odom` in simulation, and `/odom` on
hardware.

`motion_test.launch.py` publishes `TwistStamped` on `/cmd_vel` by default, for the drive
controller. For the open-loop driver on hardware, which takes `Twist` by default, pass
`cmd_vel_stamped:=false`.

`pattern` selects other scripts, used to exercise odometry (LIO, wheel odometry, the EKF):

| `pattern` | Script | Length with the defaults |
|---|---|---|
| `axes` | The wheel-mapping steps above (default) | About 2 m |
| `loop` | Per lap: forward `loop_length`, rotate 180°, forward `loop_length`, strafe right `loop_width`, rotate 180°, strafe right `loop_width`. From the restroom spawn it runs along the corridor and up into the lobby, and closes. | 12 m and 360° per lap (`loop_length` 4, `loop_width` 2) |
| `corridor` | Per lap: forward `loop_length`, then reverse `loop_length` | 8 m per lap |

`laps` repeats the script. In `loop` and `corridor`, each segment has a target distance or
angle. With `measure_odom_topic` set, a segment ends when the measured progress reaches the
target, so calibration errors do not walk the robot into the walls. A segment that takes more than twice its nominal time stops the test with a
failure. Without odometry, each segment runs for its nominal time.

## Usage

**Dry run without hardware or a simulator.** The mock transport logs the commands:
```bash
ros2 launch mecanum_base base.launch.py transport:=mock
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic echo /esp32_driver/wheel_commands
```

**Motion test in simulation.** Start the simulation first with
`ros2 launch mecanum_bringup robot.launch.py`, then:
```bash
ros2 launch mecanum_base motion_test.launch.py use_sim_time:=true \
  measure_odom_topic:=/sim/ground_truth/odom
# Odometry drift loop, about 24 m (run gt_compare from mecanum_localization alongside):
ros2 launch mecanum_base motion_test.launch.py use_sim_time:=true pattern:=loop laps:=2 \
  measure_odom_topic:=/sim/ground_truth/odom
```

## Real ESP32 transport

The ESP32 transport is pending the firmware rewrite. Selecting `transport:=esp32` stops at
startup with an explanation. To implement it, fill in `Esp32Transport` in
`mecanum_base/transport.py`. Its `send()` gets one `MotorCommand(motor_id, direction, value)`
per motor at `rate_hz` and must not block longer than one period. These facts are needed
first:

- transport, address and port;
- message format;
- value units and range, which set `max_value` and `scale`;
- direction encoding;
- motor ID to wheel mapping;
- whether the firmware has its own watchdog.

## Tests

```bash
colcon test --packages-select mecanum_base && colcon test-result --verbose
```

The tests cover:
- kinematics formulas, sign patterns, round trips and saturation;
- the calibration mapping;
- drive core timeouts and clamping;
- the transports;
- URDF consistency;
- copyright, flake8 and pep257.
