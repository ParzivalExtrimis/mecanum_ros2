# mecanum_base

This package holds the drive node, mecanum kinematics and the motion test for the
**encoderless** mecanum base.

The drive is open-loop by design. This package publishes no odometry and no joint states.
Odometry comes from LiDAR-inertial odometry only. See
[decision 0001](../../docs/decisions/0001-open-loop-drive-without-ros2-control.md), and
[encoder_upgrade.md](../../docs/encoder_upgrade.md) for the planned move to ros2_control.

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
     sim   : mecanum_interfaces/MotorCommands on /esp32/motor_commands (simulated ESP32)
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

The geometry defaults come from `mecanum_description`. `test/test_urdf_defaults.py`
checks them against the URDF:

| Parameter | Value |
|---|---|
| `wheel_radius` | 0.05 m |
| `lx` | 0.2255 m |
| `ly` | 0.207 m |

The wheel centres sit 9.5 mm behind `base_link`. The formulas rotate about the wheel
centre, and the offset is ignored for open-loop control.

## Wheel calibration model

```
value     = deadband + scale * |omega|    (capped at max_value)
direction = sign * sign(omega)
```

The calibration is set per wheel in `config/wheel_calibration.yaml`. Each wheel has a
motor ID, sign, scale, deadband and max value. All of these are stand-ins until the firmware
and motors exist; see `STANDINS.md`. The calibration tool, added in phase 5, fits scale and
deadband from LIO measurements. Open-loop calibration drifts with battery voltage and floor
condition, so re-run it when those change.

## Nodes

### `esp32_driver`

| Parameter | Default | Meaning |
|---|---|---|
| `transport` | `mock` | `mock`, `sim` or `esp32` |
| `cmd_vel_topic` | `cmd_vel` | Input topic |
| `cmd_vel_stamped` | `false` | `true` subscribes to TwistStamped. Jazzy Nav2 publishes Twist by default. |
| `motor_commands_topic` | `/esp32/motor_commands` | Output topic of the `sim` transport |
| `rate_hz` | `20.0` | Fixed send rate |
| `cmd_timeout` | `0.25` | Seconds without cmd_vel before zeros are sent |
| `wheel_radius`, `lx`, `ly` | URDF values | Geometry |
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
judged, because they depend on calibration. For example, the simulation rotates at 110% of
the commanded rate; see `mecanum_simulation/README.md`.

If `measure_odom_topic` is set, the test measures each step and prints PASS or FAIL for
direction and cross-axis drift. Use `/sim/ground_truth/odom` in simulation, and `/odom` on
hardware once LIO runs.

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
