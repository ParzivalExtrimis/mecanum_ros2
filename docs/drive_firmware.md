# Drive firmware and the ros2_control link: reference

How the real base is driven: the ESP32 sketch, the ros2_control hardware plugin that talks to
it, and every value that can be configured, with where it is set. The procedure that uses all
of this is [`hardware_drive_test.md`](hardware_drive_test.md), which also holds the serial
protocol's byte tables ("Protocol") and the results log.

**The sketch's source is [`firmware/mecanum_drive_esp32/mecanum_drive_esp32.ino`](../firmware/mecanum_drive_esp32/mecanum_drive_esp32.ino)**.
That repository file was compiled and uploaded on 2026-11-13, not a scratch copy. The copy in
[the appendix](#appendix-the-sketch) is for reading; edit the `.ino`, never this page. The
appendix copy matches commit `c894c07` (sha256
`49af11f72ff1ae1623e414a12de6d9eb2ebf755fa11f8efaea77f4c67e7bca55`); check it with
`sha256sum firmware/mecanum_drive_esp32/mecanum_drive_esp32.ino`. Build and upload steps are in
the [firmware README](../firmware/mecanum_drive_esp32/README.md).

## 1. The whole chain

```
 keyboard / drive_tune.py / Nav2 (later)
        │  TwistStamped /cmd_vel
        ▼
 safety gate (mecanum_safety) ── blocks when /joint_states stops ──►  /cmd_vel_safe
        ▼
 mecanum_drive_controller (ros2_controllers)        joint_state_broadcaster
   inverse kinematics → 4 wheel velocity commands     /joint_states
   forward kinematics ← 4 wheel velocity states  ──►  /wheel/odometry, odom → base_footprint TF
        │ command interfaces            ▲ state interfaces (position, velocity)
        ▼                               │
 ┌──────────── mecanum_hardware/Esp32System (ros2_control SystemInterface) ────────────┐
 │ write(): CMD frame, 50 Hz           read(): newest STATE → WheelEncoder → rad, rad/s │
 │ reader thread: frames from the port, keeps the newest STATE, publishes /drive/state  │
 └──────────────────────────────── USB serial 921600, COBS + CRC-16 ───────────────────┘
        │ CMD (50 Hz)                   ▲ STATE (100 Hz), HELLO, CONFIG_ACK
        ▼                               │
 ┌──────────────── ESP32 sketch (firmware/mecanum_drive_esp32) ─────────────────────────┐
 │ watchdog, speed clamp, ramp (accel_backstop) → step pulses on 4 × PUL/DIR            │
 │ 4 × pulse counter units ← encoder A/B through the tap; following error = pulses − counts│
 └──────────────────────────────────────────────────────────────────────────────────────┘
        │ PUL/DIR                       ▲ A/B (tapped)
        ▼                               │
 4 × CS-D808 closed-loop drivers ──► 4 × CS-M234120 motors (their encoders)
```

Two loops close here, and neither is in ROS:
- The **position loop** of each motor is in its CS-D808, which follows the step pulses with
  the motor's encoder.
- The **following-error check** is on the ESP32. It compares the pulses it emitted with the
  counts it read through the tap, and stops every wheel when one wheel falls behind (decision
  0006).

ROS gets the counted position and velocity, so the odometry measures the wheels' motion rather
than assuming the commanded one.

## 2. Where the code lives

| Path | What it is |
|---|---|
| `firmware/mecanum_drive_esp32/mecanum_drive_esp32.ino` | The sketch: one file, sections 1–7 (below) |
| `firmware/mecanum_drive_esp32/README.md` | Build, board options, CONFIG values to check, what each test stage may change |
| `src/mecanum_hardware/include/mecanum_hardware/drive_protocol.hpp`, `src/drive_protocol.cpp` | The host side of the protocol: message structs, CRC-16, COBS, `encode_*`, `decode_frame`, `FrameReader` (bytes → frames); `kProtocolVersion` |
| `src/mecanum_hardware/.../serial_port.hpp`, `src/serial_port.cpp` | POSIX termios serial port (raw mode, baud, read with timeout) |
| `src/mecanum_hardware/.../esp32_system.hpp`, `src/esp32_system.cpp` | `Esp32System`, the ros2_control hardware plugin (section 4) |
| `src/mecanum_hardware/.../wheel_encoder.hpp`, `src/wheel_encoder.cpp` | `WheelEncoder`: count → position (rad), and velocity from the count change over the read period; shared with the simulation's `SimEncoderSystem` |
| `src/mecanum_hardware/mecanum_hardware_plugins.xml` | pluginlib export of `mecanum_hardware/Esp32System` |
| `src/mecanum_hardware/test/` | `test_drive_protocol.cpp` (byte vectors from the protocol table), `test_wheel_encoder.cpp`, `drive_protocol.py` (Python reference), `fake_esp32.py` (a fake board on a pseudo-terminal) |
| `src/mecanum_interfaces/msg/DriveState.msg` | `/drive/state`: counts, pulses, following error, faults, mode, rx errors, the commands sent |
| `src/mecanum_controllers/urdf/mecanum.ros2_control.xacro` | The `<ros2_control>` block: `hardware == 'esp32'` selects the plugin and passes its parameters |
| `src/mecanum_controllers/urdf/mecanum_esp32.urdf.xacro` | Entry point: robot model + the esp32 `<ros2_control>` block, with the launch arguments as xacro args |
| `src/mecanum_controllers/config/controllers.yaml`, `controllers_hardware.yaml` | The controllers; the hardware overlay with the low test limits |
| `src/mecanum_controllers/launch/controllers.launch.py` | `hardware:=esp32`: robot_state_publisher, ros2_control_node, the spawners; `ESP32_ARGS` lists every drive argument (empty by default: the xacro holds the defaults) |
| `src/mecanum_controllers/test/test_esp32_fake.py` | Launch test of the whole stack on the fake ESP32 |
| `src/mecanum_bringup/launch/drive_test.launch.py` | The drive, the safety gate, optional RViz |
| `src/mecanum_bringup/launch/teleop.launch.py` | Keyboard teleop with the drive-test defaults, in its own terminal window |
| `tools/drive_monitor.py`, `tools/drive_tune.py`, `tools/fe_soak.py` | Live per-wheel table and `identify`; tuning ladders; the following-error soak |

## 3. The sketch

### Structure

| Section | Contents |
|---|---|
| 1. CONFIG | Pins, defaults, timing constants: the only part meant to be edited |
| 2. Pure logic (`namespace logic`) | No Arduino types, so it reads like the host code: `crc16`, `cobsEncode` / `cobsDecode`, `wheelToPulseHz`, `wheelAccelToPulse`, `followingError`, `rampToward`, `clampAbs`, `expired`, `validMotorOrder` |
| 3. Protocol | Message types, flag and fault bits, modes, the packed message structs (sizes checked by `static_assert`), `sendFrame`, `sendHello`, `sendConfigAck` |
| 4. Encoders | `encoderInit` (one pulse counter unit per motor, x4 quadrature, glitch filter), `encoderPoll` (folds the 16-bit hardware count into a 64-bit total) |
| 5. Step generation | One interface, two implementations chosen by `STEP_DRIVER_FASTACCEL`: FastAccelStepper on the RMT peripheral (in use), or the file's own 40 kHz timer generator. `stepInit`, `stepSetTarget(motor, Hz, Hz/s)`, `stepPoll`, `stepPulses`, `stepStill` |
| 6. Drive state | `RuntimeConfig` (what CONFIG sets), the wheel/motor mapping, the CONFIG and CMD handlers, the frame reader, `updateDrive`, `sendState` |
| 7. `setup()` / `loop()` | Start-up order and the main loop |

### Key functions

| Function | What it does | Why it matters |
|---|---|---|
| `setup()` | Opens the serial port, starts the step generator **first** (PUL pins driven low), then the encoders, then sends HELLO | Nothing pulses at boot; a failed start latches `FAULT_STEP_DRIVER` |
| `loop()` | `readSerial` → `encoderPoll` → `stepPoll` → `updateDrive`, and `sendState` every 10 ms | No blocking anywhere: a full TX buffer drops a frame (the host sees a seq gap) instead of stalling the loop |
| `readSerial()` / `handleFrame()` | Collects bytes up to each `0x00`, COBS-decodes, checks the CRC and the length, and dispatches HELLO_REQ, CONFIG and CMD | Every bad frame counts in `crc_errors` (the `MCU rx errors` in the monitor) |
| `handleConfig()` | Validates and applies the host's configuration (pulses and counts per rev, motor order, inverts, watchdog, backstop, speed cap, FE limit), then rebases the following error | Refused while any wheel moves (ack 2) or with bad values (ack 1). Motion stays off until a CMD enables it |
| `handleCmd()` | Stores the four wheel speeds (rad/s, clamped to `max_wheel_speed`, non-finite → 0), the enable flag, and the time; clears a latched fault only when asked, all wheels still, and the step generator healthy | Every CMD feeds the watchdog |
| `updateDrive()` | Decides what each motor does now: sets the watchdog bit past `watchdog_ms` without a CMD, latches a following-error fault, then sets every motor's target pulse rate (or 0) with the ramp `accel_backstop` | The single place where stopping is decided: no config, not enabled, latched fault or watchdog → every wheel ramps to 0 |
| `wheelCounts()`, `wheelPulses()` | Map wheel (FL, FR, RL, RR) → motor through `motor_for_wheel`, and apply `enc_invert` / `dir_invert` | Everything on the wire is in the wheel convention: positive rolls the robot forward |
| `logic::followingError()` | `pulses × counts_per_rev / pulses_per_rev − counts − offset` | The offset is taken at CONFIG and at a fault clear, so the error is relative to the last rebase, not to boot |
| `stepSetTarget()` (FastAccelStepper) | Sets the acceleration and the speed, runs forward or backward, or `stopMove()` below 2 Hz; an unchanged target leaves the running ramp alone | The ramp, the speed changes and the direction changes are FastAccelStepper's; DIR is set 20 µs before the next pulse |
| `encoderInit()` / `encoderPoll()` | x4 decoding on both channels' edges, `accum_count` past ±30000, 64-bit total from 16-bit differences | Counts never wrap in practice |
| `sendState()` | Seq, last CMD seq, MCU time, 4 × counts, 4 × pulses, 4 × following error, faults, rx errors, mode | 100 Hz; the host's staleness check (100 ms) relies on it |

### Modes and faults (in STATE)

| Mode | Meaning |
|---|---|
| 0 no config | No valid CONFIG yet: nothing can move |
| 1 idle | Configured, motion not enabled, or the watchdog tripped |
| 2 running | Configured, enabled, CMDs arriving |
| 3 fault | A fault is latched: zero only, until a fault clear |

| Bit | Fault | Latched? |
|---|---|---|
| `0x0001`–`0x0008` | Following error on FL, FR, RL, RR | yes |
| `0x0010` | Watchdog: no CMD within `watchdog_ms` | no: the next CMD clears it |
| `0x0020` | No configuration | no |
| `0x0040` | A command was clamped to the speed cap | no (information) |
| `0x0080` | A fault is latched | — |
| `0x0100` | The step generator or the encoders failed to start | yes, until reboot |

## 4. The host side: `Esp32System`

`mecanum_hardware/Esp32System` is a `hardware_interface::SystemInterface`, loaded by
`ros2_control_node` from the `<ros2_control>` block when `hardware:=esp32`. It owns the serial
port and has no ROS dependency beyond the interfaces it exports, plus a small node
(`esp32_drive`) for its two diagnostic topics.

### Lifecycle

| Callback | What it does | On failure |
|---|---|---|
| `on_init` | Reads the `<param>`s (section 6), checks for four wheel joints with one velocity command interface each, matches joints to FL/FR/RL/RR by name, converts `following_error_limit` from rad to counts, creates `/drive/state` and `/drive/following_error` | ERROR: the controller manager does not load it |
| `on_configure` | Opens the port, starts the reader thread, sends HELLO_REQ every 250 ms until HELLO (up to `connect_timeout_s`), checks `PROTOCOL_VERSION` | "No HELLO from the ESP32…": wrong port, the serial monitor open, or no sketch |
| `on_activate` | Sends CONFIG (3 tries, 1 s each) and waits for CONFIG_ACK, then sends CMDs with "clear fault" until the board is idle and unfaulted (2 s), and seeds each `WheelEncoder` with the current count | ERROR: the board rejected the values, never acked, or stays faulted |
| `read()` | Takes the newest STATE: ERROR if the link is lost, the STATE is older than `state_timeout_ms`, or a fault is latched (logged once, with each wheel's following error); otherwise each wheel's count goes through its `WheelEncoder` → position (rad) and velocity (rad/s) state interfaces | The controller manager deactivates the component and its controllers: the robot stops (recovery: `hardware_drive_test.md`, section 7) |
| `write()` | Sends one CMD: the four velocity command interfaces (non-finite → 0) with "enable" while active | ERROR if the port write fails |
| `on_deactivate` / `on_error` / `on_cleanup` | Send CMDs with motion off (the wheels ramp to 0), and on error or cleanup stop the reader and close the port | — |

### One control cycle (50 Hz, `update_rate` in `controllers.yaml`)

1. `read()` copies the newest STATE that the reader thread received, at most 10 ms old in normal
   running. `WheelEncoder::update(count, period)` turns the count into a position, and the
   count change over that period into a velocity. A controller that integrates velocity ×
   period therefore reproduces the count exactly, so the odometry does not drift from the
   counts.
2. `joint_state_broadcaster` publishes `/joint_states`. The safety gate watches it, and
   blocks motion when it stops.
3. `mecanum_drive_controller` updates the odometry from the four wheel velocities and
   publishes `/wheel/odometry` and the `odom → base_footprint` TF. From the newest
   `/cmd_vel_safe` twist it computes four wheel speeds, clamped by the velocity limits. With
   no new twist for 0.25 s (`reference_timeout`), it commands zero.
4. `write()` sends them as one CMD.
5. On the ESP32, the CMD refreshes the watchdog, and the pulse rates ramp toward the new speeds
   at `accel_backstop`. The drivers follow the pulses, and the encoders report back within
   the next STATE.

The reader thread runs beside this cycle. It reads the port in 50 ms slices, feeds the bytes
to `FrameReader`, and handles each message:
- HELLO and CONFIG_ACK are stored for the lifecycle callbacks.
- A STATE becomes the newest state. It is published on `/drive/state` (100 Hz) and, once a
  second, as `/drive/following_error`: four current values, then four maxima, in wheel rad.

### Ways the wheels stop

| Cause | Who acts | How fast |
|---|---|---|
| Teleop key released, or no twist | the drive controller's `reference_timeout` → zero speeds | 0.25 s + ramp |
| `/joint_states` stops | the safety gate blocks `/cmd_vel_safe` | `timeout_joint_states` (0.2 s) + the above |
| Following error past the limit on any wheel | the ESP32 latches, every wheel ramps to 0; `read()` returns ERROR | about 0.05 s at 0.1 m/s with the 0.05 rad limit |
| Host silent (crash, USB pulled) | the ESP32 watchdog | `watchdog_ms` (0.2 s) + ramp |
| STATE stale on the host | `read()` ERROR → controllers deactivate, `on_error` sends stop | `state_timeout_ms` (0.1 s) |
| Power | the power lead | immediate; the only hard stop (no E-stop, ENA/ALM not wired) |

## 5. Values that can be changed

"Stand-in" means a provisional value that still has to be measured or tuned. `STANDINS.md` is
the full register; this table says where each drive value lives. **How to change it:**
- **Re-upload:** edit the sketch, then compile and upload.
- **Launch arg:** a `drive_test.launch.py` argument. It passes through the xacro, and the
  plugin sends it to the ESP32 by CONFIG on every activation, so a relaunch is enough.
- **Rebuild:** edit the file, then run `colcon build`. Config files are installed, so the edit
  only takes effect after a rebuild.
- **Edit:** edit the Python constant; it takes effect on the next run.

### On the ESP32 (the sketch's CONFIG block, `firmware/.../mecanum_drive_esp32.ino`)

| Value | Current | Stand-in? | Notes | Change by |
|---|---|---|---|---|
| `PIN_PUL` | 18, 21, 23, 4 (motors 1–4) | No longer: every motor pulsed both ways in T2 identify | Motor 4 PUL (GPIO 4) was the unverified one | Re-upload |
| `PIN_DIR` | 19, 22, 5, 15 | As above; GPIO 5 and 15 are strapping pins, and the board boots with the drivers connected (T1) | | Re-upload |
| `PIN_ENC_A`, `PIN_ENC_B` | A 34, 33, 26, 14; B 35, 32, 25, 27 | Confirmed by T2: counts follow pulses on all four | 34/35 input-only, no pull-up | Re-upload |
| `STEP_DRIVER_FASTACCEL` | 1 (FastAccelStepper 2.0.1, RMT) | — | 0 selects the built-in timer generator; also settable at compile time with `-DSTEP_DRIVER_FASTACCEL=0` | Re-upload |
| `SERIAL_BAUD` | 921600 | — | Must equal the xacro's `baud` | Re-upload (and the xacro) |
| `STATE_PERIOD_US` | 10000 (100 Hz) | — | The host's 100 ms staleness limit assumes it | Re-upload |
| `DIR_SETUP_US` | 20 µs | — | CS-D808 needs ≥ 5 µs | Re-upload |
| `ENC_GLITCH_NS` | 1000 ns | — | Raise toward 5000 only if counts jitter at rest; max 12700 | Re-upload |
| `MAX_PULSE_RATE_HZ` | 20000 | — | Hard cap under any host value (12.5 rev/s at 1600) | Re-upload |
| `MIN_PULSE_RATE_HZ` | 2 | — | Below this a wheel is stopped | Re-upload |
| `TIMER_TICK_HZ`, `CONTROL_PERIOD_US` | 40000, 1000 µs | — | Timer generator only (not in use) | Re-upload |
| `DEFAULT_PULSES_PER_REV`, `DEFAULT_COUNTS_PER_REV` | 1600, 4000 | — | Reported in HELLO; used only until CONFIG arrives | Re-upload |
| `RuntimeConfig` defaults | watchdog 200 ms, backstop 5.0 rad/s², speed cap 3.0 rad/s, FE limit off | — | Before the first CONFIG only, and nothing moves then anyway | Re-upload |
| CONFIG validation | pulses 200–51200, counts 100–1 000 000, watchdog 20–2000 ms, backstop > 0, speed cap > 0, FE limit ≥ 0 | — | `handleConfig()` | Re-upload |
| `FW_MAJOR`.`FW_MINOR` | 0.1 | — | Logged by the host at connect | Re-upload |
| `PROTOCOL_VERSION` | 1 | — | Must equal `kProtocolVersion` in `drive_protocol.hpp`: the host refuses another | Re-upload and rebuild, with the protocol table |

### Sent to the ESP32 by CONFIG (launch args → xacro → plugin)

**The defaults live in one place:** the `xacro:arg` defaults of
`mecanum_controllers/urdf/mecanum_esp32.urdf.xacro`. The launch arguments of
`controllers.launch.py` and `drive_test.launch.py` default to empty. An empty argument is not
passed to xacro (`XacroArg` in `controllers.launch.py`), so the xacro's value applies; a set one
overrides it for that run. Two further fallbacks exist, and are not used under a launch: the
`mecanum_ros2_control` macro's `esp32_*` defaults in `mecanum.ros2_control.xacro` (the old
stand-ins, for a hand-written URDF), and the plugin's `param(...)` fallbacks in
`esp32_system.cpp` (when a `<param>` is missing).

| Launch arg | Default (the xacro) | Measured / chosen | Stand-in? | Notes |
|---|---|---|---|---|
| `port` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | the same | Changes on the Jetson | The stable by-id path of the CH340 bridge on the dev laptop |
| `pulses_per_rev` | 1600 | 1600 (T2: 2.4995 counts per pulse) | Measured; the default is right | Must match the CS-D808 DIP switches |
| `counts_per_wheel_rev` | 4000 | 4000 (T2) | Measured; the default is right | 1000-line encoder × 4, direct drive |
| `motor_for_wheel` | `3 0 2 1` | T2 identify | Measured | Motor index (0–3) driving FL FR RL RR |
| `dir_invert` | `0 0 1 1` | T2 identify | Measured | Per motor 1–4 |
| `enc_invert` | `0 0 1 1` | T2 identify | Measured | Per motor 1–4 |
| `following_error_limit` | 0.5 rad (the owner's choice) | T2 soak: peak 10 counts (0.016 rad), so 0.05 rad (32 counts) is the tightest the rule allows | The owner kept 0.5 as the default; pass 0.05 for a tight check, and revisit at each T5 speed step | rad at the wheel; 0 turns the check off |
| `watchdog_ms` | 200 | T2 check 6 passed | Stand-in until T5 checks it at speed | |
| `accel_backstop` | 2.5 rad/s² (0.26 m/s², 0.57 rad/s²) | — | **Yes**, T5 acceleration ladder, decision 0007 | The only acceleration limit (`hardware_drive_test.md` section 8) |
| `max_wheel_speed` | 3.5 rad/s (0.36 m/s at the rim) | — | **Yes**, T5 speed ladder | ESP32 clamp; `drive_tune.py` refuses steps above 90% of it |

### In ROS packages (edit, then `colcon build`)

| Value | File | Current | Stand-in? |
|---|---|---|---|
| `baud` | `mecanum_controllers/urdf/mecanum.ros2_control.xacro` | 921600 | — (must match the sketch) |
| `state_timeout_ms` | same | 100 | — |
| `connect_timeout_s` | same | 3.0 | — |
| `update_rate` | `mecanum_controllers/config/controllers.yaml` | 50 Hz | — |
| `wheels_radius` | same (and `mecanum_params.xacro`) | 0.1025 m | **Yes**, T4 (3 m straights) |
| `sum_of_robot_center_projection_on_X_Y_axis` (lx + ly) | same | 0.451 m | **Yes**, T4 (N × 360° spins) |
| `reference_timeout` | same | 0.25 s | Checked in T5 at top speed |
| Velocity limits x, y, z | `mecanum_controllers/config/controllers_hardware.yaml` | 0.1 m/s, 0.1 m/s, 0.3 rad/s | **Yes**, T5; `drive_tune.py` raises them at run time and restores them |
| Acceleration limits | same | none, on purpose | Until a ros2_controllers release fixes the timeout bug (section 8 of the test document) |
| `enable_odom_tf` | same | true (no EKF on the hardware yet) | — |
| `timeout_joint_states` | `mecanum_safety/config/safety.yaml` | 0.2 s | **Yes**, the real worst gap on the Jetson |
| `watchdog_clock` | `drive_test.launch.py` → safety launch | `steady` | — |
| Protocol constants (`kProtocolVersion`, flags, fault bits) | `mecanum_hardware/include/mecanum_hardware/drive_protocol.hpp` | 1 | — (copies of the sketch's) |
| HELLO retry 250 ms, CONFIG 3 × 1 s, fault-clear wait 2 s, `/drive/following_error` at 1 Hz | `mecanum_hardware/src/esp32_system.cpp` | as listed | — |

### Launch-time defaults for teleop (`mecanum_bringup/launch/teleop.launch.py`)

| Arg | Default | Notes |
|---|---|---|
| `speed`, `turn` | 0.1 m/s, 0.3 rad/s | Start values; `w`/`x` and `e`/`c` change them in the window. The hardware limits clamp anything higher |
| `frame_id` | `base_footprint` | `stamped` is always true |
| `cmd_vel_topic` | `/cmd_vel` | The safety gate's input |
| `terminal` | `gnome-terminal --wait --` | teleop needs a keyboard, so it opens its own window; for instance `terminal:='xterm -e'` |

### Constants in the Python tools (edit the file)

| Constant | File | Value | Meaning |
|---|---|---|---|
| `SLIP_LIMIT_PCT` | `tools/drive_tune.py` | x 3, y 8, wz 3 % | The owner's limit rule: slip that fails a step |
| `FE_SHARE` | same | 0.5 | A step fails when the following error passes this share of `following_error_limit` |
| `SPEED_MARGIN` | same | 0.9 | Steps above 90% of `max_wheel_speed` are refused |
| `RATE` | same, `tools/fe_soak.py` | 50 Hz | Command and sample rate |
| `STILL_LINEAR`, `STILL_ANGULAR` | `tools/drive_tune.py` | 0.002 m/s, 0.005 rad/s | "The robot has stopped" |
| `SILENT_DRIVE_S` | same (also used by `fe_soak.py`) | 0.3 s | No `/joint_states` this long aborts the run |
| `MIN_LIMIT_RAD`, `LIMIT_FACTOR` | `tools/fe_soak.py` | 0.05 rad, 2.5 | The suggested limit: max(0.05, 2.5 × the largest error) |
| `DRIFT_COUNTS` | same | 4 | A settled error moving more than this is a build-up, not lag |
| CLI defaults | `fe_soak.py` | speed 0.1, turn 0.3, 20 cycles, 10 reversals, 30 s still | `--help` |
| CLI defaults | `tools/drive_monitor.py identify` | `--speed 0.05`, `--turn 0.15` | The crawl speeds on blocks |
| `SIGNS`, `PATTERNS` | `tools/drive_monitor.py` | the mecanum wheel signs per motion | Kinematics, not a tuning value |

### On the drivers (outside the repo)

| Value | Where | Current | Stand-in? |
|---|---|---|---|
| Pulses per revolution | CS-D808 DIP switches | 1600 (matches T2) | — |
| The drivers' own following-error alarm | Leadshine tuning software | not set | **Yes** (`STANDINS.md`): a second bound beside the ESP32's |
| ENA, ALM | not wired | drivers always enabled when powered | — |

## Appendix: the sketch

Verbatim copy of `firmware/mecanum_drive_esp32/mecanum_drive_esp32.ino` at commit `c894c07`,
as uploaded on 2026-11-13 (sha256 above).

```cpp
// Copyright 2026 Aryan Hegde
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// mecanum_drive_esp32: the drive firmware of the mecanum base.
//
// An ESP32-DevKitC (WROOM-32E) between the host (ros2_control plugin mecanum_hardware/Esp32System,
// over USB serial) and four Leadshine CS-D808 closed-loop stepper drivers (pulse and direction).
// It reads each motor's encoder through the encoder tap (decision 0006) on the ESP32 pulse
// counter units, counts the step pulses it emits, and stops all wheels when the two disagree
// (the following error).
//
// The serial protocol is specified in docs/hardware_drive_test.md, section "Protocol". This
// file, the host (src/mecanum_hardware/src/drive_protocol.cpp) and the Python reference
// (src/mecanum_hardware/test/drive_protocol.py) implement it; PROTOCOL_VERSION must match.
//
// Arduino-ESP32 core 3.x (ESP-IDF 5). Built and uploaded by the owner; see README.md beside this
// file for the steps.
//
// Sections:
//   1. CONFIG            pins and defaults: the only part to edit
//   2. Pure logic        no Arduino types: CRC, COBS, conversions, following error, ramps
//   3. Protocol          message layouts, frame reader, senders
//   4. Encoders          pulse counter units, x4 decoding, 64-bit counts
//   5. Step generation   FastAccelStepper (RMT) or the built-in timer generator
//   6. Drive state       configuration, watchdog, fault latch
//   7. setup() / loop()

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"

// =================================================================================================
// 1. CONFIG
// =================================================================================================

// Firmware version, reported in HELLO.
#define FW_MAJOR 0
#define FW_MINOR 1
// Must equal PROTOCOL_VERSION in the host plugin. Change only with the protocol table.
#define PROTOCOL_VERSION 1

// Step generator: 1 = FastAccelStepper on the RMT peripheral (needs the FastAccelStepper library);
// 0 = the built-in timer generator of this file (no library). Both behave the same on the wire.
// Switch to 0 if FastAccelStepper does not build or misbehaves.
#ifndef STEP_DRIVER_FASTACCEL
#define STEP_DRIVER_FASTACCEL 1
#endif

// Pin map, owner's notes 2026-11-11. Index = motor 1..4 (index 0..3). Which motor drives which
// wheel is set at runtime by the host (motor_for_wheel in CONFIG), found on blocks (stage T2).
// STAND-IN: motor 3 DIR (5) and motor 4 PUL (4) were overwritten in the original notes; verify.
// GPIO 5 and 15 are strapping pins: harmless as DIR outputs, but check that the board still
// boots with the drivers connected (stage T1).
static const uint8_t PIN_PUL[4] = {18, 21, 23, 4};
static const uint8_t PIN_DIR[4] = {19, 22, 5, 15};
// Encoder A/B through the tap. 34/35 are input-only with no internal pull-up: the tap's level
// converter drives them (owner's side).
static const uint8_t PIN_ENC_A[4] = {34, 33, 26, 14};
static const uint8_t PIN_ENC_B[4] = {35, 32, 25, 27};

// Defaults until the host's CONFIG arrives (the host's values win). STAND-INs.
#define DEFAULT_PULSES_PER_REV 1600   // CS-D808 DIP setting: measured in stage T2
#define DEFAULT_COUNTS_PER_REV 4000   // 1000-line encoder, x4 decoding, direct drive

// The serial link to the host (the DevKitC's USB-UART bridge on UART0).
#define SERIAL_BAUD 921600
// STATE period: 100 Hz.
#define STATE_PERIOD_US 10000
// Direction setup before the first pulse after a direction change. The CS-D808 needs at least
// 5 us; 20 us leaves margin. (FastAccelStepper: dir_change_delay; timer generator: one or more
// control ticks, always longer.)
#define DIR_SETUP_US 20
// Encoder glitch filter: pulses shorter than this are ignored. At most 12700 ns (1023 APB
// cycles). 1000 ns is far below an encoder edge spacing at full speed (about 160 us per channel
// at 1 m/s).
#define ENC_GLITCH_NS 1000
// Hard limit on any pulse rate, whatever the host asks (Hz). The timer generator cannot exceed
// TIMER_TICK_HZ / 2.
#define MAX_PULSE_RATE_HZ 20000
// Below this rate a wheel is stopped (Hz).
#define MIN_PULSE_RATE_HZ 2
// Timer generator only: tick rate (Hz) and the ramp update period (us).
#define TIMER_TICK_HZ 40000
#define CONTROL_PERIOD_US 1000

#if STEP_DRIVER_FASTACCEL
#include <FastAccelStepper.h>
#endif

// =================================================================================================
// 2. Pure logic (no Arduino types; reviewed against the host tests)
// =================================================================================================

namespace logic
{

static const double kTwoPi = 6.283185307179586;

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// Check value: "123456789" -> 0x29B1.
uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// COBS encode `len` bytes into `out` (room for len + len / 254 + 1). Returns the encoded length,
// without the 0x00 delimiter.
size_t cobsEncode(const uint8_t * in, size_t len, uint8_t * out)
{
  size_t code_pos = 0;
  size_t o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; ++i) {
    if (in[i] == 0) {
      out[code_pos] = code;
      code_pos = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      ++code;
      if (code == 0xFF) {
        out[code_pos] = code;
        code_pos = o++;
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return o;
}

// COBS decode. Returns the decoded length, or 0 for a malformed frame.
size_t cobsDecode(const uint8_t * in, size_t len, uint8_t * out)
{
  size_t i = 0;
  size_t o = 0;
  while (i < len) {
    const uint8_t code = in[i];
    if (code == 0 || i + code > len) {
      return 0;
    }
    for (uint8_t k = 1; k < code; ++k) {
      out[o++] = in[i + k];
    }
    i += code;
    if (code < 0xFF && i < len) {
      out[o++] = 0;
    }
  }
  return o;
}

// Wheel speed (rad/s) to a signed pulse rate (Hz).
double wheelToPulseHz(double rad_s, uint32_t pulses_per_rev)
{
  return rad_s * static_cast<double>(pulses_per_rev) / kTwoPi;
}

// Wheel acceleration (rad/s^2) to pulses/s^2.
double wheelAccelToPulse(double rad_s2, uint32_t pulses_per_rev)
{
  return rad_s2 * static_cast<double>(pulses_per_rev) / kTwoPi;
}

// Following error in encoder counts: the emitted pulses scaled to counts, minus the counted
// position, minus the offset taken at the last configuration or fault clear.
int64_t followingError(
  int64_t pulses, int64_t counts, uint32_t pulses_per_rev, uint32_t counts_per_rev,
  int64_t offset)
{
  const int64_t expected = pulses * static_cast<int64_t>(counts_per_rev) /
    static_cast<int64_t>(pulses_per_rev);
  return expected - counts - offset;
}

// Move `current` toward `target` by at most `max_step`.
double rampToward(double current, double target, double max_step)
{
  if (target > current + max_step) {
    return current + max_step;
  }
  if (target < current - max_step) {
    return current - max_step;
  }
  return target;
}

// Clamp to +-limit; sets *clamped when it bites.
double clampAbs(double value, double limit, bool * clamped)
{
  if (value > limit) {
    *clamped = true;
    return limit;
  }
  if (value < -limit) {
    *clamped = true;
    return -limit;
  }
  return value;
}

// True when `elapsed` is past `timeout` (unsigned arithmetic, wraps safely).
bool expired(uint32_t now, uint32_t since, uint32_t timeout)
{
  return static_cast<uint32_t>(now - since) > timeout;
}

// Is motor_for_wheel a permutation of 0..3?
bool validMotorOrder(const uint8_t * order)
{
  bool seen[4] = {false, false, false, false};
  for (int w = 0; w < 4; ++w) {
    if (order[w] > 3 || seen[order[w]]) {
      return false;
    }
    seen[order[w]] = true;
  }
  return true;
}

}  // namespace logic

// =================================================================================================
// 3. Protocol (docs/hardware_drive_test.md, "Protocol"). Little-endian, packed.
// =================================================================================================

enum MsgType : uint8_t
{
  MSG_HELLO = 0x01,
  MSG_HELLO_REQ = 0x02,
  MSG_CONFIG = 0x10,
  MSG_CONFIG_ACK = 0x11,
  MSG_CMD = 0x20,
  MSG_STATE = 0x30,
};

enum : uint8_t
{
  FLAG_ENABLE = 0x01,
  FLAG_CLEAR_FAULT = 0x02,
};

enum : uint16_t
{
  FAULT_FE_WHEEL0 = 0x0001,   // bits 0..3: following error on wheel 0..3 (latched)
  FAULT_WATCHDOG = 0x0010,    // no CMD within the watchdog: wheels ramping to zero
  FAULT_NO_CONFIG = 0x0020,   // no valid CONFIG yet: nothing moves
  FAULT_OVERSPEED = 0x0040,   // a command was clamped to the maximum speed
  FAULT_LATCHED = 0x0080,     // a fault is latched: only zero until cleared
  FAULT_STEP_DRIVER = 0x0100, // the step generator failed to start (latched)
};

enum : uint8_t
{
  MODE_NO_CONFIG = 0,
  MODE_IDLE = 1,
  MODE_RUNNING = 2,
  MODE_FAULT = 3,
};

#pragma pack(push, 1)
struct HelloBody
{
  uint16_t protocol_version;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t wheels;
  uint32_t default_pulses_per_rev;
  uint32_t default_counts_per_rev;
};
struct ConfigBody
{
  uint32_t pulses_per_rev;
  uint32_t counts_per_rev;
  uint8_t motor_for_wheel[4];   // motor index (0..3) of FL, FR, RL, RR
  uint8_t dir_invert;           // bit m: invert motor m's direction
  uint8_t enc_invert;           // bit m: invert motor m's encoder count
  uint16_t watchdog_ms;
  float accel_backstop;         // rad/s^2 at the wheel
  float max_wheel_speed;        // rad/s
  int32_t following_error_limit;  // counts; 0 = off
};
struct ConfigAckBody
{
  uint8_t status;  // 0 ok, 1 invalid, 2 rejected while moving
};
struct CmdBody
{
  uint16_t seq;
  uint8_t flags;
  float wheel_velocity[4];  // rad/s, FL, FR, RL, RR; positive rolls the robot forward
};
struct StateBody
{
  uint16_t seq;
  uint16_t last_cmd_seq;
  uint64_t mcu_time_us;
  int64_t counts[4];          // per wheel, in the wheel convention
  int64_t pulses[4];          // emitted, per wheel, in the wheel convention
  int32_t following_error[4]; // counts
  uint16_t faults;
  uint16_t crc_errors;
  uint8_t mode;
};
#pragma pack(pop)

static_assert(sizeof(HelloBody) == 13, "HELLO layout");
static_assert(sizeof(ConfigBody) == 28, "CONFIG layout");
static_assert(sizeof(CmdBody) == 19, "CMD layout");
static_assert(sizeof(StateBody) == 97, "STATE layout");

static uint16_t g_crc_errors = 0;

static void sendFrame(uint8_t type, const void * body, size_t len)
{
  uint8_t payload[1 + sizeof(StateBody) + 2];
  payload[0] = type;
  memcpy(payload + 1, body, len);
  const uint16_t crc = logic::crc16(payload, 1 + len);
  payload[1 + len] = static_cast<uint8_t>(crc & 0xFF);
  payload[2 + len] = static_cast<uint8_t>(crc >> 8);
  uint8_t frame[sizeof(payload) + 4];
  const size_t n = logic::cobsEncode(payload, len + 3, frame);
  frame[n] = 0x00;
  // Never block the loop: drop the frame if the TX buffer is full (the host sees a seq gap).
  if (Serial.availableForWrite() >= static_cast<int>(n + 1)) {
    Serial.write(frame, n + 1);
  }
}

static void sendHello()
{
  HelloBody b;
  b.protocol_version = PROTOCOL_VERSION;
  b.fw_major = FW_MAJOR;
  b.fw_minor = FW_MINOR;
  b.wheels = 4;
  b.default_pulses_per_rev = DEFAULT_PULSES_PER_REV;
  b.default_counts_per_rev = DEFAULT_COUNTS_PER_REV;
  sendFrame(MSG_HELLO, &b, sizeof(b));
}

static void sendConfigAck(uint8_t status)
{
  ConfigAckBody b{status};
  sendFrame(MSG_CONFIG_ACK, &b, sizeof(b));
}

// =================================================================================================
// 4. Encoders: one pulse counter unit per motor, x4 quadrature, extended to 64 bits.
// =================================================================================================

static pcnt_unit_handle_t g_pcnt[4] = {nullptr, nullptr, nullptr, nullptr};
static int g_pcnt_last[4] = {0, 0, 0, 0};
static int64_t g_enc_raw[4] = {0, 0, 0, 0};  // per motor, as counted (before any invert)
static bool g_enc_ok = true;

static bool encoderInit(int m)
{
  pcnt_unit_config_t unit_config = {};
  unit_config.low_limit = -30000;
  unit_config.high_limit = 30000;
  unit_config.flags.accum_count = 1;  // keep counting past the limits
  if (pcnt_new_unit(&unit_config, &g_pcnt[m]) != ESP_OK) {
    return false;
  }
  pcnt_glitch_filter_config_t filter_config = {};
  filter_config.max_glitch_ns = ENC_GLITCH_NS;
  pcnt_unit_set_glitch_filter(g_pcnt[m], &filter_config);

  // x4 decoding: both edges of A (level of B) and both edges of B (level of A), as in the
  // ESP-IDF rotary encoder example.
  pcnt_chan_config_t a_config = {};
  a_config.edge_gpio_num = PIN_ENC_A[m];
  a_config.level_gpio_num = PIN_ENC_B[m];
  pcnt_channel_handle_t chan_a = nullptr;
  if (pcnt_new_channel(g_pcnt[m], &a_config, &chan_a) != ESP_OK) {
    return false;
  }
  pcnt_chan_config_t b_config = {};
  b_config.edge_gpio_num = PIN_ENC_B[m];
  b_config.level_gpio_num = PIN_ENC_A[m];
  pcnt_channel_handle_t chan_b = nullptr;
  if (pcnt_new_channel(g_pcnt[m], &b_config, &chan_b) != ESP_OK) {
    return false;
  }
  pcnt_channel_set_edge_action(
    chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(
    chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(
    chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(
    chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  // accum_count needs watch points at the limits.
  pcnt_unit_add_watch_point(g_pcnt[m], unit_config.high_limit);
  pcnt_unit_add_watch_point(g_pcnt[m], unit_config.low_limit);
  if (pcnt_unit_enable(g_pcnt[m]) != ESP_OK || pcnt_unit_clear_count(g_pcnt[m]) != ESP_OK ||
    pcnt_unit_start(g_pcnt[m]) != ESP_OK)
  {
    return false;
  }
  return true;
}

// Call often (every loop): folds the counter into the 64-bit count.
static void encoderPoll()
{
  for (int m = 0; m < 4; ++m) {
    if (!g_pcnt[m]) {
      continue;
    }
    int value = 0;
    if (pcnt_unit_get_count(g_pcnt[m], &value) == ESP_OK) {
      g_enc_raw[m] += static_cast<int32_t>(static_cast<uint32_t>(value) -
        static_cast<uint32_t>(g_pcnt_last[m]));
      g_pcnt_last[m] = value;
    }
  }
}

// =================================================================================================
// 5. Step generation. Interface: stepInit(), stepSetTarget(motor, signed Hz, Hz/s),
//    stepPoll() every loop, stepPulses(motor) (signed, emitted), stepStill(motor).
//    Signs here are motor signs (after the per-motor direction invert).
// =================================================================================================

#if STEP_DRIVER_FASTACCEL

static FastAccelStepperEngine g_engine;
static FastAccelStepper * g_stepper[4] = {nullptr, nullptr, nullptr, nullptr};
static int32_t g_fas_last[4] = {0, 0, 0, 0};
static int64_t g_fas_pulses[4] = {0, 0, 0, 0};
static double g_fas_target[4] = {0, 0, 0, 0};

static bool stepInit()
{
  g_engine.init();
  for (int m = 0; m < 4; ++m) {
    g_stepper[m] = g_engine.stepperConnectToPin(PIN_PUL[m], DRIVER_RMT);
    if (!g_stepper[m]) {
      return false;
    }
    g_stepper[m]->setDirectionPin(PIN_DIR[m], true, DIR_SETUP_US);
    g_stepper[m]->setAcceleration(1000);
  }
  return true;
}

static void stepSetTarget(int m, double hz, double accel_hz_s)
{
  FastAccelStepper * s = g_stepper[m];
  if (!s) {
    return;
  }
  const uint32_t accel = accel_hz_s < 1.0 ? 1 : static_cast<uint32_t>(accel_hz_s);
  const double mag = fabs(hz);
  if (hz == g_fas_target[m]) {
    return;  // unchanged: let the running ramp continue
  }
  g_fas_target[m] = hz;
  s->setAcceleration(accel);
  if (mag < MIN_PULSE_RATE_HZ) {
    s->stopMove();  // decelerates at the acceleration set above
    return;
  }
  s->setSpeedInHz(static_cast<uint32_t>(mag));
  if (hz > 0) {
    s->runForward();
  } else {
    s->runBackward();
  }
}

static void stepPoll()
{
  for (int m = 0; m < 4; ++m) {
    if (!g_stepper[m]) {
      continue;
    }
    const int32_t pos = g_stepper[m]->getCurrentPosition();
    g_fas_pulses[m] += static_cast<int32_t>(static_cast<uint32_t>(pos) -
      static_cast<uint32_t>(g_fas_last[m]));
    g_fas_last[m] = pos;
  }
}

static int64_t stepPulses(int m) {return g_fas_pulses[m];}

static bool stepStill(int m) {return !g_stepper[m] || !g_stepper[m]->isRunning();}

#else  // the built-in timer generator

// A 40 kHz timer interrupt runs one phase accumulator per motor: each overflow emits one pulse
// (high for one tick). The loop ramps each rate toward its target every CONTROL_PERIOD_US and
// changes direction only at zero rate, one control tick after setting DIR (>= DIR_SETUP_US).
static hw_timer_t * g_timer = nullptr;
static portMUX_TYPE g_step_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_inc[4] = {0, 0, 0, 0};
static volatile uint32_t g_phase[4] = {0, 0, 0, 0};
static volatile int8_t g_dir[4] = {1, 1, 1, 1};
static volatile int64_t g_pulses[4] = {0, 0, 0, 0};
static volatile uint32_t g_high_mask = 0;
static uint32_t g_pul_mask[4];
static double g_rate[4] = {0, 0, 0, 0};       // current signed rate (Hz)
static double g_target[4] = {0, 0, 0, 0};
static double g_accel[4] = {1000, 1000, 1000, 1000};
static bool g_dir_pending[4] = {false, false, false, false};
static uint32_t g_last_control_us = 0;

static void ARDUINO_ISR_ATTR onStepTick()
{
  portENTER_CRITICAL_ISR(&g_step_mux);
  if (g_high_mask) {
    REG_WRITE(GPIO_OUT_W1TC_REG, g_high_mask);
    g_high_mask = 0;
  }
  uint32_t mask = 0;
  for (int m = 0; m < 4; ++m) {
    const uint32_t inc = g_inc[m];
    if (inc) {
      const uint32_t before = g_phase[m];
      const uint32_t after = before + inc;
      g_phase[m] = after;
      if (after < before) {
        mask |= g_pul_mask[m];
        g_pulses[m] += g_dir[m];
      }
    }
  }
  if (mask) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask);
    g_high_mask = mask;
  }
  portEXIT_CRITICAL_ISR(&g_step_mux);
}

static bool stepInit()
{
  for (int m = 0; m < 4; ++m) {
    pinMode(PIN_PUL[m], OUTPUT);
    digitalWrite(PIN_PUL[m], LOW);
    pinMode(PIN_DIR[m], OUTPUT);
    digitalWrite(PIN_DIR[m], HIGH);  // HIGH = positive, as FastAccelStepper's dirHighCountsUp
    g_pul_mask[m] = 1UL << PIN_PUL[m];  // all PUL pins are below 32
  }
  g_timer = timerBegin(1000000);  // 1 MHz timer clock
  if (!g_timer) {
    return false;
  }
  timerAttachInterrupt(g_timer, &onStepTick);
  timerAlarm(g_timer, 1000000 / TIMER_TICK_HZ, true, 0);
  g_last_control_us = micros();
  return true;
}

static void stepSetTarget(int m, double hz, double accel_hz_s)
{
  g_target[m] = hz;
  g_accel[m] = accel_hz_s < 1.0 ? 1.0 : accel_hz_s;
}

static void setIncrement(int m, double hz)
{
  const double inc = fabs(hz) * 4294967296.0 / TIMER_TICK_HZ;
  const uint32_t value = inc >= 4294967295.0 ? 0xFFFFFFFFu : static_cast<uint32_t>(inc);
  portENTER_CRITICAL(&g_step_mux);
  g_inc[m] = fabs(hz) < MIN_PULSE_RATE_HZ ? 0 : value;
  portEXIT_CRITICAL(&g_step_mux);
}

static void stepPoll()
{
  const uint32_t now = micros();
  const uint32_t dt_us = now - g_last_control_us;
  if (dt_us < CONTROL_PERIOD_US) {
    return;
  }
  g_last_control_us = now;
  const double dt = dt_us * 1e-6;
  for (int m = 0; m < 4; ++m) {
    if (g_dir_pending[m]) {
      g_dir_pending[m] = false;  // DIR was set one control tick ago: pulses may start
    }
    double next = logic::rampToward(g_rate[m], g_target[m], g_accel[m] * dt);
    const int8_t want = next > 0 ? 1 : (next < 0 ? -1 : g_dir[m]);
    if (want != g_dir[m]) {
      // Pass through zero, set DIR, and wait a control tick before pulsing the other way.
      next = 0.0;
      portENTER_CRITICAL(&g_step_mux);
      g_inc[m] = 0;
      g_dir[m] = want;
      portEXIT_CRITICAL(&g_step_mux);
      digitalWrite(PIN_DIR[m], want > 0 ? HIGH : LOW);
      g_dir_pending[m] = true;
    }
    g_rate[m] = next;
    if (!g_dir_pending[m]) {
      setIncrement(m, next);
    }
  }
}

static int64_t stepPulses(int m)
{
  portENTER_CRITICAL(&g_step_mux);
  const int64_t value = g_pulses[m];
  portEXIT_CRITICAL(&g_step_mux);
  return value;
}

static bool stepStill(int m) {return fabs(g_rate[m]) < MIN_PULSE_RATE_HZ;}

#endif

// =================================================================================================
// 6. Drive state
// =================================================================================================

struct RuntimeConfig
{
  uint32_t pulses_per_rev = DEFAULT_PULSES_PER_REV;
  uint32_t counts_per_rev = DEFAULT_COUNTS_PER_REV;
  uint8_t motor_for_wheel[4] = {0, 1, 2, 3};
  uint8_t dir_invert = 0;
  uint8_t enc_invert = 0;
  uint16_t watchdog_ms = 200;
  float accel_backstop = 5.0f;     // rad/s^2
  float max_wheel_speed = 3.0f;    // rad/s, about 0.3 m/s at the rim
  int32_t following_error_limit = 0;
};

static RuntimeConfig g_cfg;
static bool g_configured = false;
static bool g_step_ok = false;
static uint16_t g_latched = 0;        // latched fault bits
static bool g_enabled = false;
static uint16_t g_last_cmd_seq = 0;
static uint32_t g_last_cmd_ms = 0;
static bool g_watchdog = false;
static bool g_overspeed = false;
static float g_cmd[4] = {0, 0, 0, 0};  // wheel rad/s, FL FR RL RR
static int64_t g_fe_offset[4] = {0, 0, 0, 0};
static uint16_t g_state_seq = 0;
static uint32_t g_last_state_us = 0;

// Per wheel, in the wheel convention.
static int64_t wheelCounts(int w)
{
  const int m = g_cfg.motor_for_wheel[w];
  return (g_cfg.enc_invert >> m) & 1 ? -g_enc_raw[m] : g_enc_raw[m];
}

static int64_t wheelPulses(int w)
{
  const int m = g_cfg.motor_for_wheel[w];
  return (g_cfg.dir_invert >> m) & 1 ? -stepPulses(m) : stepPulses(m);
}

static int64_t wheelFollowingError(int w)
{
  return logic::followingError(
    wheelPulses(w), wheelCounts(w), g_cfg.pulses_per_rev, g_cfg.counts_per_rev, g_fe_offset[w]);
}

static bool allStill()
{
  for (int m = 0; m < 4; ++m) {
    if (!stepStill(m)) {
      return false;
    }
  }
  return true;
}

static void rebaseFollowingError()
{
  for (int w = 0; w < 4; ++w) {
    g_fe_offset[w] = 0;
    g_fe_offset[w] = wheelFollowingError(w);
  }
}

static void handleConfig(const ConfigBody & c)
{
  if (!allStill()) {
    sendConfigAck(2);
    return;
  }
  if (c.pulses_per_rev < 200 || c.pulses_per_rev > 51200 || c.counts_per_rev < 100 ||
    c.counts_per_rev > 1000000 || !logic::validMotorOrder(c.motor_for_wheel) ||
    c.watchdog_ms < 20 || c.watchdog_ms > 2000 || !(c.accel_backstop > 0.0f) ||
    !(c.max_wheel_speed > 0.0f) || c.following_error_limit < 0)
  {
    sendConfigAck(1);
    return;
  }
  g_cfg.pulses_per_rev = c.pulses_per_rev;
  g_cfg.counts_per_rev = c.counts_per_rev;
  memcpy(g_cfg.motor_for_wheel, c.motor_for_wheel, 4);
  g_cfg.dir_invert = c.dir_invert;
  g_cfg.enc_invert = c.enc_invert;
  g_cfg.watchdog_ms = c.watchdog_ms;
  g_cfg.accel_backstop = c.accel_backstop;
  g_cfg.max_wheel_speed = c.max_wheel_speed;
  g_cfg.following_error_limit = c.following_error_limit;
  g_configured = true;
  g_enabled = false;
  for (int w = 0; w < 4; ++w) {
    g_cmd[w] = 0.0f;
  }
  rebaseFollowingError();
  sendConfigAck(0);
}

static void handleCmd(const CmdBody & c)
{
  g_last_cmd_seq = c.seq;
  g_last_cmd_ms = millis();
  g_watchdog = false;
  if ((c.flags & FLAG_CLEAR_FAULT) && g_latched && allStill() && g_step_ok) {
    g_latched = 0;
    rebaseFollowingError();
  }
  g_enabled = (c.flags & FLAG_ENABLE) != 0;
  bool clamped = false;
  for (int w = 0; w < 4; ++w) {
    const float v = c.wheel_velocity[w];
    g_cmd[w] = isfinite(v) ?
      static_cast<float>(logic::clampAbs(v, g_cfg.max_wheel_speed, &clamped)) : 0.0f;
  }
  g_overspeed = clamped;
}

static void handleFrame(const uint8_t * frame, size_t len)
{
  uint8_t payload[64];
  if (len > sizeof(payload)) {
    ++g_crc_errors;
    return;
  }
  const size_t n = logic::cobsDecode(frame, len, payload);
  if (n < 3) {
    ++g_crc_errors;
    return;
  }
  const uint16_t crc = static_cast<uint16_t>(payload[n - 2] | (payload[n - 1] << 8));
  if (logic::crc16(payload, n - 2) != crc) {
    ++g_crc_errors;
    return;
  }
  const uint8_t type = payload[0];
  const uint8_t * body = payload + 1;
  const size_t body_len = n - 3;
  if (type == MSG_HELLO_REQ && body_len == 0) {
    sendHello();
  } else if (type == MSG_CONFIG && body_len == sizeof(ConfigBody)) {
    ConfigBody c;
    memcpy(&c, body, sizeof(c));
    handleConfig(c);
  } else if (type == MSG_CMD && body_len == sizeof(CmdBody)) {
    CmdBody c;
    memcpy(&c, body, sizeof(c));
    handleCmd(c);
  } else {
    ++g_crc_errors;
  }
}

// Serial frame reader: bytes up to 0x00 make one frame.
static uint8_t g_rx[96];
static size_t g_rx_len = 0;
static bool g_rx_overflow = false;

static void readSerial()
{
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) {
      break;
    }
    if (c == 0) {
      if (!g_rx_overflow && g_rx_len > 0) {
        handleFrame(g_rx, g_rx_len);
      } else if (g_rx_overflow) {
        ++g_crc_errors;
      }
      g_rx_len = 0;
      g_rx_overflow = false;
    } else if (g_rx_len < sizeof(g_rx)) {
      g_rx[g_rx_len++] = static_cast<uint8_t>(c);
    } else {
      g_rx_overflow = true;
    }
  }
}

// Apply the drive logic: what each motor should do now.
static void updateDrive()
{
  if (g_configured && g_enabled && !g_latched &&
    logic::expired(millis(), g_last_cmd_ms, g_cfg.watchdog_ms))
  {
    g_watchdog = true;
  }
  const bool may_move = g_configured && g_step_ok && g_enabled && !g_latched && !g_watchdog;

  // Following error: latch, and every wheel ramps to zero.
  if (g_configured && g_cfg.following_error_limit > 0) {
    for (int w = 0; w < 4; ++w) {
      const int64_t fe = wheelFollowingError(w);
      if (fe > g_cfg.following_error_limit || fe < -g_cfg.following_error_limit) {
        g_latched |= static_cast<uint16_t>(FAULT_FE_WHEEL0 << w) | FAULT_LATCHED;
      }
    }
  }

  const double accel = logic::wheelAccelToPulse(g_cfg.accel_backstop, g_cfg.pulses_per_rev);
  const double cap = MAX_PULSE_RATE_HZ;
  for (int w = 0; w < 4; ++w) {
    const int m = g_cfg.motor_for_wheel[w];
    double hz = may_move && !g_latched ?
      logic::wheelToPulseHz(g_cmd[w], g_cfg.pulses_per_rev) : 0.0;
    bool clamped = false;
    hz = logic::clampAbs(hz, cap, &clamped);
    if (clamped) {
      g_overspeed = true;
    }
    if ((g_cfg.dir_invert >> m) & 1) {
      hz = -hz;
    }
    if (g_configured && g_step_ok) {
      stepSetTarget(m, hz, accel);
    }
  }
}

static void sendState()
{
  StateBody s;
  s.seq = g_state_seq++;
  s.last_cmd_seq = g_last_cmd_seq;
  s.mcu_time_us = static_cast<uint64_t>(esp_timer_get_time());
  uint16_t faults = g_latched;
  for (int w = 0; w < 4; ++w) {
    s.counts[w] = wheelCounts(w);
    s.pulses[w] = wheelPulses(w);
    const int64_t fe = wheelFollowingError(w);
    s.following_error[w] = fe > INT32_MAX ? INT32_MAX : (fe < INT32_MIN ? INT32_MIN :
      static_cast<int32_t>(fe));
  }
  if (g_watchdog) {
    faults |= FAULT_WATCHDOG;
  }
  if (!g_configured) {
    faults |= FAULT_NO_CONFIG;
  }
  if (g_overspeed) {
    faults |= FAULT_OVERSPEED;
  }
  if (!g_step_ok || !g_enc_ok) {
    faults |= FAULT_STEP_DRIVER | FAULT_LATCHED;
  }
  s.faults = faults;
  s.crc_errors = g_crc_errors;
  if (!g_configured) {
    s.mode = MODE_NO_CONFIG;
  } else if (faults & FAULT_LATCHED) {
    s.mode = MODE_FAULT;
  } else if (g_enabled && !g_watchdog) {
    s.mode = MODE_RUNNING;
  } else {
    s.mode = MODE_IDLE;
  }
  sendFrame(MSG_STATE, &s, sizeof(s));
}

// =================================================================================================
// 7. setup() / loop()
// =================================================================================================

void setup()
{
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);

  // Steps first, so the PUL pins are driven low before anything else.
  g_step_ok = stepInit();
  for (int m = 0; m < 4; ++m) {
    if (!encoderInit(m)) {
      g_enc_ok = false;
    }
  }
  if (!g_step_ok || !g_enc_ok) {
    g_latched |= FAULT_STEP_DRIVER | FAULT_LATCHED;
  }
  delay(100);
  sendHello();
  g_last_state_us = micros();
}

void loop()
{
  readSerial();
  encoderPoll();
  stepPoll();
  updateDrive();
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - g_last_state_us) >= STATE_PERIOD_US) {
    g_last_state_us = now;
    sendState();
  }
}
```
