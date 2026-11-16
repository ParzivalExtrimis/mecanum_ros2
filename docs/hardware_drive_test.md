# Real drive on the tapped encoders: protocol, test procedure and results

Stage 5H of [`hardware_transition.md`](hardware_transition.md), drive only: the ESP32 drive
firmware, the ros2_control plugin `mecanum_hardware/Esp32System`, and the tests that take the
real base from the desk to tuned limits under teleop. Sensors, the EKF on the hardware and
autonomy are planned later. Written 2026-11-12. T0 passed; T1 to T5 wait on the owner's
upload of the sketch.

```
laptop ──USB serial 921600──► ESP32-DevKitC ──PUL/DIR──► 4 × CS-D808 ──► 4 × CS-M234120
  ▲  Esp32System (ros2_control)      ▲                                     │ encoder A/B
  │  /joint_states, /wheel/odometry  └──── 6N137 + level converter ◄───────┘ (the tap)
```

- **The drivers close their own position loop** on the motor encoders. The ESP32 emits step
  pulses at the commanded rate, counts the same encoders through the tap, and reports both.
  The difference is the **following error**, the stack's own fault signal.
- **ENA and ALM are not wired.** The drivers are enabled whenever powered (holding torque);
  stopping means zero pulses. A driver that faults internally shows only as a following error.
- **Stops on the floor: the power lead and the software.** There is no physical E-stop. Every
  floor step is run at low speed, with a person at the power lead, after the owner confirms.
- **The tap's electrics are not verified** (out of the project's scope, decision 0006). No
  step below tests them. Section 9 lists what points back to them.

Contents: 1 Protocol · 2 Where each value lives · 3 Tools · 4 Safety rules · 5 Stages T0–T5 ·
6 Limit rule · 7 Recovery after a stop · 8 Known controller issue · 9 Not verified ·
10 Results log.

## 1. Protocol

The single reference for the link. The sketch (`firmware/mecanum_drive_esp32`), the host
(`mecanum_hardware/drive_protocol.cpp`) and the Python reference
(`mecanum_hardware/test/drive_protocol.py`, used by the fake ESP32) implement this table; the
host's unit test checks byte vectors made from it. A change here means a new
`PROTOCOL_VERSION` in all three; the plugin refuses a board that reports another version.

### Framing

- A frame is `COBS(type, body, crc) 0x00`: COBS removes every zero byte, so 0x00 only ends a
  frame. A reader that starts mid-stream resynchronizes at the next 0x00.
- `type`: one byte, the message code below. `body`: the fields below, packed, no padding.
- `crc`: CRC-16/CCITT-FALSE (polynomial 0x1021, initial 0xFFFF, no reflection, no final XOR)
  over `type` and `body`, sent little-endian. Check value: "123456789" gives 0x29B1.
- Every field is little-endian. `f32` is IEEE 754 single precision.
- A frame with a bad COBS block, CRC, unknown type or wrong body size is dropped and counted
  (`crc_errors` on the ESP32, `host_rx_errors` on the host).

### Messages

| Code | Name | Direction | Body (bytes) | When |
|---|---|---|---|---|
| 0x01 | HELLO | ESP32 → host | 13 | At boot, and in answer to HELLO_REQ |
| 0x02 | HELLO_REQ | host → ESP32 | 0 | On configure, every 250 ms until HELLO arrives (3 s timeout) |
| 0x10 | CONFIG | host → ESP32 | 28 | On activate (up to 3 attempts) |
| 0x11 | CONFIG_ACK | ESP32 → host | 1 | In answer to CONFIG |
| 0x20 | CMD | host → ESP32 | 19 | Every controller cycle (50 Hz) |
| 0x30 | STATE | ESP32 → host | 97 | Every 10 ms (100 Hz) |

**HELLO** (0x01)

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | protocol version (1) |
| 2 | u8 | firmware major |
| 3 | u8 | firmware minor |
| 4 | u8 | wheels (4) |
| 5 | u32 | default pulses per revolution (the sketch's `DEFAULT_PULSES_PER_REV`) |
| 9 | u32 | default counts per revolution (`DEFAULT_COUNTS_PER_REV`) |

**CONFIG** (0x10)

| Offset | Type | Field | Valid range |
|---|---|---|---|
| 0 | u32 | step pulses per wheel revolution | 200 to 51200 |
| 4 | u32 | encoder counts per wheel revolution (x4 decoding) | 100 to 1,000,000 |
| 8 | u8[4] | motor index (0..3) driving FL, FR, RL, RR | a permutation of 0..3 |
| 12 | u8 | direction invert: bit m inverts motor m's steps | any |
| 13 | u8 | encoder invert: bit m inverts motor m's count | any |
| 14 | u16 | watchdog (ms) | 20 to 2000 |
| 16 | f32 | acceleration backstop (rad/s² at the wheel), for every ramp | > 0 |
| 20 | f32 | maximum wheel speed (rad/s); commands are clamped to it | > 0 |
| 24 | i32 | following-error limit (counts); 0 turns the check off | ≥ 0 |

**CONFIG_ACK** (0x11): one `u8` status. 0 accepted; 1 a value out of range (nothing changed);
2 refused because a wheel is still turning (CONFIG is taken only with every wheel still).

**CMD** (0x20)

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | sequence (wraps) |
| 2 | u8 | flags: bit 0 motion enable, bit 1 clear fault |
| 3 | f32[4] | wheel velocity targets (rad/s), FL, FR, RL, RR; positive rolls the robot forward. A non-finite value counts as 0. |

**STATE** (0x30)

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | sequence (wraps) |
| 2 | u16 | sequence of the newest CMD received |
| 4 | u64 | ESP32 time (µs since boot) |
| 12 | i64[4] | encoder counts, FL, FR, RL, RR, in the wheel convention (inverts applied), continuous |
| 44 | i64[4] | step pulses emitted, same order and convention |
| 76 | i32[4] | following error (counts), same order |
| 92 | u16 | fault bits (below) |
| 94 | u16 | bad frames received |
| 96 | u8 | mode: 0 no configuration, 1 idle, 2 running, 3 fault |

Fault bits: 0x0001..0x0008 following error on FL, FR, RL, RR (latched); 0x0010 watchdog (no
CMD within the watchdog, wheels ramping to zero; clears on the next CMD); 0x0020 no
configuration yet; 0x0040 a command was clamped to the maximum speed; 0x0080 a fault is latched;
0x0100 the step generator or the encoder counters failed to start (latched, needs a reset).

### Behaviour on the ESP32

- **Boot.** PUL pins low; nothing moves until a valid CONFIG has been accepted and a CMD with
  motion enabled arrives. The motor-to-wheel order, the inverts, the watchdog, the backstop,
  the speed cap and the following-error limit all come from CONFIG: tuning needs no re-upload.
- **Motion.** Each wheel ramps toward its target at the backstop. With the watchdog expired,
  motion disabled or a fault latched, every wheel ramps to zero at the backstop.
- **Following error** per wheel = `trunc(pulses × counts_per_rev / pulses_per_rev) − counts −
  offset`. The offset is taken when CONFIG is accepted and when a fault is cleared. Over the
  limit on any wheel, the fault latches and all four wheels ramp to zero.
- **Clear fault** (CMD bit 1) works only with every wheel still, and rebases the offsets.

### Behaviour on the host (`Esp32System`)

- **Configure:** open the port, ask for HELLO, check the protocol version.
- **Activate:** send CONFIG (the xacro's values, the limit converted from rad to counts),
  clear any fault with motion off, wait for mode idle.
- **read():** position = counts × 2π / counts_per_rev, velocity = count change over the read
  period (decision 0006). Returns ERROR, which stops everything, when the newest STATE is
  older than 100 ms, the link is lost, or a fault is latched.
- **write():** one CMD per cycle, motion enabled while active.
- **ERROR** → the controller manager deactivates the controllers, `/joint_states` and the
  wheel odometry stop, the safety gate's `joint_states` watchdog blocks every command, and the
  component goes back to unconfigured (it sends a stop and releases the port).
- Publishes `/drive/state` (`mecanum_interfaces/DriveState`, every STATE, with the command
  sent and the host's bad-frame count) and `/drive/following_error` (1 Hz: four current errors,
  then four maxima, in wheel rad; the same topic as the simulation).

## 2. Where each value lives

All launch arguments below are arguments of `drive_test.launch.py` (and of
`controllers.launch.py hardware:=esp32`); they go to the xacro, then to the ESP32 by CONFIG on
every activation. Their defaults live in one place, the `xacro:arg` defaults of
`src/mecanum_controllers/urdf/mecanum_esp32.urdf.xacro`: a launch argument left out (empty)
is not passed, so the xacro's value applies. Rows marked STAND-IN are in `STANDINS.md`.

| Value | Default | Where | Set in |
|---|---|---|---|
| `port` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (the dev laptop) | xacro default, launch arg | T1: the `/dev/serial/by-id/...` path; changes on the Jetson |
| `pulses_per_rev` | 1600, measured T2 | xacro default, launch arg | T2, measured |
| `counts_per_wheel_rev` | 4000, measured T2 | xacro default, launch arg | T2, measured |
| `motor_for_wheel`, `dir_invert`, `enc_invert` | `3 0 2 1`, `0 0 1 1`, `0 0 1 1`, measured T2 | xacro defaults, launch args | T2, `drive_monitor.py identify` |
| `following_error_limit` | 0.5 rad at the wheel (owner's choice; the T2 soak allows down to 0.05) | xacro default, launch arg | T2 (0 = off for identify) |
| `watchdog_ms` | 200 | xacro default, launch arg | T2 check |
| `accel_backstop` | 2.5 rad/s², STAND-IN (0.26 m/s², 0.57 rad/s²) | xacro default, launch arg | T5 ladder |
| `max_wheel_speed` | 3.5 rad/s, STAND-IN (0.36 m/s at the rim) | xacro default, launch arg | T5, with the speed ladder |
| Speed limits | 0.1 m/s x, 0.1 m/s y, 0.3 rad/s, STAND-IN | `controllers_hardware.yaml` | T5 (`drive_tune.py` raises them at run time, then restores) |
| Reference timeout | 0.25 s | `controllers.yaml` | T5 check |
| Wheel radius, `lx + ly` | 0.1025 m, 0.451 m, STAND-IN | `controllers.yaml`, the URDF | T4 |
| Pins, step generator | the owner's pin map | the sketch's CONFIG block | Re-upload only |

## 3. Tools

| Tool | Use |
|---|---|
| `ros2 launch mecanum_bringup drive_test.launch.py port:=... [rviz:=true]` | The drive: controllers on the ESP32, the safety gate (`/cmd_vel` → `/cmd_vel_safe`, watching `/joint_states` only), RViz in `odom` |
| `ros2 launch mecanum_bringup teleop.launch.py` (or `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_footprint -p speed:=0.1 -p turn:=0.3`) | Teleop; the launch opens it in its own window, type there. Hold a key to drive; release it to stop (the 0.25 s reference timeout). `i` forward, `,` back, `j`/`l` rotate, Shift+`J`/`L` strafe, `k` stop. |
| `tools/drive_monitor.py` | Live table per wheel: command, speeds from pulses and from counts, totals since Enter, following error, faults, mode |
| `tools/drive_monitor.py identify` | On blocks: the motor order and signs from three patterns and the owner's answers |
| `tools/drive_tune.py` | The ladders: steps with confirmation, the limit rule, CSV output (`--help`) |
| [`drive_firmware.md`](drive_firmware.md) | Reference: the sketch, `Esp32System`, and every configurable value with where it lives |
| `tools/fe_soak.py` | T2 check 4 scripted: still, hard start/stop cycles per axis, reversals, still; CSV of every report, the largest following error per phase, the settled error after each stop (build-up), a suggested limit (`--help`) |
| `tools/l2_wall_range.py --side front` | Distance and angle to a wall from the L2, for setting up runs |
| `ros2 launch mecanum_base motion_test.launch.py measure_odom_topic:=/wheel/odometry linear_speed:=0.1 angular_speed:=0.3` | The axes pattern and the loop (T4) |
| `python3 src/mecanum_hardware/test/fake_esp32.py --link /tmp/fake_esp32` | A fake ESP32 on a pseudo-terminal, for rehearsals (`port:=/tmp/fake_esp32`) |

The L2, when a reference is needed: `ros2 launch mecanum_lidar l2.launch.py robot_model:=false
rviz:=false` (its URDF frames come from `drive_test.launch.py`).

## 4. Safety rules (every powered stage)

1. **Before power:** wheels clear (blocks) or the path clear (floor); a person holds the power
   lead and is ready to pull it; everyone knows that releasing the teleop key stops the robot
   and that the power lead is the only hard stop.
2. **One speed step at a time,** each one confirmed by the owner. Nothing is raised during a
   run.
3. **Stop at once** (release the key, or pull the lead) on any noise, judder, tipping, a wheel
   that does not turn, or an unexpected direction. Record it before the next try.
4. **Never add acceleration limits to the drive controller** with the installed version
   (section 8): a released key would then leave three wheels running.
5. Close the Arduino serial monitor before starting ROS: one program holds the port.

## 5. Stages

### T0: no hardware (done 2026-11-12)

```
colcon build --packages-up-to mecanum_bringup
colcon test --packages-select mecanum_hardware mecanum_controllers
colcon test-result --verbose
```

- `mecanum_hardware`: the protocol gtest (byte vectors from the table above), the encoder
  gtest, lint.
- `mecanum_controllers`, `test_esp32_fake.py`: the full stack on the fake ESP32. The odometry
  returns the twist; the hardware speed limits apply; the reference timeout stops all four
  wheels; a frozen encoder trips the fault and the recovery of section 7 works; a mute link
  stops everything and the same recovery works.
- **Optional GUI rehearsal** (four terminals, `source install/setup.bash` in each):
  1. `python3 src/mecanum_hardware/test/fake_esp32.py --link /tmp/fake_esp32`
  2. `ros2 launch mecanum_bringup drive_test.launch.py port:=/tmp/fake_esp32 rviz:=true`
  3. `tools/drive_monitor.py`
  4. the teleop command of section 3.
  Expect: RViz shows the robot moving in `odom` while a key is held and stopping on release;
  the monitor shows commands, equal pulse and count speeds, following errors of 0 to 2 counts.
  `kill -USR1 <fake pid>` freezes the front-left encoder: within about a second terminal 2
  logs "ESP32 fault, all wheels stopped: following error front_left", the controllers
  deactivate and the robot stops in RViz. Recover with section 7.

### T1: the ESP32 on USB, the drivers unpowered

After the owner has uploaded the sketch (`firmware/mecanum_drive_esp32/README.md`). The
drivers' PUL/DIR inputs may stay connected; their power stays off.

| Terminal | Command |
|---|---|
| 1 | `ls -l /dev/serial/by-id/` → note the DevKitC's path, then `ros2 launch mecanum_bringup drive_test.launch.py port:=/dev/serial/by-id/<path> following_error_limit:=0` |
| 2 | `tools/drive_monitor.py` |
| 3 | teleop (section 3) |

Checks:
1. **Connect.** Terminal 1 logs "ESP32 drive firmware 0.1, protocol 1, defaults 1600 pulses /
   4000 counts per rev", then "ESP32 drive active", and both spawners finish. Failure "No
   HELLO": wrong port, the serial monitor still open, or the sketch not running.
2. **Idle.** Terminal 2: mode idle, faults none, pulses and counts not moving, rx errors 0.
   `ros2 topic hz /drive/state` ≈ 100 Hz.
3. **Pulse rates.** Hold `i` (forward 0.1 m/s): every wheel's command is +0.976 rad/s and its
   pulse speed the same (248 pulses/s at 1600). Shift+`J` (strafe left): FL −, FR +, RL +,
   RR −. `j` (rotate left, 0.3 rad/s): FL −, FR +, RL −, RR + at 1.32 rad/s. Counts stay at 0
   (the encoders are powered by the drivers). Release: the pulse speeds ramp to 0 within
   about 0.5 s.
4. **Fault path without motion.** Stop terminal 1 (Ctrl-C) and relaunch it without
   `following_error_limit:=0`. Hold `i`: within about 0.5 s the following error passes
   0.5 rad (318 counts) because the counts do not move. Terminal 1 logs "ESP32 fault, all
   wheels stopped", the controllers deactivate, terminal 2 shows mode FAULT and pulses
   ramping to zero. Recover with section 7: it must come back to idle.
5. **Reset while connected.** Press the DevKitC's EN button. Terminal 1 logs a stale STATE and
   stops. Recover with section 7: HELLO is asked for again and the drive becomes active.
6. **Strapping pins.** Power-cycle the ESP32 (USB out and in) with the drivers' inputs
   connected, then start terminal 1 again: it must connect. If not, DIR on GPIO 5 or 15 holds
   a strapping level at boot: move it (sketch README, section 5).

Record: the port path, the firmware version, the measured pulse speeds against the commands.

### T2: on blocks, wheels in the air, the drivers powered

The robot on blocks so that no wheel touches anything. Hands clear of the wheels.

| Terminal | Command |
|---|---|
| 1 | `ros2 launch mecanum_bringup drive_test.launch.py port:=<path> following_error_limit:=0` |
| 2 | `tools/drive_monitor.py identify` (then `tools/drive_monitor.py`) |
| 3 | teleop, for the checks after identify |

Checks:
1. **Wheel order and signs.** Terminal 2 runs forward, strafe left and rotate left at a crawl
   (0.05 m/s, 0.15 rad/s), one at a time. For each, look at FL, FR, RL, RR (front: the
   D435i end) and type `f` (the top of the wheel moves forward: it would roll the robot
   forward) or `b`, then Enter. It prints `motor_for_wheel:=... dir_invert:=... enc_invert:=...`.
   Relaunch terminal 1 with them (still `following_error_limit:=0`) and run identify again:
   the answers must now be `f f f f`, `b f f b`, `b f b f`, and it must print the same values.
   A wheel that does not turn at all: its driver power, PUL/DIR wiring or the pin map.
2. **Counts agree with pulses.** Terminal 2 in watch mode, teleop `i` at 0.1 m/s for 10 s and
   `,` back: for every wheel the count speed equals the pulse speed and the following error
   stays within a few counts. Repeat with strafe and rotate.
3. **Pulses and counts per revolution.** Put a tape mark on one wheel and a fixed pointer
   beside it. Press Enter in terminal 2 (totals reset), hold `i` until the wheel has turned
   exactly 10 times (count them; release on the mark). pulses per rev = pulses total / 10;
   counts per rev = counts total / 10. Do it on two wheels. Expect 1600 (if the DIP is at
   1600) and 4000. Relaunch with the measured `pulses_per_rev:=` and `counts_per_wheel_rev:=`
   and with the default following-error limit (0.5 rad).
4. **Following error in normal running.** Teleop forward, strafe and rotate, starting and
   stopping hard (press, release) for a minute each. Read the maxima on
   `ros2 topic echo /drive/following_error` (the last four values, rad). Expect a few counts
   (under 0.01 rad). The limit for T3 to T5 is 2 to 3 times the largest seen, but not below
   0.05 rad: relaunch with it.
5. **The fault trips and stops everything.** Relaunch with `following_error_limit:=0.003`
   (about 2 counts). Hold `i`: the fault trips in normal running, every wheel stops, terminal
   1 logs the wheel, `/joint_states` stops. Recover (section 7) after relaunching with the
   real limit. Optional, at 0.05 m/s only, gloved: brake one tyre with a wooden block until
   the fault trips. Skip it if the motor holds against the block; the low-limit test above
   proves the same path.
6. **Host loss.** Hold `i` and pull the USB cable: all wheels stop within the watchdog
   (0.2 s) plus the ramp. Terminal 1 logs a lost link. Plug back in, relaunch.
7. **Teleop on blocks.** Every key of section 3 turns the wheels as the robot should move;
   releasing stops them.

Record: the motor order and signs, pulses and counts per revolution (both wheels), the largest
following error per axis, the chosen limit, the watchdog stop.

### T3: the floor, at 0.1 m/s, a person at the power lead

At least 2 m clear in every direction on the target floor (tiles). The person at the power
lead stays beside the robot, not in its path.

| Terminal | Command |
|---|---|
| 1 | `ros2 launch mecanum_bringup drive_test.launch.py port:=<path> <T2 values> rviz:=true` |
| 2 | `tools/drive_monitor.py` |
| 3 | teleop (speed 0.1, turn 0.3: the hardware limits clamp any higher teleop speed) |

Checks, short presses first (half a second), then 2 to 3 s:
1. Forward and back, strafe left and right, rotate both ways, the diagonals (`u`, `o`, `m`,
   `.`). The robot moves as commanded; RViz agrees in direction.
2. Stop on release: the robot stops within about 0.5 s (0.25 s timeout plus the ramp), all
   wheels together, straight.
3. Combined: Shift+`U`, `O`, `M`, `>` (holonomic diagonals) and a strafe while rotating.
4. Watch: following error small, no faults, no stall noise, no judder, no wheel lift.

Record: what was seen per check; any following error peak.

### T4: the floor, compliance and geometry

| Terminal | Command |
|---|---|
| 1 | as T3, plus terminal 1b: `ros2 launch mecanum_lidar l2.launch.py robot_model:=false rviz:=false` |
| 2 | the run commands below |

1. **Axes.** `ros2 launch mecanum_base motion_test.launch.py measure_odom_topic:=/wheel/odometry
   linear_speed:=0.1 angular_speed:=0.3 move_duration:=3.0`. Every step PASS (directions
   right, odometry ≈ command).
2. **Wheel radius.** The robot square to a wall 4 m ahead (`tools/l2_wall_range.py --side
   front`: yaw within 1°), a tape mark at the robot's front edge.
   `tools/drive_tune.py --axis x --speeds 0.1 --distance 3.0 --reference wall --wall front
   --tape`. Tape the travel. New radius = 0.1025 × reference / odometry (the tape when given).
   Do it three times; use the mean.
3. **Strafe.** The same against a wall to the left: `--axis y --speeds 0.1 --distance 2.0
   --wall left`. The strafe slip (rollers) is expected to be larger; do not fit the radius from
   it.
4. **lx + ly.** `tools/drive_tune.py --axis wz --speeds 0.3 --angle 1080 --reference gyro`
   (three turns and back), a tape line on the floor along the robot's side to check the final
   heading by eye. New `lx + ly` = 0.451 × (new radius / 0.1025) × odometry angle / gyro angle.
5. **Loop.** `ros2 launch mecanum_base motion_test.launch.py pattern:=loop loop_length:=2.0
   loop_width:=1.0 linear_speed:=0.1 angular_speed:=0.3 measure_odom_topic:=/wheel/odometry`;
   tape the end point against the start mark.
6. Write the radius and `lx + ly` into `controllers.yaml` (and the URDF and `esp32_driver.yaml`,
   as the `STANDINS.md` row says), rebuild, repeat 2 and 4 once: slip within the limit rule.

Record: every run's CSV directory, the fitted values, the loop's end error.

### T5: the floor, tuning ladders

The same terminals as T4. Every step is confirmed in `drive_tune.py`. The leg length must fit
the floor at the step's speed plus the stop.

1. **Speed ladder, per axis.** x: `tools/drive_tune.py --axis x --speeds
   0.1,0.15,0.2,0.25,0.3,0.4,0.5 --distance 1.5 --reference wall --wall front --tape`; y the
   same with `--axis y --wall left`; rotation `--axis wz --speeds 0.3,0.5,0.8,1.0,1.2,1.5
   --angle 360 --reference gyro`. Stop the ladder at the first FAIL. When the tool says a step
   needs more `max_wheel_speed`, relaunch terminal 1 with the value it gives.
2. **Acceleration ladder** (the ESP32 backstop, per wheel). At a mid speed (half the top
   passing speed), relaunch terminal 1 with `accel_backstop:=` 2.5, 4, 6, 8, 10, 15, 20
   (0.26 to 2.0 m/s² along x), one `drive_tune.py --axis x --speeds <mid>` run each. The
   summary has the ramp time and the stopping distance. Stop at the first FAIL.
3. **Stopping by timeout at the top speed.** `--stop timeout` at the top passing speed of each
   axis: the stop distance must be the timeout (0.25 s × speed) plus the ramp.
4. **Combined twists.** `tools/drive_tune.py --axis combined --twist <vx>,<vy>,0 --twist
   <vx>,0,<wz> --duration 3` at 70% of the axis limits: the path direction against the command
   (the dx, dy columns), following error, faults.
5. **Defaults.** For each axis: the speed limit = 70% of the last passing speed; the backstop =
   70% of the last passing value; the following-error limit = 2 to 3 times the largest seen in
   T5. Set them in `controllers_hardware.yaml` and the xacro defaults, rerun one step of each
   axis at the new defaults, then write decision 0007 (drive limits and profiles).

## 6. Limit rule (owner, 2026-11-11)

A step **fails** on any of:
- a following error over 50% of the stack's limit (`following_error_limit`);
- slip over **3%** forward, **8%** strafe or **3%** rotation, against the reference (the tape
  when measured, else the L2 wall range or the L2 gyro);
- a fault, an audible stall, visible tipping or judder.

The **default** is 70% of the last passing step. Speeds rise one step at a time, each step
confirmed by the owner. Slip is `(odometry − reference) / |reference|`: positive when the
wheels report more motion than happened.

## 7. Recovery after a stop

After a fault, a stale report or a lost link, the drive component is back to unconfigured and
the two controllers are inactive. First find and fix the cause (terminal 1's last error, the
monitor's fault bits). Then, wheels still, in any terminal:

```
ros2 control set_hardware_component_state mecanum_base active
ros2 control switch_controllers --activate joint_state_broadcaster mecanum_drive_controller
```

The first line reconnects (HELLO), sends CONFIG and clears the latched fault; it fails if a
wheel is still turning or the fault persists. Once `/joint_states` flows again the safety gate
unblocks by itself. Relaunching terminal 1 does the same.

## 8. Known controller issue: no acceleration limits in the drive controller

Found on the fake ESP32 in T0 (2026-11-12). With any acceleration limit set
(`linear.x.max_acceleration` etc.), `mecanum_drive_controller` 4.42.1 (the installed Jazzy
release) does not stop on a reference timeout: it ramps one step, then writes zero to the
front-left wheel only, and the other three keep their last command (four `set_value` calls
chained with `||`, which stops at the first success). A teleop key release, the safety gate's
single zero on a block and a stopped gate all end in that timeout, so the robot would keep
driving. The upstream jazzy branch has the fix; the installed release does not.

So `controllers_hardware.yaml` sets no acceleration limits, and every ramp, start and stop
alike, is the ESP32's `accel_backstop`. It is per wheel: in a combined twist the wheel with the
largest change takes longest, so the path bends slightly while it ramps (T5, step 4, measures
it). `test_esp32_fake.py` checks the timeout stop on every test run. Revisit when a newer
release is installed: add the limits, then the test must still pass.

## 9. Not verified (revisit only on failure)

The encoder tap's electrics: the 6N137 optocouplers, the level converter, their edge timing,
any loading of the drivers' encoder lines, noise (decision 0006, "Gaps"). The first suspects
when:
- counts drift from the emitted pulses while the driver shows no alarm;
- a wheel counts only one way, or at half or a quarter of the expected rate;
- the drivers alarm or behave differently with the tap connected;
- counts are lost only at the highest speeds (watch the following error climb with speed);
- counts jitter at standstill (the sketch's `ENC_GLITCH_NS`, firmware README section 3).

Also not verified here: the 3.3 V PUL/DIR levels into the CS-D808 inputs (missed pulses show
as a following error), and the strapping pins (T1, check 6).

## 10. Results log

| Date | Stage | Result |
|---|---|---|
| 2026-11-12 | T0 | Pass. `mecanum_hardware` tests and lint pass. `test_esp32_fake.py`, 5 of 5: odometry within 0.003 m/s and 0.01 rad/s of the twist (mean over a second); limits 0.1 m/s and 0.3 rad/s hold; the reference timeout zeroes all four wheels; a frozen front-left encoder latches at 326 counts (limit 318), everything stops, and the recovery drives again; a mute link stops everything within 0.1 s and the same recovery works. The controller issue of section 8 was found here: with acceleration limits, three wheels kept 0.73 rad/s after the timeout. `drive_tune.py` (speed steps, Ctrl-C/SIGTERM stop with the raised limits restored, `--stop timeout`, fault abort) and `drive_monitor.py identify` rehearsed on the fake. Per-sample wheel odometry at 0.1 m/s: σ 0.004 m/s, alternating 0.097 / 0.105, the count quantization (one count per 20 ms cycle is 0.008 m/s); the mean is exact. Stops at 0.1 m/s with the 2.5 rad/s² backstop: 0.024 m after a zero command, 0.044 m by the reference timeout. |
| 2026-11-13 | Upload, T1 checks 1–2 (desk) | Sketch compiled and uploaded by Claude at the owner's request: arduino-cli 1.5.1, core esp32 3.3.12, FastAccelStepper 2.0.1 (the step generator in use, `STEP_DRIVER_FASTACCEL 1`), board ESP32 Dev Module, Core Debug Level None, 921600 upload; 332 511 bytes (25%). Port `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (CH340). Passive read: 200 frames in 2 s (100 Hz). `drive_test.launch.py following_error_limit:=0`: "ESP32 drive firmware 0.1, protocol 1, defaults 1600 pulses / 4000 counts per rev", "ESP32 drive active", both controllers active, safety gate Clear; `/drive/state` faults 0, rx errors 0 both ways, counts and pulses 0. No motion commanded. T1 checks 3–6 not yet run. |
| 2026-11-13 | T2 checks 1 and 3 (on blocks, owner) | Identify: `motor_for_wheel:="3 0 2 1" dir_invert:="0 0 1 1" enc_invert:="0 0 1 1"`, re-run gave the expected answers. Ten turns at 0.1 m/s (0.976 rad/s): 15 943 pulses and 39 849–39 850 counts on all four wheels at release, 16 050 / 40 116–40 118 after the ramp. Counts ÷ pulses = 2.4995 (4000 ÷ 1600 = 2.5, within 0.02%): pulses per rev 1600 and counts per rev 4000 kept; 1594 pulses per "turn" is the tape-mark release error. Following error at speed and on the ramp: 7–9 counts behind on every wheel (since Enter), steady. Offsets built up before Enter in the same launch: FR −38, RL −18, RR +7, FL 0 counts, cause unknown (check 4 to find out whether they grow). Pulse speed 0.972 against 0.976 rad/s commanded (−0.4%, the same on all wheels). MCU rx errors 1. Following-error limit not chosen yet. |
| 2026-11-13 | T2 check 4 (on blocks, `tools/fe_soak.py`, full default sequence) | 324 s, 32 552 reports (100 Hz, largest gap 17.6 ms), no faults, host rx errors 0, MCU rx errors 1 (unchanged from before). Largest following error 10 counts (0.016 rad, FR, at a rotate stop); per phase max 5–10 counts, cruise lag 3–7 counts at 0.1 m/s / 0.3 rad/s, FR about 2.5 counts biased positive. No build-up: the error left after each of 123 stops stayed within −3…+5 counts and ended within 1 count of the start, across 60 axis cycles and 10 straight reversals, so the earlier −38 / −18 / +7 offsets did not come from driving. Limit by the rule: 2.5 × 10 counts is below the floor, so **0.05 rad (32 counts)** for T3; re-check against the lag at each higher speed in T5. |
| 2026-11-13 | Floor (owner) | RR held little torque at power-on and lagged in strafe, giving a yaw drift. Cause: the RR wheel was not mounted to the motor shaft properly (mechanical); fixed by the owner. The on-blocks soak had already shown RR's motor tracking its pulses, so the drive and the firmware were not involved. |
| 2026-11-14 | T2 checks 5–7 (on blocks, owner) | Pass, as reported by the owner. Check 5: a low following-error limit tripped the fault in normal running and every wheel stopped. Check 6: pulling the USB cable while driving stopped all wheels. Check 7: every teleop key turned the wheels as expected and releasing it stopped them. T2 complete. |
| 2026-11-14 | Floor recheck after the RR wheel was remounted (owner) | Pass, as reported by the owner: the strafe drive was repeated and the yaw drift is gone. |
