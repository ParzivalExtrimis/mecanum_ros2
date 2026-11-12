# 0006: Wheel state from the tapped motor encoders

- **Status:** accepted (2026-11-11). Amends
  [0003](0003-default-odometry-wheels-and-l2-gyro.md): its section "The wheel state without
  encoders" (the echoed step pulses) is replaced by the rules below. 0003's choice of E1 stands.
  Also amends [0002](0002-ros2-control-with-measured-wheel-state.md): the wheel state is
  measured again, as 0002 first assumed.
- **Date:** 2026-11-11
- **Revisit when:**
  - the ESP32 firmware or the hardware plugin (`mecanum_hardware/Esp32System`) is written;
  - the block tests of stage 5H (`docs/hardware_transition.md`) show counts that disagree with
    the emitted pulses, or the counts per wheel revolution differ from 4000;
  - the floor tests show slip large enough to need a second translation source.

## Context

- **The motors' encoders can now be read.** 0003 was written when the Leadshine CS-D808
  drivers closed the position loop on the CS-M234120 motor encoders and gave the MCU only a
  fault signal. The owner has since built an **encoder tap**: each motor's A and B encoder
  outputs at its driver go through a 6N137 optocoupler (8 in all), then a 5 V to 3.3 V logic
  level converter, into the ESP32, which reads them as square waves. The drivers keep their own
  closed loop.
- **The tap's electrics are not verified, and are out of this project's scope.** The owner
  built and checks them outside it. No electrical test is planned here; see "Gaps" for what
  points back to them if counts go wrong.
- **0003's echo was a stand-in for exactly this.** It counted the pulses the MCU emits and
  trusted the driver to follow them. It could not see pulses lost between the MCU and the
  driver, and it relied on the driver's alarm for a stalled wheel.

## Decision

The wheel state is the **measured quadrature count** of each motor, read through the tap.

- **The MCU counts.** It decodes A and B of each wheel in hardware counters (the ESP32's pulse
  counter units, x4 decoding, glitch filter on) and extends them to continuous 64-bit counts.
  It reports the four counts with an MCU timestamp and the per-driver fault bits, at 50 Hz or
  more. The count is the source of truth: a dropped report loses no distance.
- **The plugin derives the velocity** (kept from 0003). In `read()`, the velocity is the count
  change since the previous `read()` divided by that period. `mecanum_drive_controller`
  integrates velocity times its own period, so its odometry reproduces the count exactly, even
  when reports and controller cycles do not line up one to one.
- **The echo becomes a cross-check.** The MCU still counts the step pulses it emits. The
  difference between emitted pulses (scaled to encoder counts) and measured counts is the
  **following error**, measured by the stack itself, independent of the driver's alarm.
- **A fault stops everything** (kept from 0003). A driver fault, or a following error over the
  limit on any wheel, makes the MCU stop all four wheels. The plugin returns an error, the
  controllers deactivate, `/joint_states` and wheel odometry stop, and the safety gate's
  `joint_states` watchdog blocks every motion.
- **0002's rule holds again without an exception:** wheel commands, echoed or not, are never
  odometry.
- **The model is one piece of code for both worlds.** `mecanum_hardware`'s `WheelEncoder` holds
  the count-to-state rules and the following-error check. The simulation plugin
  (`SimEncoderSystem`) feeds it counts made from Gazebo's exact wheel angle; the ESP32 plugin
  will feed it the MCU's counts.

### In simulation

`mecanum_hardware/SimEncoderSystem` replaces `gz_ros2_control/GazeboSimSystem` as the sim
hardware (it wraps it). Its parameters, in `mecanum_controllers/urdf/mecanum.ros2_control.xacro`,
are STAND-INs:

| Parameter | Value | Why |
|---|---|---|
| `counts_per_wheel_rev` | 4000 | A 1000-line encoder, x4 decoding, direct drive (no gearbox is known) |
| `report_delay_cycles` | 1 (20 ms) | The MCU report and the link |
| `following_error_limit` | see `STANDINS.md` | Large enough for the simulated tracking; set on hardware from the block tests |

Its `/drive/inject_wheel_fault` topic freezes one wheel's count, as a dead encoder channel would,
to test the stop path. It publishes the following errors on `/drive/following_error`.

## Consequences

- **What the encoders add over the echo.** A held or stalled wheel and pulses lost between the
  MCU and the driver now show up as a following error, in the stack, without trusting the
  driver's alarm. The driver's following-error alarm stays as a second, independent bound.
- **What they still cannot see.** Like any shaft encoder, they are blind to roller slip,
  sideways slip when strafing, wet tiles and wheel deformation. Those stay the main errors of
  mecanum odometry, so:
  - the wheel covariances stay STAND-INs and are not reduced (0003's reasoning holds);
  - the L2 gyro stays the independent check on rotation, and the map localization corrects
    position drift.
- **The simulation is closer to the robot.** Shortcut S2 of `plan.md` shrinks: quantization, the
  count-difference velocity and a report delay are modelled. Still ideal: no electrical
  glitches on the tap, no slip beyond the friction model.
- **Gaps that stay open:**
  - **The tap's electrics (not verified, out of scope).** Optocoupler and level converter edge
    timing, loading of the drivers' encoder lines, noise. First suspect if counts drift from the
    emitted pulses while no driver alarm shows, a wheel counts only one way or at half rate, the
    drivers alarm or behave differently with the tap connected, or counts are lost at the
    highest speeds.
  - **Motor to wheel.** Compliance or backlash in the coupling: the encoders sit on the motor.
- **Hardware transition.** The plan is in `docs/hardware_transition.md`, stage 5H: firmware,
  `mecanum_hardware/Esp32System`, block tests, floor tests, then the EKF owns the TF on hardware
  and the open-loop `esp32_driver` retires.
