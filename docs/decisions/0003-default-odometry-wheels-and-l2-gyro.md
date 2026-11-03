# 0003: Default odometry: echoed wheel motion and the L2 gyro (decision D1)

- **Status:** accepted (2026-11-03). Decision D1 of [`plan.md`](../plan.md) Phase 3. Amends
  [0002](0002-ros2-control-with-measured-wheel-state.md): the motors report no encoder state,
  and the wheel state comes from echoed step pulses instead (see "The wheel state without
  encoders").
- **Date:** 2026-11-03
- **Revisit when:**
  - the floor tests on the real robot (see "Consequences") show large wheel slip on tiles;
  - the driver's following-error alarm cannot be set tight enough to bound the wheel error;
  - the MCU firmware or the hardware plugin (`mecanum_hardware`) is written, to check the echo
    rules below;
  - visual odometry or 2D scan matching is measured on hardware.

## Context

- **The Phase 3 candidates were measured in simulation** (2026-11-02, results in `plan.md`):

  | Candidate | Loop, 24.6 m | Long corridor, 40 m |
  |---|---|---|
  | E0 wheels | 0.098 m / 0.82° | 0.002 m / 0.00° |
  | E1 wheels + L2 gyro | 0.088 m / 0.20° | 0.042 m / 0.26° |
  | E2 E1 + D435i gyro | 0.087 m / 0.18° | 0.026 m / 0.13° |
  | E3a E1 + LIO twist | 0.079 m / 0.22° | 0.702 m / 0.25° |
  | E3b E1 + LIO pose | 0.074 m / 0.20° | 1.525 m / 0.25°: FAIL |
  | LIO alone | 0.119 m / 1.69° | 20.01 m: FAIL |

  - **E1 against E0.** E1 cuts the wheels' yaw error on the loop by three quarters.
  - **LIO.** It reports no motion along the long corridor. On the loop it helps by 10 to 16%,
    below the 25% margin required to add it.
  - **E2.** Its gain comes from averaging two identical stand-in noise models.
- **The drive has no encoder feedback.** It uses Leadshine CS-M234120 closed-loop stepper
  motors and CS-D808 drivers. Each driver takes pulse and direction and closes the position
  loop on its own motor encoder. Its only output is a fault (alarm) signal. The encoder counts
  never reach the MCU or ROS.
- **The other sensors have other jobs** (`plan.md`, sensor table):
  - **RPLIDAR C1.** The map and global localization scan (Phases 5 and 6, `map → odom`).
  - **D435i.** Obstacles ahead and docking; its gyro is E2.
  - **Ultrasonics.** Glass, mirrors and reversing.

## Decision

- **The default is E1.** The EKF (`config/ekf.yaml`) fuses the wheel odometry twist (vx, vy,
  wz) with the L2 gyro's yaw rate. It owns `/odom` and `odom → base_footprint`.
- **LIO is out of the EKF and off by default in simulation.** `robot.launch.py`'s `lio`
  defaults to the opposite of `sim`. With `sim:=true` it is off, and `lio:=true` still runs it
  on `/lio/odom` for evaluation. With `sim:=false` it stays on, because the hardware path has no
  wheel odometry or EKF yet and LIO is its only odometry. Turning it off in simulation saves
  about 11% of a core.
- **E2 is not the default.** The D435i gyro stays available as `ekf_e2.yaml`. Its benefit can
  only be judged with real, independent gyro noise.
- **Not used for local odometry:**
  - **2D scan matching on the C1 (rf2o).** Not measured. A single 2D slice with a 200° view is
    expected to fail the long corridor as LIO does. The same scan also feeds AMCL, which
    assumes its motion input is independent of the scans it matches. It stays the documented
    fallback.
  - **Visual odometry on the D435i.** The plan listed it for Phase 3, but it is deferred to
    hardware. Gazebo's plain textures make a simulated result meaningless. Washrooms (plain
    tiles, mirrors, glass) and the camera's 87°, 0.3 to 3 m view are a poor case for it, and it
    costs much CPU.
  - **Ultrasonics.** A single range per sensor, with no bearing, cannot give motion.

### The wheel state without encoders

The wheel odometry is computed from the **echoed command**: the step pulses the MCU actually
emits, after its ramps and limiters. This is treated as equivalent to a motor encoder, for the
following reasons.

- **Why the echo can be trusted.**
  - **The driver bounds the error.** The closed-loop driver makes the shaft follow every pulse
    it receives, and catches up lost steps. If the gap between commanded and actual position
    passes its following-error alarm threshold, it raises the fault.
  - **The error does not add up.** While no fault is present, the shaft position equals the
    received pulse count within that threshold, so the error stays bounded instead of growing.
  - **A stall is caught.** A held or stalled wheel raises the fault.
- **What a motor encoder would add is small.** A motor encoder also measures the shaft only. It
  cannot see roller slip, sideways slip when strafing, wet tiles or the wheel deforming under
  load either. Those are the main errors of mecanum odometry, so the echo loses little. What it
  loses is the error below the alarm threshold, mostly during acceleration, and that is
  bounded.
- **This amends the rule "wheel commands are never odometry"** of 0001 and 0002. A velocity
  *command* is still never odometry. What is echoed is the pulse train *after* the MCU's ramps
  and limiters, whose tracking the driver checks.
- **The gaps the driver does not cover:**
  - **MCU to driver.** The driver checks the pulses it receives, not the pulses sent. Noise, a
    direction change too close to a step, pulses too short, or a rate near the maximum make it
    miss or miscount pulses, and no fault follows. Mitigations: differential signalling,
    shielded cable, the datasheet's setup and pulse width timings with margin, and a pulse rate
    well below the maximum (a few kHz at 0.3 m/s).
  - **Wheel to floor.** Slip is invisible, as with any encoder.
  - **Motor to wheel.** Compliance or backlash in the coupling, or a gearbox if any.

The MCU and the hardware plugin follow these rules:

- **Echo the position.** The MCU counts the pulses it emits, per wheel, as a continuous 64-bit
  count. It reports the count with an MCU timestamp and the per-driver fault bits. The count is
  the source of truth, so a dropped report loses no distance.
- **Derive the velocity in the plugin.** `mecanum_drive_controller` integrates wheel velocity
  times its own update period. In `read()`, the plugin sets velocity to the count change since
  the previous `read()` divided by that period. The integration then reproduces the echoed
  count exactly, even when reports and controller cycles do not line up one to one.
- **A fault stops everything.** On any driver fault, the MCU stops all four wheels. The plugin
  returns an error, so the controller deactivates and wheel odometry stops. It does not keep
  publishing with a large covariance.
- **Set the alarm threshold tight.** The driver's following-error threshold, set in Leadshine's
  tuning software, is the measurement bound. It is recorded as a STAND-IN until set.

## Consequences

- **The wheel covariances stay STAND-INs, and are not reduced.** The echo is almost free of
  noise, but the EKF treats covariance as white noise, while the real error is slip, a bias
  roughly proportional to speed and largest sideways. A covariance that is too small makes the
  filter overconfident against any later input. The stand-ins (vx 0.001, vy 0.004) are about
  10% and 20% of the 0.3 m/s limit as a standard deviation, a little above a plausible slip
  budget of 2 to 5% forward and 5 to 15% strafing.
- **In E1 the wheel covariances correct nothing.** The wheels are E1's only source of vx and vy.
  The covariance sets only how much the filter smooths them, and how fast the reported pose
  covariance grows. It matters once a second translation source is added.
- **The gyro becomes the only independent motion check.** The echo cannot show that the robot
  failed to move as commanded. On hardware, a sustained mismatch between the wheels' yaw rate
  and the gyro flags rotational slip. Drift in position is corrected by the map localization
  (Phase 6).
- **The real L2 gyro covariance must be set.** Its driver publishes zero, which
  robot_localization takes as 1e-9. The static recording gives σ 0.003 to 0.005 rad/s.
- **Validation before trusting the echo on hardware:**
  - **Bench.** Run each wheel back and forth for hours, and check it returns to a mark on the
    shaft. This catches lost pulses between the MCU and the driver.
  - **Floor.** Drive measured straight lines, strafes and squares on real tiles. Fit the wheel
    radius and the centre-to-wheel distance from them, and set the covariances from what is
    left.
- **The simulation stays a fair stand-in for the shaft side.** Its ideal wheel sensing (shortcut
  S2) matches the echo, but not the slip.
- **The hardware path changes once the plugin exists.** The EKF with E1 then owns the TF on
  hardware too, and LIO's default there is reconsidered.
