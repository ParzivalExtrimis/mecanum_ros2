# 0001: Open-loop drive through a Python driver node, without ros2_control

- **Status:** accepted
- **Date:** 2026-10-01
- **Revisit when:** the ESP32 firmware reports measured wheel position or velocity. See
  [encoder_upgrade.md](../encoder_upgrade.md).

## Context

- **No feedback.** The base's four mecanum motors have no encoders, so there is no wheel
  feedback of any kind. The ESP32 accepts per-motor direction and velocity commands only.
- **LIO is the only odometry source.** Odometry comes from LiDAR-inertial odometry on the
  Unitree L2. Wheel commands must never be published as odometry.
- **Option A:** a driver node subscribes to `cmd_vel`, runs mecanum inverse kinematics and
  per-wheel calibration, and sends commands through a pluggable transport.
- **Option B:** ros2_control. That is `mecanum_drive_controller` plus a hardware plugin,
  with `gz_ros2_control` in simulation.

Option B was rejected for now. The Jazzy `mecanum_drive_controller` (4.42.1):

- **Requires wheel feedback.** It needs velocity *state* interfaces on all four wheels.
- **Always publishes odometry from those states.** Only its TF broadcast can be disabled.
- **Takes TwistStamped on its reference topic.** Jazzy Nav2 publishes Twist by default.

Without encoders, a hardware plugin's `read()` could only copy the last command back
into the state interfaces. `/joint_states` and the controller's odometry would then be
command echoes, which breaks the rule above. In simulation, Gazebo would report real wheel
states while hardware reported echoes. The sim would then look better than the robot.

## Decision

- **Driver.** `mecanum_base/esp32_driver_node` (Python) owns the drive path. It clamps the
  twist, runs inverse kinematics, scales all wheels proportionally if any would saturate, and
  applies the per-wheel calibration (sign, scale, deadband, max). It then sends one command per
  motor at a fixed rate. It sends zeros on timeout, exception and shutdown.
- **Transports.** `mock` logs commands, `sim` publishes the simulated ESP32 wire message, and
  `esp32` is the real link. The real link is implemented once the firmware protocol is fixed.
- **Simulation.** A simulated ESP32 and motors node turns motor commands into wheel joint
  velocities. Gazebo drives the wheel joints physically, using anisotropic friction for the
  rollers. **Wheel joint states from Gazebo are never used for odometry.**
- **Odometry.** It comes from LIO only, through the LIO adapter, which owns
  `odom -> base_footprint`.
- **Wheel joints.** joint_state_publisher publishes zero wheel angles for robot_state_publisher,
  identically in sim and on hardware.

## Consequences

- **Sim matches hardware.** The sim is exactly as blind as the hardware, so results transfer.
- **Python stays.** The kinematics and calibration live in a pure-Python module with unit
  tests.
- **Non-standard interface.** There is no ros2_control. ros2_control tooling, such as the
  controller manager, mock hardware and the joint state broadcaster, is not used.
- **Open-loop wheel speed.** Real wheel speed varies with battery voltage, load and floor.
  Nav2 corrects the resulting error, because it closes the loop through LIO.
- **Migration cost later.** Moving to ros2_control later needs a C++ hardware plugin and a
  switch to stamped velocity commands. The steps are in
  [encoder_upgrade.md](../encoder_upgrade.md).
