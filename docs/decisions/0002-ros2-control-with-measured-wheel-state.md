# 0002: ros2_control with measured wheel state

- **Status:** accepted (2026-10-28). Supersedes
  [0001](0001-open-loop-drive-without-ros2-control.md) in simulation; on hardware, 0001's
  open-loop driver stays until the ESP32 hardware plugin exists.
- **Date:** 2026-10-28
- **Revisit when:** the EKF takes over `odom -> base_footprint` (plan Phase 3), or the ESP32
  firmware's state reporting is defined ([encoder_upgrade.md](../encoder_upgrade.md),
  sections 0 and 2).

## Context

- **The motors have encoders now.** The model v2 motors report wheel position and velocity, so
  0001's main reason, no feedback, no longer holds.
- **The standard path exists.** The Jazzy `mecanum_drive_controller` (4.42.1) needs velocity
  state interfaces on all four wheels and publishes odometry from them, and `gz_ros2_control`
  (1.2.20) provides those states from Gazebo physics. Wheel odometry is then computed from
  *measured* states, not commands, which keeps the rule "wheel commands are never odometry".
- **The previous sim LIO results were poor** (18.5% loop error on the old model), so a second
  odometry source was needed anyway.

## Decision

- **Package.** `mecanum_controllers` holds the ros2_control xacro (`hardware` mock or sim), the
  controller configuration and the controller launch. `mecanum_description` stays free of
  ros2_control.
- **Simulation.** The gz_ros2_control plugin with `GazeboSimSystem` replaces 0001's simulated
  ESP32 (`sim_motor_plant`) and the JointController plugins, which are deleted. The anisotropic
  roller friction stays.
- **Command contract.** Every velocity source publishes `geometry_msgs/TwistStamped` on
  `/cmd_vel`. The drive controller's reference is remapped to `/cmd_vel`. In Phase 4 the safety
  gate subscribes to `/cmd_vel`, and the controller's reference moves to the gate's output; no
  source changes.
- **Limits.** The controller applies the speed limits (0.3 m/s, 0.3 m/s, 0.8 rad/s, as the
  driver's stand-ins) and the 0.25 s reference timeout.
- **Odometry and TF.** The controller publishes `/wheel/odometry` and, until the EKF in Phase
  3, owns `odom -> base_footprint` (`enable_odom_tf: true`). LIO publishes `/lio/odom` without
  TF in simulation, as an evaluation and future EKF input.
- **Joint states.** `joint_state_broadcaster` publishes `/joint_states`; joint_state_publisher
  is gone from the simulation.
- **Hardware.** Unchanged for now: `robot.launch.py sim:=false` runs 0001's open-loop driver,
  and LIO owns the TF there. The ESP32 hardware plugin (`mecanum_hardware`) follows the
  firmware.

## Consequences

- **Standard tooling.** The controller manager, mock hardware and the joint state
  broadcaster are used; `controllers.launch.py hardware:=mock` and a launch test check the
  kinematics against `mecanum_base/kinematics.py` without Gazebo.
- **Wheel slip is visible.** Slip now shows up as wheel odometry error against ground truth
  instead of being hidden. The sim encoders are ideal (shortcut S2 in `plan.md`).
- **Sim and hardware differ until the plugin exists.** The sim has wheel odometry and
  measured joint states; the hardware path does not.
- **Stamped commands.** Nav2 (Phase 6) must set `enable_stamped_cmd_vel: true`; the open-loop
  driver on hardware still takes `Twist` by default.
