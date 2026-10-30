# Stand-in values

Every value below is a placeholder for a hardware fact that is not known yet. Each one is
marked in its file with a `STAND-IN:` comment (`# STAND-IN:` in YAML/Python,
`<!-- STAND-IN: -->` in xacro). Replace the value, remove the comment, and delete the row
here once the real value is known.

## Active stand-ins

| File | Parameter | Placeholder | How to obtain the real value |
|---|---|---|---|
| `src/mecanum_base/config/esp32_driver.yaml`, `src/mecanum_controllers/config/controllers.yaml` | `max_vx`, `max_vy`, `max_wz`; the controller's `linear.*` / `angular.*` limits | `0.3` m/s, `0.3` m/s, `0.8` rad/s | Raise them step by step during hardware tests while stopping, tracking and LIO quality stay good. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.motor_id` | `0`, `1`, `2`, `3` for FL, FR, RL, RR | Take them from the rewritten firmware. Otherwise run `motion_test` on blocks and note which wheel each ID drives. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.sign` | `1` | Run `motion_test`. Flip the sign of any wheel that turns backward on the forward step. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.scale` | `0.0666667` value per rad/s, assuming value 1.0 equals 15 rad/s | On hardware, command a few values and read the wheel speed from the encoders. Needed only by the open-loop driver; ros2_control (plan Phase 1) closes the loop on the encoders. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.deadband` | `0.0` | On hardware, raise the command until the encoders report motion. Open-loop driver only, as `scale`. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.max_value` | `1.0`, a normalized 0 to 1 value | Take the value range and units from the rewritten firmware's protocol. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | `sim_wheel_mu` | `1.0` | Tune it until simulated slip matches the real robot on the target floor. |
| `src/mecanum_simulation/worlds/restroom.sdf` | ground plane `mu` | `50`, from Gazebo's mecanum demo | Measure the sideways sliding force on the real floor, or tune it with `sim_wheel_mu`. |
| `src/mecanum_simulation/worlds/restroom.sdf` | Whole layout, including the `toilet_1` shape and pose | Primitive corridor, lobby, restroom and stall. `toilet_1` bowl front edge at x = 6.34 m, y = 3.075 m. | Replace it with the real site's layout once known. A floor plan is enough for walls and fixtures. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 `gpu_lidar` samples | 360 horizontal × 32 vertical per sweep at 5.55 Hz, a regular grid | Compare with a real L2 cloud from the `mecanum_lidar` bench test: points per revolution and vertical spacing. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 range noise | Gaussian σ 0.01 m. The datasheet gives accuracy ≤ 2.0 cm. | Record a flat wall with the real L2 and measure the spread. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 IMU rate | 250 Hz | Read the IMU rate from the `mecanum_lidar` status line. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 IMU noise | σ 0.002 rad/s gyro, σ 0.02 m/s² accelerometer | Record the IMU at rest for a few minutes and take the standard deviations. |
| `src/mecanum_simulation/worlds/restroom.sdf` | Ceiling height | 2.4 m | Use the real site's ceiling height. |
| `src/mecanum_localization/config/cloud_to_scan.yaml` | `min_height`, `max_height` | 0.10 m, 0.50 m | Tune on the real site, for example for low fixtures and the toilet bowl height. |
| `src/mecanum_localization/config/rko_lio.yaml` | `min_range`, `max_range`, `voxel_size` | 0.3 m, 30 m, 0.25 m: indoor guesses | Re-tune in simulation for the upright L2 in plan Phase 3 (candidate E3); on hardware, in stage 4H of `docs/hardware_lio_test.md`. |
| `src/mecanum_localization/mecanum_localization/lio_odom_adapter.py` | `pose_covariance_diagonal`, `twist_covariance_diagonal` | x, y 0.0025 m²; yaw 0.0012 rad²; vx, vy, wz 0.0025 | Set from the drift measured in plan Phase 3 (sim) and stage 4H (hardware). They matter once the EKF uses LIO (candidate E3). |
| `src/mecanum_controllers/config/controllers.yaml` | `pose_covariance_diagonal`, `twist_covariance_diagonal` | x 0.001, y 0.002, yaw 0.002; vx 0.001, vy 0.004, wz 0.002 (vy larger for roller slip) | Set from the wheel odometry drift measured against ground truth in plan Phase 3, then from hardware. They matter once the EKF uses `/wheel/odometry`. |
| `src/mecanum_localization/mecanum_localization/gt_compare.py` | `max_drift_percent`, `max_yaw_error_deg` | 2% of the path, 2° | The plan Phase 3 targets. Replace them with what docking and navigation actually need. |

### Robot model (model v2)

The model follows the reference URDF `src/mecanum_description/refs/washroom_robot.urdf`. Its
geometry comes from the STEP model; the values below are the ones the STEP and the sensor
placement PDF do not settle.

| File | Parameter | Placeholder | How to obtain the real value |
|---|---|---|---|
| `src/mecanum_description/urdf/mecanum_params.xacro` | `chassis_mass`, chassis inertial origin and tensor | 25 kg at (−0.065, 0, 0.243) m in `base_link`, from the STEP solid at uniform density | Weigh the robot. Find the centre of mass by weighing each wheel (four scales, or one scale per corner in turn). In sim the 6.5 cm offset makes the base translate about 5.7 cm during a 86° turn (plan Phase 1). |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `wheel_mass` | 1.5 kg per wheel; the drive motors are in the chassis | Weigh one wheel with its hub off the robot. |
| `src/mecanum_description/urdf/mecanum_params.xacro` (and `wheel_radius` in `esp32_driver.yaml`, `wheels_radius` in `controllers.yaml`) | `wheel_radius` | 0.1025 m, the unloaded CAD radius | Measure the axle height with the robot loaded, or drive a measured 5 m straight line and divide by the encoder angle. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | roller friction directions (`fdir1`) | X layout: front-left and rear-right rollers along (1, −1) in the base frame, front-right and rear-left along (1, 1), matching `kinematics.py` | The CAD has no rollers. Look at the wheels from above: the rollers on the top of the four wheels form an X. A wrong layout makes the strafe step of `motion_test` go the wrong way. |
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_mount_z` | Bottom face 0.767 m above the floor: the STEP box top plus the PDF's 30 mm standoff. The STEP sinks the L2 40 mm into the box. | Measure the height of the mounted L2's bottom face. |
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_cable_yaw` | 0: vendor +X forward, cable outlet to the rear | Read the cable outlet direction on the robot; vendor +X points away from it. The yaw is counter-clockwise from above. |
| `src/mecanum_lidar/config/l2.yaml` | `work_mode` (wide FOV, bit 0) | Left at `-1` (keep the device setting); the upright mount needs bit 0 = 1. The simulation uses wide mode (`sim_l2_wide_fov` true). | Set `work_mode: 1` when the L2 is mounted, after checking the other bits on the unit (see `docs/hardware_l2_test.md` step 1.1). |
| `src/mecanum_description/urdf/sensors/realsense_d435i.xacro` | `d435i_pitch` | 8° down, from the PDF | Measure the bracket angle, or fit the floor plane in the D435i depth cloud. |
| `src/mecanum_description/urdf/sensors/lidar_2d.xacro` | `lidar_2d_z`, scan 0° direction | Scan plane 0.130 m above the floor (PDF); 0° along `lidar_2d_link` +x, forward | Measure the C1's optical centre height. Check its 0° mark on the unit, and look at `/scan` against a known wall. |
| `src/mecanum_description/urdf/sensors/cliff_tof.xacro` | `cliff_tof_z` | 0.119 m above the floor (STEP); the PDF says 0.08 m | Measure the mounted sensor height. Phase 4's cliff threshold depends on it. |
| `src/mecanum_description/urdf/sensors/bumpers.xacro` | `bumper_x` | Plank faces at ±0.404 m, as in the STEP. The 2D lidar (0.420 m) and the front ultrasonics (0.408 m) stick out further, so the plank cannot hit first; the PDF wants the planks 3 cm proud. | Plan Phase 4, decision D2. Then measure the real planks. |
| `src/mecanum_localization/config/self_filter.yaml` | crop box | x −0.43 to 0.45 m, y ±0.29 m, z −0.05 to 0.86 m: the robot extent plus about 2.5 cm | Check a real L2 cloud for points on the robot (`tools/l2_mount_check.py`, "points on the robot"). |
