# Stand-in values

Every value below is a placeholder for a hardware fact that is not known yet. Each one is
marked in its file with a `STAND-IN:` comment (`# STAND-IN:` in YAML/Python,
`<!-- STAND-IN: -->` in xacro). Replace the value, remove the comment, and delete the row
here once the real value is known.

## Active stand-ins

| File | Parameter | Placeholder | How to obtain the real value |
|---|---|---|---|
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_mast_mass` | `0.9` kg for the pole plus bracket, assuming a 2 mm wall aluminium tube | Weigh the pole and bracket assembly without the L2. It only affects simulation dynamics. |
| `src/mecanum_base/config/esp32_driver.yaml` | `max_vx`, `max_vy`, `max_wz` | `0.3` m/s, `0.3` m/s, `0.8` rad/s | Raise them step by step during hardware tests while stopping, tracking and LIO quality stay good. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.motor_id` | `0`, `1`, `2`, `3` for FL, FR, RL, RR | Take them from the rewritten firmware. Otherwise run `motion_test` on blocks and note which wheel each ID drives. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.sign` | `1` | Run `motion_test`. Flip the sign of any wheel that turns backward on the forward step. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.scale` | `0.0666667` value per rad/s, assuming value 1.0 equals 15 rad/s | Run the calibration tool in phase 5 with LIO running. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.deadband` | `0.0` | Run the calibration tool. It finds the smallest value that makes the wheel turn. |
| `src/mecanum_base/config/wheel_calibration.yaml` | `wheels.<wheel>.max_value` | `1.0`, a normalized 0 to 1 value | Take the value range and units from the rewritten firmware's protocol. |
| `src/mecanum_interfaces/msg/MotorCommands.msg` | `value` units, `motor_id` mapping | Normalized 0 to 1. IDs 0 to 3. | Same as `max_value` and `motor_id` above. The real ESP32 protocol replaces this message on hardware. |
| `src/mecanum_simulation/config/sim_plant.yaml` | `plant.<wheel>.max_omega` | `15.0` rad/s, about 143 rpm | Use the motor and gearbox no-load speed from the datasheet, or measure the wheel speed at full command with the wheels off the ground. |
| `src/mecanum_simulation/config/sim_plant.yaml` | `watchdog_timeout` | `0.0`, meaning off | Check whether the rewritten firmware stops the motors when commands stop, and after how long. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | `sim_wheel_mu` | `1.0` | Tune it until simulated slip matches the real robot on the target floor. |
| `src/mecanum_simulation/worlds/restroom.sdf` | ground plane `mu` | `50`, from Gazebo's mecanum demo | Measure the sideways sliding force on the real floor, or tune it with `sim_wheel_mu`. |
| `src/mecanum_simulation/worlds/restroom.sdf` | Whole layout, including the `toilet_1` shape and pose | Primitive corridor, lobby, restroom and stall. `toilet_1` bowl front edge at x = 6.34 m, y = 3.075 m. | Replace it with the real site's layout once known. A floor plan is enough for walls and fixtures. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 `gpu_lidar` samples | 360 horizontal × 32 vertical per sweep at 5.55 Hz, a regular grid | Compare with a real L2 cloud from the `mecanum_lidar` bench test: points per revolution and vertical spacing. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 range noise | Gaussian σ 0.01 m. The datasheet gives accuracy ≤ 2.0 cm. | Record a flat wall with the real L2 and measure the spread. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 IMU rate | 250 Hz | Read the IMU rate from the `mecanum_lidar` status line. |
| `src/mecanum_simulation/urdf/mecanum.gazebo.xacro` | L2 IMU noise | σ 0.002 rad/s gyro, σ 0.02 m/s² accelerometer | Record the IMU at rest for a few minutes and take the standard deviations. |
| `src/mecanum_simulation/worlds/restroom.sdf` | Ceiling height | 2.4 m | Use the real site's ceiling height. |
| `src/mecanum_localization/config/self_filter.yaml` | Self-filter box margins | x ±0.33 m, y ±0.27 m: 2.75 cm and 2.3 cm beyond the chassis and wheels | Record a cloud on the real robot and check for self-hits outside the box, or for nearby obstacles wrongly removed. |
| `src/mecanum_localization/config/cloud_to_scan.yaml` | `min_height`, `max_height` | 0.10 m, 0.50 m | Tune on the real site, for example for low fixtures and the toilet bowl height. |
| `src/mecanum_localization/config/rko_lio.yaml` | `min_range`, `max_range`, `voxel_size` | 0.3 m, 30 m, 0.25 m: indoor guesses | Tune in phase 4H on the real robot: loop drift, corridor and static drift. |
| `src/mecanum_localization/mecanum_localization/lio_odom_adapter.py` | `pose_covariance_diagonal`, `twist_covariance_diagonal` | x, y 0.0025 m²; yaw 0.0012 rad²; vx, vy, wz 0.0025 | Set from the drift measured in phase 4H. They matter once an EKF or AMCL uses them. |
| `src/mecanum_localization/mecanum_localization/gt_compare.py` | `max_drift_percent`, `max_yaw_error_deg` | 1% of the path, 2° | The phase 4 targets from `docs/plan.md`. Replace them with what docking and navigation actually need. |

## Provisional design choices

These values are decided, not unknown, but they may still change. Each is set in one place.

| Choice | Current value | Set by | If it changes |
|---|---|---|---|
| L2 cable outlet direction on the bracket | 22.9° clockwise seen from above, from pointing straight at the pole, so toward the rear-left. Measured on the actual mount. The L2's +X points forward-right. | xacro arg `l2_bracket_yaw` in `src/mecanum_description/urdf/sensors/unitree_l2.xacro`, default `0.3996804` rad. It is a rotation about the L2's own tilted axis; seen from above it appears as 26.0°. | Set the new default. `0.0` means the cable points straight at the pole, and `3.14159` toward the overhang tip; positive is clockwise from above. Re-mount the real L2 to match. Check any LIO config that hard-codes a lidar or IMU extrinsic; extrinsics should come from TF. Frames, the simulated sensor, the self-filter and the L2 driver follow automatically. |
| Mast position | Pole at the rear edge, centred, L2 looking forward and down | xacro args `l2_mast_x`, `l2_mast_y`, `l2_mast_z`, `l2_mast_yaw` | Pole at the front is `l2_mast_x:=0.2875 l2_mast_yaw:=3.14159`. Re-check the self-filter and the footprint. |

## Existing description values of uncertain accuracy

These values are in `mecanum_description/urdf/mecanum_params.xacro`, where each carries a
`STAND-IN` comment. Per the owner, they combine CAD values and measured approximations.
Kinematics, footprint and simulation all derive from them.

| File | Parameter | Current value | How to obtain the real value |
|---|---|---|---|
| `src/mecanum_description/urdf/mecanum_params.xacro` | `wheel_radius` | `0.05` m | Roll the robot through ten wheel revolutions on the floor and divide the distance by 20π. Measure under load, because rollers compress. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `front_axle_x`, `rear_axle_x` | `0.216`, `-0.235` m | Measure the axle centres relative to the chassis centre. Their difference is the wheelbase, and half of it is `lx` in the kinematics. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `track_width` | `0.414` m | Measure between the left and right wheel mid-planes. Half of it is `ly` in the kinematics. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `chassis_length`, `chassis_width`, `chassis_height` | `0.605`, `0.394`, `0.064` m | Measure the base plate. These values set the footprint and the self-filter box. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `chassis_mass`, `wheel_mass` | `5.0`, `0.3` kg | Weigh the assembled base without wheels, then weigh one wheel with its motor mount. These values only affect simulation dynamics. |
