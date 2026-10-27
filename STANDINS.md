# Stand-in values

Every value below is a placeholder for a hardware fact that is not known yet. Each one is
marked in its file with a `STAND-IN:` comment (`# STAND-IN:` in YAML/Python,
`<!-- STAND-IN: -->` in xacro). Replace the value, remove the comment, and delete the row
here once the real value is known.

## Active stand-ins

| File | Parameter | Placeholder | How to obtain the real value |
|---|---|---|---|
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
| `src/mecanum_localization/config/cloud_to_scan.yaml` | `min_height`, `max_height` | 0.10 m, 0.50 m | Tune on the real site, for example for low fixtures and the toilet bowl height. |
| `src/mecanum_localization/config/rko_lio.yaml` | `min_range`, `max_range`, `voxel_size` | 0.3 m, 30 m, 0.25 m: indoor guesses | Tune in phase 4H on the real robot: loop drift, corridor and static drift. |
| `src/mecanum_localization/mecanum_localization/lio_odom_adapter.py` | `pose_covariance_diagonal`, `twist_covariance_diagonal` | x, y 0.0025 m²; yaw 0.0012 rad²; vx, vy, wz 0.0025 | Set from the drift measured in phase 4H. They matter once an EKF or AMCL uses them. |
| `src/mecanum_localization/mecanum_localization/gt_compare.py` | `max_drift_percent`, `max_yaw_error_deg` | 1% of the path, 2° | The phase 4 targets from `docs/plan.md`. Replace them with what docking and navigation actually need. |

## Removed with the previous robot model

On branch `feature/model-v2` the robot model is being replaced. The previous model's stand-ins
and provisional design choices were removed with its geometry: the description dimensions and
masses, the L2 mast mass, the mast position, the L2 cable direction and the self-filter box
margins. They are listed, with how each was to be measured, in [`REWRITE.md`](REWRITE.md). Add
new rows here for the new model's uncertain values when its URDF is written.
