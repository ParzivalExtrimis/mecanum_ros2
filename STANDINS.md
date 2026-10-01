# Stand-in values

Every value below is a placeholder for a hardware fact that is not known yet. Each one is
marked in its file with a `STAND-IN:` comment (`# STAND-IN:` in YAML/Python,
`<!-- STAND-IN: -->` in xacro). Replace the value, remove the comment, and delete the row
here once the real value is known.

## Active stand-ins

| File | Parameter | Placeholder | How to obtain the real value |
|---|---|---|---|
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_x`, `l2_y`, `l2_z` | `0.0`, `0.0`, `0.14` m (upright, centred, on a 0.10 m spacer above the 0.040 m chassis top) | After mounting, measure the centre of the L2's bottom mounting face relative to `base_link` (chassis centre, wheel-axle height, 0.05 m above the floor). Or read it from the bracket CAD. |
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_roll`, `l2_pitch` | `0.0`, `0.0` (upright, level) | Read from the bracket CAD, or measure the tilt of the mounting face with a digital level. An inverted mount is `l2_roll:=3.14159`. |
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_yaw` | `0.0` (cable outlet to the rear) | The L2's +X axis points away from the cable outlet. Measure the angle between that direction and the robot's forward axis. |
| `src/mecanum_description/urdf/sensors/unitree_l2.xacro` | `l2_spacer_length` | `0.10` m | Measure the real spacer or bracket between the mounting face and the structure it is fixed to. Use `0` if the L2 sits directly on the plate. |
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

## Existing description values of uncertain accuracy

These values are in `mecanum_description`, which is extend-only, so they are **not**
annotated in the files. Per the owner, they combine CAD values and measured approximations.
Kinematics, footprint and simulation all derive from them.

| File | Parameter | Current value | How to obtain the real value |
|---|---|---|---|
| `src/mecanum_description/urdf/mecanum_params.xacro` | `wheel_radius` | `0.05` m | Roll the robot through ten wheel revolutions on the floor and divide the distance by 20π. Measure under load, because rollers compress. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `front_axle_x`, `rear_axle_x` | `0.216`, `-0.235` m | Measure the axle centres relative to the chassis centre. Their difference is the wheelbase, and half of it is `lx` in the kinematics. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `track_width` | `0.414` m | Measure between the left and right wheel mid-planes. Half of it is `ly` in the kinematics. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `chassis_length`, `chassis_width`, `chassis_height` | `0.605`, `0.394`, `0.064` m | Measure the base plate. These values set the footprint and the self-filter box. |
| `src/mecanum_description/urdf/mecanum_params.xacro` | `chassis_mass`, `wheel_mass` | `5.0`, `0.3` kg | Weigh the assembled base without wheels, then weigh one wheel with its motor mount. These values only affect simulation dynamics. |
