# Robot model rewrite (branch `feature/model-v2`)

The robot is getting a new model. It has new visual and collision meshes, additional geometry
meshes and several additional sensors, and none of the previous meshes or dimensions carry over.
This branch removed the previous model's meshes and numbers from the URDF and from everything
that copied them. The URDF is now a **frame skeleton**: the link and joint names and the tree
are kept, but there are no meshes, poses, sizes or masses, apart from the Unitree L2's vendor
values. The L2 stays on the new robot, on a new mount.

This file lists what was removed, what was kept but must be re-checked, and where the new
sensors plug in. Every spot in the code is marked with a `REWRITE:` comment
(`<!-- REWRITE: -->` in xacro, `# REWRITE:` in YAML and Python, a `REWRITE` note in Markdown):

```bash
grep -rn "REWRITE" src tools docs README.md STANDINS.md
```

Remove each marker and its row here once it is done, then delete this file. The full previous
model is in commit `64523c8` on `feature/sim-drive-stack`, for example
`git show 64523c8:src/mecanum_description/urdf/mecanum_params.xacro`.

**Until the rewrite is done, nothing drives.** The xacro processes and RViz shows the frames,
all at the origin. The drive node refuses to start (no geometry), the Gazebo model has no
wheels to drive on, and the self-filter uses pcl_ros's default box.

## 1. Removed from `mecanum_description`

The previous values are given for reference only. Do not reuse them for the new model.

| Item | Previous value | Was used by | The new model must provide |
|---|---|---|---|
| Visual meshes (`meshes/visual/`) | `macchumwheel_body_001_nowheel.stl` (chassis), `macchumwheelFL.stl` and `macchumwheelbase_FR.stl` (left- and right-hand wheels, X layout), `810mmL2stand.stl` (pole), `L2lidar30degree.stl` (bracket with the L2 block) | `base_link`, wheel and `l2_mast` visuals; the pole and bracket were also the `l2_mast` collisions | New visual meshes, collision meshes in `meshes/collision/`, the additional geometry meshes. Metres, origin at the link frame. |
| Reference photo `refs/mecanum_ros2_lidar_placement.jpeg` | The earlier 725 mm, 20° mast | Documentation only | New reference photos, if useful |
| `chassis_length`, `chassis_width`, `chassis_height`, `chassis_z_offset` | 0.605, 0.394, 0.064, 0.008 m | `base_link` collision box and inertia; the self-filter box; the planned Nav2 footprint (±0.3025 × ±0.247 m) | Chassis collision and extent, including every mount and sensor |
| `chassis_mass`, `wheel_mass` | 5.0 kg, 0.3 kg | Inertias; simulation dynamics only | Masses and inertias. To measure: weigh the base without wheels, then one wheel with its motor mount. |
| `wheel_radius`, `wheel_width` | 0.05 m, 0.08 m | `base_joint` height, wheel collision cylinder and inertia, the driver's `wheel_radius`, `test_urdf_defaults.py` | Wheel radius and collision. To measure: roll ten wheel revolutions on the floor and divide the distance by 20π, under load. |
| `front_axle_x`, `rear_axle_x`, `track_width` | 0.216, -0.235, 0.414 m (wheelbase 0.451 m) | Wheel joint origins; driver `lx` = 0.2255 and `ly` = 0.207 | Wheel centres. To measure: axle centres from the chassis centre, and the distance between the wheel mid-planes. |
| `mesh_dir`, `chassis_mesh`, `wheel_mesh_left_hand`, `wheel_mesh_right_hand` | `package://mecanum_description/meshes/visual/...` | The visuals above | The new mesh paths |
| `l2_mast` link and joint; xacro args `l2_mast_x`, `l2_mast_y`, `l2_mast_z`, `l2_mast_yaw` | -0.2875, 0, 0.040 m, 0 rad: pole flush with the rear plate edge, centred | Mast pose; the L2 pose followed from it | The new mount's links, meshes and pose |
| `l2_face_x`, `l2_face_y`, `l2_face_z`, `l2_bracket_tilt` | 0.06592, 0.0075, 0.83999 m in `l2_mast`; 30° | `l2_mount` pose: L2 face 0.930 m above the floor, inverted, tilted 30° forward | The `base_link → l2_mount` origin (in `unitree_l2.xacro`) |
| Xacro arg `l2_bracket_yaw` | 0.3996804 rad = 22.9°, cable toward the rear-left | `l2_mount → l2_lidar` rotation and the rotated IMU offset | The `l2_mount → l2_lidar` rotation about z for the new cable direction (vendor +X points away from the cable) |
| `l2_pole_x/y/height`, `l2_mast_mass` | 0.030 × 0.070 × 0.810 m, 0.9 kg (STAND-IN) | Mast inertia | The mount's inertia |

**Tree change.** `l2_imu` is now a child of `l2_lidar`, at the vendor offset (-0.007698,
-0.014655, 0.00667) m. It used to be a child of `l2_mount`, with the offset rotated by
`l2_bracket_yaw`. That was the same pose; the new form does not depend on the mount.

**Kept in the skeleton:**
- the link and joint names (section 4);
- the `prefix` and `use_l2` xacro args;
- the L2's vendor values: body 75 × 75 × 65 mm, 0.230 kg, the IMU offset;
- `materials.xacro` and `inertial_macros.xacro`;
- the display launch file.

## 2. Removed from configs and code

| File | What | Previous value | Now |
|---|---|---|---|
| `src/mecanum_base/config/esp32_driver.yaml` | `wheel_radius`, `lx`, `ly` | 0.05, 0.2255, 0.207 m | Commented out. Set them from the new URDF: `lx` = (front_axle_x − rear_axle_x) / 2, `ly` = track / 2. |
| `src/mecanum_base/mecanum_base/esp32_driver_node.py` | `DEFAULT_WHEEL_RADIUS`, `DEFAULT_LX`, `DEFAULT_LY` | The same values | Removed. The three parameters are required, and the node raises an error naming any that are missing, so it can never drive on old geometry. |
| `src/mecanum_localization/config/self_filter.yaml` | `min_x`, `max_x`, `min_y`, `max_y`, `min_z`, `max_z` | -0.33, 0.33, -0.27, 0.27, -0.05, 1.00 m: the chassis and wheels plus 2.75 cm and 2.3 cm STAND-IN margins, up to the 0.954 m bracket top | Commented out. Until they are set, pcl_ros uses its default ±1 m cube. Set them from the new URDF, then check for self-hits in a cloud recorded on the robot. |

## 3. Tests disabled

`src/mecanum_base/test/test_urdf_defaults.py` is skipped at module level, and
`test_code_defaults_match_urdf` was deleted, because there are no code defaults any more.

Re-enable the test by removing the skip once the URDF has wheel joint origins and a wheel
collision, and `esp32_driver.yaml` has the new values. The test assumes:
- the wheel radius is the `radius` of the cylinder collision on `front_left_wheel_link`.
  Change the lookup if the new collision is a mesh or a sphere;
- the `*_wheel_joint` joints are direct children of `base_link`;
- the left and right wheels are symmetric.

## 4. Kept, re-check after the rewrite

| What | Where | Check |
|---|---|---|
| Frame and joint names: `base_footprint`, `base_link`, `{front,rear}_{left,right}_wheel_link` / `_wheel_joint`, `l2_mount`, `l2_lidar`, `l2_imu` | `mecanum_base` `WHEEL_NAMES`, `mecanum.gazebo.xacro`, `gz_bridge.yaml` topics, `l2.yaml` and the L2 driver defaults, `rko_lio*.yaml`, `self_filter.yaml`, `cloud_to_scan.yaml`, `lio_odom_adapter.py`, `motion_test.py`, the RViz files, `docs/encoder_upgrade.md` | Keep them, or rename them everywhere at once |
| X-roller, 4-wheel kinematics | `mecanum_base/kinematics.py` | The new wheels are in an X layout: FL and RR share one roller handedness, FR and RL the other |
| Wheel friction directions `fdir` and `sim_wheel_mu` (STAND-IN) | `mecanum_simulation/urdf/mecanum.gazebo.xacro` | `fdir` matches the new rollers |
| One unnamed collision per wheel link | `mecanum_wheel.xacro`, `mecanum.gazebo.xacro` | Needed for the friction block to merge. If it does not merge, the wheels silently lose their mecanum behaviour. |
| Spawn height `-z 0.01` | `mecanum_simulation/launch/sim.launch.py` | `base_footprint` is at the wheel contact |
| Model name `mecanum` | `sim.launch.py` (`-name`), `gz_bridge.yaml` topics, `<robot name>` in both top-level xacros, the gz commands in `docs/hardware_l2_test.md` | Keep all of them in step |
| `GZ_SIM_RESOURCE_PATH` | `sim.launch.py` | Still needed so `package://mecanum_description` meshes resolve in Gazebo |
| Scan height band 0.10 to 0.50 m (STAND-IN) | `mecanum_localization/config/cloud_to_scan.yaml` | The new L2 mount sees the band around the robot |
| RViz link lists and views | `mecanum_description/rviz/display.rviz` and `mecanum_bringup/rviz/robot.rviz` (base and wheel links only, view distance 1.5 m for the previous base) | Add the new links. `sensors.rviz` and `l2.rviz` have no per-link lists. |
| Mount check constants | `tools/l2_mount_check.py`: `--floor-radius` 0.4 m, wall band 0.5 to 0.9 m with \|y\| > 0.35 m, "~25% (sim)" points on the robot | Fit them to the new footprint and L2 height. The on-robot checks are skipped until the self-filter box is set. |
| CAD conversion example and `--report-l2` | `tools/cad_stl_to_ros.py` | These fit the previous bracket. The converter itself is generic. |
| Old numbers in math tests | `mecanum_base/test/test_kinematics.py`, `test_drive_core.py` | Arbitrary inputs to the formulas; no change needed |
| L2 Gazebo sensor model (scan grid, noise, IMU rate) | `mecanum.gazebo.xacro` | The L2's own model; it follows `l2_lidar` / `l2_imu` |

## 5. Follows automatically through TF

No change is needed for these once the URDF has the new poses:
- the rko_lio extrinsics: `base_frame: base_footprint`, frames from the message headers;
- the LIO odometry adapter;
- the L2 driver frames;
- the self-filter and cloud-to-scan frames;
- the URDF comparisons in `tools/l2_mount_check.py`;
- the simulated L2's pose.

## 6. Where the additional sensors plug in

For each new sensor:
1. **Description.** Add `mecanum_description/urdf/sensors/<sensor>.xacro` with its frames on the
   new mount links. Include it from `mecanum.urdf.xacro`, behind a `use_<sensor>` arg like
   `use_l2`.
2. **Simulation.**
   - Add a Gazebo sensor on that frame in `mecanum_simulation/urdf/mecanum.gazebo.xacro`. Add
     world systems if needed (the L2 needs Sensors and Imu).
   - Bridge it in `config/gz_bridge.yaml`, on the same topic and frame as the hardware driver.
   - Expose the new `use_<sensor>` arg in `sim.launch.py` and `robot.launch.py` if it should be
     switchable.
3. **Hardware.** Add a driver package, or a vendor node, and include it in
   `mecanum_bringup/launch/hardware_sensors.launch.py`. Wrap the include in
   `GroupAction(scoped=True)`, or its launch arguments leak into the parent.
4. **Consumers.**
   - Self-filter: grow the box if the L2 sees the sensor.
   - Perception and LIO, if they use the sensor.
   - The costmaps (Phase 7).
5. **RViz.** Add a display to `mecanum_bringup/rviz/sensors.rviz`.
6. **Records.**
   - `docs/hardware_transition.md`: contracts (topic and frame), assumptions (for example A5,
     rear safety), change log.
   - `STANDINS.md` rows for unknown values.
   - `package.xml` `exec_depend`s.

## 7. Stale docs

These describe the previous model. Each has a `REWRITE` note where it matters.

| Doc | Sections |
|---|---|
| `README.md` | "View the robot description" (now a note), the rear-coverage paragraph under perception |
| `docs/plan.md` | "Where things stand" (mast); Phase 3 results (25% self-hits, visibility model); Phase 5 sphere-wheel option and the 110.2% rotation; Phase 7 footprint ±0.3025 × ±0.247 m, voxel height 0.954 m, pole shadow |
| `docs/hardware_transition.md` | Section 1 hardware facts (mast, floor 8 cm low, IMU tilt); Contracts "Mount"; A4, A5, A13; section 6 paused 3H/4H data (previous mast) |
| `docs/hardware_l2_test.md` | 2a measurement table, 2c tolerance actions, 3 and 3a sim comparison and visibility model (L2 world pose, 11,520 points, 25% on the robot) |
| `docs/hardware_lio_test.md` | Self-filter box, the chassis plate front edge as the measuring point, pole shadow |
| `docs/encoder_upgrade.md` | Controller YAML (`wheels_radius`, `sum_of_robot_center_projection_on_X_Y_axis`), the mapping table (9.5 mm offset) |
| `src/mecanum_simulation/README.md` | Measured L2 results at spawn; phase 2 measured behaviour (rotation 110.2%, 9.5 mm) |
| `src/mecanum_localization/README.md` | Self-filter section; rear coverage, pole shadow, measured box table; floor share |
| `src/mecanum_lidar/README.md` | Bench check 3 (cable direction); "Mounted on the robot" |
| `src/mecanum_base/README.md` | `motion_test`: "the simulation rotates at 110%" |

## 8. Suggested order once the meshes arrive

- [ ] Convert the CAD exports to metres in their link frames (`tools/cad_stl_to_ros.py`). Put them
      in `meshes/visual/` and `meshes/collision/`.
- [ ] `mecanum_params.xacro`: dimensions, masses and mesh paths, with `STAND-IN` comments where
      a value is uncertain.
- [ ] `mecanum_robot.xacro` and `mecanum_wheel.xacro`:
  - [ ] the `base_joint` origin;
  - [ ] `base_link` visual, collision and inertial, and the additional geometry meshes;
  - [ ] wheel meshes, positions, exactly one collision per wheel, inertias.
- [ ] `unitree_l2.xacro`: the new mount links and meshes, the `l2_mount` pose, and the
      `l2_lidar` yaw from the cable direction.
- [ ] The additional sensors (section 6).
- [ ] `esp32_driver.yaml`: set `wheel_radius`, `lx` and `ly`. Remove the skip in
      `test_urdf_defaults.py`, then run `colcon test`.
- [ ] `self_filter.yaml`: the box.
- [ ] Simulation:
  - [ ] check `fdir`, the friction merge and the spawn height;
  - [ ] run the sim and `motion_test`;
  - [ ] check the rotation scale (the previous model rotated at 110.2%).
- [ ] `cloud_to_scan.yaml` heights and the `tools/l2_mount_check.py` constants.
- [ ] RViz link lists and views.
- [ ] The docs in section 7, new `STANDINS.md` rows, and a `docs/hardware_transition.md`
      change-log entry. Then delete this file.
