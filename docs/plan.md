# Development plan: end-to-end simulation build

This plan covers an end-to-end build in simulation only: from the robot model to a full
mission (home → staging → dock at the toilet → task → home) in Gazebo. Hardware is not
planned here. Keep this file in step with the work.

## Context

The robot model was replaced on branch `feature/model-v2` (2026-10-27). The new robot is the
washroom robot prototype, described in `mecanum_description` from the reference URDF
`src/mecanum_description/refs/washroom_robot.urdf` and the sensor placement PDF
`docs/Washroom_Robot_Prototype_Sensors_and_BOM.pdf`. Two changes make the previous plan
obsolete:

- **The L2's role changed.** It used to hang inverted on an 810 mm mast, looking down at the
  floor around the robot, and was the only sensor. It now stands upright on the top of the
  robot, 0.80 m up. It sees the walls above the robot and the ceiling, but nothing near the
  floor closer than about 3 m. Near-field obstacles are now the job of the new sensors.
- **The motors now have position and velocity encoders.** Wheel odometry exists, so the drive
  can move to ros2_control, and odometry no longer depends on LIO alone.

The previous plan's decisions are therefore withdrawn:
- the open-loop drive (decision 0001);
- LIO as the only odometry, and decision D4 on how to make it work;
- the L2 band as the 2D scan;
- the rear-visibility model of the old mast;
- the open-loop motor calibration phase.

The SLAM, odometry and navigation strategy below is designed from scratch.

## Carried over

These were built for the previous model and are reused:

| What | Where | Reuse |
|---|---|---|
| The restroom and corridor worlds | `mecanum_simulation/worlds/` | Test worlds; the restroom gets sensor test objects in Phase 2 |
| The simulated L2, cloud and IMU | `mecanum_simulation/urdf/mecanum.gazebo.xacro` | Unchanged sensor model, now in wide mode and on the new pose |
| The self-filter | `mecanum_localization/config/self_filter.yaml` | Box set for the new robot |
| `gt_compare` | `mecanum_localization` | Judges every odometry candidate against Gazebo ground truth |
| `motion_test` patterns (steps, `loop`, `corridor`) | `mecanum_base` | Odometry and drive tests; moves to TwistStamped in Phase 1 |
| The rko_lio pipeline and `lio_odom_adapter` | `mecanum_localization` | An optional EKF input (Phase 3); the adapter's `publish_tf:=false` and `odom_topic` were made for this |
| The mecanum kinematics and its tests | `mecanum_base/kinematics.py` | A cross-check for the ros2_control controller |
| The encoder upgrade guide | `docs/encoder_upgrade.md` | Phase 1 follows its simulation steps |

## Rules that hold in every phase

- Every unknown is marked `STAND-IN` and listed in `STANDINS.md`.
- `use_sim_time` is honoured.
- Every package has a README.
- Upstream packages are preferred. Each one's installed parameters are read before it is
  configured.
- **TF has a single publisher for each link:**

  | Transform | Publisher |
  |---|---|
  | `map → odom` | AMCL or slam_toolbox, never both |
  | `odom → base_footprint` | the EKF (from Phase 3) |
  | wheel joints | robot_state_publisher, from `/joint_states` from `joint_state_broadcaster` |
  | everything else | robot_state_publisher, from the URDF |

  LIO packages broadcast their own TF; it is always remapped away.
- **Odometry comes only from measurements:** the wheel encoders, the IMUs and the sensors.
  Wheel *commands* are never odometry.
- **Every velocity command passes through the safety gate** (Phase 4), from every source:
  Nav2, docking, teleop and the test tools.
- **Simulation-only shortcuts are allowed, but never silent.** Anything that works only because
  it is a simulation is added to the "Sim-only shortcuts" list at the end of this file when it
  is introduced.
- **The contract stays stable.** The topics and frames in "Sensor contract" are what later
  hardware drivers must match. Changing one is allowed, but it is recorded there.

## Phase workflow

Each phase runs in this order:

1. **Implement** in simulation.
2. **Verify.** `colcon build`, `colcon test` and the sim verification listed for the phase.
3. **Update the records:** `STANDINS.md`, the sensor contract and the sim-only shortcuts in
   this file, and the package READMEs.
4. **Report and stop** for the owner's confirmation.
5. **Commit and push** only when the owner asks.

When a phase reaches a **decision point** (marked "Decision"), the options are reported with a
recommendation, and work stops until the owner chooses.

---

## Strategy

### Sensor roles

| Sensor | Where (`base_footprint`) | Sees | Role |
|---|---|---|---|
| **RPLIDAR C1** 2D lidar | Front face, scan plane 0.13 m, 200° slot | A horizontal slice ahead and to the sides, up to 12 m. Blind behind (160°). | **Primary SLAM and localization scan.** Near-field obstacles ahead. Collision monitor source. |
| **Unitree L2** | Top, upright, 0.80 m, wide mode (−6..90°) | Walls above 0.78 m, the ceiling, partitions, counters, doors, all around. Low objects only beyond about 3 m. | **Upper structure.** 3D costmap source for anything at the robot's height that the C1 plane misses. Long-range coverage behind. Its IMU is the EKF's gyro. LIO candidate (Phase 3), no longer odometry-critical. |
| **RealSense D435i** | Front face, 0.26 m, pitched 8° down | Depth ahead, 0.3–3 m (87° wide) | **Front 3D obstacles:** low objects under the C1 plane, partitions with a floor gap, overhangs below 0.83 m, the toilet. **Dock perception** (Phase 7). Second IMU. |
| **Ultrasonics** ×4 | Front pair facing +x, rear pair facing −x, 0.30 m | A 60° cone each, 0.03–4.5 m | **Glass and mirrors**, which the lidars and the camera miss. **The rear pair is the main reverse sensor.** Range costmap layer and collision monitor source. |
| **Cliff ToF** ×4 | Underside corners, 0.12 m, pointing down | The floor below each corner | **Drop detection** (stairs, a step down, a missing floor), through the safety supervisor |
| **Bumpers** F+R | Planks across the full width, 0.06–0.09 m | Contact | Last-resort contact stop, through the safety supervisor |
| **E-stop** | Rear face (not modelled) | — | A latched stop; a topic in simulation |
| **Wheel encoders** ×4 | Motors | Wheel position and velocity | **Wheel odometry** through `mecanum_drive_controller`, the EKF's main motion input |

### State estimation

- **Local:** a robot_localization EKF in `two_d_mode` owns `odom → base_footprint`. The
  baseline inputs are:
  - the wheel odometry twist (vx, vy, wz);
  - the L2 IMU's yaw rate.

  Mecanum rollers slip, mostly sideways, so vy gets a larger covariance. LIO, a second IMU and
  visual odometry are evaluated as additional inputs in Phase 3.
- **Global:** AMCL (OmniMotionModel) or slam_toolbox localization on the C1 scan publishes
  `map → odom` (Phase 6).

### Mapping and navigation

- **Map:** a 2D map from slam_toolbox on the C1 scan (Phase 5). The scan plane at 0.13 m does
  not see everything, so the map is not the only obstacle source.
- **Costmaps:**
  - the C1 scan;
  - a 3D voxel layer from the D435i and the L2;
  - a range layer from the ultrasonics.
- **Safety:** a collision monitor on the scan, the D435i and the ultrasonics. A separate safety
  supervisor handles the cliff sensors, bumpers, e-stop and sensor loss.
- **Planning and control:** Smac 2D and MPPI with the omnidirectional motion model, so the
  robot strafes into the stall.

### Command chain

```
Nav2 controller → velocity smoother → collision monitor ─┐
docking server ──────────────────────────────────────────┤
teleop / test tools ─────────────────────────────────────┴→ safety gate → mecanum_drive_controller
```

All velocity messages are `TwistStamped`, as `mecanum_drive_controller` requires. Every source
publishes on `/cmd_vel`. Until Phase 4 the controller's reference is remapped straight to
`/cmd_vel`; from Phase 4 the safety gate subscribes to `/cmd_vel` and the controller listens to
the gate's output, so no source changes ([decision 0002](decisions/0002-ros2-control-with-measured-wheel-state.md)).

### Sensor contract

Topics and frames that the simulation provides and that hardware drivers must match later:

| Topic | Type | Frame | Rate | From phase |
|---|---|---|---|---|
| `/l2/points` | `sensor_msgs/PointCloud2` | `l2_lidar` | 5.55 Hz | carried over |
| `/l2/imu` | `sensor_msgs/Imu` | `l2_imu` | 250 Hz | carried over |
| `/l2/points_filtered` | `sensor_msgs/PointCloud2` | `l2_lidar` | 5.55 Hz | carried over |
| `/scan` | `sensor_msgs/LaserScan`, ±100° | `lidar_2d_link` | 10 Hz | 2 |
| `/d435i/color/image_raw`, `/d435i/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `d435i_optical_frame` | 15 Hz (STAND-IN) | 2 |
| `/d435i/depth/image_rect_raw`, `/d435i/depth/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `d435i_optical_frame` | 15 Hz (STAND-IN) | 2 |
| `/d435i/depth/color/points` | `sensor_msgs/PointCloud2` | `d435i_optical_frame` | 15 Hz (STAND-IN) | 2 |
| `/d435i/imu` | `sensor_msgs/Imu` | `d435i_link` | 200 Hz | 2 |
| `/ultrasonic/{front_left,front_right,rear_left,rear_right}` | `sensor_msgs/Range` (ULTRASOUND) | `ultrasonic_<pos>_link` | 10 Hz | 2 |
| `/cliff/{front_left,front_right,rear_left,rear_right}` | `sensor_msgs/Range` (INFRARED) | `cliff_tof_<pos>_link` | 20 Hz | 2 |
| `/bumper/{front,rear}` | `std_msgs/Bool` | — | 20 Hz | 2 |
| `/estop` | `std_msgs/Bool`, transient local | — | on change | 4 |
| `/joint_states` | `sensor_msgs/JointState` | — | 50 Hz | 1 |
| `/wheel/odometry` | `nav_msgs/Odometry` | `odom` → `base_footprint` | 50 Hz | 1 |
| `/odom` | `nav_msgs/Odometry` (EKF) | `odom` → `base_footprint` | 50 Hz | 3 |
| `/safety/state` | to be defined in Phase 4 (`mecanum_interfaces`) | — | 10 Hz | 4 |

The frames come from `mecanum_description`. Gazebo-only topics live under `/sim/`.

---

## Phase 0: model v2 description (done 2026-10-27)

- **`mecanum_description`** was rewritten from the reference URDF. It has the body as 18 boxes,
  the wheels, the L2 on its new mount and one xacro per new sensor, each behind a `use_<sensor>`
  arg.
- **`test_reference_urdf.py`** checks every frame against the reference.
- **The values removed with the previous model were set again:**
  - the driver geometry: `wheel_radius` 0.1025, `lx` 0.2360, `ly` 0.2150;
  - the self-filter box;
  - `test_urdf_defaults.py` re-enabled.
- **Every marker left by the model strip (commit 08310da) was resolved,** and its tracking
  list was deleted.
- **The simulation still drives open-loop** (`sim_motor_plant`) until Phase 1.

## Phase 1: drive on encoders (ros2_control in simulation) (done 2026-10-28)

- **New package `mecanum_controllers`,** following `docs/encoder_upgrade.md`, sections 1, 3, 5
  and 6:
  - `urdf/mecanum.ros2_control.xacro` with a `hardware` arg, `mock` or `sim`. Each wheel joint
    has a velocity command interface and position and velocity state interfaces;
  - `config/controllers.yaml`: `mecanum_drive_controller` with `wheels_radius` 0.1025,
    `sum_of_robot_center_projection_on_X_Y_axis` 0.451, the speed limits (0.3, 0.3, 0.8) and
    `enable_odom_tf: true`, plus `joint_state_broadcaster`;
  - `launch/controllers.launch.py`: in sim only the spawners (started when the robot is
    spawned); with `hardware:=mock` also robot_state_publisher and `ros2_control_node`;
  - `test/test_mock_kinematics.py`, a launch test on mock hardware: wheel speeds equal
    `mecanum_base/kinematics.py`, the odometry echoes the twist, the limits and the reference
    timeout apply.
- **Simulation:** gz_ros2_control replaced the four `JointController` blocks and
  `sim_motor_plant`, which were deleted with `plant_model.py`, `sim_plant.yaml`, the wheel
  command bridges, the driver's `sim` transport and the `MotorCommands` message.
  joint_state_publisher is gone from the sim launch. The anisotropic wheel friction and the
  ground truth are unchanged.
- **Commands:** `TwistStamped` on `/cmd_vel` (see "Command chain"). `motion_test` publishes
  stamped by default; teleop needs `-p stamped:=true`.
- **Odometry:** the controller publishes `/wheel/odometry` and owns `odom → base_footprint`
  until Phase 3. LIO publishes `/lio/odom` without TF.
- **Decision record [0002](decisions/0002-ros2-control-with-measured-wheel-state.md):**
  ros2_control with measured wheel state. It supersedes 0001.
- **Measured** (restroom world, headless, 2026-10-28):

  | Test | Result |
  |---|---|
  | `motion_test` forward 0.45 m | 99% of commanded, cross-axis drift 0.0% |
  | `motion_test` strafe left 0.45 m | 99%, drift 0.1% |
  | `motion_test` rotate 85.9° | 100% of commanded, but the base translates **5.7 cm** (limit 5 cm): FAIL |
  | `/wheel/odometry` vs ground truth, `loop` × 2 (24.6 m) | final 0.03 m (0.13%), max 0.096 m (0.39%), yaw max 0.73°: PASS |
  | `/lio/odom` vs ground truth, same run | final 0.005 m, max 0.056 m (0.23%), yaw max 1.35°: PASS |
  | `/joint_states` | 50 Hz; the wheels turn in TF |

- **The translation during rotation is the centre of mass.** The stand-in COM sits 6.5 cm
  behind the wheel centre, so the wheels carry unequal loads and the friction-limited rollers
  slip unevenly. With the COM moved to the centre (a test only, not kept), the rotation step
  showed no translation but 109% rotation, the cylinder-rim effect of the previous model. The
  wheel odometry cannot see this slip; it is part of the rotation error the EKF has to absorb
  (Phase 3). The COM is a stand-in (`STANDINS.md`); `rotate_translation_tolerance` is left at
  5 cm until the owner decides.
- **LIO tracks on the upright L2, so far.** Unlike the previous model (18.5% loop error),
  rko_lio stays within 0.23% on the restroom loop. The corridor world, its hard case, was not
  re-run; Phase 3 evaluates it as candidate E3.
- **Shortcut S2:** ideal encoders, with no quantization or noise.
- **The ESP32 hardware plugin (`mecanum_hardware`) is out of scope.** `mecanum_base`'s driver
  stays for the hardware path (`sim:=false`), with the `mock` and `esp32` transports.

## Phase 2: simulated sensor suite

- **New Gazebo sensors** in `mecanum.gazebo.xacro`, on the URDF frames, bridged in
  `gz_bridge.yaml` to the contract topics. Each is switchable with its `use_<sensor>` arg in
  `sim.launch.py` and `robot.launch.py`.
  - **C1:** a `gpu_lidar` with one vertical sample, ±100°, about 0.7° steps, 0.05–12 m,
    10 Hz, Gaussian noise. STAND-IN: the noise, from the datasheet.
  - **D435i:** an `rgbd_camera`, 87° horizontal field of view, 0.3–3 m, 640 × 480 at 15 Hz
    (STAND-IN for the sim load), plus an `imu` sensor. If the simulator cannot render it at
    15 Hz, lower the resolution and record the change.
  - **Ultrasonics and cliff ToF:**
    - Gazebo Harmonic has no sonar sensor, so each is a small `gpu_lidar` cone: 60° for the
      ultrasonics and 27° for the ToF.
    - A sim-only node, `mecanum_simulation/sim_range_sensor`, turns each cone into a `Range`
      message: the minimum return, clamped to the sensor's limits.
  - **Bumpers:**
    - a `contact` sensor on each plank's collision. The plank joints need
      `preserveFixedJoint`, so that Gazebo does not lump the planks into `base_link`;
    - a sim-only node, `mecanum_simulation/sim_bumper`, turns the contacts into `Bool`.
  - **L2:** unchanged, wide mode by default.
- **Test objects**, in a new `worlds/sensor_test.sdf`, each placed for one sensor:
  - a low threshold (5 cm) under the C1 plane, which the D435i must see;
  - a stall partition with a 0.15 m floor gap, which the C1 passes under and the D435i and
    the L2 must see;
  - a counter overhang at 0.6–0.85 m, which only the D435i and the L2 see;
  - a glass panel, hidden from the lidars and the camera by a visibility mask, which only the
    ultrasonics see (shortcut S4);
  - a raised deck with an edge, as a drop for the cliff sensors;
  - a low wall for the bumpers.

  Add the partition, threshold and glass to `restroom.sdf` as well.
- **Self-filter:** check that the L2 sees no part of the robot outside the box. Check that the
  D435i and the C1 see no part of the robot; the C1 is limited to its slot.
- **RViz:** displays in `mecanum_bringup/rviz/sensors.rviz` for each sensor.
- **Verify:**
  - each sensor's topic, frame and rate match the contract;
  - each test object is detected, in `base_footprint`, within a few cm of its ground-truth
    pose, by the sensors that should see it and not by those that should not;
  - the sim real-time factor and CPU load with every sensor on. Record them; they bound the
    later phases.

## Phase 3: state estimation

- **EKF:** robot_localization `ekf_node`, `mecanum_localization/config/ekf.yaml`:
  - `two_d_mode: true`, 50 Hz, `world_frame: odom`, `base_link_frame: base_footprint`;
  - `odom0` `/wheel/odometry`: twist only (vx, vy, wz), with a larger covariance on vy;
  - `imu0` `/l2/imu`: yaw rate only;
  - optional `imu1` `/d435i/imu`: yaw rate only;
  - output `/odom` and `odom → base_footprint`.
- **LIO, re-evaluated as an optional input:**
  - rko_lio on the upright L2 (walls and ceiling, no floor), through `lio_odom_adapter` with
    `publish_tf:=false` on `/lio/odom`;
  - in the EKF, its twist or its pose differential.

  The previous failure modes, a cloud mostly of floor and a fixed scan pattern on a down-facing
  sensor, may not apply to the new view. They are measured, not assumed.
- **Candidates**, each measured with `gt_compare` on the `loop` pattern, the restroom corridor
  and `corridor.sdf` (the long featureless corridor):

  | Candidate | Inputs |
  |---|---|
  | E0 | wheel odometry alone (`/wheel/odometry`) |
  | E1 | the EKF on wheel odometry + the L2 gyro |
  | E2 | E1 + the D435i gyro |
  | E3 | E1 + LIO |

- **Targets** (STAND-IN, from what docking and navigation need): under 2% drift over the loop
  and under 2° yaw error; on the corridor, report the along-corridor error.
- **Decision D1: the default odometry.** It covers which EKF inputs are the default, and
  whether LIO stays, is replaced (for example by a point-to-plane LIO) or is dropped.
  **Recommendation:** E1 as the baseline. Add LIO only if E3 beats E1 by a clear margin on all
  three tests, because it costs CPU and is the input most likely to differ on hardware.
- **Verify:** TF has one publisher per link; the EKF's `/odom` beats each of its inputs alone.

## Phase 4: safety layer

- **New package `mecanum_safety`,** with a node `safety_supervisor`. Its logic is pure Python
  with unit tests, like `kinematics.py`. It handles:
  - **Cliffs:** a cliff sensor whose range exceeds the floor distance plus a margin (STAND-IN:
    0.119 + 0.05 m), or which returns nothing, blocks motion toward its corner. Motion away is
    allowed.
  - **Bumpers:** a press stops the robot. Only motion away from the pressed plank is allowed
    until it releases.
  - **E-stop:** `/estop` true stops everything and latches. A `~/reset` service clears it once
    `/estop` is false.
  - **Sensor loss:** a watchdog per contract topic, with timeouts as STAND-INs. A lost C1, L2,
    D435i, `/joint_states` or `/odom` stops the robot. A lost ultrasonic or cliff sensor blocks
    motion toward its side.
  - **Outputs:**
    - the gated `TwistStamped` to the controller reference;
    - `/safety/state`, a new `mecanum_interfaces` message with the reason for each block.
- **Gate placement:** between every command source and `mecanum_drive_controller`, as in the
  command chain. The Phase 1 remap is replaced.
- **Decision D2: the front bumper protrusion.** In the reference model the 2D lidar sticks out
  16 mm past the front plank, and the front ultrasonics 4 mm, so a wall hits them before the
  plank can switch. The PDF intends the plank to stand 3 cm proud of the face.
  **Recommendation:** move both planks 3 cm proud in the description, marked STAND-IN until the
  mechanical design is confirmed. The bumper tests then use real walls.
- **Verify, in `sensor_test.sdf`:**
  - driving at the deck edge stops before a wheel leaves the deck, forward, sideways and in
    reverse;
  - a bumper press stops the robot and allows only backing off;
  - the e-stop latches and resets;
  - killing each sensor's bridge triggers its watchdog;
  - teleop is gated too.

## Phase 5: mapping

- **slam_toolbox,** online async, on the C1 `/scan`:
  - `odom_frame odom`, from the EKF;
  - `base_frame base_footprint`;
  - resolution 0.05 m.

  Launched from `mecanum_localization/launch/slam_mapping.launch.py` and
  `mecanum_bringup/launch/mapping.launch.py`.
- **Saving.** A documented save command writes the `.pgm`/`.yaml` map and a posegraph to
  `mecanum_navigation/maps/`.
- **Rear blindness.** The C1 sees only the front 200°. The README explains how to map with it:
  forward passes, turning in place at the ends, strafing past openings.
- **Known gap.** The 0.13 m scan plane passes under the stall partitions (their floor gap ends
  at 0.15 m in `restroom.sdf`) and misses overhangs and low thresholds. The map shows those
  places as free.
- **Decision D3: what the map is built from.**

  | Option | What | For | Against |
  |---|---|---|---|
  | **D3-a** | The C1 alone. Known gaps are covered by a keepout mask, and the live 3D costmap layers (Phase 6) see the rest. | Simple; the C1 is a clean, long-range 2D sensor | The static map is incomplete; the mask is hand-drawn |
  | **D3-b** | A merged virtual scan into slam_toolbox: the C1 plus the D435i and L2 clouds, cut to the robot's height band | A more complete map | The D435i reaches only 3 m and covers 87°, so the merged scan's quality varies with direction; more CPU; a new merge node |

  **Recommendation:** D3-a. Revisit if Phase 6 shows planning failures from the gaps.
- **Verify.** Map the restroom world by teleop. Wall positions match the world SDF within a few
  cm. Report the partitions and fixtures the map misses.

## Phase 6: localization and Nav2

- **Localization.**
  - AMCL with `OmniMotionModel` on `/scan` is the default.
  - slam_toolbox in localization mode can be selected with `localization:=slam_toolbox`.
  - Only one of the two publishes `map → odom`.
- **`mecanum_navigation`:**
  - **Motion.** MPPI with the Omni motion model and Smac 2D, with a velocity smoother that has
    vy enabled. `enable_stamped_cmd_vel: true` everywhere.
  - **Footprint.** The polygon x −0.405..0.421, y −0.258..0.258, from the description. It is
    re-checked if Decision D2 moves the bumpers.
  - **Local costmap** (rolling, 0.05 m):
    - an obstacle layer on `/scan`;
    - a spatio-temporal voxel layer on `/d435i/depth/color/points` and `/l2/points_filtered`,
      from 0.04 m up to the robot's top at 0.83 m plus a margin. The floor is below 0.04 m;
    - a range layer on the four ultrasonics;
    - inflation.
  - **Global costmap:** static, obstacle (`/scan`), voxel (as local), inflation and a keepout
    filter. The keepout mask covers the map gaps from Phase 5.
  - **collision_monitor,** between the smoother and the safety gate:
    - sources `/scan`, `/d435i/depth/color/points` and the four ultrasonic ranges;
    - a front slowdown and stop polygon;
    - a rear stop polygon fed by the rear ultrasonics;
    - footprint approach.
  - **Reverse motion.** Behind the robot only the rear ultrasonics see the near field, and the
    L2 sees beyond about 3 m. MPPI critics prefer forward and sideways motion, and reverse speed
    is capped (STAND-IN).
  - **Goal tolerances.** Loose at staging, tight at the dock. Both are STAND-INs.
  - **Launch.** `mecanum_bringup/launch/navigation.launch.py`.
- **Verify:**
  - goals across the restroom world, including strafing moves;
  - each test object from Phase 2 is avoided, including the ones the C1 misses and the glass;
  - an obstacle inserted ahead stops the robot;
  - reversing toward an obstacle: the rear ultrasonics stop the robot;
  - relocalization from an offset initial pose;
  - the CPU load of the whole stack is recorded.
- **Branch point.** If MPPI behaves poorly with the EKF's odometry, tune the EKF's output rate
  and covariances before changing the controller.

## Phase 7: docking and the mission

- **Docking.** opennav_docking with `SimpleNonChargingDock`, and a static database holding
  `dock_toilet_1`. The staging pose is about 1 m before the dock. The final approach faces the
  toilet with the D435i.
- **Interfaces.** `mecanum_interfaces` gains `PerformTask.action`, `RunMission.action` and
  `MissionState.msg`.
- **`mecanum_mission`:**
  - `placeholder_task_server`, where the arm will plug in later;
  - `mission_executive` on nav2_simple_commander: home → staging → dock → task → undock →
    home. Each step is retried once; if it still fails, the robot goes home. It publishes
    `/mission_state` and reacts to `/safety/state`;
  - `record_pose.py` and `config/mission_poses.yaml`;
  - the launch file `mecanum_bringup/launch/mission.launch.py`.
- **Decision D4: dock detection.** First dock on the map pose alone and measure the final pose
  error against ground truth (shortcut S6 while poses come from the world file). If the error
  is too large for the arm (STAND-IN tolerance: 2 cm, 2°), choose a detector that publishes
  `detected_dock_pose` from the D435i:
  - **D4-a,** a fiducial (AprilTag) near the toilet: robust and cheap, but it needs a marker in
    the washroom;
  - **D4-b,** a depth-based detector of the bowl's front edge: no marker, but more work and more
    sensitive to the toilet model.

  **Recommendation:** measure first; if a detector is needed, D4-a in simulation to finish the
  end-to-end build, with D4-b as a later option.
- **Verify:**
  - the full mission end to end;
  - the dock pose error against ground truth;
  - injected failures, such as a blocked staging pose, trigger the retry and the return home.

## Phase 8: end-to-end acceptance in simulation

- **Missions.** 10 full missions in the restroom world from varied start poses. Report the
  success rate, the time and the dock pose errors.
- **Injected failures,** each with the expected safe outcome:
  - the loss of the C1, L2 or D435i mid-mission: stop, abort the mission;
  - a blocked staging pose: retry, then home;
  - a bumper press: stop and recover;
  - a person-sized obstacle on the path: stop or replan;
  - an e-stop mid-mission: latch; the mission resumes or aborts on reset, as chosen in Phase 7.
- **Clean-up:**
  - retire what the new strategy no longer uses from the default launch paths, such as the L2
    cloud-to-scan;
  - update the root README's run instructions for mapping, navigation and the mission.
- **Close-out.** Review the sim-only shortcuts list and `STANDINS.md` as a whole, then report.

---

## Sim-only shortcuts

Each entry works only because this is a simulation. Add entries as they are introduced.

| # | Shortcut | Introduced | Effect |
|---|---|---|---|
| S1 | Gazebo ground truth (`/sim/ground_truth/odom`) | carried over | Evaluation only (`gt_compare`, dock error). Never used for control or odometry. |
| S2 | Ideal wheel encoders | Phase 1 | No quantization, noise or latency. Slip comes only from the friction model. |
| S3 | Idealized sensors | Phase 2 | Gaussian noise only. No sonar crosstalk or multipath, no IR depth artefacts on shiny surfaces, no motion distortion in the lidars, a regular L2 scan pattern. |
| S4 | Glass via visibility masks | Phase 2 | Glass is hidden from the lidars and the camera by a Gazebo visibility mask and seen only by the ultrasonics. Real glass is partly visible and partly not. |
| S5 | The E-stop is a topic | Phase 4 | No hardware stop circuit. |
| S6 | Poses from the world file | Phase 7 | The dock and staging poses come from the known `toilet_1` pose until detection exists. |

## After the simulation build

Hardware is not planned here. When this build is complete, a hardware plan is written from:
- the sensor contract;
- the sim-only shortcuts;
- `STANDINS.md`.

`docs/hardware_transition.md` holds the previous hardware register and the paused L2 work,
kept for reference.
