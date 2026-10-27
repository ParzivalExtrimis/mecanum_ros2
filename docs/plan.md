# Development plan: the sim track

> **REWRITE (branch `feature/model-v2`):** the robot model is being replaced. Geometry
> quoted here (the 810 mm mast, the 0.93 m L2 height, the pole shadow, the Nav2 footprint
> ±0.3025 × ±0.247 m, the 0.954 m robot height) belongs to the previous model. See
> [`REWRITE.md`](../REWRITE.md).

This plan covers development in simulation only. Keep this file in step with the work.

Hardware work is planned separately, in [`hardware_transition.md`](hardware_transition.md). It
starts when this sim track is complete. Until then, that file is a living register: each phase
here ends by updating it (see "Phase workflow"), so the decisions and shortcuts made in
simulation are recorded where the hardware work will meet them.

## Context

The sim track was first planned with an optional hardware step after each phase. On
2026-10-16 the hardware steps were paused, after 3H and 4H had started; their progress is kept
in `hardware_transition.md`, section 6. From here, the phases run in simulation only, and some
of them make branching decisions, starting with Phase 4.

**Where things stand (2026-10-16):**
- **Done and pushed** on `feature/sim-drive-stack`: Phases 1 to 3, the `mecanum_lidar` driver,
  the corrected mast (810 mm pole, 30° bracket, cable at 22.9°), and commit 9033dfd (the L2
  hardware procedure, `tools/l2_mount_check.py`, `hardware_sensors.launch.py`).
- **Done, not committed:** Step 0, and the Phase 4 implementation and evaluation.
- **Phase 4 is blocked on a decision:** rko_lio misses its drift targets in sim. See Phase 4.

**Rules that hold in every phase:**
- `mecanum_description` is an ordinary package and may be edited freely (since 2026-10-15).
- Every unknown is marked `STAND-IN` and listed in `STANDINS.md`.
- `use_sim_time` is honoured.
- Every package has a README.
- Upstream packages are preferred. Each one's installed parameters are read before it is
  configured.
- TF has a single publisher for each link:
  - `map → odom` comes from slam_toolbox or AMCL;
  - `odom → base_footprint` comes from the odometry adapter;
  - everything else comes from robot_state_publisher.
- There are no encoders, so wheel commands are never used as odometry.
- **Simulation-only shortcuts are allowed, but never silent.** Anything that works only
  because it is a simulation (ground truth, perfect sensors, known poses) is recorded in
  `hardware_transition.md`, section 4, when it is introduced.
- **Contracts stay stable where possible.** The topics, frames and launch arguments in
  `hardware_transition.md`, section 2, are what the hardware work builds on. Changing one is
  allowed, but it is recorded there.

## Phase workflow

Each phase runs in this order:

1. **Implement** in simulation.
2. **Verify.** `colcon build`, `colcon test` and the sim verification listed for the phase.
3. **Update the hardware register** in `hardware_transition.md`:
   - add any sim-only shortcut the phase took;
   - re-score the assumptions listed under the phase's "Hardware register";
   - update contracts the phase changed, and the confidence of the affected stages;
   - add a change-log line.
4. **Report and stop** for the owner's confirmation.
5. **Commit and push** only when the owner asks.

When a phase reaches a **decision point** (marked "Decision"), the options are reported with a
recommendation, and work stops until the owner chooses.

---

## Done

### Phases 1 to 3

- **Phase 1.** L2 sensor frames in the description, `STANDINS.md`, the root README.
- **Phase 2.** Gazebo world, simulated drive plant, ESP32 driver with mock and sim transports,
  `motion_test`. Decision 0001: open-loop drive without ros2_control.
- **Phase 3.** Simulated L2 cloud and IMU, CropBox self-filter, cloud-to-scan. 25% of raw points
  hit the robot and are removed; about 45% hit the floor. The rear visibility model is
  z ≤ 0.93 + r·tan30·cosφ, with the pole shadowing 150 to 210°.

### Step 0: housekeeping (2026-10-15)

Stale serial wording updated to Ethernet in the docs; `mecanum_description` copyright fix,
`STAND-IN` comments and README; build and tests fully green (162 tests).

## Phase 4: LIO and the odometry adapter

### Built (2026-10-15)

- **rko_lio** (`online_node`) from `mecanum_localization/launch/lio.launch.py`: inputs
  `/l2/points_filtered` and `/l2/imu`; `base_frame base_footprint` with extrinsics from TF;
  `odom_frame lio_odom`; output `/lio/raw_odom`; its `/tf` remapped to `/lio/tf_unused`.
  `config/rko_lio.yaml`: deskew off (no time field in sim), initialization off (LIO starts while
  the robot settles after the spawn), `min_range` 0.3, `max_range` 30, `voxel_size` 0.25.
- **`lio_odom_adapter`:** flattens to x, y, yaw from the start pose; publishes `/odom` and
  `odom → base_footprint`; `publish_tf` and `odom_topic` parameters for the encoder upgrade;
  math in `planar_math.py` with unit tests.
- **`gt_compare`:** drift against `/sim/ground_truth/odom`, judged on both the final and the
  maximum error.
- **`motion_test` patterns** `loop` and `corridor`, whose segments end on measured progress.
- **`worlds/corridor.sdf`:** a 70 m featureless corridor for the degeneracy check.
- **`robot.launch.py lio:=true`** (default).
- 183 tests, 0 failures.

### Results: targets not met

| Run | Result |
|---|---|
| Loop, 2 laps, 24 m, 720° | Ended 4.48 m off (18.5%, target 1%); yaw error up to 9.3° (target 2°) |
| 4 m and 3 m out and back along the restroom corridor | LIO saw almost no motion: 0.03 m read at 3 m |
| Floor points removed before LIO | About 80% of the motion seen |
| `voxel_size` 0.1 or 0.5 | No better |

TF ownership and `publish_tf:=false` work as designed. When LIO tracks, its twist matches
ground truth. Details: `src/mecanum_localization/README.md`.

### What the paused hardware work added

From one static recording with the real L2 (`hardware_transition.md`, section 6):
- **The real L2 draws about 39 meridian scan lines per revolution,** at about 216 lines/s,
  and the lines shift between revolutions. The sim draws a fixed 360 × 32 grid.
- **rko_lio is sensitive to the scan pattern on real data too.** With half-revolution clouds it
  turned about 10°/s while standing still. With near-full sweeps (two clouds merged) it held
  0.014 m after 5 minutes.
- The common cause: rko_lio matches points to points, so on large flat surfaces it follows the
  scan lines instead of the geometry. In sim the fixed lines read as "no motion"; in the real
  half-sweeps the moving lines read as rotation.

### Decision D4: how to get trustworthy odometry in sim

This decides whether later phases run on representative odometry. Options:

| Option | What | For | Against |
|---|---|---|---|
| **D4-a** | **Make the simulated L2 draw the real pattern:** meridian lines that shift between revolutions. For example, a dense render thinned by a sim-only node to about 39 lines per revolution with a moving phase, or a single-line sensor on a spinning joint at the real rates. | Sim LIO, costmaps and scan behave like the real sensor. Any LIO can then be judged in sim. | Effort. Large clouds over DDS failed once in a quick prototype; this must be solved or avoided. |
| **D4-b** | **Evaluate a LIO that is not point-to-point,** for example a point-to-plane LIO such as FAST-LIO2 or Point-LIO (Unitree provides L2 settings for Point-LIO), behind the same adapter and `/odom`. | Likely robust to scan patterns on both sim and real data. The adapter, tools and recordings stay. | Built from source; its TF must be suppressed or remapped; evaluating it fairly needs D4-a or real motion data. |
| **D4-c** | **Keep rko_lio and pre-process** its input: floor removal, full-sweep accumulation. | Small change. | In sim, still about 80%: misses the targets. |
| **D4-d** | **Sim-only shortcut:** odometry from ground truth plus realistic noise, behind the same `/odom`, so Phases 5 to 8 can proceed. | Unblocks the rest of the sim track at once. | Defers the LIO risk to hardware; calibration (Phase 5) would be validated against an idealised odometry. Must be recorded as shortcut S8. |

**Recommendation:** D4-a first, then judge rko_lio and one D4-b candidate on the realistic sim
sensor, with the real static recording as a second check. Use D4-d only as an explicitly
recorded shortcut if D4-a stalls. **Owner's choice pending.**

### Verification once D4 is decided

- The loop: under 1% drift and under 2° yaw (stand-in targets).
- The restroom corridor and `corridor.sdf`: LIO holds, or the along-corridor error is reported.
- TF: one publisher per link.

### Hardware register

Re-score A1 (LIO method), A3 (sim sensor fidelity), A10 (time stamps). Record S1, S2 (done) and
S8 if D4-d is used. If the LIO changes, update the odometry contract notes and the 4H stage.

## Phase 5: motor calibration tool

- **`mecanum_base/calibrate_node.py`.**
  - Drives pure vx, vy and wz at several command levels and measures the achieved velocity from
    `/odom`.
  - Fits a scale and a deadband for each wheel group, plus **per-axis twist scales**, which the
    driver applies before inverse kinematics. This corrects the sim's 110.2% rotation rate.
  - Writes a calibration YAML that the driver loads.
- **Optional sim fix.** Change the wheel collision in `mecanum_description` to touch the floor
  at the wheel mid-plane, for example with a sphere, to remove the 110.2% at its source. The
  per-axis scales stay, because real mecanum wheels need them.
- **Verify.** Errors injected through `sim_plant_errors.yaml` are recovered within tolerance;
  rotation comes out at about 100%.
- **README.** Open-loop calibration drifts with battery voltage and floor condition.
- **Depends on D4.** The calibration is only as good as `/odom`. If D4-d is in use, the
  calibration is validated against idealised odometry: record that.

### Hardware register

Re-score A2 (LIO-only odometry for control), A6 (open-loop drive), A7 (open-loop calibration).
Record the tolerance chosen here; 5H uses it.

## Phase 6: mapping

- **Mapping.** slam_toolbox, online async, on `/scan` with `odom_frame odom`. Launched from
  `mecanum_localization/launch/slam_mapping.launch.py` and `mecanum_bringup/launch/mapping.launch.py`.
- **Saving.** A documented save command writes the `.pgm`/`.yaml` map and a posegraph to
  `mecanum_navigation/maps/`.
- **Rear blindness.** The README explains mapping around it: forward passes, rotating in place.
- **Verify.** Map the restroom world by teleop. Wall positions match the world SDF within a few
  cm.
- **Branch point.** If the 0.10 to 0.50 m scan band misses fixtures (the toilet bowl, the
  stall's floor gap), decide between another band, several bands, or a 3D map.

### Hardware register

Re-score A11 (2D mapping from the cloud band). Record shortcut S6 (no mirrors, glass or wet
floors) as still open.

## Phase 7: localization and Nav2

- **Localization.**
  - slam_toolbox in localization mode is the default.
  - AMCL with `OmniMotionModel` can be selected with `localization:=amcl`.
  - Only one of the two publishes `map → odom`.
- **`mecanum_navigation`:**
  - **Motion.** MPPI (Omni) and Smac 2D, with a velocity smoother that has vy enabled. All of
    it uses Twist.
  - **Footprint.** ±0.3025 × ±0.247 m. It already contains the mast.
  - **Local costmap.** A spatio-temporal voxel layer on `/l2/points_filtered`, from 0.05 m up to
    the bracket top at 0.954 m, plus a margin.
  - **Global costmap.** Static layer, plus the voxel layer, plus inflation.
  - **Keepout.** A keepout filter with an empty stand-in mask.
  - **collision_monitor** between the smoother and `/cmd_vel`, with slowdown and stop
    polygons, sourced from `/l2/points_filtered` and `/scan`.
  - **Rear safety** from the visibility model: MPPI critics limit reverse motion; the rear stop
    zone uses only the visible near field; the 150 to 210° pole shadow is never seen.
  - **Goal tolerances.** Loose at staging, tight at the dock. Both are stand-ins.
  - **Launch.** `mecanum_bringup/launch/navigation.launch.py`.
- **Verify.**
  - Goals across the world, including strafing moves.
  - An obstacle inserted ahead stops the robot.
  - Reversing toward an obstacle: reverse motion is limited, and the collision monitor stops
    the robot.
  - Relocalization from an offset initial pose.
  - **Measure the CPU load** of the whole stack, for the onboard computer choice.
- **Branch point.** If MPPI behaves poorly on LIO-rate odometry (about 5 to 12 Hz), decide
  between IMU-rate LIO output and a different controller.

### Hardware register

Re-score A2 (odometry rate for control), A5 (rear safety from the L2 alone), A8 (compute).
Record the speed limits and rear-motion limits chosen; 7H-b uses them.

## Phase 8: docking and the mission

- **Docking.** opennav_docking with `SimpleNonChargingDock`, and a static database holding
  `dock_toilet_1`. The staging pose is about 1 m before the dock.
- **Interfaces.** `mecanum_interfaces` gains `PerformTask.action`, `RunMission.action` and
  `MissionState.msg`.
- **`mecanum_mission`:**
  - `placeholder_task_server`, where the arm will plug in later.
  - `mission_executive` on nav2_simple_commander: home → staging → dock → task → undock → home.
    Each step is retried once; if it still fails, the robot goes home. Publishes
    `/mission_state`.
  - `record_pose.py` and `config/mission_poses.yaml`.
  - The launch file `mecanum_bringup/launch/mission.launch.py`.
- **Verify.**
  - The full mission end to end, about 10 times.
  - The dock pose error against ground truth.
  - Injected failures, such as a blocked staging pose, trigger the retry and the return home.
  - **Sensor loss:** stopping the L2 stream mid-mission must stop the robot and abort the
    mission (A9).
- **Branch point.** If pose-only docking misses the needed accuracy in sim, decide on a dock
  detector before hardware.

### Hardware register

Re-score A12 (pose-only docking), A9 (behaviour on sensor loss). Record shortcut S7 (poses from
the world file).

## End of the sim track: moving to hardware

When Phase 8 is done:

1. Review `hardware_transition.md` as a whole: every assumption re-scored, every shortcut with
   a removal stage, and the stage order still right.
2. Report the hardware plan with its confidence per stage, and the riskiest assumptions first.
3. The owner decides where to restart. The paused 3H and 4H work resumes from section 6 of that
   file.
