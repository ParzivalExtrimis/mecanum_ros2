# Hardware transition plan

This is the plan for taking the stack from simulation to the real robot. It is used when the
sim track in `docs/plan.md` is complete. Until then it is a living register, so that the sim
track's decisions and shortcuts are recorded where the hardware work will meet them.

**Status (2026-10-16): paused.** Stages 3H and 4H had started. Their progress, data and
findings are kept in [section 6](#6-paused-work-3h-and-4h) and in the procedure documents, so
they can resume without redoing anything.

**Superseded strategy (2026-10-27).** The robot model was replaced (model v2: the L2 upright on
the robot's top, plus a D435i, a 2D lidar, ultrasonics, cliff sensors and bumpers), and the new
motors have encoders. `docs/plan.md` was rewritten as an end-to-end simulation build with a new
odometry, mapping and navigation strategy, and it no longer keeps this register up to date.
This file is kept for its hardware facts, measurements and paused work. The stages in
section 5 follow the previous plan's phases; rewrite them from the new plan's sensor contract
and sim-only shortcuts when hardware is planned again. Assumptions that the new model or the
encoders made obsolete are marked "Superseded" in section 3.

The stages here are a starting point, not a commitment. The sim track will make branching
decisions, for example on the LIO method, and some of them will change these stages. The
document is built so that such a change stays local:

- **Assumptions are numbered** (A1, A2, ...) and each stage lists the ones it depends on. When
  an assumption changes, only those stages need review.
- **Sim-only shortcuts are numbered** (S1, S2, ...) with the stage that removes each one.
- **Stable contracts** (topics, frames, launch arguments) are listed once. Hardware procedures
  and tools are written against them, not against a particular implementation, so they survive
  a component swap.
- **Each stage separates kept progress from open work,** and states its confidence.

## How to keep this file current

At the end of each sim phase, and whenever a sim-only shortcut is taken:

1. **Shortcuts.** Add any new sim-only shortcut to [section 4](#4-sim-only-shortcuts), with the
   stage that must remove it.
2. **Assumptions.** Re-score the assumptions that `docs/plan.md` lists for that phase under
   "Hardware register". Add new ones.
3. **Contracts.** If the phase changed a contract in [section 2](#2-contracts), update every
   stage that names it.
4. **Stages.** Adjust the confidence and the open work of the affected stages only. Keep their
   "progress kept" lists.
5. **Change log.** Add a line to [section 7](#7-change-log).

### Confidence scale

| Level | Meaning |
|---|---|
| **High** | Expected to work as planned. Only parameters or stand-ins should change. |
| **Medium** | May need a component swapped or redesigned behind an unchanged contract, such as another LIO behind the same `/odom`. |
| **Low** | May change the architecture: contracts, packages, sensors or the drive design. |

## 1. Hardware facts

### Available

- **Unitree L2,** over Ethernet: L2 at 192.168.1.62, host 192.168.1.2 on `enp2s0`,
  `connection:=udp`, work mode 0 (the new mount wants work mode 1, wide FOV). USB-UART crashed
  the laptop's USB controller and is a fallback only. The robot model v2 puts it upright on a
  30 mm standoff on the top of the tall box, bottom face 0.767 m above the floor. The 3H/4H data
  below were recorded on the previous model's mast (810 mm pole, 30° bracket, inverted).
- **RealSense D435i** (owned) and **Jetson Orin Nano Super** (owned), per the sensor placement
  and BOM document (`docs/Washroom_Robot_Prototype_Sensors_and_BOM.pdf`).
- **The robot base,** pushable by hand with unpowered motors. Whether it slides sideways on its
  rollers is not yet checked. The new motors have position and velocity encoders.
- **The development laptop,** which runs everything for now.

### Not available yet

| Item | Blocks |
|---|---|
| ESP32 firmware protocol (framing, rates, timeout, units, motor IDs, watchdog) | 5H, 7H-b, 8H |
| ESP32 state reporting of the new encoders | The encoder upgrade (`docs/encoder_upgrade.md`) on hardware |
| The BOM parts: RPLIDAR C1, ultrasonics, cliff ToF, bumper switches, E-stop, drive relay, sensor hub | Their drivers; any driving (E-stop) |
| Onboard computer OS and ROS distro on the Jetson Orin Nano Super | Untethered runs |
| Physical dock and its design | 8H |
| Arm specification | The dock accuracy target, the arm self-filter |
| Real site floor plan | The site map and mission poses (`STANDINS.md`) |

### Measured so far (provisional)

From one static recording (`lio_4h_stage2_start`, 2026-10-15), before the mount check, with
the previous model's mast. The rows that depend on the mount (floor and IMU gravity through the
URDF, points outside the self-filter box) do not apply to the new mount; the sensor's own values
(rates, noise, points) do.

| Quantity | Measured | Sim assumption | Confidence in the value |
|---|---|---|---|
| IMU rate | 250 Hz | 250 Hz | High |
| Accelerometer noise σ | 0.09 to 0.16 m/s² | 0.02 | Medium: one session, robot still |
| Gyro noise σ | 0.004 to 0.007 rad/s | 0.002 | Medium |
| Accelerometer magnitude at rest | 9.47 m/s², 3.5% low | 9.81 | Medium |
| Scan line rate | about 216 Hz: 18 lines in 81 to 91 ms | n/a | High |
| Points | about 5,200 per 18-line cloud at 12 Hz, about 62,000 points/s | 11,520 per sweep at 5.55 Hz | High |
| Floor scatter per cloud | 3.2 cm rms | about 1 cm | Medium: includes floor flatness |
| Floor through the URDF | 8 cm below z = 0, tilted 1.9° forward | 0, level | Low: mount not yet checked |
| IMU gravity through the URDF | Tilted 3.8° forward | Level | Low: mount not yet checked |
| Points just outside the self-filter box | About 75 per cloud | None | Low: cause unknown |
| L2 stream | Stopped after 15.5 min for 23 min, then resumed | Never stops | Low: one occurrence |

## 2. Contracts

The hardware stages, procedures and tools depend on these. Changing one is a review trigger.

| Contract | Value | Used by |
|---|---|---|
| L2 topics | `/l2/points` (PointCloud2 in `l2_lidar`: x y z intensity time ring; `time` = float32 s after the stamp), `/l2/imu` (`l2_imu`); reliable QoS | 3H, 4H, all later |
| Filtered cloud and scan | `/l2/points_filtered` in `l2_lidar`; `/scan` in `base_footprint` | 3H, 4H, 6H, 7H |
| Odometry | `/odom` (`nav_msgs/Odometry`, `odom → base_footprint`, planar) | 4H to 8H, `tools/lio_marks.py` |
| TF ownership | `map → odom`: slam_toolbox or AMCL. `odom → base_footprint`: the odometry adapter (an EKF after the encoder upgrade). The rest: robot_state_publisher. | All |
| Drive input | `/cmd_vel` (Twist) into `esp32_driver`; zero on timeout | 5H, 7H-b, 8H |
| Hardware launch | `mecanum_bringup hardware_sensors.launch.py` with `lio:=`, `lio_config:=`, `replay:=` | 3H, 4H |
| Recording | `ros2 bag record /l2/points /l2/imu /odom /lio/raw_odom /tf_static`, started before LIO | 4H onward; replayable through `replay:=true` |
| Mount | The properties `l2_mount_x/y/z` and `l2_cable_yaw` in `mecanum_description/urdf/sensors/unitree_l2.xacro`: upright, (−0.100, 0, 0.767) m in `base_footprint`, cable to the rear (stand-in) | 3H; frames, self-filter and sim follow |
| Other sensors | The sensor contract in `docs/plan.md` (topics and frames of the D435i, 2D lidar, ultrasonics, cliff ToF, bumpers, E-stop) | Their future drivers |

## 3. Assumptions and risks

Each row: what the architecture assumes, the evidence so far, the confidence, what changes if
it is wrong, and when to re-score it.

| ID | Assumption | Evidence so far | Confidence | If wrong | Stages | Re-score at |
|---|---|---|---|---|---|---|
| A1 | **Superseded (2026-10-27):** odometry is now wheel odometry fused with the IMUs in an EKF, with LIO optional (plan Phase 3). Previously: **LIO from the L2 alone gives usable odometry, with rko_lio.** | Sim: rko_lio misses motion along a corridor (fixed scan grid). Real, still: diverges with 18-line clouds, holds with 36 (0.014 m after 5 min). Real motion untested. rko_lio matches points to points and is sensitive to the scan pattern in both cases. | **Low** for rko_lio specifically; **Medium** for "some LIO behind `/odom`". | Swap the LIO behind the adapter (for example a point-to-plane LIO), or feed it differently (full sweeps, floor removal). The `/odom` contract, procedures, tools and recordings survive. | 4H, 5H, 6H, 7H, 8H | Phase 4 decision; 4H |
| A2 | **Superseded (2026-10-27):** wheel odometry and the EKF run at 50 Hz. Previously: **LIO-only odometry is enough for control:** about 5 to 12 Hz, without wheel odometry. | MPPI and the velocity smoother not yet run on it. | Medium | Raise the LIO rate (IMU-rate output), or bring the encoder upgrade forward (EKF, ros2_control). | 5H, 7H-b, 8H | Phase 7 |
| A3 | **The simulated L2 represents the real one well enough** for LIO, costmaps and the blind zone. | Real pattern: about 39 meridian lines per revolution, precessing; sim: a fixed 360 × 32 grid. Noise 5 to 8 times lower in sim. | Low (as of now) | Sim results for LIO and costmaps do not transfer; re-tune on hardware. | 4H, 6H, 7H | Phase 4 decision |
| A4 | **The real mount matches the URDF** closely enough that only xacro properties change. | New mount (upright on a standoff) unmeasured. Its height is a stand-in: the STEP and the PDF disagree by 40 mm. The previous mast's provisional errors (floor 8 cm low, 1.9° tilt) do not carry over. | Medium | Re-measure with the ceiling check, then shim the standoff or change `l2_mount_*` / `l2_cable_yaw`. Self-filter and costmap heights re-tuned. Architecture unaffected. | 3H, then all | 3H stage 2 |
| A5 | **The rear ultrasonics cover rear safety** in the near field, with the L2 beyond about 3 m. | The upright L2 sees nothing low within about 3 m in any direction, and the 2D lidar sees only the front 200°. Not yet simulated (plan Phases 2 and 6). | Medium | Reverse motion restricted further, or another rear sensor. | 7H-b, 8H | Plan Phase 6 |
| A6 | **Superseded (2026-10-27):** the motors have encoders; the drive moves to ros2_control (plan Phase 1). Previously: **Open-loop drive through the ESP32,** no ros2_control (decision 0001); the firmware takes wheel commands and stops on link loss. | Firmware protocol unknown. | Medium | If the firmware runs closed-loop wheel speeds or reports encoders, move to ros2_control earlier (`docs/encoder_upgrade.md`). The `/cmd_vel` contract survives. | 5H, 7H-b, 8H | When the protocol is known |
| A7 | **Superseded (2026-10-27):** wheel speed is closed-loop with encoders; no open-loop calibration phase. Previously: **Open-loop calibration from LIO odometry** gives commanded and achieved velocity within tolerance on the real floor. | Sim only, with an ideal plant and dry friction. Restroom floors can be wet. | Low | Closed-loop speed control (encoders) or online velocity correction. | 5H, 7H-b, 8H | Phase 5 |
| A8 | **The onboard computer can run the stack** (L2 driver, LIO, slam_toolbox or AMCL, Nav2 with MPPI and the voxel layer). | rko_lio used 6 to 15% of one laptop core. Nav2 not yet measured. | Medium | A bigger computer, or lighter components (for example a 2D costmap). The vendored L2 SDK supports aarch64. | All untethered work | Phase 7 |
| A9 | **The L2 link is reliable for a full mission,** and the stack survives a dropout. | Ethernet stream stopped once for 23 min after 15.5 min. LIO did not recover. Serial stopped after about 7 s. | Low | Driver reconnect logic; LIO reset and the adapter re-zeroing; mission abort on sensor loss. Possibly a hardware cause: cable, power, network config. | 4H onward | Before 4H resumes |
| A10 | **Host-clock stamping is accurate enough** (`use_system_timestamp: true`; the cloud stamp is the sweep start, IMU stamped on receipt). | Replay with stamps moved to the sweep end changed nothing while still. Motion untested. | Medium | Hardware time sync, or a measured offset. | 4H | 4H stage 2 |
| A11 | **2D mapping on the 0.10 to 0.50 m band of the cloud** gives a good restroom map. Superseded (2026-10-27): the map comes from the 2D lidar (plan Phase 5); the same glass, mirror and tile risk applies to it. | Sim only. Restrooms have mirrors, glass and shiny tiles, which the sim does not model. | Medium | Different band, filters for reflections, or 3D mapping. | 6H, 7H | Phase 6 |
| A12 | **Pose-only docking** (opennav_docking with `SimpleNonChargingDock`, no dock perception) is precise enough for the arm. | Dock design and the arm's tolerance unknown. | Low | A dock detector (a tag or a shape), a new perception input. | 8H | Phase 8 |
| A13 | **A CropBox self-filter is enough** until the arm arrives. | Sim, new mount: 0.6% of the points hit the robot (the top of the tall box), all removed. The previous mast's 75 real points per cloud outside the box do not carry over. | High until the arm arrives | URDF-based self-filter earlier. | 3H, 4H, 7H | 3H stage 2 |
| A14 | **The laptop and later the onboard computer can share the L2 subnet** (192.168.1.0/24) with site Wi-Fi. | NetworkManager profile with a /32 route works on the laptop. | High | Re-address the L2 (needs a one-time serial write). | All | Onboard computer choice |

## 4. Sim-only shortcuts

Things the sim track does that hardware must replace or re-check. Each sim phase adds its own.

| ID | Shortcut | Introduced | Removed or re-checked by |
|---|---|---|---|
| S1 | Ground truth (`/sim/ground_truth/odom`) for evaluation (`gt_compare`) and for ending `motion_test` segments | Phase 4 | 4H: tape marks and `tools/lio_marks.py` |
| S2 | LIO sim config: deskew off (no time field), start-up initialization off (spawn settling) | Phase 4 | 4H: `rko_lio_hardware.yaml` (deskew and initialization on) |
| S3 | Simulated L2: fixed 360 × 32 grid, one full sweep per message, instant render, low noise | Phase 3 | 3H stage 3 (compare), 4H |
| S4 | Simulated drive: `sim_motor_plant`, JointController, anisotropic friction, 110.2% rotation, no watchdog | Phase 2 | 5H |
| S5 | Perfect extrinsics; the robot starts level and still; no people or moving objects | Phases 3, 4 | 3H stage 2, 4H, 7H |
| S6 | World: primitive restroom; no mirrors, glass or wet floor | Phase 2 | 6H, 7H |
| S7 | Dock and mission poses known exactly from the world file | Phase 8 (planned) | 8H: `record_pose.py` on the real map |

## 5. Stages

The stages follow the sim phases. A stage starts only when its sim phase is done and its
prerequisites exist. Motors are needed only from 5H on; 3H, 4H, 6H and 7H-a work with the
robot pushed by hand. Every stage writes or extends a procedure in
`docs/hardware_<topic>_test.md` (prerequisites, numbered stages, acceptance, results log),
replaces measured stand-ins in `STANDINS.md`, and stops for the owner's review.

### 3H: the L2 on the robot

- **Confidence: High** for the procedure and tools. **Medium** overall, because of A4, A5 and
  A13.
- **Depends on.** Phase 3; contracts: L2 topics, filtered cloud, mount; A4, A5, A13, A9.
- **Procedure.** `docs/hardware_l2_test.md`.
- **Progress kept.** The procedure, `tools/l2_mount_check.py` (validated against the sim),
  `hardware_sensors.launch.py`, the Ethernet setup, and the provisional measurements in
  [section 1](#measured-so-far-provisional).
- **Open work.**
  1. Stage 1: set `cloud_scan_num` for full sweeps (about 39) and the 0.05 to 30 m range in
     `l2.yaml`; confirm one revolution per cloud at about 5.5 Hz. **4H depends on this.**
  2. Stage 2: the mount check on the new mount (ceiling plane, wall heading).
  3. Stage 3: sim against real, including the edge of the field of view.
- **Acceptance.** The mount check within its tolerances; no points on the robot.

### 4H: LIO with the robot pushed by hand

- **Confidence: Low.** The LIO method is an open decision (A1), and real motion is untested.
  The procedure, tools and recordings do not depend on which LIO runs behind `/odom`.
- **Depends on.** Phase 4 (and its LIO decision), 3H stage 1; contracts: odometry, hardware
  launch, recording; A1, A3, A9, A10.
- **Procedure.** `docs/hardware_lio_test.md`, sized for the current test area: a static test,
  one 2 m push away from a wall (the sim comparison), and a loop of three laps of a 2 m × 1 m
  rectangle.
- **Progress kept.** `rko_lio_hardware.yaml`, `hardware_sensors.launch.py lio:=true` and
  `replay:=true`, `tools/lio_marks.py`, the first static recording and its analysis
  ([section 6](#6-paused-work-3h-and-4h)).
- **Open work.**
  1. Re-run the static test with full-sweep clouds.
  2. The 2 m push: does the real L2 see motion where the sim's does not?
  3. The loop, then CPU load.
- **If the Phase 4 decision changes the LIO:** add its hardware config next to
  `rko_lio_hardware.yaml`, select it with `lio_config:=`, and replay the existing recordings
  through it first.
- **Acceptance.** Drift within the Phase 4 targets (1% of the path, 2° of yaw).

### 5H: ESP32 drive and calibration

- **Confidence: Medium-Low.** A6 (firmware), A7 (open-loop calibration on a real floor), the
  missing e-stop.
- **Depends on.** Phase 5, 4H; the ESP32 protocol; a way to stop the robot; contracts: drive
  input, odometry; A2, A6, A7.
- **Progress kept.** The driver's transport interface (`Esp32Transport` in `transport.py`),
  `motion_test`, the calibration file format.
- **Steps.**
  1. Implement the `esp32` transport against the protocol, tested on a loopback fake, as
     `mecanum_lidar` was.
  2. On blocks: wheel directions and signs with `motion_test`; losing the link stops the
     motors.
  3. On the floor at low speed, after confirming how to stop the robot.
  4. Calibration with `calibrate_node` and LIO odometry, at two battery levels.
- **Acceptance.** Commanded and achieved velocity agree within the Phase 5 tolerance on all
  three axes.
- **Architecture-sensitive.** Firmware that closes the wheel-speed loop or reports encoders
  (A6): then follow `docs/encoder_upgrade.md` instead of open-loop calibration.

### 6H: map the real restroom

- **Confidence: Medium.** Depends on 4H working, and on A11.
- **Depends on.** Phase 6, 4H; contracts: scan, odometry; A1, A4, A11.
- **Steps.** `mapping.launch.py` gains a `sim:=false` path that reuses the hardware launch.
  Map the room by pushing (or driving, after 5H). Check tape-measured distances, and look for
  ghost walls from the robot itself, glass and mirrors. Keep the real map separate from the
  sim map.
- **Acceptance.** Distances within about 5 cm (stand-in).

### 7H: real localization, then autonomous driving

- **7H-a, pushed. Confidence: Medium.** Load the 6H map and push the robot around;
  localization must track and must recover from a wrong start.
- **7H-b, driving. Confidence: Low-Medium.** A2, A5, A7 and A8 all meet here.
  - Needs 5H and a navigation speed cap.
  - Stages: short goals in open space; a box in the path must stop the robot; a rear-stop
    test; goals across the room. Someone stays ready to stop the robot.
- **Depends on.** Phase 7, 6H; for 7H-b also 5H.
- **Acceptance.** Goals reached within tolerance; every obstacle test stops with margin.

### 8H: the mission in the real restroom

- **Confidence: Low.** The dock design and the arm's tolerance are unknown (A12).
- **Depends on.** Phase 8, 7H-b, a physical dock.
- **Steps.** Record `home`, `staging_toilet_1` and `dock_toilet_1` on the 6H map with
  `record_pose.py`; set dock tolerances from what can be measured; dock about 10 times and
  measure repeatability; run the full mission under supervision.
- **Acceptance.** The dock pose repeats within what the arm needs (a stand-in until the arm is
  specified).

### Cross-stage items

| Item | Confidence | Notes |
|---|---|---|
| Onboard computer | Medium (A8) | Choose model, OS and ROS distro from the 4H and 7H CPU measurements. The laptop runs everything until then. |
| Encoders | High for the plan; the timing is open | Follow `docs/encoder_upgrade.md`: `mecanum_controllers`, `mecanum_hardware`, an EKF. `publish_tf:=false` on the adapter hands `odom → base_footprint` to the EKF. |
| Arm | Low | Needs a URDF-based self-filter that replaces the CropBox; moving arm points must not reach LIO. |
| L2 link reliability | Low (A9) | Before 4H resumes: a long static run with the stream monitored. Then driver reconnect and LIO recovery if dropouts repeat. |

## 6. Paused work: 3H and 4H

Kept so that the work resumes where it stopped.

- **3H.** Not run. The procedure and tools are ready. The provisional values in
  [section 1](#measured-so-far-provisional) came from the 4H recording, not from 3H.
- **4H, first static run (2026-10-15).**
  - Recording: `lio_4h_stage2_start/` (2.1 GB, mcap) and `lio_4h_stage2_start.csv` in the
    repo root, untracked. The recording started after LIO was already running.
  - Result: with `cloud_scan_num` 18, `/odom` wandered by up to 4.3 m and 120° while still.
  - The sensor data itself is consistent: a plain point-matching check holds within about
    1 cm and 0.5°.
- **4H, offline replay of that recording (2026-10-16).**
  - Every one of these still diverged: deskew off, more threads, a noise-free IMU, an IMU
    agreeing with the floor, shifted stamps, a 0.1 m map, the raw cloud, initialization off.
  - Two clouds merged per scan (36 lines) held still: 0.014 m and 0.01° after 5 min, with the
    largest excursion 0.067 m and 2.2° in the first seconds. rko_lio used about 6% of a core.
  - Cause: half-revolution clouds give each scan different line positions, and rko_lio's
    point-to-point matching follows them.
  - The replay scripts are in `build/scratch_4h/` (not tracked): `replay.sh`, relays that
    merge clouds or replace the IMU, and `summarize.py`.
- **To resume.** 3H stage 1 (`cloud_scan_num`, work mode 1), then the 4H static test, then
  the 2 m push. Check the L2 link first (A9). The recording above is from the previous mast;
  its LIO findings on scan lines still apply to the sensor, but its mount numbers do not.

## 7. Change log

| Date | Change |
|---|---|
| 2026-10-16 | Created from the hardware steps in `docs/plan.md`. 3H and 4H paused, with the first 4H results. Assumptions A1 to A14 and shortcuts S1 to S7 recorded. |
| 2026-10-27 | Branch `feature/model-v2`: the robot model is being replaced (new meshes, geometry and additional sensors). The previous model's meshes, dimensions and L2 mast were removed from the URDF and the dependent configs. The Mount contract, A4 and A5 need re-scoring for the new model, and the 3H/4H mount data describe the previous mast. |
| 2026-10-27 | Model v2 written from the reference URDF (`mecanum_description/refs/washroom_robot.urdf`): the L2 upright at 0.767 m, the D435i, 2D lidar, ultrasonics, cliff ToF and bumpers. The motors have encoders. `docs/plan.md` rewritten as an end-to-end simulation build; this register is no longer updated per phase. Updated: hardware facts, the Mount contract (and a new Other sensors contract), A4, A5, A13. Superseded: A1, A2, A6, A7, A11. `docs/hardware_l2_test.md` and `docs/hardware_lio_test.md` updated for the upright mount. |
