# mecanum_simulation

This package is the Gazebo Harmonic (gz-sim 8) simulation of the mecanum base. It contains
the restroom, corridor and sensor test worlds, the simulation-only additions to the robot
description, the simulated sensor suite and the ros_gz bridge.

The drive runs on ros2_control with Gazebo hardware (`mecanum_controllers`,
[decision 0002](../../docs/decisions/0002-ros2-control-with-measured-wheel-state.md)): the
wheel joints are velocity-commanded, and their measured positions and velocities come from
physics. These are ideal encoders, with no noise or quantization (shortcut S2 in
[docs/plan.md](../../docs/plan.md)).

## Data flow

```
/cmd_vel (geometry_msgs/TwistStamped)
  -> mecanum_drive_controller     inverse kinematics, limits, 0.25 s reference timeout
  -> gz_ros2_control (GazeboSimSystem, inside Gazebo)
  -> wheel joint velocities -> anisotropic roller friction -> the chassis moves
  -> measured wheel states -> joint_state_broadcaster -> /joint_states
                           -> mecanum_drive_controller -> /wheel/odometry, odom -> base_footprint

gz OdometryPublisher -> /sim/ground_truth/odom   evaluation only, never TF or /odom
```

The sensors, on the topics and frames of the plan's sensor contract:

```
gz gpu_lidar            -> bridge            -> /scan                     2D lidar
gz gpu_lidar, IMU       -> bridge            -> /l2/points, /l2/imu       L2
gz rgbd_camera, IMU     -> bridge            -> /d435i/color/*, /d435i/depth/image_rect_raw,
                                                /d435i/depth/camera_info, /d435i/imu
  depth image           -> sim_depth_cloud   -> /d435i/depth/color/points
gz gpu_lidar cones      -> bridge, as clouds -> /sim/ultrasonic/*, /sim/cliff/*
                        -> sim_range_sensor  -> /ultrasonic/*, /cliff/*   sensor_msgs/Range
gz contact              -> bridge            -> /sim/bumper/*             while in contact
                        -> sim_bumper        -> /bumper/*                 std_msgs/Bool, 20 Hz
```

## How mecanum wheels are simulated

Each wheel keeps its cylinder collision from `mecanum_description`. The file
`urdf/mecanum.gazebo.xacro` merges friction settings into that collision. Friction is
`mu = 1.0` along the roller axis and `mu2 = 0` across it. The direction is fixed in the base
frame through `fdir1 gz:expressed_in="base_footprint"`, so it does not spin with the wheel.

| Wheels | Roller axis |
|---|---|
| front_left, rear_right | (1, -1) |
| front_right, rear_left | (1, 1) |

This is the method of Gazebo's own `mecanum_drive` demo. The rollers are idealized: there are
no roller dynamics, vibration or wear.

The merge only works because the `<collision name>` in the Gazebo block matches the name the
URDF-to-SDF converter generates, `<link>_collision`. A bare `<fdir1>` under
`<gazebo reference>` silently loses its `gz:expressed_in` attribute.

## World: `worlds/restroom.sdf`

The world is a stand-in site built from primitives only, so nothing is downloaded. Layout,
with z up, in metres:

| Area | Extent and contents |
|---|---|
| Corridor | x -1 to 7, y -2 to 0. Spawn and home at (0, -1), facing +x. Contains a bench. |
| Lobby | x -1 to 2, y 0 to 4. Contains a column. |
| Restroom | x 2 to 7, y 0 to 4. A 0.9 m doorway in the wall at y = 0, a sink counter and a bin. |
| Stall | x 5.4 to 7, y 2.2 to 4, open towards -x. Partitions leave a 0.15 m floor gap. |
| `toilet_1` | Against the east wall at y = 3.075. The bowl's front edge is at x = 6.34. |

| Test objects | A 5 cm threshold across the corridor at x = 5.5, and a glass screen in the lobby at y = 2.8. Both are off the routes used so far. |

The world is open on top in the Gazebo GUI, and the four outer walls are translucent (tinted
blue), so the robot can be watched from above and from outside. The inner walls and fixtures
are opaque. Only the GUI is affected: the visual's `<transparency>` is ignored by `gpu_lidar`,
so the simulated sensors still see the outer walls and a 2.4 m ceiling. The ceiling is a
separate model, `ceiling`, with a fully transparent visual. Checked headless on 2026-10-27:
still 11,520 points per sweep, about half of them on the ceiling.

## World: `worlds/corridor.sdf`

A test world for the odometry degeneracy check (plan Phase 3), not a site model. It is a
straight, featureless corridor, 2 m wide and 70 m long, with a 2.4 m ceiling: x −35 to 35,
y −2 to 0.
The default spawn (0, −1) is on its centre line, facing along it.

The walls, floor and ceiling pin down sideways, vertical and rotational motion. Nothing pins
down motion along the corridor while both end walls are beyond the L2's 30 m range, which holds
within x −5 to 5. That is the worst case for scan matching.

```bash
ros2 launch mecanum_bringup robot.launch.py gui:=false \
  world:=$(ros2 pkg prefix mecanum_simulation)/share/mecanum_simulation/worlds/corridor.sdf
```

## World: `worlds/sensor_test.sdf`

One object per sensor, in the open, each with its own station for `tools/sensor_check.py`. The
floor is thick boxes around a hole, with the friction of the restroom floor, so there is no
ground plane.

| Object | Where | Size, m | Must be seen by | Must not be seen by |
|---|---|---|---|---|
| `low_threshold` | (1.5, 0) | 0.3 × 2 × 0.05 | D435i | 2D lidar, L2, ultrasonics |
| `stall_partition` | (1.5, 3) | 0.05 × 1.6, z 0.15 to 1.75 | D435i, L2, ultrasonics | 2D lidar |
| `counter_overhang` | (2.6, -3) | 0.6 × 2, z 0.6 to 0.85 | D435i, L2 | 2D lidar, ultrasonics |
| `glass_panel` | (1.5, -6) | 0.02 × 1.5 × 2 | ultrasonics | 2D lidar, L2, D435i |
| floor hole | x 1 to 2.2, y 5.4 to 6.6 | 1 m deep | cliff ToF | |
| `bumper_wall` | (1, 9) | 0.1 × 1 × 0.09 | bumper planks | |

The bumper wall is lower than the body and the 2D lidar, so the plank hits first even though the
lidar stands further forward (decision D2 in the plan).

```bash
ros2 launch mecanum_bringup robot.launch.py gui:=false lio:=false \
  world:=$(ros2 pkg prefix mecanum_simulation)/share/mecanum_simulation/worlds/sensor_test.sdf
tools/sensor_check.py     # topics, frames, rates, self view, every object, cliffs, bumpers
```

Run the check headless, as above. The Gazebo window takes the real-time factor from 0.96 to 0.81,
and with RViz to 0.71, and the extra load costs `/l2/points_filtered` and the D435i their rates.

## Simulated sensor suite

All in `urdf/mecanum.gazebo.xacro`, on the URDF frames. Each has a `use_<sensor>` argument.

| Sensor | Model | Output |
|---|---|---|
| RPLIDAR C1 | `gpu_lidar`, 250 rays over ±89.5°, 10 Hz, 0.05 to 12 m, σ 0.01 m | `/scan` in `lidar_2d_link` |
| D435i | `rgbd_camera`, 87°, 848 × 480, 15 Hz, depth 0.3 to 3 m, IMU at 200 Hz | `/d435i/...` in `d435i_optical_frame`, the IMU in `d435i_link` |
| Ultrasonics | `gpu_lidar` cone, 60° × 6°, 9 × 3 rays, 10 Hz, 0.03 to 4.5 m, σ 0.01 m | `/ultrasonic/<pos>` in `ultrasonic_<pos>_link` |
| Cliff ToF | `gpu_lidar` cone, 27° × 27°, 3 × 3 rays, 20 Hz, 0.01 to 4 m, σ 3 mm | `/cliff/<pos>` in `cliff_tof_<pos>_link` |
| Bumpers | `contact` on each plank's collision | `/bumper/{front,rear}` at 20 Hz |

Sim-only nodes, each with a pure helper tested in `test/test_sim_nodes.py`:

| Node | Does | Why |
|---|---|---|
| `sim_range_sensor` | Nearest return of a cone as a `Range`; `+inf` when there is none. One instance per group (`config/sim_range_sensors.yaml`). | No sonar sensor in Gazebo, no `Range` in the bridge |
| `sim_bumper` | Contacts to a level `Bool`, held for 0.1 s after the last contact | Gazebo publishes contacts only while they last |
| `sim_depth_cloud` | The depth image as a cloud in the optical frame, every second pixel, x, y, z only | Gazebo's own cloud has wrong axes for its frame |

These Gazebo behaviours shaped the design. Each was measured here, and none is obvious:

- **A bridged `LaserScan` keeps only the middle row of a cone,** so the cones are bridged as
  clouds. As a scan, the cliff cone over a hole reads 1.12 m where the real cone reads 0.27 m.
- **`rgbd_camera` labels its cloud `gz_frame_id` but keeps x forward.** Through TF the cloud is
  rotated. `<gz_frame_id>` and `<optical_frame_id>` set every frame together, so they cannot fix
  it. `sim_depth_cloud` replaces it; the cloud is not bridged, so Gazebo does not compute it.
- **Glass needs visibility bit 28.** The `rgbd_camera`'s depth pass ignores `<visibility_mask>`
  and renders only `GZ_VISIBILITY_ALL` (`0x0FFFFFFF`). So the glass flag must lie above it; the
  glass visual has flags `0x10000000`, and the other optical sensors mask it out with
  `0xEFFFFFFF`. Bit 20 fails for another reason: ogre2 keeps it for particles and takes it out of
  every `gpu_lidar` mask, so not even the ultrasonics see the glass.
- **A massless link on a preserved joint is deleted.** The URDF converter drops it with its
  collision and sensor, so each bumper plank has 0.1 kg, taken out of `chassis_mass`.
- **Contacts need a world system and their own topic tag.** Every world loads
  `gz::sim::systems::Contact`, and the topic is set in `<contact><topic>`.
- **The model has no slot for the 2D lidar.** The sensor is 0.7 mm ahead of a solid front face,
  and from ±90° on the rays run into the body and return from inside it. So the scan stops at
  ±89.5°, not at the real ±100°.
- **At long range the 2D lidar sees things a little below its plane:** it reads the 9 cm bumper
  wall from 11 m. Its one row of pixels has a height. There is no such effect near the robot.
- **Python nodes on `/clock` cost half a core each,** because it runs at 500 Hz. The three
  sim-only nodes need no clock and stay off it. `sim_bumper` works on wall time.

## Simulated Unitree L2

`urdf/mecanum.gazebo.xacro` adds two sensors. Both are bridged to the real driver's topics and
frames:

| Sensor | Model | Output |
|---|---|---|
| Lidar | `gpu_lidar` on `l2_lidar`. 360 × 32 samples per sweep at 5.55 Hz, about 64k points/s. Elevation −6° to 90° in the L2 frame, the wide mode the upright mount uses (the default), or the factory standard 0° to 90° with `sim_l2_wide_fov:=false`. Range 0.05 to 30 m, noise σ 0.01 m. | `/l2/points` in `l2_lidar` |
| IMU | IMU on `l2_imu` at 250 Hz, Gaussian noise | `/l2/imu` in `l2_imu` |

The sample counts, the noise and the IMU rate are stand-ins. The sim differs from the real L2
in three ways:
- **No per-point time field.** Gazebo renders each sweep instantly, so the cloud has no `time`
  field and no motion distortion. LIO must not rely on deskewing in sim.
- **Regular grid.** The points form a regular grid, not the L2's non-repetitive pattern.
- **Full sweeps.** One message is a full sweep. The real driver's 18-scan clouds may be partial;
  this is checked in the hardware phase.

The L2 stands upright on the robot's top, in wide mode by default (`sim_l2_wide_fov` is
`true`), as the sensor placement PDF asks.

Measured in the restroom world at spawn (2026-10-27), with `tools/l2_mount_check.py`:

| Quantity | Value |
|---|---|
| Points per sweep | 11,520: every ray hits something under the ceiling |
| Points on the robot itself | 0.6% of each raw cloud (the top of the tall box), removed by the self-filter |
| Floor points | 0.2% of the raw cloud: the floor is in view only beyond about 7.3 m |
| Points below the L2's own height | 6% |
| Ceiling | Fitted 1.633 m above the L2 (ceiling 2.40 m), tilt 0.01° |
| IMU at rest | (0, 0, +9.80) m/s², as predicted by the upright URDF orientation |

## Usage

The normal entry point starts the simulation and the drive node:

```bash
ros2 launch mecanum_bringup robot.launch.py
```

This package's launch file starts Gazebo, the robot model, the bridge and, once the robot is
spawned, the controllers (`mecanum_controllers/controllers.launch.py`). It starts no
perception or LIO:

```bash
ros2 launch mecanum_simulation sim.launch.py gui:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `world` | `worlds/restroom.sdf` | World file |
| `gui` | `true` | `false` runs a headless server |
| `x`, `y`, `yaw` | `0.0`, `-1.0`, `0.0` | Spawn pose |
| `use_l2`, `use_d435i`, `use_lidar_2d`, `use_ultrasonics`, `use_cliff_tof`, `use_bumpers` | `true` | Include that sensor in the model, and start its sim-only nodes |

## Measured behaviour

These figures come from the restroom world, headless, with the robot model of 2026-10-27
(31.2 kg, 0.1025 m wheels) on ros2_control (2026-10-28). The motion test
(`motion_test.launch.py`, default `axes` pattern) is judged against `/sim/ground_truth/odom`:

| Motion | Result |
|---|---|
| Forward 0.15 m/s, 3 s | 99% of commanded travel, cross-axis drift 0.0% |
| Strafe left 0.15 m/s, 3 s | 99% of commanded travel, cross-axis drift 0.1% |
| Rotation 0.5 rad/s, 3 s | 100% of commanded rotation |
| Translation during a rotation | 5.7 cm over 86°: over the motion test's 5 cm limit, so the test stops there |
| `/wheel/odometry` against ground truth, `loop` pattern, 2 laps (24.6 m) | final 0.03 m (0.13% of the path), max 0.096 m (0.39%), yaw max 0.73° |

- **The robot walks during a rotation, and the centre of mass causes it.** The stand-in COM
  sits 6.5 cm behind the wheel centre, so the wheels carry unequal loads and the
  friction-limited rollers slip unevenly while the base turns. With the COM moved to the
  centre as a test, the rotation step showed no translation but turned 109% of command, the
  cylinder-rim effect seen on the previous model. The wheel odometry cannot see this slip,
  which is most of its error on the loop. The COM is a stand-in; measure it on the robot.
- **Stop time.** It is `reference_timeout` of 0.25 s plus one 20 ms controller period.

## Known issues

- **Exit code -2 on Ctrl-C.** Gazebo reports this at shutdown. That is upstream behaviour.
- **Start-up warnings.** gz_ros2_control logs "IMU sensor 'l2_imu' not found in hardware_info,
  skipping" (the IMUs are bridged, not ros2_control sensors; `d435i_imu` gives the same line)
  and that the 50 Hz controller period is slower than the 2 ms physics step. All are expected.
- **Big messages need a reliable subscriber.** At Linux's default socket buffers
  (`net.core.rmem_max` 212 kB) Cyclone DDS loses fragments of large best-effort messages. On the
  1 to 1.6 MB D435i topics a best-effort subscriber got 9 to 13 Hz where a reliable one got
  13.9 Hz, and on a 13 MB cloud it got nothing. So subscribe to the D435i with reliable QoS.
  This matters for the costmap layers (plan Phase 6). Raising the buffers needs `sudo sysctl`
  and a Cyclone configuration; neither is set here.
- **The L2 self-filter loses 3 to 5% of its clouds while the D435i runs.** It is the same cause:
  `self_filter` subscribes best-effort, and pcl_ros offers no way to change that. Without the
  D435i it loses 0.3%. So `tools/sensor_check.py` can fail `/l2/points_filtered`, which reads 4.7
  to 5.6 Hz in a 10 s window, while `/l2/points` holds 5.5 Hz.
- **Measured load** (headless restroom with LIO, 2026-10-30): real-time factor 1.00. Gazebo uses
  188% of a core, against 103% with only the L2. The bridge uses 35%, `sim_depth_cloud` 28% and
  each `sim_range_sensor` 11%. There is no GPU rendering here.

## Assumptions

- The model name is `mecanum` and the URDF prefix is empty. The bridge topics and the joint
  names in `mecanum_controllers/config/controllers.yaml` depend on both.
- `GZ_SIM_RESOURCE_PATH` is extended at launch, so `package://mecanum_description` meshes
  resolve without changing the description package.
