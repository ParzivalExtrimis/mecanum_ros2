# mecanum_simulation

This package is the Gazebo Harmonic (gz-sim 8) simulation of the mecanum base. It contains
the restroom and corridor test worlds, the simulation-only additions to the robot description
and the ros_gz bridge.

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
| `use_l2` | `true` | Include the L2 in the model |

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
  skipping" (the L2 IMU is bridged, not a ros2_control sensor) and that the 50 Hz controller
  period is slower than the 2 ms physics step. Both are expected.

## Assumptions

- The model name is `mecanum` and the URDF prefix is empty. The bridge topics and the joint
  names in `mecanum_controllers/config/controllers.yaml` depend on both.
- `GZ_SIM_RESOURCE_PATH` is extended at launch, so `package://mecanum_description` meshes
  resolve without changing the description package.
