# mecanum_simulation

This package is the Gazebo Harmonic (gz-sim 8) simulation of the mecanum base. It contains
the restroom test world, the simulation-only additions to the robot description, the ros_gz
bridge and a simulated ESP32 with its motors.

The simulation is as blind as the real robot. Wheel joint states are never published or
used for odometry, because the real motors have no encoders. See
[decision 0001](../../docs/decisions/0001-open-loop-drive-without-ros2-control.md).

## Data flow

```
esp32_driver (mecanum_base, transport:=sim)
  -> /esp32/motor_commands        mecanum_interfaces/MotorCommands (motor_id, direction, value)
  -> sim_motor_plant              simulated ESP32 + motors: wiring table, per-wheel plant model
  -> /sim/wheel_cmd/<wheel>       std_msgs/Float64, wheel rad/s
  -> ros_gz_bridge
  -> /model/mecanum/joint/<wheel>_wheel_joint/cmd_vel   gz JointController (velocity mode)
  -> wheel joints spin -> anisotropic roller friction -> the chassis moves

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

## Simulated ESP32 and motors (`sim_motor_plant`)

This node maps each motor ID to a wheel through `wiring.<wheel>`. It then applies the "true"
plant for that wheel:

```
omega = sign * direction * (value - deadband) / scale,   |omega| <= max_omega
```

The default `config/sim_plant.yaml` is ideal and matches the driver's stand-in calibration.
Changing scale, deadband, sign or wiring injects realistic errors, which tests the motion test
and the calibration tool. `watchdog_timeout`, the simulated firmware watchdog, is off by
default. Whether the real firmware has one is unknown.

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

A 2.4 m ceiling is a separate model, `ceiling`. The simulated L2 sees it. To look at the robot
from above in the Gazebo GUI, make it transparent: in the entity tree, right-click `ceiling`,
then choose View > Transparent.

## World: `worlds/corridor.sdf`

A test world for the LIO degeneracy check in phase 4, not a site model. It is a straight,
featureless corridor, 2 m wide and 70 m long, with a 2.4 m ceiling: x −35 to 35, y −2 to 0.
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
| Lidar | `gpu_lidar` on `l2_lidar`. 360 × 32 samples per sweep at 5.55 Hz, about 64k points/s. Elevation 0° to 90° in the L2 frame, the factory standard FOV, or −6° to 90° with `sim_l2_wide_fov:=true`. Range 0.05 to 30 m, noise σ 0.01 m. | `/l2/points` in `l2_lidar` |
| IMU | IMU on `l2_imu` at 250 Hz, Gaussian noise | `/l2/imu` in `l2_imu` |

The sample counts, the noise and the IMU rate are stand-ins. The sim differs from the real L2
in three ways:
- **No per-point time field.** Gazebo renders each sweep instantly, so the cloud has no `time`
  field and no motion distortion. LIO must not rely on deskewing in sim.
- **Regular grid.** The points form a regular grid, not the L2's non-repetitive pattern.
- **Full sweeps.** One message is a full sweep. The real driver's 18-scan clouds may be partial;
  this is checked in the hardware phase.

Measured in the restroom world at spawn:

> **REWRITE (branch `feature/model-v2`):** measured with the previous model and L2 mast. Redo
> after the new URDF exists. See the root `REWRITE.md`.

| Quantity | Value |
|---|---|
| Points on the robot itself | 25% of each raw cloud, removed by the self-filter |
| Floor points | About 45% of the raw cloud |
| IMU at rest | (4.52, −1.88, −8.49) m/s², as predicted by the URDF orientation |

## Usage

The normal entry point starts the simulation and the drive node:

```bash
ros2 launch mecanum_bringup robot.launch.py
```

This package's launch file starts only Gazebo, the robot model, the bridge and the simulated
ESP32:

```bash
ros2 launch mecanum_simulation sim.launch.py gui:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `world` | `worlds/restroom.sdf` | World file |
| `gui` | `true` | `false` runs a headless server |
| `x`, `y`, `yaw` | `0.0`, `-1.0`, `0.0` | Spawn pose |
| `use_l2` | `true` | Include the L2 in the model |
| `plant_file` | `config/sim_plant.yaml` | Plant model; swap it to inject motor errors |

## Measured behaviour (phase 2, ideal plant)

These figures come from the motion test and probes against `/sim/ground_truth/odom`:

> **REWRITE (branch `feature/model-v2`):** measured with the previous model's wheels (cylinder
> collision, 9.5 mm wheel-centre offset). Redo after the new URDF exists.

| Motion | Result |
|---|---|
| Forward 0.15 to 0.25 m/s | 100% of commanded travel, cross-axis drift below 0.1% |
| Strafe left 0.15 to 0.25 m/s | 100% of commanded travel, cross-axis drift below 0.1% |
| Rotation 0.3 to 0.8 rad/s | **110.2%** of commanded rate at every speed |
| Translation during a rotation | 1.4 cm over about 95° |
| Stop after `cmd_vel` stops | 0.34 s |

- **Rotation is 10% fast.** The effective lever arm is 0.04 m shorter than lx + ly, which is
  half the wheel width. The cylinder collision most likely touches the floor near a rim edge
  instead of at the wheel mid-plane. A sphere collision would avoid this. The gazebo merge
  mechanism cannot replace the geometry, so that change would go in the wheel xacro of
  `mecanum_description` itself; it is an option for phase 5.
  - Real mecanum wheels also rotate at a rate different from the ideal, because the contact
    point moves along each roller.
  - The phase 5 calibration fits a per-axis scale, which covers both cases.
- **Translation during a rotation is expected.** The kinematics rotate about the wheel centre,
  which is 9.5 mm behind `base_link`.
- **The stop time adds up as expected.** It is `cmd_timeout` of 0.25 s, plus one 50 ms
  control period, plus about 40 ms of bridge and physics latency.

## Known issues

- **Slow shutdown of `sim_motor_plant`.** It sometimes takes about 5 s to exit after Ctrl-C.
  Launch then logs "failed to terminate ... escalating to SIGTERM", followed by "finished
  cleanly".
  - A stack dump shows the main thread inside rclpy's `Publisher.destroy()` while it destroys
    the node.
  - The likely cause is Cyclone DDS waiting for acknowledgements of the last wheel commands
    from ros_gz_bridge, which is exiting at the same time.
  - It only affects simulation shutdown time. The stock `joint_state_publisher` shows the
    same kind of hang in its own destroy call.
- **Exit code -2 on Ctrl-C.** Gazebo and `joint_state_publisher` report this at shutdown.
  That is upstream behaviour.

## Assumptions

- The model name is `mecanum` and the URDF prefix is empty. The bridge topics depend on both.
- `GZ_SIM_RESOURCE_PATH` is extended at launch, so `package://mecanum_description` meshes
  resolve without changing the description package.
