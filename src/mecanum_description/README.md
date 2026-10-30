# mecanum_description

URDF/xacro description of the washroom robot prototype: a 4-wheel mecanum base with its
sensors, and a display launch file. Every other package takes its frames, dimensions and sensor
poses from here.

The model is built from boxes and cylinders, with no meshes. It follows the reference URDF
[`refs/washroom_robot.urdf`](refs/washroom_robot.urdf), which was generated from the STEP model
and the sensor placement PDF
([`docs/Washroom_Robot_Prototype_Sensors_and_BOM.pdf`](../../docs/Washroom_Robot_Prototype_Sensors_and_BOM.pdf)).
Where the two sources disagree, the reference URDF wins; its header lists every difference.
[`docs/cad_vs_urdf.png`](../../docs/cad_vs_urdf.png) compares the CAD with the URDF.
`test/test_reference_urdf.py` checks that every frame here matches the reference.

## Files

| Path | Content |
|---|---|
| `urdf/mecanum.urdf.xacro` | Top-level file. Includes everything below. |
| `urdf/mecanum_params.xacro` | Wheel and body dimensions, masses and inertias |
| `urdf/mecanum_robot.xacro` | `base_footprint`, `base_link`, `chassis_link` (18 boxes for visual and collision) and the four wheel calls |
| `urdf/mecanum_wheel.xacro` | Wheel macro: `<name>_wheel_link` with a continuous joint and one cylinder collision |
| `urdf/sensors/unitree_l2.xacro` | `l2_mount`, `l2_lidar`, `l2_imu` |
| `urdf/sensors/realsense_d435i.xacro` | `d435i_link`, `d435i_optical_frame` |
| `urdf/sensors/lidar_2d.xacro` | `lidar_2d_link` (RPLIDAR C1) |
| `urdf/sensors/ultrasonics.xacro` | `ultrasonic_{front,rear}_{left,right}_link` |
| `urdf/sensors/cliff_tof.xacro` | `cliff_tof_{front,rear}_{left,right}_link` |
| `urdf/sensors/bumpers.xacro` | `bumper_{front,rear}_link` |
| `urdf/materials.xacro`, `urdf/inertial_macros.xacro` | Helpers. The colours match the PDF callouts. |
| `refs/washroom_robot.urdf` | The reference URDF, kept unchanged |
| `meshes/visual/`, `meshes/collision/` | Empty. The model uses primitives only. |
| `launch/display.launch.py`, `rviz/display.rviz` | View the model in RViz |

## Robot

| Quantity | Value |
|---|---|
| Body envelope | 0.79 × 0.51 m, 0.737 m tall (the top of the tall box). 0.832 m to the top of the L2. |
| Overall extent, x | −0.404 m (rear bumper) to +0.420 m (2D lidar) |
| Overall extent, y | ±0.258 m (the wheels) |
| Wheel radius, width | 0.1025 m, 0.085 m |
| Wheel centres | x +0.2363 / −0.2357 m, y ±0.2150 m: lx = 0.2360 m, ly = 0.2150 m |
| Mass | 31.23 kg: chassis 25 kg, wheels 1.5 kg each, L2 0.23 kg (all STAND-IN except the L2) |

## Frames

Every frame is published by robot_state_publisher; nothing else may publish TF below
`base_footprint`. All joints are fixed except the wheels.

```
base_footprint                       on the floor, below the wheel-rectangle centre
└─ base_link                         at axle height, 0.1025 m
   ├─ chassis_link                   body envelope, carries the body mass
   ├─ {front,rear}_{left,right}_wheel_link   continuous, axis +y, positive = rolls forward
   ├─ l2_mount → l2_lidar → l2_imu
   ├─ d435i_link → d435i_optical_frame
   ├─ lidar_2d_link
   ├─ ultrasonic_{front,rear}_{left,right}_link   +x along the beam
   ├─ cliff_tof_{front,rear}_{left,right}_link    +x straight down
   └─ bumper_{front,rear}_link                    +x outward
```

| Sensor | Frame | Position in `base_footprint` (m) | Orientation |
|---|---|---|---|
| Unitree L2 | `l2_mount` (bottom face) | (−0.100, 0, 0.767) | Upright, on a 30 mm standoff on the top of the tall box. Housing centre at z 0.799. |
| RealSense D435i | `d435i_link` (front face) | (0.404, 0, 0.264) | Facing +x, pitched 8° down |
| RPLIDAR C1 | `lidar_2d_link` (scan centre) | (0.400, 0, 0.130) | Horizontal scan in a 200° front slot |
| Ultrasonic ×4 | `ultrasonic_*_link` | front (0.408, ±0.191, 0.296), rear (−0.401, ±0.211, 0.298) | Front pair +x, rear pair −x |
| Cliff ToF ×4 | `cliff_tof_*_link` | (±0.354, ±0.208, 0.119) | Straight down |
| Bumpers | `bumper_*_link` | (±0.392, 0, 0.076) | Outward |

Not modelled: the E-stop (rear face of the tall box), the arm (not in the CAD yet), the
internal electronics and the mecanum rollers. The roller handedness (X layout) is set in
`mecanum_base/kinematics.py` and the simulation's friction directions.

## Check on the robot

Each of these is a `STAND-IN` in the xacro and in the root `STANDINS.md`:
- the total mass and the centre of mass;
- the loaded wheel radius;
- the L2 height and its cable direction, which sets `l2_cable_yaw`;
- the D435i pitch;
- the 2D lidar's scan height and its 0° direction;
- the cliff ToF height above the floor: 0.119 m in the STEP, 0.08 m in the PDF;
- the bumper protrusion. In the STEP the 2D lidar sticks out 16 mm past the front plank, so the
  plank cannot hit first as the PDF intends.

## Xacro arguments

| Argument | Default | Meaning |
|---|---|---|
| `prefix` | `""` | Prefix for every link and joint name |
| `use_l2` | `true` | Include the L2 frames |
| `use_d435i` | `true` | Include the D435i frames |
| `use_lidar_2d` | `true` | Include the 2D lidar frame |
| `use_ultrasonics` | `true` | Include the four ultrasonic frames |
| `use_cliff_tof` | `true` | Include the four cliff ToF frames |
| `use_bumpers` | `true` | Include the two bumper planks |

```bash
xacro $(ros2 pkg prefix mecanum_description)/share/mecanum_description/urdf/mecanum.urdf.xacro \
  > /tmp/mecanum.urdf && check_urdf /tmp/mecanum.urdf
```

## Launch

```bash
ros2 launch mecanum_description display.launch.py            # RViz + joint slider GUI
ros2 launch mecanum_description display.launch.py gui:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | joint_state_publisher_gui instead of joint_state_publisher |
| `rviz` | `true` | Start RViz |
| `prefix` | `""` | Passed to the xacro |
| `use_sim_time` | `false` | Use the `/clock` topic |

Gazebo additions (sensors, friction, plugins) live in `mecanum_simulation/urdf/`, not here.

## Tests

`colcon test --packages-select mecanum_description` runs the ament lint suite and
`test_reference_urdf.py`. That test checks:
- every reference frame against the xacro, to within 0.1 mm. The wheels are allowed 0.25 mm in
  y, because they are made symmetric here;
- one collision per wheel;
- the total mass.
