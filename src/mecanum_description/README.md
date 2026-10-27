# mecanum_description

URDF/xacro description of the 4-wheel mecanum base and its Unitree L2 lidar, with meshes and a
display launch file. Every other package takes its frames, dimensions and sensor poses from
here.

> **REWRITE (branch `feature/model-v2`):** the previous model's meshes, dimensions, masses and
> L2 mast were removed for the new model. The URDF is a frame skeleton: the link and joint names
> and the tree are kept, but there are no meshes, poses, sizes or masses, apart from the L2's
> vendor values. What was removed, and what must change once the new meshes and URDF exist, is
> listed in the root [`REWRITE.md`](../../REWRITE.md).

## Files

| Path | Content |
|---|---|
| `urdf/mecanum.urdf.xacro` | Top-level file. Includes everything below. |
| `urdf/mecanum_params.xacro` | Dimensions and masses. Empty until the rewrite; its header lists what the new geometry must provide. |
| `urdf/mecanum_robot.xacro` | `base_footprint`, `base_link` and the four wheel calls |
| `urdf/mecanum_wheel.xacro` | Wheel macro: `<name>_wheel_link` with a continuous joint |
| `urdf/sensors/unitree_l2.xacro` | `l2_mount`, `l2_lidar`, `l2_imu` with the L2's vendor values. The new mount is still to be added. |
| `urdf/materials.xacro`, `urdf/inertial_macros.xacro` | Helpers |
| `meshes/visual/`, `meshes/collision/` | STL meshes in metres. Empty until the new meshes are added. |
| `launch/display.launch.py`, `rviz/display.rviz` | View the model in RViz |

## Frames

`base_footprint` → `base_link` → the four wheel links and `l2_mount`; `l2_mount` → `l2_lidar` →
`l2_imu`. robot_state_publisher publishes all of them; nothing else may publish TF below
`base_footprint`. In the skeleton every joint has an identity origin, so all frames coincide.

## Xacro arguments

| Argument | Default | Meaning |
|---|---|---|
| `prefix` | `""` | Prefix for every link and joint name |
| `use_l2` | `true` | Include the L2 sensor frames |

```bash
xacro $(ros2 pkg prefix mecanum_description)/share/mecanum_description/urdf/mecanum.urdf.xacro \
  use_l2:=true
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

`colcon test --packages-select mecanum_description` runs the ament lint suite.
