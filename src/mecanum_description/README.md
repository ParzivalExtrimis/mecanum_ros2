# mecanum_description

URDF/xacro description of the 4-wheel mecanum base and its Unitree L2 lidar mast, with meshes and
a display launch file. Every other package takes its frames, dimensions and sensor poses from
here.

## Files

| Path | Content |
|---|---|
| `urdf/mecanum.urdf.xacro` | Top-level file. Includes everything below. |
| `urdf/mecanum_params.xacro` | Dimensions and masses. Uncertain values are marked `STAND-IN` (also in the root `STANDINS.md`). |
| `urdf/mecanum_robot.xacro` | `base_footprint`, `base_link` and the chassis |
| `urdf/mecanum_wheel.xacro` | Wheel macro: `<name>_wheel_link` with a continuous joint |
| `urdf/sensors/unitree_l2.xacro` | L2 mast and bracket (CAD meshes), `l2_mount`, `l2_lidar`, `l2_imu` |
| `urdf/materials.xacro`, `urdf/inertial_macros.xacro` | Helpers |
| `meshes/visual/` | STL meshes in metres. The mast meshes come from `tools/cad_stl_to_ros.py`. |
| `launch/display.launch.py`, `rviz/display.rviz` | View the model in RViz |
| `refs/` | Reference photo of the lidar placement |

## Frames

`base_footprint` → `base_link` → the four wheel links, `l2_mast` and `l2_mount`;
`l2_mount` → `l2_lidar` and `l2_imu`. robot_state_publisher publishes all of them; nothing
else may publish TF below `base_footprint`.

## Xacro arguments

| Argument | Default | Meaning |
|---|---|---|
| `prefix` | `""` | Prefix for every link and joint name |
| `use_l2` | `true` | Include the L2 mast and sensor frames |
| `l2_mast_x`, `l2_mast_y`, `l2_mast_z`, `l2_mast_yaw` | `-0.2875`, `0`, `0.040`, `0` | Pole position on the chassis. The default is the rear edge, centred, with the L2 looking forward. |
| `l2_bracket_yaw` | `0.3996804` | L2 rotation about its own axis on the bracket (22.9°, cable toward the rear-left). See `STANDINS.md` before changing it. |

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
