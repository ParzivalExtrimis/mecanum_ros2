# Encoder upgrade: migrating the drive to ros2_control

The current drive is open-loop, because the first motors had no encoders. See
[decision 0001](decisions/0001-open-loop-drive-without-ros2-control.md). The new motors have
position and velocity encoders, so the drive moves to ros2_control. In simulation that is
Phase 1 of [`plan.md`](plan.md), which follows sections 1 and 3 to 6 below; on hardware it also
needs the ESP32 to report the **measured** wheel state (section 0) and the hardware plugin
(section 2). This guide lists the steps in order. The numbers are those of the robot model v2.

**Status (2026-10-28):** the simulation steps (sections 1, 3, 5 and 6) are done; see
[decision 0002](decisions/0002-ros2-control-with-measured-wheel-state.md) and
`src/mecanum_controllers`. Two choices differ from the sketches below: the controller's
reference is remapped to `/cmd_vel` (all sources publish stamped there), and
`enable_odom_tf` is `true` until the EKF of section 4 exists (plan Phase 3). Sections 0, 2, 4,
7 and the hardware steps of 8 are open.

The names and parameters below were checked against ROS 2 Jazzy:

| Package | Version |
|---|---|
| ros2_control | 4.48 |
| ros2_controllers | 4.42.1 |
| gz_ros2_control | 1.2.20 |

Re-check them against the installed versions at upgrade time:

```bash
ros2 control list_hardware_components
```

You can also read the controller's generated parameter header at
`/opt/ros/jazzy/include/mecanum_drive_controller/`.

## 0. Firmware requirements, before any ROS work

The ESP32 must provide the following before the migration is worth doing:

- **Per-wheel position.** Report encoder ticks, plus counts per *wheel* revolution after the
  gearbox. Positions must be continuous and must not wrap between reports.
- **Per-wheel velocity.** Measure it on the ESP32, not by differencing on the Jetson, and
  report it at 50 Hz or more.
- **A timestamp or sequence number on each state report,** so stale or dropped reports can be
  detected.
- **One sign convention for all wheels.** Positive means the wheel rolls the robot forward.
  Document it per wheel, or apply it in the plugin.
- **Measured latency** from command to wheel response, and from measurement to report.
- **Documented watchdog behaviour:** the timeout, and what the motors do when it fires.
- **Recommended: speed PID on the ESP32.** Commands then become wheel speed targets in rad/s
  instead of open-loop values, and the per-wheel scale and deadband calibration largely
  disappears.

## 1. New package `mecanum_controllers`

Keep ros2_control files out of `mecanum_description`.

```
src/mecanum_controllers/
  package.xml            exec_depend: controller_manager, mecanum_drive_controller,
                         joint_state_broadcaster, gz_ros2_control, mecanum_hardware
  urdf/mecanum.ros2_control.xacro
  config/controllers.yaml
  launch/controllers.launch.py
  README.md
```

The `urdf/mecanum.ros2_control.xacro` file has one `<ros2_control type="system">` block. Each
wheel joint, `<prefix>{front,rear}_{left,right}_wheel_joint`, gets a velocity command
interface and position and velocity state interfaces. A `hardware` xacro arg selects the
plugin:

| `hardware` | Plugin | Use |
|---|---|---|
| `mock` | `mock_components/GenericSystem` | Dry runs. Commands are echoed to states. |
| `sim` | `gz_ros2_control/GazeboSimSystem` | Gazebo, with real wheel states from physics. |
| `esp32` | `mecanum_hardware/Esp32System` | The real robot. |

Sketch:

```xml
<ros2_control name="mecanum_base" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>   <!-- chosen by the hardware arg -->
  </hardware>
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... the other three wheels ... -->
</ros2_control>
```

Include this xacro from the simulation and bringup top-level xacros, not from the description.

### `config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController

mecanum_drive_controller:
  ros__parameters:
    front_left_wheel_command_joint_name: front_left_wheel_joint
    front_right_wheel_command_joint_name: front_right_wheel_joint
    rear_left_wheel_command_joint_name: rear_left_wheel_joint
    rear_right_wheel_command_joint_name: rear_right_wheel_joint
    front_left_wheel_state_joint_name: front_left_wheel_joint
    front_right_wheel_state_joint_name: front_right_wheel_joint
    rear_left_wheel_state_joint_name: rear_left_wheel_joint
    rear_right_wheel_state_joint_name: rear_right_wheel_joint
    kinematics:
      wheels_radius: 0.1025
      sum_of_robot_center_projection_on_X_Y_axis: 0.451
      base_frame_offset: {x: 0.0, y: 0.0, theta: 0.0}
    reference_timeout: 0.25
    enable_odom_tf: false        # the EKF owns odom -> base_footprint (step 4)
    odom_frame_id: odom
    base_frame_id: base_footprint
```

`sum_of_robot_center_projection_on_X_Y_axis` is lx + ly, which is 0.2360 + 0.2150.

Carry each setting over from the current driver config:

| Current file | Current setting | Controller setting |
|---|---|---|
| `mecanum_base/config/esp32_driver.yaml` | `wheel_radius` | `kinematics.wheels_radius` |
| `mecanum_base/config/esp32_driver.yaml` | `lx` + `ly` | `kinematics.sum_of_robot_center_projection_on_X_Y_axis` |
| `mecanum_description/urdf/mecanum_params.xacro` | wheel-rectangle centre 0.3 mm ahead of `base_link` (`front_axle_x` 0.2363, `rear_axle_x` −0.2357) | `kinematics.base_frame_offset.x`: 0, the offset is negligible. Check the sign in the parameter description before setting a non-zero one. |
| `mecanum_base/config/esp32_driver.yaml` | `max_vx`, `max_vy`, `max_wz` | `linear.x.*`, `linear.y.*`, `angular.z.*` limits |
| `mecanum_base/config/esp32_driver.yaml` | `cmd_timeout` | `reference_timeout` |
| `mecanum_base/config/wheel_calibration.yaml` | sign, scale, deadband, max, motor_id | Parameters of the hardware plugin, not the controller |

### Controller topics

| Topic | Type | Notes |
|---|---|---|
| `~/reference` | `geometry_msgs/TwistStamped` | The input, remapped to `/cmd_vel`. Stale stamps are rejected by `reference_timeout`. |
| `~/odometry` | `nav_msgs/Odometry` | Remapped to `/wheel/odometry`. |
| `~/tf_odometry` | `tf2_msgs/TFMessage` | Remapped to `/tf`; used while `enable_odom_tf` is true (until the EKF). |

## 2. New package `mecanum_hardware`

This is a C++ ament_cmake package with pluginlib. It provides a
`hardware_interface::SystemInterface` that is exported as `mecanum_hardware/Esp32System`.

| Method | Responsibility |
|---|---|
| `on_init` | Read the joint list, ESP32 address and calibration from the `<hardware>` params. Validate four joints with the expected interfaces. |
| `on_configure` / `on_cleanup` | Open and close the ESP32 link. |
| `on_activate` | Zero all commands, read the initial positions, and start. |
| `on_deactivate` / `on_shutdown` / `on_error` | Send zeros to all motors. |
| `export_state_interfaces` | Position and velocity per wheel. |
| `export_command_interfaces` | Velocity per wheel. |
| `read()` | Take the latest ESP32 state report. Convert ticks to rad and rad/s, apply per-wheel sign, and check its age. Return an error if the report is stale. |
| `write()` | Convert wheel rad/s to the ESP32 command. That is a rad/s target if the firmware runs PID. Otherwise it is the sign, scale, deadband and max mapping. Then send. |

Port this logic from `mecanum_base`:
- `mecanum_base/wheel_calibration.py` provides the command mapping, if still needed.
- `mecanum_base/transport.py`'s `Esp32Transport` provides the protocol.

Keep the Python unit tests' numeric cases as test vectors for the C++ port.

## 3. Switch velocity commands to TwistStamped

- **Nav2.** Set `enable_stamped_cmd_vel: true` on every node that publishes or subscribes to
  velocity. That means `controller_server`, `velocity_smoother`, `collision_monitor`,
  `behavior_server` and `docking_server`. The file is
  `mecanum_navigation/config/nav2_params.yaml`.
- **Collision monitor.** Its output, the final velocity in the chain, goes to `/cmd_vel`,
  which the safety gate (plan Phase 4) passes on to the controller.
- **Teleop.** Run it stamped:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
  ```
- **Motion test.** Set `cmd_vel_stamped: true` on `motion_test`.

## 4. Odometry and TF ownership

Wheel odometry is now real, but mecanum rollers slip, especially sideways. Fuse it with LIO
rather than replacing LIO.

- **Add a robot_localization EKF** with config in `mecanum_localization/config/ekf.yaml`
  and `two_d_mode: true`. Its inputs:
  - LIO odometry, `/lio/odom` from the adapter: x, y and yaw pose plus vx, vy and wz.
  - Wheel odometry, `/wheel/odometry`: **twist only**, vx, vy and wz. Inflate the covariance
    on vy.
- **The EKF publishes `odom -> base_footprint`** and `/odom`.
- **The LIO adapter gets `publish_tf: false`.** It publishes on `/lio/odom` instead of
  `/odom`.
- **The mecanum controller keeps `enable_odom_tf: false`.**
- **Update the TF ownership table** in the root `README.md`.

The rule against publishing wheel *commands* as odometry still holds. `/wheel/odometry` is
now computed from measured states.

## 5. Joint states

- **Remove joint_state_publisher** from all launch files. It currently publishes zero wheel
  angles.
- **Spawn `joint_state_broadcaster`.** It publishes `/joint_states` from the state
  interfaces, and robot_state_publisher shows the wheels turning.
  - In simulation, the states come from Gazebo's physics through `GazeboSimSystem`, inside
    the Gazebo process. **ros_gz_bridge is not involved.**
  - On hardware, `read()` fills the state interfaces each control cycle. The broadcaster
    publishes them. `read()` itself publishes nothing.

## 6. Simulation changes

In `mecanum_simulation/urdf/mecanum.gazebo.xacro`:
- Remove the four `JointController` plugin blocks.
- Add the gz_ros2_control plugin, and include `mecanum.ros2_control.xacro` with
  `hardware:=sim`:
  ```xml
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find mecanum_controllers)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>
  ```
- Keep the anisotropic wheel friction and the ground-truth OdometryPublisher unchanged.

In `gz_bridge.yaml` and `sim.launch.py`:
- Remove the `/sim/wheel_cmd/*` bridges and the `sim_motor_plant` node. The controller drives
  the joints directly.
- Spawn the controllers after the robot appears in Gazebo, for example with
  `OnProcessExit` on the `create` node.

## 7. Retire or repurpose

| Item | Action |
|---|---|
| `mecanum_base/esp32_driver_node.py`, `drive_core.py`, `transport.py` | Remove after the C++ plugin is in service. The protocol logic moves into `mecanum_hardware`. |
| `mecanum_base/kinematics.py` and its tests | Keep as a reference to cross-check the controller's inverse kinematics. |
| `mecanum_base/motion_test.py` | Keep. Set it stamped and point it at the controller reference. |
| `mecanum_base/config/wheel_calibration.yaml` | Move the values that remain into the hardware plugin parameters (section 2), then remove it with the driver. With firmware PID there is no open-loop scale or deadband to fit; `motion_test` checks wheel speed tracking. |
| `mecanum_simulation/sim_motor_plant.py` | Removed in plan Phase 1 (2026-10-28), with `plant_model.py`, `sim_plant.yaml`, the `sim` transport and `MotorCommands`. |
| `mecanum_bringup/launch/*` | Include `controllers.launch.py`. Drop the driver node and joint_state_publisher. |
| `STANDINS.md` | Remove the calibration stand-ins that no longer apply. Add the encoder counts per revolution until they are confirmed. |

## 8. Verification order

1. **Mock.** Run with `hardware:=mock`. Check that the controllers load, that
   `/joint_states` shows the four wheels, and that a TwistStamped on the reference turns the
   commanded joint velocities with the right signs. Use the existing kinematics test vectors.
2. **Gazebo.** Run with `hardware:=sim`. Run `motion_test` and compare `/wheel/odometry`
   against `/sim/ground_truth/odom`. Then check that the EKF output beats both inputs.
3. **Hardware on blocks, wheels in the air.** Check that each wheel's sign and position
   counting is consistent with a hand rotation, and that the watchdog stops the motors.
4. **Hardware on the floor.** Run `motion_test`, then the calibration and validation run,
   then mapping and navigation regression runs.
