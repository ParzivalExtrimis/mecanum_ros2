# Unitree L2 on hardware: configuration, mount verification and sim comparison

> **Paused.** Hardware is not planned while the end-to-end simulation build
> ([`plan.md`](plan.md)) runs. This procedure was updated on 2026-10-27 for the robot model v2:
> the L2 stands upright on a 30 mm standoff on the top of the robot. Re-read the plan's sensor
> contract before running it, because the simulation build may have changed what it tests.

This procedure takes the real L2 from the desk to the robot. It makes sure that everything
built in simulation still holds on hardware: the frames, the mount pose, the stream format, the
self-filter and the field of view.

It has four stages. Do them in order, because each one assumes the previous one passed.

| Stage | Where | Proves |
|---|---|---|
| 1. Configure and bench test | L2 on a desk, Ethernet | The driver works, and the L2 is configured to match the sim |
| 2. Mount verification | L2 on the robot, robot still on a flat floor under a flat ceiling | The real mount matches the URDF |
| 3. Sim against real | Same setup, plus a few props | The sim's sensor model is close enough, so stand-ins can be replaced |
| 4. Moving | Later hardware work | LIO, mapping and navigation on hardware |

Stage 4 is covered by the phase test lists, not here.

The main tool is `tools/l2_mount_check.py`. It reads `/l2/points`, `/l2/imu` and the URDF TF,
and prints a report with an OK or WARN verdict per check. The upright L2 sees the floor only
from about 7 m out, so the tool fits the **ceiling** by default (`--plane ceiling`); given a
tape-measured `--ceiling-height`, the ceiling also gives the L2's height. Validated against the
simulation (2026-10-27), where the true pose is known: height 0.767 m against 0.767 m, tilt
0.01° against 0°, wall heading −0.06°, wall distance 1.002 m against 1.0 m.

## 0. What the simulation assumes

The sim's L2 behaves as listed below. The real L2 must be configured or measured to match. Any
remaining difference goes into `STANDINS.md` and the sim model.

| Assumption | Sim value | Real L2 setting or check |
|---|---|---|
| Connection | n/a | Ethernet/UDP, factory addresses: L2 192.168.1.62, host 192.168.1.2. Work-mode bit 3 = 0. USB-UART crashed the dev laptop's USB controller. |
| Field of view | **Wide:** elevation −6° to 90° in the L2 frame (`sim_l2_wide_fov` defaults to `true`), the PDF's negative-angle mode | Work-mode bit 0 = 1. For standard mode (bit 0 = 0, 0° to 90°), launch the sim with `sim_l2_wide_fov:=false`. |
| Scan mode | 3D | Work-mode bit 1 = 0 |
| IMU | On | Work-mode bit 2 = 0 |
| Start | Spins at power-on | Work-mode bit 4 = 0 |
| **Work mode** | n/a | **1** for Ethernet, wide FOV, 3D, IMU on, auto start: the robot's mode. **0** is the factory mode, the same with standard FOV. |
| One message | **One full 360° sweep at 5.55 Hz** | `cloud_scan_num`. The default of 18 scans covers only about 166° at about 12 Hz. A revolution is 216 Hz vertical scans divided by 5.55 Hz, about **39 scans**. Verify in stage 1. |
| Points per sweep | 11,520, on a regular 360 × 32 grid | Measured in stage 1. The real pattern is non-repetitive. |
| Range | 0.05 to 30 m | `range_min: 0.05`, `range_max: 30.0`. The driver defaults are 0 and 100. |
| Timestamps | Simulation clock | `use_system_timestamp: true`, the default |
| Per-point `time` field | **Absent**: Gazebo renders instantly | Present on hardware, spanning about 180 ms per full sweep. LIO deskews with it. |
| Frames | `l2_lidar`, `l2_imu` | `cloud_frame_id`, `imu_frame_id`, the defaults |
| IMU units | m/s², rad/s | The driver converts. Check that the norm is about 9.8 at rest. |
| IMU rate | 250 Hz | Measured in stage 1 |
| Noise | Range σ 0.01 m; gyro σ 0.002 rad/s; accel σ 0.02 m/s² | Measured in stages 1 and 2 |
| Mount | Upright on a 30 mm standoff on the tall box, bottom face 0.767 m above the floor, 0.100 m behind the robot centre, on the centreline. Cable toward the rear (`l2_cable_yaw` 0, a stand-in). | Measured in stage 2 |

The driver parameters live in `src/mecanum_lidar/config/l2.yaml`. Once stage 1 has confirmed
them, set them there so they apply everywhere:

```yaml
    work_mode: -1          # leave as is after the one-time write below
    cloud_scan_num: 39     # full sweep per message; confirm in stage 1
    range_min: 0.05
    range_max: 30.0
```

## 1. Configure and bench test (L2 on a desk)

Follow "Ethernet setup" in `src/mecanum_lidar/README.md` for the network profile. Then:

1. **Set the work mode to 1: Ethernet, wide FOV.** The factory setting is 0 (standard FOV).
   Write mode 1 once over Ethernet, or over serial if the L2 was switched to serial, then
   power-cycle:
   ```bash
   ros2 run mecanum_lidar l2_driver --ros-args -p work_mode:=1
   ros2 run mecanum_lidar l2_driver --ros-args -p connection:=serial \
     -p serial_port:=/dev/ttyACM0 -p work_mode:=1     # if it is in serial mode
   ```
   After the power cycle, with the Ethernet cable in, `ping 192.168.1.62` answers, and the
   driver logs `Receiving data from the L2`.
2. **Find `cloud_scan_num` for full sweeps.** Start the driver with a candidate value:
   ```bash
   ros2 launch mecanum_lidar l2.launch.py robot_model:=true rviz:=false
   ros2 param set /l2_driver cloud_scan_num 39   # if not settable at runtime, edit l2.yaml and relaunch
   tools/l2_mount_check.py --duration 15
   ```
   On the desk, ignore the mount rows; they assume the L2 is on the robot. Read the
   **azimuth per cloud** and **cloud rate** rows:
   - **Target.** About 360° per cloud at about 5.5 Hz.
   - **If it is less.** Use `new = current × 360 / coverage`.
   - **If it is more than one turn,** for example rate below 5 Hz, decrease the value.
   - **Record.** Note the **points per cloud** and the **per-point time span**. The span should
     be about 180 ms for a full sweep.
3. **Check the IMU axes and units.** Place the L2 upright and level. In the report:
   - |accel| should be about 9.81. A value of 1.0 means the units are g, which is a driver bug.
   - IMU tilt from vertical should be about 0°. The tool measures the tilt of the L2's Z axis
     from the vertical, up or down.
   - Tip the L2 so the side away from the cable rises: accel x must go positive. That is the
     +X convention the URDF's `l2_cable_yaw` relies on.
4. **Measure IMU noise.** Leave the L2 untouched on a solid surface:
   ```bash
   tools/l2_mount_check.py --duration 180
   ```
   Copy the **IMU rate**, **gyro std** and **accel std** into the stage 3 table.
5. **Check the cable and +X direction.** In RViz with fixed frame `l2_lidar`, an object placed
   in front of the side away from the cable must appear along the red +X axis.

**Pass:** the driver streams, `cloud_scan_num` gives about 360° per cloud, |accel| is about 9.8,
and the IMU axes follow the vendor convention.

## 2. Mount verification (L2 on the robot)

**Setup:**
- Put the robot on a flat, level floor under a flat ceiling; check the floor with a spirit
  level, and tape-measure the ceiling height at the robot.
- Park it **parallel to a long straight wall**, about 1 m to its right or left. Use a wall
  without skirting cabinets.
- Keep at least 3 m clear ahead.
- Nobody stands near the robot.
- Power the L2 for 2 minutes first, so the IMU warms up.

### 2a. Physical measurements (tape, angle gauge)

Measure these before trusting any software. Each one checks a value the URDF assumes:

| Quantity | Design (URDF) | How to measure |
|---|---|---|
| Tall box top above the floor | 0.737 m | Tape, floor to the box top next to the standoff |
| Standoff height | 30 mm | Tape or calipers |
| L2 bottom face above the floor | 0.767 m | The two above added; a stand-in, because the STEP sinks the L2 40 mm into the box |
| L2 position | Centre 0.100 m behind the robot centre (the wheel-rectangle centre), on the centreline | Square and tape from the front axle line |
| L2 level | Upright, 0° tilt | Angle gauge on the L2 top, zeroed on the floor |
| Cable direction | Toward the rear (vendor +X forward) | Look from straight above. Set any other direction as `l2_cable_yaw`, counter-clockwise positive. |

### 2b. Software check

Start the driver, the URDF TF, the self-filter, the scan conversion and the sensors RViz view.
This is the same view as the simulation's `robot.launch.py rviz:=true`:

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py
```

**Look in RViz first:**
- The ceiling is level, at its tape-measured height.
- Walls are vertical and straight.
- `/l2/points_filtered` has no points on the top of the robot.

Then run the check. Tape-measure the wall distance from the robot's **centreline**, which is
the body's side face plus 0.255 m:

```bash
tools/l2_mount_check.py --duration 20 --wall-side right --wall-distance 1.03 --ceiling-height 2.70
```

### 2c. Reading the report

The tolerances below are what the navigation stack can absorb. At 3 m range, 0.5° of tilt error
moves a point by 2.6 cm.

| Row | Tolerance | If WARN |
|---|---|---|
| L2 height above floor (from the ceiling) | ±2 cm | The box top or standoff differs from the design. Check 2a. Fix the hardware, or set `l2_mount_z` in `unitree_l2.xacro`. The tolerance includes the tape measurement of the ceiling. |
| L2 tilt from vertical, from the ceiling | ±0.5° | The standoff or the box top is not level. Shim the standoff, rather than encoding a tilt in the URDF. |
| Mount roll or pitch error | ±0.5° | The same error split into its two directions. A ceiling that is not level shows here too: check the floor with the spirit level and the ceiling with a laser line. |
| IMU tilt from vertical | ±1° | It should agree with the ceiling tilt within about 0.5°. If only the IMU disagrees, the floor isn't level or the robot is rocking. |
| L2 spin about own axis | n/a | Not measurable from gravity on an upright mount. The wall heading gives the yaw. |
| Wall heading | ±0.5° | First make sure the robot really is parallel to the wall, then rerun. If it persists, the L2 is rotated on its standoff: set the measured value in `l2_cable_yaw`, or turn the L2. |
| Wall distance | ±2 cm | A sideways L2 offset (`l2_mount_y`). Check 2a. |
| Points just outside the box | about 0 | Self-hits the box misses, for example cables or the arm. Look in RViz, then grow `self_filter.yaml`. |

**Which to change, the hardware or the URDF?**
- **Fix the hardware** when it deviates from the CAD design: a loose bolt, a tilted standoff, a
  wrong cable direction.
- **Change the URDF** only when the design itself is different and the measurement repeats
  after re-tightening. Do that through the properties in `unitree_l2.xacro` (`l2_mount_*`,
  `l2_cable_yaw`).

Rerun until the report has no warnings.

**Pass:** zero warnings with the robot parallel to the wall. Ceiling and IMU tilt agree within
0.5°.

## 3. Sim against real

Run the same tool against the sim at the same spot type: the spawn point is 1.0 m from the
corridor wall on the right. Fill in the table:

```bash
ros2 launch mecanum_bringup robot.launch.py gui:=false
tools/l2_mount_check.py --use-sim-time --wall-side right --wall-distance 1.0 --ceiling-height 2.4
```

| Row | Sim | Real | Action if they differ |
|---|---|---|---|
| Cloud rate | 5.55 Hz | | Tune `cloud_scan_num` (stage 1) |
| Azimuth per cloud | 360° | | Tune `cloud_scan_num` (stage 1) |
| Points per cloud | 11,520 | | Set the gpu_lidar samples in `mecanum_simulation/urdf/mecanum.gazebo.xacro` to about the same count, for example 360 × N |
| Points on the robot | about 0.6% | | A big difference means a frame or mount error, or something on the robot top (cables, the arm); go back to stage 2 |
| Ceiling residual (range noise) | 0.009 m | | Set the gpu_lidar noise `stddev` |
| IMU rate | 250 Hz | | Set the IMU `update_rate` |
| Gyro std, accel std | 0.002, 0.02 | | Set the IMU noise |
| Per-point time span | none | about 180 ms | Expected difference. LIO deskews on hardware only. |

After each change, update the matching row in `STANDINS.md`. Remove a row once its value is
measured.

### 3a. Field of view

The L2 stands upright with its bottom face 0.767 m above the floor. In wide mode it sees the
hemisphere above its horizon plus 6° below it. A point at horizontal distance r from the L2 is
in view, in every direction, only if its height z satisfies:

```
z ≥ 0.767 − r · tan(6°)
```

So a 0.45 m object comes into view only about 3 m out, and the floor about 7.3 m out. The robot
itself shadows almost nothing: the tall box top is 3 cm below the L2's face.

Check two points with a box about 0.5 m tall:
- **Near field is blind:** at 1.5 m in any direction, nothing of the box appears in
  `/l2/points_filtered`.
- **The edge of the view:** move the box out along a line until its top first appears. It
  should be at about 2.5 m (0.5 m tall). Compare with the sim, which uses the same formula.

To reproduce a position in the sim, spawn the same box with Gazebo's `create` service. This
example puts the robot in the open restroom centre first:

```bash
gz service -s /world/restroom/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: 'mecanum', position: {x: 4.5, y: 2.0, z: 0.01}"
# box 2.5 m ahead of the L2: L2 is at world (4.4, 2.0)
gz service -s /world/restroom/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"probe\"><static>true</static><pose>6.9 2.0 0.25 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.3 0.3 0.5</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.3 0.3 0.5</size></box></geometry></visual></link></model></sdf>"'
gz service -s /world/restroom/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: 'probe', type: MODEL"
```

**If the real result differs:** a different edge distance means a height or tilt error (go back
to stage 2), or the work mode is not wide (stage 1).

**Pass:** the sim and real rows agree, or the sim was updated; the edge of the view matches the
prediction within about 0.3 m; and `STANDINS.md` was updated.

## 4. After this procedure

- **Commit the config.** Commit the confirmed driver config (`cloud_scan_num`, ranges) and
  any sim and stand-in updates.
- **Hardware bringup.** `robot.launch.py sim:=false` does not start the L2 driver yet. It will
  include `mecanum_lidar/l2.launch.py` when hardware is planned, so the hardware bringup
  becomes one command.
- **LIO on hardware.** Push the robot by hand while LIO runs, and check that `/odom` follows:
  `docs/hardware_lio_test.md`. LIO then uses the per-point `time` field the sim lacks.

## Results log

Copy this table for each run:

| Date | Who | Stage | Setup notes | Warnings | Values changed |
|---|---|---|---|---|---|
| | | | | | |
