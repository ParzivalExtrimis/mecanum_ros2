# Unitree L2 on hardware: configuration, mount verification and sim comparison

This procedure takes the real L2 from the desk to the robot. It makes sure that everything
built in simulation still holds on hardware: the frames, the mount pose, the stream format, the
self-filter and the field of view.

It has four stages. Do them in order, because each one assumes the previous one passed.

| Stage | Where | Proves |
|---|---|---|
| 1. Configure and bench test | L2 on a desk, Ethernet | The driver works, and the L2 is configured to match the sim |
| 2. Mount verification | L2 on the mast, robot still on a flat floor | The real mount matches the URDF |
| 3. Sim against real | Same setup, plus a few props | The sim's sensor model is close enough, so stand-ins can be replaced |
| 4. Moving | Phase 4 onward | LIO, mapping and navigation on hardware |

Stage 4 is covered by the phase test lists, not here.

The main tool is `tools/l2_mount_check.py`. It reads `/l2/points`, `/l2/imu` and the URDF TF,
and prints a report with an OK or WARN verdict per check. It was validated against the
simulation, where the true pose is known:
- **Exact pose.** Height 0.930 m, tilt 30.01° and spin 22.90°, against URDF values of 0.930 m,
  30.00° and 22.90°.
- **Wall.** A deliberate 3° heading error was reported as −3.00°.
- **Synthetic mount error.** A wrong mount was recovered to within 0.5 mm and 0.02°.

## 0. What the simulation assumes

The sim's L2 behaves as listed below. The real L2 must be configured or measured to match. Any
remaining difference goes into `STANDINS.md` and the sim model.

| Assumption | Sim value | Real L2 setting or check |
|---|---|---|
| Connection | n/a | Ethernet/UDP, factory addresses: L2 192.168.1.62, host 192.168.1.2. Work-mode bit 3 = 0. USB-UART crashed the dev laptop's USB controller. |
| Field of view | Standard: elevation 0° to 90° in the L2 frame | Work-mode bit 0 = 0. For wide mode (bit 0 = 1, −6° to 90°), launch the sim with `sim_l2_wide_fov:=true`. |
| Scan mode | 3D | Work-mode bit 1 = 0 |
| IMU | On | Work-mode bit 2 = 0 |
| Start | Spins at power-on | Work-mode bit 4 = 0 |
| **Work mode** | n/a | **0** for Ethernet, standard FOV, 3D, IMU on, auto start: the factory mode. **1** for the same with wide FOV. |
| One message | **One full 360° sweep at 5.55 Hz** | `cloud_scan_num`. The default of 18 scans covers only about 166° at about 12 Hz. A revolution is 216 Hz vertical scans divided by 5.55 Hz, about **39 scans**. Verify in stage 1. |
| Points per sweep | 11,520, on a regular 360 × 32 grid | Measured in stage 1. The real pattern is non-repetitive. |
| Range | 0.05 to 30 m | `range_min: 0.05`, `range_max: 30.0`. The driver defaults are 0 and 100. |
| Timestamps | Simulation clock | `use_system_timestamp: true`, the default |
| Per-point `time` field | **Absent**: Gazebo renders instantly | Present on hardware, spanning about 180 ms per full sweep. LIO deskews with it in phase 4. |
| Frames | `l2_lidar`, `l2_imu` | `cloud_frame_id`, `imu_frame_id`, the defaults |
| IMU units | m/s², rad/s | The driver converts. Check that the norm is about 9.8 at rest. |
| IMU rate | 250 Hz | Measured in stage 1 |
| Noise | Range σ 0.01 m; gyro σ 0.002 rad/s; accel σ 0.02 m/s² | Measured in stages 1 and 2 |
| Mount | 810 mm pole at the rear edge, bracket 30°, L2 inverted, cable 22.9° clockwise from above | Measured in stage 2 |

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

1. **Check the work mode is 0, Ethernet.** That is the factory setting. If the L2 was switched to
   serial, write mode 0 over serial once, then power-cycle:
   ```bash
   ros2 run mecanum_lidar l2_driver --ros-args -p connection:=serial \
     -p serial_port:=/dev/ttyACM0 -p work_mode:=0
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
   - IMU tilt from vertical should be about 180°, because the upright L2's +Z points up,
     away from "down". It is about 0° when the L2 is inverted.
   - Tip the L2 so the side away from the cable rises: accel x must go positive. That is the
     +X convention the URDF's `l2_bracket_yaw` relies on.
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
- Put the robot on a flat, level floor; check the floor with a spirit level.
- Park it **parallel to a long straight wall**, about 1 m to its right or left. Use a wall
  without skirting cabinets.
- Keep at least 3 m clear ahead.
- Nobody stands near the robot.
- Power the L2 for 2 minutes first, so the IMU warms up.

### 2a. Physical measurements (tape, angle gauge)

Measure these before trusting any software. Each one checks a value the URDF assumes:

| Quantity | Design (URDF) | How to measure |
|---|---|---|
| Pole height above the chassis plate | 810 mm | Tape, plate top to pole top |
| Pole position | Rear face flush with the plate's rear edge, centred left-right | Square and tape |
| Pole vertical | 90° to the plate | Angle gauge on two faces of the pole |
| Bracket tilt | 30° up toward the front | Digital angle gauge on the bracket plate. Zero it on the chassis plate first. |
| L2 face centre | 66 mm ahead of the pole centre, 7.5 mm left of the centreline | Tape. This comes from the CAD. |
| Cable direction | 22.9° clockwise from above, from pointing straight at the pole | Protractor on the bracket, or a photo from straight above |

### 2b. Software check

Start the driver, the URDF TF, the self-filter, the scan conversion and the sensors RViz view.
This is the same view as the simulation's `robot.launch.py rviz:=true`:

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py
```

**Look in RViz first:**
- The floor lies on the grid at z = 0.
- Walls are vertical and straight.
- `/l2/points_filtered` has no points on the chassis, wheels, pole or bracket.
- `/scan` traces the walls.

Then run the check. Tape-measure the wall distance from the robot's **centreline**, which is
the chassis side face plus 0.197 m:

```bash
tools/l2_mount_check.py --duration 20 --wall-side right --wall-distance 1.03
```

### 2c. Reading the report

The tolerances below are what the navigation stack can absorb. At 3 m range, 0.5° of tilt error
moves the floor by 2.6 cm.

| Row | Tolerance | If WARN |
|---|---|---|
| L2 height above floor | ±1 cm | The pole or bracket height differs from the CAD. Check 2a. Fix the hardware, or set `l2_mast_z`, which moves the whole mast. |
| L2 tilt from vertical, from the floor | ±0.5° | The bracket angle differs from 30°. Check it with the angle gauge, then shim the bracket or update `l2_bracket_tilt` and `l2_face_*`. Re-export and convert the CAD so the meshes match. |
| Mount roll or pitch error | ±0.5° | Roll means the pole leans sideways or the bracket is twisted. Pitch is the same error as the tilt row. Shim it, rather than encoding a lean in the URDF. |
| IMU tilt from vertical | ±1° | It should agree with the floor tilt within about 0.5°. If only the IMU disagrees, the floor isn't level (check with the spirit level) or the robot is rocking. |
| L2 spin about own axis | ±2° | The cable direction differs from 22.9°. Set the measured value as `l2_bracket_yaw` in radians; see `STANDINS.md` "Provisional design choices". A roll error also shifts this row, so fix roll first. |
| Wall heading | ±0.5° | First make sure the robot really is parallel to the wall, then rerun. If it persists, the mast is rotated on the plate (`l2_mast_yaw`) or the bracket is twisted on the pole. |
| Wall distance | ±2 cm | A sideways mast offset (`l2_mast_y`) or a wrong L2 face position. Check 2a. |
| Points just outside the box | about 0 | Self-hits the box misses, for example cables or a larger bracket. Look in RViz, then grow `self_filter.yaml`. |

**Which to change, the hardware or the URDF?**
- **Fix the hardware** when it deviates from the CAD design: a loose bolt, a bent bracket, a
  wrong cable direction.
- **Change the URDF** only when the design itself is different and the measurement repeats
  after re-tightening. Do that through the xacro args (`l2_mast_*`, `l2_bracket_yaw`) or a CAD
  re-export with `tools/cad_stl_to_ros.py`.

Rerun until the report has no warnings.

**Pass:** zero warnings with the robot parallel to the wall. Floor and IMU tilt agree within
0.5°.

## 3. Sim against real

Run the same tool against the sim at the same spot type: the spawn point is 1.0 m from the
corridor wall on the right. Fill in the table:

```bash
ros2 launch mecanum_bringup robot.launch.py gui:=false
tools/l2_mount_check.py --use-sim-time --wall-side right --wall-distance 1.0
```

| Row | Sim | Real | Action if they differ |
|---|---|---|---|
| Cloud rate | 5.55 Hz | | Tune `cloud_scan_num` (stage 1) |
| Azimuth per cloud | 360° | | Tune `cloud_scan_num` (stage 1) |
| Points per cloud | 11,520 | | Set the gpu_lidar samples in `mecanum_simulation/urdf/mecanum.gazebo.xacro` to about the same count, for example 360 × N |
| Points on the robot | about 25% | | A big difference means a frame or mount error; go back to stage 2 |
| Floor residual (range noise) | 0.008 m | | Set the gpu_lidar noise `stddev` |
| IMU rate | 250 Hz | | Set the IMU `update_rate` |
| Gyro std, accel std | 0.002, 0.02 | | Set the IMU noise |
| Per-point time span | none | about 180 ms | Expected difference. LIO deskews on hardware only. |

After each change, update the matching row in `STANDINS.md`. Remove a row once its value is
measured.

### 3a. Field of view and rear blind zone

The L2 hangs inverted, tilted 30° forward. Take a point at horizontal distance r and azimuth φ
from the L2, with φ = 0 straight ahead. It is in the L2's field of view only if its height z
satisfies:

```
z ≤ 0.93 + r · tan(30°) · cos φ
```

Ahead, the L2 sees above its own height. Toward the rear the limit drops with distance, so low,
near objects are visible and tall or far ones are not. In addition, the **pole shadows a sector
about 60° wide straight back**. Nothing there is ever visible. This is the biggest hardware
risk to confirm, because the real pole, cable and bracket may shadow more or less than the CAD.

Use a box about 0.3 m tall, placed at a measured distance from the L2's centre. That centre is
0.22 m behind the chassis centre, on the centreline. For each position, note whether the box
appears in `/l2/points_filtered`, and the highest point seen on it, and whether it appears in
`/scan`. The predicted and sim results:

| Box position, from the L2 | Predicted | Sim (measured) | Real |
|---|---|---|---|
| 45° rear-left or rear-right, 1.0 m | Fully visible | 83 points up to 0.31 m; in `/scan` | |
| 45° rear-left or rear-right, 2.0 m | Lowest part only, about 0.18 m at its near face | Up to 0.21 m; in `/scan` | |
| 45° rear-left or rear-right, 2.6 m | Invisible; the floor leaves the view at about 2.3 m | Nothing | |
| Straight back, 1.0 m | Invisible, because of the pole shadow | Nothing | |
| Straight back, sweep the azimuth | Shadow edges at about ±30° from straight back | No points between about 150° and 210° | |

To find the shadow edges on the real robot, walk the box around the rear at 1 m. Note the
azimuth at which it disappears and reappears. Compare with the sim's 150° to 210° sector.

To reproduce a position in the sim, spawn the same box with Gazebo's `create` service. This
example puts the robot in the open restroom centre first:

```bash
gz service -s /world/restroom/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: 'mecanum', position: {x: 4.5, y: 2.0, z: 0.01}"
# box 1.0 m from the L2 at 45 deg rear-right: L2 is at world (4.278, 2.008)
gz service -s /world/restroom/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"probe\"><static>true</static><pose>3.571 1.300 0.15 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></visual></link></model></sdf>"'
gz service -s /world/restroom/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: 'probe', type: MODEL"
```

**If the real result differs:**
- **A wider shadow** means the cable, bracket or pole is bigger than in the CAD. Update the
  meshes or grow the self-filter, and tell Phase 7: the rear safety zone relies on this sector.
- **A different edge distance** means a tilt or height error. Go back to stage 2.

Also compare the forward and side coverage. In a corridor of known width, the scan's side
returns must match the tape-measured distances to within about 2 cm, in both the sim and the
real run.

**Pass:** the sim and real rows agree, or the sim was updated; the blind-zone boundary matches
the prediction within about 0.15 m; and `STANDINS.md` was updated.

## 4. After this procedure

- **Commit the config.** Commit the confirmed driver config (`cloud_scan_num`, ranges) and
  any sim and stand-in updates.
- **Phase H.** `robot.launch.py sim:=false` does not start the L2 driver yet. It will include
  `mecanum_lidar/l2.launch.py` in phase H, so the hardware bringup becomes one command.
- **Phase 4 on hardware.** Push the robot by hand while LIO runs, and check that `/odom`
  follows. LIO then uses the per-point `time` field the sim lacks.

## Results log

Copy this table for each run:

| Date | Who | Stage | Setup notes | Warnings | Values changed |
|---|---|---|---|---|---|
| | | | | | |
