# mecanum_demo

The robot side of the booth demo. It is the same in Gazebo and on the robot's SBC. The
supervisor decides everything; the panel and the TV (`mecanum_panel`) only draw its state and
send requests. The interfaces are in `mecanum_interfaces`, and the whole design is in
[docs/demo_app.md](../../docs/demo_app.md).

## Files

| File | Purpose |
|---|---|
| `config/booth_demo.yaml` | Site data: footprint, named poses, headings, speeds, hold detection and teleop timings, features as step lists, mode playlists. A new booth needs only this file and a map. |
| `mecanum_demo/demo_logic.py` | The ROS-free state machine, ported from `docs/Booth demo simulator.html`. |
| `mecanum_demo/demo_supervisor.py` | The node: `/supervisor/state`, `/supervisor/command`, `/demo/plan`, Manual driving, the e-stop, the controller's STOP and the hold detector. |
| `mecanum_demo/segment_driver.py` | The `/demo/move_segment` action server: one prewritten base move at a time, closed-loop on TF `map → base_footprint`, out on `/cmd_vel`. |
| `mecanum_demo/segment_control.py` | The segment driver's control law, without ROS. |
| `mecanum_demo/hold_detector.py` | "Someone is close": lidar points near the robot's outline that the static map does not explain. |
| `mecanum_demo/health_monitor.py` | `/robot/health` for the panel's Health screen. |
| `mecanum_demo/geometry.py`, `ros_util.py` | Planar geometry and message conversions. |
| `config/booth_amcl.yaml` | The booth's AMCL tuning, on top of Phase 6's (passed as `amcl_params`). |
| `config/joy_teleop.yaml` | The booth controller's drive mapping for `teleop_twist_joy` (STAND-IN buttons). |
| `launch/demo.launch.py` | The supervisor, segment driver, health monitor, rosbridge and the web server. Optional `joy_node` and `teleop_twist_joy`. |

## Interfaces

| Interface | Type | Direction | Notes |
|---|---|---|---|
| `/supervisor/state` | `mecanum_interfaces/SupervisorState` | supervisor → panel, TV | 10 Hz and on change, transient local |
| `/supervisor/command` | `mecanum_interfaces/SupervisorCommand` | panel → supervisor | the action list is in the `.srv` |
| `/demo/move_segment` | `mecanum_interfaces/MoveSegment` (action) | supervisor → segment driver | one base move |
| `/demo/plan` | `nav_msgs/Path` | supervisor → TV | the base moves still ahead |
| `/panel/drive` | `geometry_msgs/Twist` | panel → supervisor | 20 Hz while a D-pad key is held; restamped onto `/cmd_vel` |
| `/panel/heartbeat` | `std_msgs/Header` | panel → supervisor | 2 Hz; a gap over 1 s stops the D-pad's driving |
| `/manual/cmd_vel` | `geometry_msgs/TwistStamped` | joystick, keyboard → supervisor | external controller, forwarded (capped) only in Manual driving; no heartbeat |
| `/safety/bypass` | `std_msgs/Bool` | supervisor → safety gate | 5 Hz in Manual with the checks off |
| `/demo/hold_points` | `sensor_msgs/PointCloud2` | supervisor → RViz | the points counted toward a hold (map frame) |
| `/robot/health` | `diagnostic_msgs/DiagnosticArray` | health monitor → panel | 1 Hz |
| `/joy` | `sensor_msgs/Joy` | controller → supervisor | button `joy_stop_button` sends STOP |
| `/estop` | `std_msgs/Bool`, transient local | e-stop → supervisor, safety gate | pressed gives `estop`; released lands on `stopped` |
| `/supervisor/dev/arm_checks_pass` | `std_srvs/SetBool` | development | the teleop gate's arm checks pass |

## Velocity sources

Only two nodes publish `/cmd_vel` in the demo, and never both at once. The segment driver
publishes only while a goal runs, and the supervisor only in Manual driving, from the D-pad or
from an external controller on `/manual/cmd_vel` (`demo.launch.py joy:=true` adds
`teleop_twist_joy` with `config/joy_teleop.yaml`). Manual starts with its safety checks off:
no visitor hold, and the safety gate's sensor blocks bypassed; only the e-stop stops the base
(`docs/demo_app.md`, "Manual"). Nav2 is not
launched: the booth is too small for its inflation. The safety gate stays between `/cmd_vel`
and the drive, as everywhere else.

## The arm

The arm is not part of this workspace. Every arm step (the replay, the greeting wave, a teleop
session) is a timed placeholder: the panel and the TV show it, and `notice` says nothing
moves. The teleop gate's three arm checks fail unless the development switch is set.

## Run

```bash
ros2 launch mecanum_bringup demo_sim.launch.py          # everything, in Gazebo
ros2 launch mecanum_demo demo.launch.py                 # on the robot, on its bringup
```

## Tests

```bash
colcon test --packages-select mecanum_demo && colcon test-result --verbose
```

- `test_demo_logic.py`: every flow of the simulator page, on the state machine.
- `test_booth_layout.py`: every prewritten move is swept with the real footprint through `booth.sdf`. It needs 5 cm of clearance, or 4 cm for turns at centre stage.
- `test_segment_control.py`: the control law on an ideal base.
- `test_hold_detector.py`: people count; the booth's walls do not.
- `test_supervisor_launch.py`: the nodes on ROS_DOMAIN_ID 93, with a fake localization. It checks commands and state, STOP, drive expiry and heartbeat loss. Run it with no simulation up.
