# Booth demo app: supervisor, panel and TV

This is how the booth demo was built from
[Robot control panel app architecture.md](Robot%20control%20panel%20app%20architecture.md)
(the design) and [Booth demo simulator.html](Booth%20demo%20simulator.html) (the reference for
behaviour). The design still holds: the panel is thin and draws the state, and a robot-side
supervisor decides everything. STOP on the panel is not a safety device; the hardware e-stop
cuts motor power.

The same nodes, topics, services and actions run in Gazebo on the laptop and on the robot's
SBC. Moving to the robot means three things: swapping the simulation bringup for the hardware
bringup, mapping the real booth, and re-measuring the named poses. The list is in
[hardware_transition.md](hardware_transition.md) (S11 to S13 and the SBC launch list).

## Pieces

```
 panel (800x480)   TV (1280x720)   sim controls          browser pages, mecanum_panel
        \              |              /
         +---- rosbridge websocket :9090 ----+           rosbridge_server
                       |
   /supervisor/state  /supervisor/command  /panel/drive  /panel/heartbeat  /robot/health
                       |                      joystick / keyboard -> /manual/cmd_vel
                       |
                demo_supervisor  ---- /demo/move_segment ---->  segment_driver
                 |   |   |                                          |
        /scan /l2/points_filtered /odom /estop /safety/state     /cmd_vel (auto)
                 |
              /cmd_vel (Manual) -> safety gate -> /cmd_vel_safe -> drive
              /safety/bypass (Manual, checks off) ---^
```

- **`mecanum_demo`** is the robot side.
  - `demo_supervisor` holds the state machine (`demo_logic.py`, a port of the simulator's
    supervisor), the hold detector, Manual driving and the e-stop handling.
  - `segment_driver` runs one prewritten base move at a time.
  - `health_monitor` feeds the Health screen.
  - All site data is in `config/booth_demo.yaml`.
- **`mecanum_panel`** holds the web pages and a static file server. The pages are plain ES
  modules with no build step.
- **`mecanum_interfaces`** gained `SupervisorState`, `GateCheck` and `FeatureStatus` (msgs),
  `SupervisorCommand` (srv) and `MoveSegment` (action).
- **`mecanum_simulation`** gained the booth world (`worlds/booth.sdf`) and `booth_visitor`,
  which moves a visitor cylinder in and out of the booth front.
- **`mecanum_bringup/launch/demo_sim.launch.py`** starts the whole thing in Gazebo.

## How the base moves

Nav2 is not used. With its 0.55 m inflation, a 2 × 2 m booth has no free space left. Every
feature is a list of steps in `booth_demo.yaml`:

- **Base steps** are straight holonomic moves, turns in place and spins, each to a named
  map-frame pose: home, w1, stage, b1, a1, a2, dock.
- **Arm steps** are timed placeholders.
- **Wait steps** are pauses.

The segment driver follows a trapezoidal profile and corrects on TF `map → base_footprint`
(AMCL). Before a move starts, it sweeps the robot's outline along the path against `/map` and
refuses the move if it would touch, reporting "Path blocked". It also aborts a move that makes
no progress for 3 s, for example when the safety gate holds the base. Only two nodes ever
publish `/cmd_vel`, and never at the same time: the segment driver while a goal runs, and the
supervisor in Manual. The safety gate stays between `/cmd_vel` and the drive, as everywhere
else.

Going home from anywhere goes through centre stage, the only place with room to turn. When the
robot is off the marked spots, for example after a Manual drive, it first moves to the nearest
spot.

AMCL starts at home: `demo_sim.launch.py` passes the pose, and the supervisor publishes
`/initialpose` once AMCL listens. The booth uses its own AMCL tuning, in
`mecanum_demo/config/booth_amcl.yaml`, on top of the Phase 6 file: every beam, a sharper
likelihood field, and updates every 2 cm.

The simulated booth map (`tools/make_booth_map.py`) keeps only the outline of each wall and
fixture occupied; the inside is unknown, as in a map built from the lidar. With solid walls,
AMCL scored a beam ending anywhere inside a wall as a perfect hit, and its estimate slid 7 to
10 cm into the walls in front of the robot. The scan itself fitted the map to 1 cm at the true
pose. The error put wall points inside the hold radius, so the robot held for nobody.

## The hold

"Someone is close" means lidar points near the robot that the static map does not explain. All
the values are in `booth_demo.yaml` and are STAND-INs. They were relaxed on 2026-11-16, after
false holds in GUI runs (they were 0.45 m, 0.10 m, 3 points and no confirmation). The detector:

- uses both the C1 `/scan` and the L2 `/l2/points_filtered`, flattened over 0.1 to 1.8 m,
  because the C1 sees only the front half;
- counts points within `hold_radius` (0.35 m) of the robot's outline;
- skips points within `hold_map_margin` (0.20 m) of an occupied or unknown map cell, and points
  inside `static_zones` (the toilet tank, which is above the map's slice height). Unknown cells
  count because, near the robot, they are the insides of walls and furniture: the L2 sees the
  counter's top over its unknown inside;
- needs at least `hold_min_points` (10) points;
- needs them continuously for `hold_confirm_s` (0.5 s, ROS time) before it calls someone
  close.

The hold clears after `hold_clear_s` (3 s) with nobody close, and the playlist resumes. A hold
only pauses the playlist; it is not a safety stop.

**Evidence for tuning.** When a hold starts, the supervisor logs a line such as
`Hold evidence: 23 points (C1 9, L2 14), nearest 0.21 m from the outline, around (0.44,
-0.98)`. The points it counts are published on `/demo/hold_points` (PointCloud2, map frame,
empty when none); RViz's navigation view shows them as "Hold points (booth demo)". If a false
hold happens, the log line and the red points show which sensor and which part of the booth
caused it.

**Overriding the hold** (`hold_check` in the state):

- **"False alarm · resume"** on the hold screen sends `hold_skip`. The robot resumes at once and
  ignores the hold until the current loop ends (`off_loop`). It is also cleared by a stop, Go
  home, a reset or a new mode. `hold_skip` is accepted in `running` too.
- **"Visitor hold On | Off"** on Home and on the Health screen sends `hold_check on|off`. Off
  stays off until it is switched back on, in every state but the e-stop. It is kept in the
  supervisor's memory only and is on again at every launch. Switching it off during a hold also
  resumes.
- The status bar shows an amber "Hold off · this loop" or "Hold off" while the check is not on.

While the check is off, `person_close` still updates, so the panel and the log show what the
detector sees.

## Manual

Manual has two inputs. Both go to `/cmd_vel` only in Manual driving, with the arm stowed, and
both are capped at 0.2 m/s and 0.6 rad/s:

- **The panel's D-pad** (`/panel/drive`). It keeps the deadman: the drive stops 0.2 s after the
  last message, or after 1 s without the panel's heartbeat.
- **An external controller** on `/manual/cmd_vel` (`geometry_msgs/TwistStamped`). It is meant
  for the dedicated booth controller, so it has no heartbeat or expiry check: the operator
  holds it. The drive controller's own 0.25 s command timeout still stops the base when the
  stream stops. Outside Manual its commands are dropped, with a log line ("External drive
  ignored: not in Manual driving"), so a controller cannot fight the show. The sources are:
  - a USB pad: `demo_sim.launch.py joy:=true` (or `demo.launch.py joy:=true`) adds
    `teleop_twist_joy` with `config/joy_teleop.yaml`. Hold LB (button 4) to drive; button 1
    is STOP. The buttons are STAND-INs.
  - the keyboard: `ros2 launch mecanum_bringup teleop.launch.py cmd_vel_topic:=/manual/cmd_vel
    speed:=0.2 turn:=0.5`.

**Safety checks are off by default in Manual**, which relies on the operator's judgement. On
every entry to Manual (and on Resume into Manual) `manual_checks` is false:

- the visitor hold does not block Manual driving;
- the supervisor sends `/safety/bypass` true at 5 Hz, and the safety gate then ignores the cliff
  sensors, the bumpers and lost sensors. It still lists them in `/safety/state` `reasons`, sets
  `bypassed`, and logs "Would block (bypassed): ...". A bypass lasts only 0.5 s past the last
  message (`bypass_timeout`), so a dead supervisor cannot leave the gate open. The supervisor
  sends false at once when Manual ends or the checks are turned on;
- **the e-stop always stops the base.**

The Manual screen has a "Safety checks Off | On" switch, and the status bar shows an amber
"Checks off". With the checks on, Manual behaves as before: someone close blocks driving and
the safety gate's blocks apply.

## Interfaces

| Interface | Type | Direction | Notes |
|---|---|---|---|
| `/supervisor/state` | `mecanum_interfaces/SupervisorState` | supervisor → panel, TV | 10 Hz and on change, transient local |
| `/supervisor/command` | `mecanum_interfaces/SupervisorCommand` | panel → supervisor | actions below |
| `/demo/move_segment` | `mecanum_interfaces/MoveSegment` (action) | supervisor → segment driver | one base move |
| `/demo/plan` | `nav_msgs/Path` | supervisor → TV | the base moves still ahead |
| `/panel/drive` | `geometry_msgs/Twist` | panel → supervisor | 20 Hz while a D-pad key is held |
| `/panel/heartbeat` | `std_msgs/Header` | panel → supervisor | 2 Hz; the D-pad's deadman |
| `/manual/cmd_vel` | `geometry_msgs/TwistStamped` | joystick, keyboard → supervisor | Manual driving only, no heartbeat |
| `/safety/bypass` | `std_msgs/Bool` | supervisor → safety gate | 5 Hz in Manual with the checks off; expires after 0.5 s |
| `/demo/hold_points` | `sensor_msgs/PointCloud2` | supervisor → RViz | the points that count toward a hold, map frame |
| `/robot/health` | `diagnostic_msgs/DiagnosticArray` | health monitor → panel | 1 Hz |
| `/joy` | `sensor_msgs/Joy` | controller → supervisor | button `joy_stop_button` sends STOP |
| `/estop` | `std_msgs/Bool`, transient local | e-stop → supervisor, safety gate | |
| `/supervisor/dev/arm_checks_pass` | `std_srvs/SetBool` | development | the teleop gate's arm checks pass |
| `/sim/visitor` | `std_srvs/SetBool` | sim controls → `booth_visitor` | simulation only |

**Actions** (`SupervisorCommand.action`):

| Action | Args |
|---|---|
| `start_mode` | `1`..`5` |
| `run_feature` | `<key> [loop]` |
| `stop`, `reset`, `resume`, `go_home` | |
| `teleop_start`, `teleop_stop`, `teleop_skip`, `teleop_cancel` | |
| `save_run`, `discard_run` | |
| `manual_sub` | `drive` or `arm` |
| `manual_teleop`, `exit_manual` | |
| `manual_checks` | `on` or `off` (Manual only) |
| `hold_skip` | no holds until this loop ends (running, hold) |
| `hold_check` | `on` or `off` (every state but the e-stop) |

A refused command changes nothing and says why. The panel enables a button only when its action
is in `allowed_actions`, and a feature chip only when its `features[]` entry is `ready`. The
robot decides both.

**States:** idle, running, hold, gate, live, stowing, ended, stopping, stopped, manual, estop.

### Changes from the architecture doc

- **New actions:** `teleop_skip` (skip the gate in a playlist), `teleop_cancel`, `exit_manual`
  and `manual_teleop`. The doc's table sends `manual_sub` for Exit Manual, which looks like a
  slip, so `manual_sub` now only switches between drive and arm.
- **More state fields:**
  - the doc's fields;
  - plus `loop`, `loop_count`, `location`, `pose`, `base_moving`, `arm_stowed`,
    `arm_integrated`, `manual_sub`, `hold_clear_s`, `gate_remaining_s`, `ended_reason`,
    `ended_length_s`, `stop_kind`, `can_resume`, `notice`, `features[]`, `person_close`,
    `hold_check` and `manual_checks`.

  With these, the panel and the TV never compute anything themselves.
- **Restamped drive:** the panel sends `/panel/drive` as a plain Twist. The supervisor checks
  it, restamps it and forwards it on `/cmd_vel` at 20 Hz, only in Manual drive. The drive stops
  0.2 s after the last message, or after 1 s without a heartbeat. Speeds are capped at
  0.2 m/s and 0.6 rad/s.
- **Rule 4 (drive expiry) covers the D-pad only.** An external controller on `/manual/cmd_vel`
  has no heartbeat check (owner, 2026-11-16); see [Manual](#manual).
- **Manual with the checks off** (owner, 2026-11-16): the doc has someone close block Manual
  driving. That now happens only with the Manual screen's safety checks switched on.
- **Hold overrides** (owner, 2026-11-16): `hold_skip` and `hold_check`; see [The hold](#the-hold).
- **Timers:** countdowns (gate, live idle, hold clear) run on ROS time. In Gazebo that is sim
  time, so "20 s" takes longer on the wall clock when the simulation runs slower than real
  time. Heartbeat and drive expiry run on the steady clock, as the panel does.
- **Layout:** feature chips are about 88 px wide instead of 120, so that all eight fit on one
  row. Home has no title row, and Health uses two columns of 376 px.

## The arm (not integrated)

Every arm step is a timed placeholder:

- the replay of a recorded demo;
- the greeting wave;
- a teleop session;
- the arm stow.

The panel and the TV show these steps as in the design, and `notice` says "the arm is not
integrated yet, nothing moves". The teleop gate shows its three arm checks failing, and the
20 s fallback then skips teleop. Manual's "Teleop arm" is shown, but only reports the notice.

For a look at Live and Ended in simulation, the sim controls have **Arm checks pass (dev)**,
which calls `/supervisor/dev/arm_checks_pass`. This switch is never used on the robot. Wiring
the real arm means replacing the placeholder steps and the three gate checks in
`demo_supervisor.py` and `demo_logic.py`. Their names are kept so the screens need no change.

## Run it in Gazebo

The bridge has to be installed once:

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

Then start the demo:

```bash
ros2 launch mecanum_bringup demo_sim.launch.py
```

Gazebo shows the booth and Firefox opens the dashboard at `http://localhost:8080/`. The launch takes these arguments:

| Argument | Effect |
|---|---|
| `gui:=false` | no Gazebo window |
| `rviz:=true` | RViz, with the navigation view |
| `browser:=false` | Firefox is not opened |
| `joy:=true` | adds `joy_node` and `teleop_twist_joy` (STOP button, Manual driving) |
| `http_port`, `ws_port` | web server and bridge ports |

The pages:

| Page | What it is |
|---|---|
| `http://localhost:8080/panel/` | the panel; `?fit=0` shows it at 1:1 |
| `http://localhost:8080/tv/` | the TV |
| `http://localhost:8080/sim/` | the sim controls |
| `http://localhost:8080/panel/?fixture=gate` | the panel drawing one sample state, with no robot |

The fixtures are `home`, `running`, `replay`, `hold`, `hold_off`, `gate`, `live`, `ended`,
`stopped`, `manual`, `manual_checks_on`, `manual_arm`, `estop`, `health` and `linklost`.

## Run it on the robot

The SBC runs:

- the hardware bringup: drive, sensors and the safety gate;
- localization on the real booth map, with `amcl_params` set to the booth tuning;
- `ros2 launch mecanum_demo demo.launch.py`, with `use_sim_time:=false`.

The panel's browser opens `http://<sbc>:8080/panel/`. The combined hardware launch is left for
the hardware stage; see [hardware_transition.md](hardware_transition.md).

## Tests

```bash
colcon test --packages-select mecanum_interfaces mecanum_demo mecanum_panel mecanum_simulation \
    mecanum_safety
colcon test-result --verbose
```

The unit tests cover:

- `test_demo_logic.py`: every flow of the simulator;
- `test_booth_layout.py`: sweeps every base step with the real footprint against the booth
  world's shapes, and needs 5 cm of clearance (4 cm for turns and spins);
- the segment control law and the hold detector;
- a launch test of the supervisor against a fake segment server.

`tools/demo_check.py` is the acceptance check on a running headless simulation. It drives the
supervisor through `/supervisor/command` and judges against Gazebo's ground truth. Its stages:

1. **Mode 1:** three loops (`--loops`) with nobody near: no hold at all, and the pose error at
   each named stop (the worst loop is judged).
2. **Hold:** a hold for the scripted visitor within 1.2 s, and the resume.
3. **STOP:** STOP while moving; the safety gate's output is zero within 0.2 s.
4. **E-stop:** press while moving, then release, then Go home.
5. **Manual:** D-pad drive and heartbeat loss; then, at centre stage, an external controller on
   `/manual/cmd_vel`: ignored outside Manual, driving with no heartbeat, driving with the
   visitor close and the checks off (the gate says bypassed), and blocked with the checks on.
6. **Hold bypass:** "False alarm" resumes with the visitor still there, and no hold follows
   until the loop ends; the next loop holds again; `hold_check off` and `on`.

```bash
ros2 launch mecanum_bringup demo_sim.launch.py gui:=false browser:=false
python3 tools/demo_check.py              # in a second terminal; --stages 123456 by default
```
