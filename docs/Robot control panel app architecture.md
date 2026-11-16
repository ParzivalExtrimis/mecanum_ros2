# Robot control panel app: architecture

Nov 14, 2026 · @Homo Sapien

The panel app is a thin, state-driven client: the supervisor node on the robot decides everything, and the panel only shows the current state and sends requests. Every screen is a pure function of one message, `/supervisor/state`, so the panel, the TV and the controller can never disagree.

## Purpose, scope and display assumptions

The panel is the staff-facing touchscreen on the back of the robot. It selects modes, runs single features, gates teleop, and shows warnings. It is not a safety device: the hardware e-stop cuts motor power on its own.

In scope: screen layout, screen states and transitions, the app's internal modules, and the ROS interface. Out of scope: the TV app (it reads the same state topic), the supervisor's internals, and the feature executors.

All sizes below assume a 7-inch 800 × 480 capacitive touchscreen in landscape. That is an assumption to confirm against the real display.

| Property | Value | Physical size at 7 in, 800 × 480 |
| --- | --- | --- |
| Pixel density | about 133 ppi | 1 px ≈ 0.19 mm |
| Primary touch target | 96 px tall (STOP, START, Reset) | ≈ 18 mm |
| Standard touch target | 48 px minimum | ≈ 9 mm |
| Outer margin | 16 px | ≈ 3 mm |
| Gap between controls | 8 to 12 px | 1.5 to 2.3 mm |
| Body text | 15 px, Inter | ≈ 2.9 mm cap-to-descender |
| Screen title | 22 px, Space Grotesk | ≈ 4.2 mm |
| Large numbers (ETA, timer) | 36 to 44 px | ≈ 7 to 8.4 mm |

The 96 px targets matter because staff press STOP and START in a hurry, often at an angle, possibly with gloves.

## System context

The panel is one of three clients of the supervisor. It talks to ROS through the same websocket bridge as the TV, so both read identical state.

&#91;embedded content: system context · 3 clients, 1 supervisor, 3 executors\]

The controller reaches the supervisor directly through the joy node, so STOP still works if the bridge or the panel browser dies. Only the base and the follower arm are on the e-stop circuit; the leader arm has no motors to cut.

## App architecture: layers and modules

The app has five layers. State only ever flows up from the robot to the screens, and requests only ever flow down from a tap to the robot. No layer skips another.

&#91;embedded content: panel app layers · state flows up the left, commands go down the right\]

Read the left column bottom to top: state arrives over the bridge, lands in the store, and the router turns it into one screen. Read the right column top to bottom: a tap becomes a command, a drive stream or a heartbeat, and goes out the same websocket. The layers hold whether the app is built for a kiosk browser or in Qt/QML.

| Module | Owns | Never does |
| --- | --- | --- |
| Screens | layout, labels, button press events | call ROS, start timers, decide what is allowed |
| Screen router | state to screen mapping, overlay stack, the 60 s lock timer | change state on the robot |
| State store | the latest state and health, link age, enabled buttons | keep its own copy of mode or progress |
| Command client | request ids, pending and error display, one command in flight at a time | retry a refused command |
| Drive streamer | the 20 Hz Twist while a D-pad button is held | send while not in Manual |
| Heartbeat | the 2 Hz liveness signal | stop while any screen is open |
| ROS adapter | the websocket, subscriptions, advertisements, reconnects | interpret messages |

Two rules keep this honest. The store holds only what the robot last said, so after a reconnect the panel shows whatever the supervisor reports, not what the panel remembers. And the command client allows one command in flight: a second tap while one is pending is ignored, except STOP, which always goes out at once.

## Screen frame and layout grid with measurements

Every screen uses the same frame. Three bands never move: a 56 px status bar, a 280 px content area, and a 96 px action band. Content can split into a 432 px main column and a 320 px side panel that lines up exactly with STOP below it.

&#91;embedded content: screen frame · 800 × 480 px, three fixed bands, STOP bottom right\]

Because the side panel and STOP share one column, anything a screen puts on the right sits directly above STOP. That keeps the eye travelling straight down to it.

| Region | x | y | Width × height (px) | About (mm) |
| --- | --- | --- | --- | --- |
| Status bar | 0 | 0 | 800 × 56 | 152 × 10.7 |
| Content area | 16 | 72 | 768 × 280 | 146 × 53 |
| Main column | 16 | 72 | 432 × 280 | 82 × 53 |
| Side panel | 464 | 72 | 320 × 280 | 61 × 53 |
| Primary action | 16 | 368 | 432 × 96 | 82 × 18 |
| Split action, two buttons | 16 and 240 | 368 | 208 × 96 each | 40 × 18 each |
| STOP | 464 | 368 | 320 × 96 | 61 × 18 |
| Hold banner (overlay) | 16 | 72 | 768 × 96 | 146 × 18 |

Repeating components have fixed sizes too:

| Component | Size (px) | About (mm) | Used on |
| --- | --- | --- | --- |
| Mode tile | 144 × 144, 12 px apart | 27 × 27 | Home |
| Feature chip | 48 tall, at least 120 wide | 9 tall | Home |
| Gate check row | 432 × 48, 12 px apart | 82 × 9 | Teleop gate |
| List row | 768 × 48, 8 px apart | 146 × 9 | Health |
| D-pad key | 88 × 88, 8 px apart | 17 × 17 | Manual |
| Base / Arm toggle | 160 × 64 per half | 30 × 12 | Manual |
| Secondary button | 320 × 64 | 61 × 12 | Stopped |
| Unlock target | 192 × 192 | 37 × 37 | Locked |

Colour carries meaning and nothing else: red only for STOP, Stopped and E-stop; amber only for Hold and countdowns; the accent only for the one action the screen wants next. A disabled button keeps its size and place, drawn in grey, so the layout never jumps when it enables.

## Screen catalogue with labelled wireframes and sizes

There are eight base screens and four overlays. Each wireframe below is the 800 × 480 frame drawn at 43 % scale, with the size of the key controls written on them in pixels. Words on the wireframes are placeholders for layout, not final copy.

&#91;embedded content: staff screens · Home, Running, Manual, Health · drawn at 43 % scale\]

The staff screens are the ones booth staff use between demos. Home is the only screen where a mode can be chosen; tapping a tile selects it and Start mode sends it, so a stray tap never starts the robot. Health is the one screen the panel opens on its own, without the robot changing state.

&#91;embedded content: teleop screens · Teleop gate, Live, Ended, Stopped · 43 % scale\]

The teleop screens follow one visitor session from gate to save. The gate shows the four checks in the main column and, in Mode 4, the 20 s fallback countdown above STOP. Live keeps the auto-end bar in amber so staff can see a session about to time out.

&#91;embedded content: overlays · Hold, Locked, E-stop, Link lost · 43 % scale\]

Overlays never replace the screen underneath; they sit on top of it and leave STOP uncovered. The one exception is Link lost: a STOP sent over a dead link would not arrive, so the panel says so and points to the controller and the e-stop instead.

| Screen | Shown when the robot reports | Main action, and the command it sends | Size of main action |
| --- | --- | --- | --- |
| Home | idle, at home | Start mode (`start_mode`) or a feature chip (`run_feature`) | 432 × 96 |
| Running | running | Go home (`go_home`) | 432 × 96 |
| Teleop gate | gate | START (`teleop_start`) | 432 × 96 |
| Live | live | End session (`teleop_stop`) | 432 × 96 |
| Ended | ended | Save run (`save_run`) or Discard (`discard_run`) | 208 × 96 each |
| Stopped | stopped | Reset (`reset`) | 432 × 96 |
| Manual | manual | Exit Manual (`manual_sub`), D-pad on `/panel/drive` | 432 × 96 |
| Health | any state (opened on the panel) | Back (no command) | 432 × 96 |
| Hold overlay | hold | none, clears by itself | banner 768 × 96 |
| Locked overlay | idle at home for 60 s | 2 s long-press (no command) | 192 × 192 |
| E-stop overlay | estop | Reset after release (`reset`) | 432 × 96 |
| Link lost overlay | state older than 1 s | none | full screen |

STOP (`stop`, 320 × 96) appears on every screen and overlay except Link lost.

## Screen relationships: state and navigation flow

The panel has almost no navigation of its own. Each arrow below is a change in the robot's state, and the router follows it. Only two moves belong to the panel: opening Health and the lock timer.

&#91;embedded content: screen relationships · 6 base screens, Stopped, 4 overlays, 1 panel-only view\]

Read it as one loop per visitor. Modes 1, 2 and 4 run along the top row; Mode 4 drops into the teleop gate at its teleop step, and either goes Live on START or returns to Running after 20 s. A session ends by tap, by STOP, or after 15 s of no leader motion, and the panel shows Ended once the arm is stowed. From there Mode 4 resumes its playlist and Mode 5 waits at the gate for the next visitor.

| From | To | Trigger | Who causes it |
| --- | --- | --- | --- |
| Home | Running | Start mode 1, 2 or 4 | staff tap, robot confirms |
| Home | Teleop gate | Start mode 5 | staff tap, robot confirms |
| Home | Manual | Start mode 3 | staff tap, robot confirms |
| Running | Teleop gate | Mode 4 reaches its teleop step | robot |
| Running | Home | playlist done, or Go home arrives | robot |
| Teleop gate | Live | START with all four checks ok | staff tap, robot confirms |
| Teleop gate | Running | 20 s with no START (Mode 4) | robot |
| Live | Ended | End session, or 15 s with no leader motion; arm stows first | staff tap or robot |
| Ended | Running or Teleop gate | Save or Discard; Mode 4 resumes, Mode 5 waits for the next visitor | staff tap, robot confirms |
| Manual | Home | Exit Manual | staff tap, robot confirms |
| any screen | Stopped | STOP on panel or controller, or a fault | staff or robot |
| Stopped | Home | Reset | staff tap, robot confirms |
| any screen | Health and back | status bar tap, Back | panel only |
| Home | Locked | 60 s idle at home | panel only |

"Robot confirms" matters: a tap sends a command and shows it as pending, but the screen changes only when the next `/supervisor/state` arrives with the new state. If the robot refuses, the panel stays where it was and shows the reason from the reply.

## ROS interface contract

The panel touches the robot through four interfaces, all over the websocket bridge. Message and field names are proposals.

| Interface | ROS type | Direction | Rate | What it carries |
| --- | --- | --- | --- | --- |
| `/supervisor/state` | topic, custom `SupervisorState` | robot to panel and TV | 10 Hz, latched | state, mode, playlist, current index, step label, ETA in s, gate checks, live-session stats, message, `allowed_actions[]` |
| `/supervisor/command` | service, custom `Command` | panel to robot | on tap | action (`start_mode`, `run_feature`, `stop`, `reset`, `resume`, `go_home`, `teleop_start`, `teleop_stop`, `save_run`, `discard_run`, `manual_sub`) + args + request id; reply accepted or refused with a reason |
| `/panel/drive` | topic, `geometry_msgs/Twist` | panel to supervisor | 20 Hz while a D-pad button is held | requested base motion in Manual; the supervisor forwards it only when Manual, arm stowed and no hold, and zeroes it after 200 ms without a message |
| `/panel/heartbeat` | topic, `std_msgs/Header` | panel to supervisor | 2 Hz | proves the panel is alive; loss for more than 1 s stops Manual driving |
| `/robot/health` | topic, `diagnostic_msgs/DiagnosticArray` | robot to panel | 1 Hz | battery, lidar, CAN links, motor temperatures |

`SupervisorState` proposed fields:

```
string state          # idle, running, hold, gate, live, stowing, ended, stopping, stopped, manual, estop
string mode           # 1..5, single, none
string[] playlist     # feature names
uint8 index           # current feature in playlist
string step_label     # "Slide in sideways"
float32 eta_s         # -1 when unknown (teleop ahead)
GateCheck[] checks    # name, ok, detail
float32 live_elapsed_s
float32 live_idle_s   # counts to 15 for auto-end
uint32 demo_count
string message        # fault or stop reason
string[] allowed_actions
```

The TV app subscribes to the same `/supervisor/state`, plus `/tf`, `/scan`, the Nav2 plan and the wrist camera.

## Safety rules the UI enforces

The robot is the authority; the panel enforces these rules only to keep the screen honest.

1. **Buttons come from the robot.** A button is enabled only if its action is in `allowed_actions[]`. The panel never decides on its own that an action is allowed.
2. **STOP lives in one place.** On every screen where the robot can move, STOP is 320 × 96 px at the bottom right (x 464 to 784, y 368 to 464). Staff build muscle memory for it.
3. **START waits for every check.** The START button stays disabled until all four gate checks report ok, including leader and follower poses matching.
4. **Drive commands expire.** D-pad input streams at 20 Hz and the supervisor zeroes motion 200 ms after the last message, so a frozen panel cannot leave the base driving.
5. **Lost link means a full-screen notice.** If `/supervisor/state` is older than 1 s, the panel covers everything with "Link lost" and the supervisor stops any Manual driving.
6. **The panel locks, STOP does not.** At home and idle for 60 s, the panel locks; a 2 s long-press unlocks it. STOP and the e-stop work locked or not.
7. **The e-stop is hardware.** The panel only reports it. Releasing it lands on Stopped, never straight back into motion.

## As built (2026-11-15)

The panel, the TV page and the supervisor exist now; [demo_app.md](demo_app.md) describes them. The design above still holds. These are the changes from it:

- **The command service** is `mecanum_interfaces/srv/SupervisorCommand`, not `Command`. It has four more actions: `teleop_skip`, `teleop_cancel`, `exit_manual` and `manual_teleop`. The table above sends `manual_sub` for Exit Manual. That looked like a slip, so `exit_manual` was added and `manual_sub` only switches between drive and arm.
- **`SupervisorState`** has all the fields above. It adds the ones the screens and the TV would otherwise have to work out themselves:
  - `loop`, `loop_count`, `location`, `pose`;
  - `base_moving`, `arm_stowed`, `arm_integrated`, `manual_sub`;
  - `hold_clear_s`, `gate_remaining_s`, `ended_reason`, `ended_length_s`;
  - `stop_kind`, `can_resume`, `notice`;
  - `features[]`, which says per feature whether it is ready and why not, so the Home chips also come from the robot (rule 1).
- **`/panel/drive`** is restamped by the supervisor and forwarded on `/cmd_vel` at 20 Hz, capped at 0.2 m/s and 0.6 rad/s.
- **The TV** draws the path ahead from `/demo/plan`. There is no Nav2 plan, because the booth moves are prewritten.
- **The arm** is not integrated. Its steps are timed placeholders, and `notice` says so.
- **Changes on 2026-11-16 (owner):**
  - **Hold overrides.** New actions `hold_skip` ("False alarm · resume" on the hold screen: no holds until the current loop ends) and `hold_check on|off` (a "Visitor hold" switch on Home and Health that stays off until switched back, reset at launch). `SupervisorState` gains `hold_check` (`on`, `off_loop` or `off`) and `person_close`. The hold detection was relaxed and its values moved to `booth_demo.yaml`.
  - **Manual with the safety checks off by default.** On every entry to Manual the visitor hold and the safety gate's cliff, bumper and lost-sensor blocks are off, through `/safety/bypass`. Only the e-stop stops the base. A "Safety checks" switch on the Manual screen (`manual_checks on|off`, field `manual_checks`) turns them back on for the session. `manual_blocked` is true only with the checks on.
  - **Rule 4 applies to the D-pad only.** The dedicated booth controller (joystick or keyboard) drives through `/manual/cmd_vel`, with no heartbeat or 200 ms expiry check; the drive controller's own command timeout still stops the base when its stream stops. It is ignored outside Manual.
- **Two of the open questions** have a working answer, without closing them:
  - the panel is a web app in a browser, served by the robot;
  - the unlock is a 2 s long-press.

## Open questions

- [ ] Which display is it, exactly: size, resolution and mounting angle? Every pixel figure here assumes 7 in at 800 × 480.
- [ ] Web app in a kiosk browser, or Qt/QML? Web reuses the TV and simulator code, but Chromium on the Orin Nano competes with Nav2 for memory.
- [ ] Is the unlock a long-press, a PIN, or a key switch?
- [ ] Who presses START during visitor teleop: staff at the panel, or a button on the controller?
- [ ] Does Manual need a speed selector, or is one capped speed enough?
