# mecanum_safety

The safety gate between every velocity source and the drive controller
([docs/plan.md](../../docs/plan.md), Phase 4).

```
Nav2, docking, teleop, test tools --/cmd_vel--> safety_supervisor --/cmd_vel_safe--> mecanum_drive_controller
                                                        |
                                                        +--> /safety/state (mecanum_interfaces/SafetyState)
```

`robot.launch.py` starts it in simulation (`safety:=false` turns it off, and then nothing
drives the robot). The hardware path is unchanged until the hardware plugin exists.

## Files

| File | Purpose |
|---|---|
| `mecanum_safety/safety_logic.py` | The rules, pure Python with no ROS: guards, the cliff threshold, the watchdog, the e-stop latch |
| `mecanum_safety/safety_supervisor.py` | The node |
| `config/safety.yaml` | Positions (from the URDF), thresholds and watchdog timeouts |
| `launch/safety.launch.py` | The node, with a `watch_<sensor>` argument per sensor group |
| `test/test_safety_logic.py` | Every rule, corner, plank, the latch and the watchdog timing |
| `test/test_layout.py` | The positions in `safety.yaml` against the URDF |
| `test/test_supervisor_launch.py` | The node with fake sensors: each topic silenced in turn, then the cliff, bumper and e-stop inputs |

## Rules

A **guard** is a set of robot points with outward normals. A command (vx, vy, wz) passes it only
if every point moves with no speed along any normal; a point (x, y) moves at
(vx − wz·y, vy + wz·x), so rotation is judged exactly. A command that fails any guard is
replaced with zero, never reduced.

| Input | Condition | Effect |
|---|---|---|
| `/cliff/<pos>` | range over `floor_distance + cliff_margin` (0.169 m), +inf or NaN | Guards its corner in both outward axes: only motion away from the corner |
| `/bumper/<front\|rear>` | true | Guards the plank's two ends along its normal: backing off, sliding along the wall, and turning only while both ends leave |
| `/estop` (transient local) | true | Stops everything and latches. `~/reset` (`std_srvs/Trigger`) clears it, only while `/estop` is false |
| Loss of `/scan`, `/l2/points_filtered`, `/d435i/depth/color/points`, `/joint_states`, `/odom` | no message within the timeout | Stops everything |
| Loss of `/ultrasonic/<pos>` | as above | Guards its face (front or rear) |
| Loss of `/cliff/<pos>` or `/bumper/<plank>` | as above | As a cliff or a pressed plank |

- **Before the first message,** a topic counts as lost. The robot cannot move until every
  watched sensor is up.
- **A sensor that is not fitted** (`use_<sensor>:=false` in `robot.launch.py`) is not watched and
  gives no protection. `/odom` is watched only with the EKF.
- **The ultrasonic ranges themselves** are not used here. Obstacles are the collision monitor's
  job (Phase 6).
- **Commands.** Each allowed command is forwarded with its stamp. A blocked one is replaced by a
  zero with the same stamp. When a new block appears, a zero goes out at once, with the last
  command's stamp. When the sources go quiet the gate sends nothing, so the controller's 0.25 s
  reference timeout still applies.
- **`/safety/state`** is published at 10 Hz and on every change: `estop_latched`, `stopped` (all
  motion blocked), and `reasons`, for example `cliff front_left`, `bumper front` or
  `lost /scan`.

## Clocks and executor

- **The wall clock, also in simulation.**
  - **The bridge carries `/clock`.** The simulation's one bridge also carries `/clock`. A
    watchdog on sim time would freeze with it and never fire, so the watchdogs run on the steady
    clock.
  - **Cost.** Following `/clock` (500 Hz) would also cost most of a core in Python.
  - **Stamps.** The gate never stamps a command itself. It reuses the sources' stamps, which
    the controller checks.
- **rclpy's `EventsExecutor`.** With some 300 messages a second, the default executor took
  34–40% of a core. The events executor takes 5–7%.

## Run

```bash
ros2 launch mecanum_bringup robot.launch.py           # starts the gate in simulation
ros2 topic echo /safety/state
ros2 topic pub --once -w 0 /estop std_msgs/msg/Bool "{data: true}" \
  --qos-durability transient_local --qos-reliability reliable
ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger
tools/safety_check.py                                  # in sensor_test.sdf, see docs/plan.md
```

Run the tests with no simulation running. The launch test publishes on the real topic names,
and a running simulation feeds the same topics.
