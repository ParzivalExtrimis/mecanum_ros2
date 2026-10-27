# LIO on hardware: the robot pushed by hand (phase 4H)

> **Paused (2026-10-16).** Hardware work resumes after the sim track; see
> [`hardware_transition.md`](hardware_transition.md) for this stage's status, dependencies and
> confidence. Re-read its section 2 (contracts) and the stage's "Depends on" before running this
> procedure, because sim-track decisions may have changed what it tests.

This procedure runs LIO (rko_lio and the odometry adapter) on the real L2 while the robot is
pushed by hand. No motors are needed.

It answers two questions, in this order:

1. **Does the real L2 have the simulation's failure?** In simulation, LIO sees almost no motion
   when the robot moves along a corridor (see "Phase 4 results" in
   `src/mecanum_localization/README.md`). The suspected cause is the simulated L2's fixed scan
   grid. The real L2's pattern shifts between revolutions, so it may not fail. Stage 2 tests
   exactly the motion that fails in simulation. Its answer decides the next step: fix the
   simulated sensor, or change the LIO approach.
2. **How large is the drift?** Stages 1, 3 and 4 measure static drift, loop drift and the
   corridor case against the phase 4 targets: under 1% of the path and under 2° of yaw.

| Stage | Motion | Proves |
|---|---|---|
| 1. Start-up and static drift | None, 5 minutes | LIO initializes, and does not drift while still |
| 2. Straight push, the sim comparison | One 2 m push, away from a wall | Whether the real L2 sees motion where the sim's does not |
| 3. Closed loop (only if stage 2 tracks) | Three laps of a 2 m × 1 m rectangle, 18 m | Drift against the targets |
| 4. Long corridor (optional) | 10 m or more, only if such a corridor is available | Degeneracy along a featureless corridor |

All distances fit a test area with 2 m of straight floor in front of a wall: tape marks at 0,
1, 2 and 3 m, where the 3 m mark is at the wall.

There is no ground truth on hardware. Tape marks on the floor take its place. The tool
`tools/lio_marks.py` records the `/odom` pose each time the operator presses Enter at a mark.
Every run is also recorded to a bag, so it can be replayed later with other LIO settings.

## 0. Prerequisites

- **The L2 on the mast, over Ethernet.** The NetworkManager profile on `enp2s0` is up and
  `ping 192.168.1.62` answers. See "Ethernet setup" in `src/mecanum_lidar/README.md`.
- **The L2 configured, stage 1 of `docs/hardware_l2_test.md`.** In particular step 2,
  `cloud_scan_num` for full sweeps (about 39). **This is required.** With the driver default of
  18, each cloud holds half a revolution of scan lines, and the next cloud's lines lie elsewhere.
  rko_lio matches points to points, so it "follows" the moving line pattern: in the first static
  run (2026-10-15) it turned about 10°/s and wandered by metres while the robot stood still.
  Replaying that recording with two clouds merged into one held still. In the
  `tools/l2_mount_check.py` report, the per-point time span must also be present: deskewing
  depends on it.
- **The mount checked, stage 2b of `docs/hardware_l2_test.md`.** LIO takes the L2's position
  and tilt from the URDF; no warnings on height and tilt. If this is skipped, run anyway, but
  write it in the results log: a mount error biases the result.
- **The laptop rides on the robot,** inside the self-filter box (the chassis footprint, below
  1 m), so it is removed from the cloud. The Ethernet cable must not trail into view.
- **The robot can be pushed.** The motors are unpowered. Check that it rolls forward and
  backward, and whether it slides sideways on its rollers. If sideways pushes need a lot of
  force or jerk the robot, leave them out and note it.
- **The pusher stays behind the robot,** low and close. Behind is the L2's blind side: the pole
  shadows the sector straight back. Nobody else walks near the robot during a run.
- **Tape marks.** See each stage. Measure them with a tape to the nearest centimetre. Use the
  same point on the robot every time, for example the front edge of the chassis plate at its
  centre, over the mark.

Build first: `colcon build`, then `source install/setup.bash` in every terminal.

## Running a stage

Each stage uses three terminals. Start them in this order, **with the robot standing still on
the start mark, facing along the first leg.** LIO estimates the IMU bias at start-up and needs
the robot still for about a second. Start the recording first, so that it holds LIO's start:
a recording that begins after LIO is already running cannot show what went wrong at start-up.

```bash
# Terminal 1: record the run (name it after the stage and attempt)
ros2 bag record -o lio_4h_stage2_run1 /l2/points /l2/imu /odom /lio/raw_odom /tf_static

# Terminal 2: driver, URDF TF, self-filter, scan, LIO, RViz
ros2 launch mecanum_bringup hardware_sensors.launch.py lio:=true

# Terminal 3: the mark logger
tools/lio_marks.py --csv lio_4h_stage2_run1.csv
```

In terminal 2, check for `Odometry map frame initialized using N IMU measurements` and an
`Estimated accel bias` near zero (a few hundredths of m/s²). In RViz, set Global Options >
Fixed Frame to `odom` to watch the robot move.

At each mark, type its label in terminal 3 and press Enter. At the end, type `q` and Enter;
the tool prints the table and writes the CSV. Then stop the recording and the launch with
Ctrl-C. rko_lio sometimes crashes on shutdown (exit code -11); that is harmless.

**Restart all three for each run,** so that `/odom` starts at zero on the start mark.

Push slowly and steadily, at about 0.15 m/s: 2 m in about 13 s. That is the speed the
simulation used.

**Measurement uncertainty.** A tape mark is placed to about ±1 cm, and the robot is stopped
over it to about ±1 to 2 cm. A reading at a mark is therefore uncertain by about ±0.03 m. Over
a 2 m push that is ±1.5%, more than the 1% drift target. So the 2 m push can tell "tracks"
from "does not track", but cannot judge the 1% target. The 18 m loop in stage 3 does that:
there the same ±0.03 m is under 0.2% of the path.

## 1. Start-up and static drift

1. Start the three terminals with the robot on any mark.
2. Press Enter at once (label `start`), then leave the robot alone for 5 minutes. Nobody moves
   near it.
3. Press Enter again (label `5min`), then `q`.

**Pass (stand-in targets):** under 0.05 m and under 0.5° after 5 minutes, and `lio z` stays
within ±0.05 m.

## 2. Straight push: the simulation comparison

This is the decisive stage: one 2 m push that repeats the motion failing in simulation.

**Which way to push.** Push **away from the wall**, not toward it. The direction matters:

- In simulation, LIO failed when the robot moved with the nearest end wall **behind** it, where
  the L2 sees little (the pole shadows the sector straight back), and the far end 7 m ahead.
  That is the start of the simulated corridor: the robot 0.7 m in front of a wall, facing away
  from it. LIO read 0.05 m after 2 m of true motion.
- When the robot drove **toward** a wall close ahead, LIO tracked well in the same simulation.
  A push toward the wall at the 3 m mark would end with the wall 1 m ahead, so LIO would likely
  track even if the real L2 had the problem. The test could then not show the failure.

**Setup.** Use the existing tape: marks at 0, 1, 2 and 3 m, with the wall at the 3 m mark.
Measure with the robot's **front edge** (the front of the chassis plate, at its centre).
1. Put the robot with its front edge on the **2 m mark, facing the 0 m mark**, so its back is
   to the wall. Its rear edge is then about 0.4 m from the wall.
2. Check there is room for the robot beyond the 0 m mark: its body is 0.6 m long, but it only
   needs to reach the 0 m mark with its front edge. Clear the area ahead of the 0 m mark; the
   more open floor ahead, the closer this is to the simulation.

**Run.** One run:
1. Start the three terminals (see "Running a stage") with the robot standing still on the
   2 m mark. Press Enter with the label `2m`.
2. Push it slowly and straight, at about 0.15 m/s, until its front edge is on the 1 m mark.
   Stop, and press Enter with the label `1m`.
3. Push on to the 0 m mark. Stop, and press Enter with the label `0m`.
4. Type `q`.

`/odom` starts at zero on the 2 m mark and its x axis points toward the 0 m mark, so `x`
should read about 1.00 at `1m` and 2.00 at `0m`.

**What the simulation showed for the same motion.** The simulated robot, starting 0.7 m in
front of a wall and pushed 2 m away from it, read **0.05 m** at the 2 m point: LIO saw almost
none of the motion. With the floor points removed before LIO, it saw about 75 to 80%.

**Reading the result.** Look at the `x` column at `0m`:

| `x` at `0m` | Share of the true 2 m | Meaning | Next step |
|---|---|---|---|
| 1.92 to 2.08 m | 96 to 104% | **The real L2 tracks.** The simulation failure is an artifact of the simulated scan grid. | Fix the simulated L2 (option 1), then re-run phase 4 in simulation. Continue with stage 3. |
| 1.60 to 1.92 m | 80 to 96% | Partial: LIO under-reads the motion. | Analyse the bag offline (below) before deciding. |
| Under 1.60 m | Under 80% | **The real L2 has the same failure.** Under about 0.3 m, it is the simulation's failure exactly. | Stop here. The LIO approach needs to change: other rko_lio settings, floor removal, another LIO, or the rf2o fallback. The bag allows testing these offline. |

With a single run there is no median to take, so the bands are wide: the "tracks" band,
±0.08 m, is about 2.5 times the ±0.03 m reading uncertainty. A reading inside it cannot be a
failing LIO that got lucky, and a reading outside it is not just a misplaced mark. The bands
are far apart from the simulation's 0.05 m, so the answer should be clear either way. If the
reading lands within 0.03 m of a boundary, repeat the run once.

Also note `y` (sideways error, expected within ±0.05 m) and `yaw` (within ±1°) at `0m`.

## 3. Closed loop

Only if stage 2 shows that LIO tracks; otherwise the drift numbers mean nothing.

**Setup.** Tape a rectangle 2 m long and 1 m wide between the 0 m and 2 m marks, with a start
mark at one corner and a direction arrow along a long side. One lap is 6 m; three laps, 18 m,
come close to the plan's 20 m. Each corner gets a mark.

The lap, starting on the start mark facing along the arrow:
1. Push forward 2 m to corner `c1`.
2. **Short side.** If the robot slides sideways, push it 1 m sideways to `c2` without turning.
   Otherwise, turn it 90° on the spot, push it 1 m forward to `c2`, and turn it back.
3. Turn 180° on the spot, and push forward 2 m to `c3`.
4. Short side to the start mark, the same way as step 2, then turn 180° to face along the
   arrow again.

That is like the simulation's loop: straight legs, sideways legs and turns on the spot.

**Run.**
1. Start the three terminals with the robot on the start mark, facing along the arrow. Press
   Enter with the label `start`.
2. Push three laps. Press Enter at each corner if you like (`c1`, `c2`, `c3`, ...).
3. Stop exactly on the start mark, facing along the arrow, and press Enter with `end`.
4. Type `q`.

**Result.** At `end`, the `|xy|` column is the position error, and `yaw` the heading error.
Divide the error by 18 m. The corner marks show where the error grew.

**Pass:** under 0.18 m (1% of 18 m) and under 2° of yaw. These are the phase 4 targets
(stand-ins). The ±0.03 m reading uncertainty is small against 0.18 m.

## 4. Long corridor (optional)

Only if a corridor of 10 m or more is available somewhere. The 2 m test area cannot show
degeneracy along a long featureless corridor; the simulation's `corridor.sdf` world covers it
instead.

**Setup.** The longest straight corridor available, with plain walls.
Tape a start mark and an end mark, and measure the distance between them.

**Run.** Start on the start mark, facing along the corridor. Push to the end mark and press
Enter (`end`), then push back to the start mark and press Enter (`back`).

**Pass:** at `end`, `x` is within 1% of the taped length; at `back`, `|xy|` is under 1% of
twice the length.

## CPU load

During stage 3, in another terminal:

```bash
top -b -d 5 -n 6 -p $(pgrep -f rko_lio/online_node) | grep online_node
```

Note the %CPU and the laptop model. In simulation rko_lio used 9 to 15% of one core. This
informs the choice of the onboard computer.

## Offline replay of a recording

Every bag can be re-run through LIO without the robot, for example with other settings in a
copy of `rko_lio_hardware.yaml`. Start the launch first, then play only the sensor topics:

```bash
ros2 launch mecanum_bringup hardware_sensors.launch.py replay:=true lio:=true \
  lio_config:=/path/to/my_rko_lio.yaml
ros2 bag play lio_4h_stage2_run1 --clock --topics /l2/points /l2/imu
```

`replay:=true` leaves out the driver and puts every node on the bag's clock. The recorded
`/odom` is not played, so the new LIO's output does not mix with the old one.

## What to report back

For each run: the table printed by `lio_marks.py` (or its CSV), the bag directory, and any
notes, such as people walking by, a stuck roller, or the cable in view.

## Results log

| Date | Who | Stage | Run | Setup notes | Result | Pass? |
|---|---|---|---|---|---|---|
| 2026-10-15 | Owner | 1 | `lio_4h_stage2_start` | Robot still. `cloud_scan_num` 18 (driver default), 3H not run. Recording started after LIO. The L2 stream stopped after 15.5 min for 23 min, then resumed; LIO did not recover. | `/odom` wandered by up to 4.3 m and 120° of yaw while still; 1344 m of phantom path. | No |
| 2026-10-16 | Replay | 1 | Same recording, offline | LIO restarted on the recording. Each change tested on its own: deskew off, all threads, a noise-free IMU, an IMU agreeing with the lidar floor, cloud stamps at sweep end, a 0.1 m map, the raw (unfiltered) cloud, initialization off. | All diverged the same way: about 16° of yaw in the first 1.2 s, about 50° after 6 s. | No |
| 2026-10-16 | Replay | 1 | Same recording, 2 clouds merged per LIO scan | 36 scan lines per cloud, close to a full sweep, as with `cloud_scan_num` about 39. | After 5 min: 0.014 m and 0.01°. Largest excursion 0.067 m and 2.2° yaw in the first seconds. rko_lio used about 6% of one core. | Yes, at 5 min |

Stage 2 summary:

| Run | `x` at `1m` | `x` at `0m` | `y` at `0m` | `yaw` at `0m` |
|---|---|---|---|---|
| Real L2, pushed 2 m away from the wall | | | | |
| Simulation, for reference | 0.05 | 0.05 | 0.03 | about −0.5° |
