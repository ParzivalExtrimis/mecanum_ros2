# 0004: Bumper protrusion deferred, sideways cliff gap accepted (decisions D2 and D5)

- **Status:** accepted (2026-11-03). Decisions D2 and D5 of [`plan.md`](../plan.md) Phase 4.
- **Date:** 2026-11-03
- **Revisit when:**
  - the mechanical design of the bumpers or the front sensor mounts is settled (D2);
  - a site has a step, stair, ramp edge or other drop beside the robot's path (D5);
  - the cliff sensors move, or more are added (D5);
  - the real VL53L1X is measured over an edge, since the simulated cone reports its nearest
    return (D5).

## D2: the front bumper protrusion, noted and deferred

- **The problem.** In the STEP the front plank face is at x = 0.404 m. The 2D lidar stands
  16 mm past it (0.420 m) and the front ultrasonics 4 mm (0.408 m). A wall at their height hits
  them before the plank can switch, so the bumper does not protect them, and a collision there
  goes undetected by the bumpers. The PDF intends the planks to stand 3 cm proud of the faces.
  The rear plank is 3 mm proud of the rear ultrasonics.
- **Decision.** Nothing changes for now. The planks stay where the STEP puts them
  (`bumper_x` in `bumpers.xacro`, a STAND-IN), and `config/safety.yaml` matches them.
- **Consequences.**
  - **The bumper test uses the low wall.** The bumper wall in `sensor_test.sdf` is lower than
    the lidar and the body, so the bumper checks pass without depending on D2. Against a full
    height wall the lidar touches first.
  - **The collision monitor matters more.** Until the planks move, frontal contact at the
    lidar's height is left to the collision monitor (Phase 6), which acts on the scan before
    contact.
  - **If the planks move later,** `bumper_x` changes in `bumpers.xacro` and `safety.yaml`
    together; `test_layout.py` checks that they match.

## D5: the sideways cliff gap, accepted

- **The problem** (measured, plan Phase 4).
  - **The sensors are inboard of the wheels.** The cliff sensors sit at y ±0.208 m, 7 mm
    inboard of the wheel centres (±0.215 m), and the wheels reach ±0.258 m.
  - **The simulated cone sees the drop late.** It reports the nearest return, so it flags a
    drop only once its whole footprint (2.9 cm radius) is past the edge.
  - **Strafing toward a drop.** The leading wheel is 6 to 7 cm over the edge when the block
    comes, and the robot falls. That happened in 12 of 12 runs, at 0.1, 0.2 and 0.3 m/s.
  - **Forward and reverse pass,** with 4.7 to 7.4 cm to spare.
- **The options.**
  - **D5-a.** Outboard or sideways cliff sensors.
  - **D5-b.** A capped sideways speed with keepout zones.
  - **D5-c.** Accept and record the gap.
- **Decision: D5-c.** The gap is accepted, for sites with no drop beside the robot's path. The
  sideways cliff check in `tools/safety_check.py` stays, and is expected to fail. It reports
  the gap instead of hiding it.
- **Consequences.**
  - **Cliff protection is forward and reverse only.** A drop beside the robot, or one reached
    while turning, is not detected in time.
  - **Site requirement.** Each site must be checked for drops beside the robot's path. A site
    with any reopens D5. D5-b's keepout zones (Phases 5 and 6) are then the fastest mitigation,
    and D5-a is the fix.
