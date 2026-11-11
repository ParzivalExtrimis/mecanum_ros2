# 0005: Ultrasonics and glass in navigation (decision D6)

- **Status:** accepted (2026-11-05). Decision D6 of [`plan.md`](../plan.md) Phase 6, chosen by
  the owner as D6-a.
- **Date:** 2026-11-05
- **Revisit when:**
  - a site has glass or mirrors that are not known in advance (doors, moving screens);
  - the real ultrasonics (DFRobot A02YYUW) are measured: their beam, their response to glass
    at an angle, and crosstalk between the four;
  - the robot gets stuck at doorways or the stall opening because of the range layer.

## The problem

- **Only the ultrasonics see glass and mirrors.** The lidars and the camera see through glass
  in simulation (shortcut S4), and real ones mostly do.
- **A 60° cone reports only its nearest echo.** Nav2's range layer then marks the whole arc at
  that distance, most strongly at its centre. Approaching the 0.9 m doorway, the jambs enter
  the front cones at about 0.5 m and the arc lies across the doorway's middle: a phantom
  obstacle where the robot has to go. The same happens at the stall opening and, in Phase 7,
  in front of the toilet.
- **The planner cannot use what only the local costmap knows.** An obstacle in the local
  costmap alone stops MPPI but does not move the global path.

## The options

| Option | What | For | Against |
|---|---|---|---|
| **D6-a** | Known glass in the keepout mask. Ultrasonics feed the collision monitor (slow down, stop) and a range layer in the local costmap only. | Known glass is planned around; the global plan is never blocked by a phantom | Unknown glass is not planned around: the robot stops before it, and Nav2 gives up |
| D6-b | The range layer in both costmaps | The planner detours around unknown glass | Phantom obstacles can close the doorway and the stall opening to the planner |
| D6-c | The collision monitor only, no range layer | Least phantom risk | Unknown glass is only ever a stop |

## Decision: D6-a

- **Keepout mask.** The glass screen in `restroom.sdf` is one of the keepout zones
  (`mecanum_navigation/maps/restroom_keepout_zones.yaml`; taken from the world file, shortcut
  S10).
- **Collision monitor.** Sources include all four ultrasonics. The front stop (0.15 m) and
  slowdown (0.5 m) polygons act when moving forward or turning; the rear stop polygon
  (0.15 m) acts only when reversing and uses only the rear ultrasonics. All STAND-IN.
- **Range layer, local costmap only.** Through `range_relay`, which turns "no echo" (+inf) into
  a max-range reading so that the layer clears its cone; the layer ignores +inf.
- **Fallback if the layer blocks the doorway:** `mark_max_range` caps what the layer may mark
  (owner: about 0.4 m). It is a `range_relay` parameter and a launch argument.
- **Pass criterion for unknown glass:** the robot stops before contact. No detour is expected.

## Consequences

Measured in plan Phase 6 (2026-11-06, two full `tools/nav_check.py` runs; details there):

- **Known glass is planned around.** The goal behind the glass screen is reached with 7 to
  8.5 cm to the nearest object. This needed the mask as a static layer before inflation, not
  Nav2's keepout filter: filtered cells are not inflated, the footprint checks skipped them,
  and the robot's corner cut the end of the screen.
- **Unknown glass is a stop.** A glass panel across the corridor, seen only by the
  ultrasonics: the robot stopped 10 to 18 cm short, without contact, and Nav2 gave up, as
  decided.
- **The fallback is on.** Uncapped, the range layer marked the doorway's jambs across the
  opening, and in a run of four goals through the doorway the robot was held twice until a
  recovery cleared the costmap. `mark_max_range` 0.4 m is now the default; with it, none.
- **Reverse.** The rear stop polygon on the rear ultrasonics stopped the robot 16.5 cm short of
  a box behind it when driven straight into the collision monitor at −0.12 m/s, and Nav2's
  BackUp stopped 21.4 cm short.
- **The range layer constrains the recoveries.** A partial clear of the local costmap leaves
  Nav2's range layer not current for good (Nav2 1.3.13), so the behaviour trees never clear
  the local costmap; its marks clear when a cone reads its maximum.
