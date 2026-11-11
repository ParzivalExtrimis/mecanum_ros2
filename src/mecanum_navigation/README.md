# mecanum_navigation

Maps, keepout masks, global localization and the Nav2 configuration for the mecanum base
([docs/plan.md](../../docs/plan.md) Phases 5 and 6). Started in simulation by
`mecanum_bringup/launch/navigation.launch.py`.

| What | Where |
|---|---|
| Nav2 parameters | `config/nav2.yaml`, from the upstream `nav2_bringup` parameters; every change says why |
| AMCL and map_server | `config/localization.yaml` |
| Nav2 launch | `launch/nav2.launch.py` |
| Localization launch | `launch/localization.launch.py`: AMCL (default) or slam_toolbox localization |
| Behaviour trees | `behavior_trees/`: the upstream trees plus a goal checker selector; the recoveries never clear the local costmap, clear the global one except around the robot, and also run on "start occupied" |
| Maps and keepout masks | `maps/` |
| Ultrasonic relay | `range_relay` (`mecanum_navigation/range_relay.py`) |
| D435i cloud for Nav2 | `cloud_thinner` (`mecanum_navigation/cloud_thinner.py`) |
| Keepout geometry | `mecanum_navigation/keepout.py`; `tools/make_keepout_mask.py` is its command line |

## Localization

`launch/localization.launch.py`, argument `localization`:
- **`amcl`** (default): map_server serves `maps/restroom.yaml` on `/map`, and AMCL with the
  `OmniMotionModel` on the 2D lidar's `/scan` publishes `map -> odom`. It starts at home, the
  map frame's origin (the robot's spawn pose). Give another start pose on `/initialpose`
  (RViz "2D Pose Estimate").
- **`slam_toolbox`**: slam_toolbox in localization mode
  (`mecanum_localization/launch/slam_localization.launch.py`) on a pose graph (`posegraph`,
  default `maps/restroom`). It serves `/map` and `map -> odom` itself; AMCL and the map's
  map_server do not run. **It loses track in the corridor (Phase 6): use AMCL.** In both runs
  it lost the robot near the corridor's east end (x 4.5 m), 30 cm off at the goal and up to
  95 cm later; one run recovered in the restroom, one did not.

  The pose graph is not in git (12 MB, `maps/.gitignore`). Make it under its own name, not
  `restroom`: saving over `restroom` replaces the committed map, and the keepout mask is on
  that map's grid. slam_toolbox serves the map of its own session, so the mask must be made on
  that map's grid too. In Phase 6 the global costmap resized itself 19 times in a run with
  the committed map's mask, 7 times with a mask on slam_toolbox's map, once with AMCL:

  ```bash
  ros2 launch mecanum_bringup mapping.launch.py gui:=false rviz:=false
  tools/map_route.py
  tools/save_map.py restroom_slam            # .pgm, .yaml, .posegraph, .data in maps/
  tools/make_keepout_mask.py src/mecanum_navigation/maps/restroom_slam.yaml \
    --zones src/mecanum_navigation/maps/restroom_keepout_zones.yaml
  colcon build --packages-select mecanum_navigation
  # stop the mapping launch first
  maps=$(ros2 pkg prefix mecanum_navigation)/share/mecanum_navigation/maps
  ros2 launch mecanum_bringup navigation.launch.py localization:=slam_toolbox \
    posegraph:=$maps/restroom_slam keepout_mask:=$maps/restroom_slam_keepout.yaml
  ```

  The zones are in the committed map's frame; a new map from the spawn pose is within about
  1 cm of it.

Only one of the two publishes `map -> odom`. `odom -> base_footprint` stays the EKF's.

## Nav2

`launch/nav2.launch.py` is modelled on the upstream `navigation_launch.py`. It leaves out
route_server (it needs a route graph, and a node that fails to configure stops the whole
bringup) and docking_server (Phase 7).

```
controller_server (MPPI), behavior_server --/cmd_vel_nav--> velocity_smoother
  --/cmd_vel_smoothed--> collision_monitor --/cmd_vel--> safety gate (mecanum_safety)
```

- **Stamped.** Every node that publishes or takes a velocity has `enable_stamped_cmd_vel: true`;
  `mecanum_drive_controller` takes `TwistStamped` only.
- **Speed limits.** The controller's 0.3 m/s, 0.3 m/s and 0.8 rad/s, never more: it clamps each
  axis on its own (Phase 3), so an over-limit command changes direction. Reverse is capped at
  0.15 m/s (STAND-IN), because only the rear ultrasonics see near objects behind.
- **Planner:** Smac State Lattice with the installed omnidirectional primitives (forward,
  sideways, diagonal, in-place turns), checked against the full footprint. Its path carries
  the heading; the robot faces a doorway before it reaches it.
- **Controller:** MPPI, `Omni` motion model, upstream sizes. It follows the path's headings
  (`PathAlignCritic` `use_path_orientations`), prefers forward and sideways motion
  (`PreferForwardCritic`), and checks the full footprint (`CostCritic` `consider_footprint`).
- **Goal checkers:** `staging` (0.10 m, 0.10 rad, the default) and `precise` (0.05 m,
  0.05 rad), both STAND-IN. A goal picks one by publishing its name on
  `/goal_checker_selector` (`std_msgs/String`, transient local) before it is sent; the
  behaviour trees' `GoalCheckerSelector` passes it to `FollowPath`.
- **Footprint:** x −0.405..0.421, y ±0.258 m, from `mecanum_description`
  (`test/test_nav2_config.py` checks it against the URDF).
- **Local costmap** (odom, 4 × 4 m, 0.05 m): the keepout mask; the 2D lidar; a
  spatio-temporal voxel layer on the D435i cloud (marks and clears in its frustum) and the
  filtered L2 cloud (marks only, the marks decay), 0.04 to 0.90 m; a range layer on the
  ultrasonics; inflation 0.55 m.
- **Global costmap** (map): the map, the keepout mask, the 2D lidar, the voxel layer,
  inflation. No range layer (decision 0005).
- **Keepout mask: a static layer, not Nav2's keepout filter.** Costmap filters run after
  inflation, so keepout cells are not inflated, and Smac's and MPPI's footprint checks only run
  where the robot's centre is on an inflated cell. With the filter the robot's corner cut the
  end of the glass screen. As a second `StaticLayer` (`keepout_layer`, on `/keepout_mask`)
  before inflation, the zones are inflated like walls. The mask is grey (unknown) outside the
  zones, and the global costmap merges by maximum (`use_maximum`), so the layer changes no
  other cell. The mask must be on the map's grid.
- **The D435i cloud is thinned for Nav2.** The voxel layers and the collision monitor subscribe
  best-effort, which cannot be set from outside, and Cyclone DDS loses whole 1.2 MB
  best-effort messages at the default socket buffers (Phase 2). Facing a wall, the collision
  monitor received no cloud for minutes and held the robot stopped. `cloud_thinner` subscribes
  reliably and publishes `/costmap/d435i/points`, one point per 5 cm cube (under 70 kB).
- **Recoveries keep what is near the robot.** Upstream's recoveries clear both costmaps
  entirely. The sensors look forward, so a cleared costmap forgets what is beside and behind
  the robot, and what is too close below the D435i to be seen again; the next spin, back-up
  or approach hit the corridor bench and a low box (Phase 6). The trees here clear only the
  global costmap, outside a 2 × 2 m square around the robot (`ClearCostmapExceptRegion`).
  The local costmap is not cleared at all: a partial clear leaves Nav2's range layer not
  current for good, after which the controller fails every goal ("Costmap timed out waiting
  for update", Nav2 1.3.13). Its stale marks clear themselves: voxels decay in 15 s, the scan
  raytraces, the range cones clear on a max reading.
- **"Start occupied" runs the recoveries.** Upstream fails the goal at once when the planner
  finds the start in collision. Next to a jamb the robot's clearance is close to the costmap's
  error (5 cm cells, a few cm of localization): in a Phase 6 run the footprint touched a wall
  cell 12 cm from the real wall, and every later goal failed in under a second. The trees here
  run the recoveries for it too (`AreErrorCodesPresent` 205 / 305), and the back-up moves the
  robot clear.
- **Collision monitor:** a front stop zone 0.15 m and a front slowdown zone 0.5 m deep when
  driving forward; a rear stop zone 0.15 m deep when reversing, on the rear ultrasonics; and
  footprint approach on the scan and the D435i. Sizes STAND-IN. The zones are velocity
  polygons, so a wall behind never stops forward motion, and the robot may turn in place
  next to a wall.

### Ultrasonics and glass (decision D6-a, [0005](../../docs/decisions/0005-ultrasonics-and-glass-in-navigation.md))

Known glass is a keepout zone. The ultrasonics feed the collision monitor, and a range layer
in the local costmap only, through `range_relay`:

```
/ultrasonic/<pos> --range_relay--> /costmap/ultrasonic/<pos>
```

The layer ignores a reading of +inf ("no echo"), so the marks it made would never clear. The
relay turns +inf into a max-range reading, which clears the cone. Its `mark_max_range` (the
`mark_max_range` launch argument, 0 = off) caps what the layer may mark: a reading beyond the
cap clears the cone up to the cap and marks nothing. The default is 0.4 m, the D6-a fallback:
without it the wide cones marked the doorway's jambs across the opening, and the robot waited
in the doorway until a recovery cleared the costmap (Phase 6; the recoveries no longer clear
the local costmap).

## Maps

| Map | Built from | Notes |
|---|---|---|
| `restroom.pgm`, `restroom.yaml` | `restroom.sdf`, `tools/map_route.py`, slam_toolbox on the 2D lidar (2026-11-05) | 0.05 m cells. The map frame is the robot's spawn pose, world (0, −1, 0°). `tools/map_check.py`: every wall face within 2.3 cm, residual 0.8 cm after a best fit. |
| `restroom_keepout_zones.yaml` | `restroom.sdf`, `tools/make_keepout_mask.py --from-world` (shortcut S10) | Five zones in the map frame, each grown by 5 cm: the two stall partitions, the toilet tank, the threshold, the glass screen |
| `restroom_keepout.pgm`, `.yaml` | the zones, on the map's grid | Black is keepout, grey elsewhere |

What the map misses, by design (decision D3-a): the stall partitions (only their post is in the
map), the 5 cm threshold in the corridor, the glass screen in the lobby, and the toilet tank's
overhang. The keepout mask covers them; the 3D costmap layers cover what is not known in
advance.

Make or replace a map while `mecanum_bringup mapping.launch.py` runs:
```bash
tools/save_map.py <name>        # writes <name>.pgm, .yaml, .posegraph, .data here
tools/map_check.py src/mecanum_navigation/maps/<name>.yaml
tools/make_keepout_mask.py src/mecanum_navigation/maps/<name>.yaml --from-world   # sim only
```
On a real site, write `<name>_keepout_zones.yaml` by hand (polygons in the map frame, as in the
restroom's), then run `tools/make_keepout_mask.py src/mecanum_navigation/maps/<name>.yaml`
without `--from-world`. Rebuild the package after adding files to `maps/`.

The pose graph (`.posegraph`, `.data`, about 12 MB) is what slam_toolbox's localization mode and
continued mapping load. It is kept out of git (`maps/.gitignore`).

## Checking

```bash
ros2 launch mecanum_bringup navigation.launch.py gui:=false rviz:=false
tools/nav_check.py                    # goals, map gaps, obstacles, reversing, relocalization
tools/nav_check.py --only suite --goals corridor_east doorway_in
```

`tools/nav_check.py` judges on Gazebo ground truth (shortcut S1); see its help for the stages
and the criteria. Results are in `docs/plan.md`, Phase 6.
