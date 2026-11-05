# mecanum_navigation

Maps and, from plan Phase 6, the Nav2 configuration for the mecanum base
([docs/plan.md](../../docs/plan.md)).

- **Built (Phase 5):** `maps/`, installed to `share/mecanum_navigation/maps`.
- **Planned (Phase 6):** Nav2 parameters (MPPI Omni, Smac 2D, costmaps, collision monitor),
  AMCL, the keepout mask and the launch file used by `mecanum_bringup/navigation.launch.py`.

## Maps

| Map | Built from | Notes |
|---|---|---|
| `restroom.pgm`, `restroom.yaml` | `restroom.sdf`, `tools/map_route.py`, slam_toolbox on the 2D lidar (2026-11-05) | 0.05 m cells. The map frame is the robot's spawn pose, world (0, −1, 0°). `tools/map_check.py`: every wall face within 2.3 cm, residual 0.8 cm after a best fit. |

What the map misses, by design (decision D3-a): the stall partitions (only their post is in the
map), the 5 cm threshold in the corridor, the glass screen in the lobby, and the toilet tank's
overhang. Phase 6 covers them with a keepout mask and the 3D costmap layers.

Make or replace a map while `mecanum_bringup mapping.launch.py` runs:
```bash
tools/save_map.py <name>        # writes <name>.pgm, .yaml, .posegraph, .data here
tools/map_check.py src/mecanum_navigation/maps/<name>.yaml
```
The pose graph (`.posegraph`, `.data`, about 12 MB) is what slam_toolbox's localization mode and
continued mapping load. It is kept out of git (`maps/.gitignore`); make it again with the
route if it is needed.
