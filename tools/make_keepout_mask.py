#!/usr/bin/env python3
# Copyright 2026 Aryan Hegde
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Make the keepout mask for a map (docs/plan.md Phase 6; a static layer in both costmaps).

    tools/make_keepout_mask.py src/mecanum_navigation/maps/restroom.yaml --from-world

Two steps; both by default write next to the map:
  1. With --from-world, write <map>_keepout_zones.yaml from the world SDF (shortcut S10): every
     static shape between the floor and the robot's top that the 2D map cannot hold (it misses
     the 2D lidar's plane, or the glass mask hides it), grown by --margin. Without
     --from-world the zones file is read as it is, so a hand-drawn one for a real site works.
  2. Rasterize the zones onto the map's grid: <map>_keepout.pgm and .yaml (black is keepout,
     grey elsewhere).

Needs no running ROS. The map frame is placed in the world at --map-pose, the spawn pose.
"""

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from mecanum_localization import map_compare as mc
from mecanum_navigation import keepout
import yaml

SCAN_HEIGHT = 0.13   # m, the C1's scan plane (mecanum_description)
ROBOT_TOP = 0.90     # m, the robot's top (0.83) plus a margin, as the costmaps' 3D layers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map', help='map_server YAML file')
    parser.add_argument('--from-world', action='store_true',
                        help='Write the zones file from the world SDF first (S10)')
    parser.add_argument('--world-file', default=os.path.join(
        get_package_share_directory('mecanum_simulation'), 'worlds', 'restroom.sdf'))
    parser.add_argument('--map-pose', type=float, nargs=3, default=(0.0, -1.0, 0.0),
                        metavar=('X', 'Y', 'YAW'),
                        help='Pose of the map frame in the world (default: the spawn pose)')
    parser.add_argument('--margin', type=float, default=0.05,
                        help='Growth of each zone, m: covers the map offset from the world')
    parser.add_argument('--zones', help='Zones file (default <map>_keepout_zones.yaml)')
    parser.add_argument('--out', help='Mask prefix (default <map>_keepout)')
    args = parser.parse_args(argv)

    base = os.path.splitext(os.path.abspath(args.map))[0]
    zones_path = args.zones or base + '_keepout_zones.yaml'
    out = args.out or base + '_keepout'

    if args.from_world:
        shapes = mc.load_world_shapes(args.world_file)
        zones = keepout.zones_from_world(shapes, SCAN_HEIGHT, (0.0, ROBOT_TOP), args.margin,
                                         tuple(args.map_pose))
        header = (
            '# Keepout zones for Nav2 (docs/plan.md Phase 6), polygons in the map frame (m).\n'
            '# Written by tools/make_keepout_mask.py --from-world from '
            f'{os.path.basename(args.world_file)}\n'
            '# (shortcut S10: on a real site, measure and draw them). The map frame sits at\n'
            f'# {list(args.map_pose)} in the world; each zone is grown by {args.margin} m.\n')
        with open(zones_path, 'w') as f:
            f.write(header)
            yaml.safe_dump({'frame': 'map', 'zones': zones}, f, default_flow_style=None,
                           sort_keys=False)
        print(f'wrote {zones_path}: {len(zones)} zones')
        for zone in zones:
            print(f"  {zone['name']}: {zone['why']}")

    zones = keepout.load_zones(zones_path)
    with open(args.map) as f:
        meta = yaml.safe_load(f)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(args.map)), image)
    height, width = mc.read_pgm(image).shape
    grid = keepout.rasterize(zones, width, height, float(meta['resolution']), meta['origin'])
    keepout.write_mask(out, grid, float(meta['resolution']), meta['origin'])
    print(f'wrote {out}.pgm/.yaml: {int(grid.sum())} keepout cells of {grid.size}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
