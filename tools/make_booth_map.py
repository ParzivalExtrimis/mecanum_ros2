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
Write the booth's occupancy map straight from the world file (sim shortcut S11).

    tools/make_booth_map.py                      # booth.sdf -> mecanum_navigation/maps/booth.*
    tools/make_booth_map.py --world W --out PREFIX --bounds -4.2 -4.2 4.2 1.2

Every static box and cylinder that crosses the 2D lidar's plane (0.13 m) becomes occupied
cells; everything else inside the bounds is free. The map frame is the world frame. A cell is
occupied when its centre is inside a shape, so the outlines are within half a cell diagonal
(1.8 cm at the default 2.5 cm) of the world.

Only each shape's outline is kept occupied; its inside is unknown, as in a map built from the
lidar, which never sees inside a wall. With solid walls, AMCL's likelihood field scores a beam
ending anywhere inside a wall as a perfect hit, and in the booth the estimate slid 7 to 10 cm
into the walls in front of it.

On the real booth this tool is not used: the map comes from mapping.launch.py and
tools/save_map.py, as in Phase 5, and the demo poses are then re-recorded in that map.
"""

import argparse
import os
import sys

from mecanum_localization import map_compare as mc
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCAN_HEIGHT = 0.13


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--world', default=os.path.join(
        REPO, 'src', 'mecanum_simulation', 'worlds', 'booth.sdf'))
    parser.add_argument('--out', default=os.path.join(
        REPO, 'src', 'mecanum_navigation', 'maps', 'booth'))
    parser.add_argument('--resolution', type=float, default=0.025)
    parser.add_argument('--bounds', type=float, nargs=4, default=[-4.2, -4.2, 4.2, 1.2],
                        metavar=('X0', 'Y0', 'X1', 'Y1'))
    args = parser.parse_args(argv)

    shapes = mc.expected_shapes(mc.load_world_shapes(args.world), SCAN_HEIGHT)
    x0, y0, x1, y1 = args.bounds
    res = args.resolution
    width = int(round((x1 - x0) / res))
    height = int(round((y1 - y0) / res))
    cols, rows = np.meshgrid(np.arange(width), np.arange(height))
    centres = np.column_stack([(x0 + (cols + 0.5) * res).ravel(),
                               (y0 + (height - 1 - rows + 0.5) * res).ravel()])
    occupied = np.zeros(len(centres), dtype=bool)
    for shape in shapes:
        occupied |= shape.signed_distance(centres) <= 0.0
    occupied = occupied.reshape(height, width)
    padded = np.pad(occupied, 1, constant_values=False)
    inside = (occupied & padded[:-2, 1:-1] & padded[2:, 1:-1]
              & padded[1:-1, :-2] & padded[1:-1, 2:])
    outline = occupied & ~inside
    pixels = np.where(outline, 0, np.where(inside, 205, 254)).astype(np.uint8)

    with open(args.out + '.pgm', 'wb') as f:
        f.write(f'P5\n# booth map from {os.path.basename(args.world)}\n'
                f'{width} {height}\n255\n'.encode())
        f.write(pixels.tobytes())
    meta = {'image': os.path.basename(args.out) + '.pgm', 'mode': 'trinary',
            'resolution': float(res), 'origin': [float(x0), float(y0), 0.0], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.196}
    with open(args.out + '.yaml', 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print(f'{len(shapes)} shapes, {width} x {height} cells, {int(outline.sum())} occupied, '
          f'{int(inside.sum())} unknown inside shapes '
          f'-> {args.out}.pgm/.yaml')
    return 0


if __name__ == '__main__':
    sys.exit(main())
