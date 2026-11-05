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
Compare a saved 2D map with the world it was built in (docs/plan.md Phase 5).

    tools/map_check.py src/mecanum_navigation/maps/restroom.yaml

Needs no running ROS: it reads the map_server files and the world SDF (shortcut S1: the world
file is evaluation only). The map frame is placed in the world at --map-pose, the robot's spawn
pose, because slam_toolbox starts the map where the robot starts.

Judged, PASS or FAIL:
  each wall face at least --min-length long with at least --min-cells cells along it: the
  median signed offset of its cells from the true face is within --tolerance (owner,
  2026-11-04: 5 cm, one cell). Positive is into the room.
Reported only:
  the cells' distance to the nearest expected shape (median, 95th percentile, max);
  the best-fit alignment of the whole map onto the world, and the residuals after it;
  the expected fixtures with no cells, which the route never showed the lidar, and the ones
  the 2D lidar cannot see (above or below its plane, or hidden like the glass).
"""

import argparse
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from mecanum_localization import map_compare as mc
import numpy as np

SCAN_HEIGHT = 0.13  # m, the C1's scan plane (mecanum_description)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map', help='map_server YAML file')
    parser.add_argument('--world-file', default=os.path.join(
        get_package_share_directory('mecanum_simulation'), 'worlds', 'restroom.sdf'))
    parser.add_argument('--map-pose', type=float, nargs=3, default=(0.0, -1.0, 0.0),
                        metavar=('X', 'Y', 'YAW'),
                        help='Pose of the map frame in the world (default: the spawn pose)')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='Largest |median offset| of a judged face, m')
    parser.add_argument('--min-length', type=float, default=1.0,
                        help='Shortest face that is judged, m')
    parser.add_argument('--min-cells', type=int, default=10,
                        help='Fewest cells along a face for it to be judged')
    parser.add_argument('--all-faces', action='store_true',
                        help='Also list the faces that are not judged')
    args = parser.parse_args(argv)

    shapes = mc.load_world_shapes(args.world_file)
    expected = mc.expected_shapes(shapes, SCAN_HEIGHT)
    cells, resolution = mc.load_occupied_cells(args.map)
    if len(cells) == 0:
        sys.exit('The map has no occupied cells')
    world_cells = mc.transform(cells, tuple(args.map_pose))
    print(f'map {args.map}: {len(cells)} occupied cells at {resolution} m; '
          f'world {os.path.basename(args.world_file)}: {len(expected)} of {len(shapes)} shapes '
          f'in the scan plane ({SCAN_HEIGHT} m) and visible')

    failures = 0
    print('\nWall faces (median signed offset, + into the room):')
    for face in mc.face_offsets(world_cells, expected, min_cells=args.min_cells,
                                min_length=args.min_length):
        if not face['judged']:
            if args.all_faces and face['cells']:
                print(f"  ----  {face['shape']:<40} {face['face']}  {face['length']:5.2f} m  "
                      f"{face['cells']:4d} cells  {face['median'] * 100:+6.1f} cm  (not judged)")
            continue
        ok = abs(face['median']) <= args.tolerance
        failures += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {face['shape']:<40} {face['face']}  "
              f"{face['length']:5.2f} m  {face['cells']:4d} cells  "
              f"{face['median'] * 100:+6.1f} cm")

    _, distance = mc.nearest_shape(world_cells, expected)
    print('\nCells to the nearest expected shape, as anchored: '
          f'median {np.median(distance) * 100:.1f} cm, '
          f'95th {np.percentile(distance, 95) * 100:.1f} cm, max {distance.max() * 100:.1f} cm; '
          f'{int((distance > 0.10).sum())} cells farther than 10 cm')
    for d, (x, y) in sorted(zip(distance, world_cells), key=lambda t: -t[0])[:5]:
        if d > 0.10:
            print(f'    stray cell at world ({x:.2f}, {y:.2f}), {d * 100:.0f} cm from any shape')

    pose, residual = mc.best_fit(cells, expected, initial=tuple(args.map_pose))
    dx, dy = pose[0] - args.map_pose[0], pose[1] - args.map_pose[1]
    dyaw = math.degrees(pose[2] - args.map_pose[2])
    print(f'Best fit of the whole map: moved by ({dx * 100:+.1f}, {dy * 100:+.1f}) cm, '
          f'{dyaw:+.2f} deg; residual median {np.median(residual) * 100:.1f} cm, '
          f'95th {np.percentile(residual, 95) * 100:.1f} cm')

    counts = mc.shape_coverage(world_cells, shapes)
    print('\nExpected fixtures with no cells within 10 cm (never shown to the lidar):')
    for shape, count in zip(shapes, counts):
        if shape in expected and not count:
            print(f'  {shape.name}')
    print('Not expected in a 2D lidar map (cells within 10 cm are usually a wall beside it):')
    for shape, count in zip(shapes, counts):
        if shape in expected:
            continue
        if shape.hidden:
            why = 'hidden from the lidars (glass)'
        else:
            low, high = shape.z_range
            why = f'outside the scan plane (z {low:.2f}..{high:.2f} m)'
        print(f'  {shape.name:<40} {why}, {count} cells')

    print(f'\n{failures} failure(s)')
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
