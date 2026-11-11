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
Keepout zones and their mask for the Nav2 costmaps (docs/plan.md Phase 6).

Pure geometry, no ROS; tools/make_keepout_mask.py is the command line over it.

- A zone is a named polygon in the map frame. Zones live in a YAML file
  (maps/<map>_keepout_zones.yaml), which can be written by hand on a real site.
- In simulation the zones are taken from the world file (shortcut S10): every static shape
  that reaches into the robot's height band but that the 2D map cannot hold, because the 2D
  lidar's plane misses it or the glass mask hides it from the lidar. A margin covers the map's
  few-cm offset from the world.
- The mask is a map_server image on the map's grid: black (occupied) cells are keepout, the
  rest is grey (unknown). Both costmaps load it as a second static layer, before inflation.
  Nav2's KeepoutFilter is not used: filters run after inflation, so keepout cells would not be
  inflated, and the planner's and MPPI's footprint checks only look at the footprint near
  inflated cells. With the filter the robot's corner cut the glass screen's end (Phase 6).
"""

import math
import os

import numpy as np
import yaml

# map_server's unknown (trinary, free_thresh 0.196), as in maps saved by slam_toolbox.
UNKNOWN_PIXEL = 205


def _box_corners(shape, margin):
    """Return the 4 corners of a box Shape grown by ``margin``, world frame, CCW."""
    hx, hy = shape.size[0] / 2.0 + margin, shape.size[1] / 2.0 + margin
    c, s = math.cos(shape.yaw), math.sin(shape.yaw)
    corners = []
    for lx, ly in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)):
        corners.append((shape.x + c * lx - s * ly, shape.y + s * lx + c * ly))
    return corners


def _cylinder_corners(shape, margin, sides=12):
    r = shape.size[0] + margin
    return [(shape.x + r * math.cos(2 * math.pi * k / sides),
             shape.y + r * math.sin(2 * math.pi * k / sides)) for k in range(sides)]


def world_to_map(points, map_pose):
    """Move world-frame points into the map frame placed at ``map_pose`` (x, y, yaw)."""
    mx, my, myaw = map_pose
    c, s = math.cos(myaw), math.sin(myaw)
    out = []
    for x, y in points:
        dx, dy = x - mx, y - my
        out.append((c * dx + s * dy, -s * dx + c * dy))
    return out


def unmapped_shapes(shapes, scan_height, band):
    """
    Return the shapes the 2D map cannot hold but the robot can hit.

    A shape counts if its height range overlaps ``band`` (low, high: the floor to the robot's
    top) and it either does not cross the 2D lidar's plane or is hidden from the lidar.
    """
    low, high = band
    out = []
    for shape in shapes:
        z_low, z_high = shape.z_range
        if z_high <= low or z_low >= high:
            continue
        if shape.hidden or not shape.crosses(scan_height):
            out.append(shape)
    return out


def zones_from_world(shapes, scan_height, band, margin, map_pose):
    """Return keepout zones (dicts of name, polygon in the map frame) from world shapes."""
    zones = []
    for shape in unmapped_shapes(shapes, scan_height, band):
        corners = (_box_corners(shape, margin) if shape.kind == 'box'
                   else _cylinder_corners(shape, margin))
        polygon = [[round(x, 3), round(y, 3)] for x, y in world_to_map(corners, map_pose)]
        z_low, z_high = shape.z_range
        zones.append({
            'name': shape.name,
            'why': ('hidden from the lidars' if shape.hidden
                    else f'z {z_low:.2f}..{z_high:.2f} m, misses the 2D lidar plane'),
            'polygon': polygon,
        })
    return zones


def points_in_polygon(points, polygon):
    """Return a bool per Nx2 point: inside the polygon (even-odd rule)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    px, py = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        crosses = (y1 > py) != (y2 > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_at = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (px < x_at)
    return inside


def rasterize(zones, width, height, resolution, origin):
    """
    Return a (height, width) bool grid, row 0 at the top: True where a cell is keepout.

    A cell is keepout if its centre is inside a zone, or if a zone's corner falls in it, so
    that zones thinner than a cell still mark one.
    """
    ox, oy = origin[0], origin[1]
    cols, rows = np.meshgrid(np.arange(width), np.arange(height))
    cx = ox + (cols + 0.5) * resolution
    cy = oy + (height - 1 - rows + 0.5) * resolution
    centres = np.column_stack([cx.ravel(), cy.ravel()])
    grid = np.zeros(width * height, dtype=bool)
    for zone in zones:
        grid |= points_in_polygon(centres, zone['polygon'])
    grid = grid.reshape(height, width)
    for zone in zones:
        for x, y in zone['polygon']:
            col = int(math.floor((x - ox) / resolution))
            row = height - 1 - int(math.floor((y - oy) / resolution))
            if 0 <= col < width and 0 <= row < height:
                grid[row, col] = True
    return grid


def write_mask(prefix, grid, resolution, origin):
    """
    Write <prefix>.pgm and <prefix>.yaml: black is keepout, grey (unknown) is everywhere else.

    The mask is a second static layer in both costmaps, merged by maximum before inflation, so
    keepout cells are inflated like walls. Grey, not white: the layer then leaves the map's own
    cells alone, unknown ones included.
    """
    height, width = grid.shape
    pixels = np.where(grid, 0, UNKNOWN_PIXEL).astype(np.uint8)
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n# keepout mask\n{width} {height}\n255\n'.encode())
        f.write(pixels.tobytes())
    meta = {
        'image': os.path.basename(prefix) + '.pgm',
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with open(prefix + '.yaml', 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)


def load_zones(path):
    """Return the zones of a zones YAML file."""
    with open(path) as f:
        return yaml.safe_load(f)['zones']
