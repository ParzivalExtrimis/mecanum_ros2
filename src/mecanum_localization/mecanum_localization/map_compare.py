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
Compare a saved 2D occupancy map with the world it was built in (docs/plan.md Phase 5).

Pure geometry, no ROS: tools/map_check.py is the command line over it.

- The world's static boxes and cylinders are read from the SDF. A shape is *expected* in the
  map if it crosses the 2D lidar's scan plane and the lidar can see it (its visual is not
  hidden by the lidar's visibility mask, like the glass).
- The map's occupied cells are moved into the world frame with the pose of the map frame in the
  world (the robot's spawn pose: slam_toolbox starts the map where the robot starts).
- Each box face gets the signed offset of the occupied cells along it: positive is out of the
  box, into the room.
- A best-fit rigid alignment of the cells onto the expected shapes separates a global offset of
  the whole map from local distortion.
"""

from dataclasses import dataclass
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# Bit mask of the simulated lidars (mecanum_simulation's sim_optical_visibility_mask): every bit
# except 28, which the glass sets.
LIDAR_VISIBILITY_MASK = 0xEFFFFFFF


def _pose(element):
    """Return (x, y, z, roll, pitch, yaw) of an element's <pose>, zeros if it has none."""
    node = element.find('pose') if element is not None else None
    if node is None or not node.text:
        return (0.0,) * 6
    values = [float(v) for v in node.text.split()]
    return tuple(values + [0.0] * (6 - len(values)))


def _compose(parent, child):
    """Compose two planar-yaw poses (roll and pitch must be zero)."""
    px, py, pz, _, _, pyaw = parent
    cx, cy, cz, _, _, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, 0.0, 0.0, pyaw + cyaw)


@dataclass
class Shape:
    """A static box or vertical cylinder of the world, in the world frame."""

    name: str
    kind: str        # 'box' or 'cylinder'
    x: float
    y: float
    z: float
    yaw: float
    size: tuple      # box: (sx, sy, sz); cylinder: (radius, length)
    hidden: bool     # hidden from the lidars by the visibility mask

    @property
    def z_range(self):
        half = (self.size[2] if self.kind == 'box' else self.size[1]) / 2.0
        return self.z - half, self.z + half

    def crosses(self, height):
        low, high = self.z_range
        return low <= height <= high

    def signed_distance(self, points):
        """Return the planar signed distance of Nx2 points to the outline (negative inside)."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        dx = points[:, 0] - self.x
        dy = points[:, 1] - self.y
        if self.kind == 'cylinder':
            return np.hypot(dx, dy) - self.size[0]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx = np.abs(c * dx + s * dy) - self.size[0] / 2.0
        ly = np.abs(-s * dx + c * dy) - self.size[1] / 2.0
        outside = np.hypot(np.maximum(lx, 0.0), np.maximum(ly, 0.0))
        inside = np.minimum(np.maximum(lx, ly), 0.0)
        return outside + inside

    def closest_points(self, points):
        """Return the closest point on the outline to each of Nx2 points."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        dx = points[:, 0] - self.x
        dy = points[:, 1] - self.y
        if self.kind == 'cylinder':
            r = np.maximum(np.hypot(dx, dy), 1e-9)
            k = self.size[0] / r
            return np.column_stack([self.x + dx * k, self.y + dy * k])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.size[0] / 2.0, self.size[1] / 2.0
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        qx = np.clip(lx, -hx, hx)
        qy = np.clip(ly, -hy, hy)
        # Points inside: push to the nearer side.
        inside = (np.abs(lx) < hx) & (np.abs(ly) < hy)
        to_x = (hx - np.abs(lx)) < (hy - np.abs(ly))
        qx = np.where(inside & to_x, np.sign(lx) * hx, qx)
        qy = np.where(inside & ~to_x, np.sign(ly) * hy, qy)
        return np.column_stack([self.x + c * qx - s * qy, self.y + s * qx + c * qy])

    def faces(self):
        """Return a box's faces as (label, start, end, outward normal), world frame."""
        if self.kind != 'box':
            return []
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.size[0] / 2.0, self.size[1] / 2.0
        out = []
        for label, (ax, ay), (bx, by), (nx, ny) in (
                ('+x', (hx, -hy), (hx, hy), (1.0, 0.0)),
                ('-x', (-hx, hy), (-hx, -hy), (-1.0, 0.0)),
                ('+y', (hx, hy), (-hx, hy), (0.0, 1.0)),
                ('-y', (-hx, -hy), (hx, -hy), (0.0, -1.0))):
            start = (self.x + c * ax - s * ay, self.y + s * ax + c * ay)
            end = (self.x + c * bx - s * by, self.y + s * bx + c * by)
            normal = (c * nx - s * ny, s * nx + c * ny)
            out.append((label, start, end, normal))
        return out


def load_world_shapes(sdf_path, visibility_mask=LIDAR_VISIBILITY_MASK):
    """Return every box and cylinder collision of the world's static models as Shapes."""
    root = ET.parse(sdf_path).getroot()
    world = root.find('world')
    shapes = []
    for model in world.findall('model'):
        static = model.find('static')
        if static is None or static.text.strip() != 'true':
            continue
        model_pose = _pose(model)
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose(link))
            hidden = False
            for visual in link.findall('visual'):
                flags = visual.find('visibility_flags')
                if flags is not None and int(flags.text) & visibility_mask == 0:
                    hidden = True
            for collision in link.findall('collision'):
                pose = _compose(link_pose, _pose(collision))
                geometry = collision.find('geometry')
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                name = f"{model.get('name')}/{link.get('name')}"
                if box is not None:
                    size = tuple(float(v) for v in box.find('size').text.split())
                    shapes.append(Shape(name, 'box', pose[0], pose[1], pose[2], pose[5],
                                        size, hidden))
                elif cylinder is not None:
                    size = (float(cylinder.find('radius').text),
                            float(cylinder.find('length').text))
                    shapes.append(Shape(name, 'cylinder', pose[0], pose[1], pose[2], pose[5],
                                        size, hidden))
    return shapes


def expected_shapes(shapes, scan_height):
    """Return the shapes the 2D lidar should see: crossing its plane and not hidden."""
    return [s for s in shapes if s.crosses(scan_height) and not s.hidden]


def read_pgm(path):
    """Read a binary (P5) 8-bit PGM image into a 2D uint8 array, row 0 at the top."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: not a binary PGM (P5)')
    width, height, maxval = (int(t) for t in tokens[1:])
    if maxval > 255:
        raise ValueError(f'{path}: 16-bit PGM not supported')
    pos += 1  # the single whitespace after maxval
    pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos)
    return pixels.reshape(height, width)


def load_occupied_cells(map_yaml):
    """
    Return (cells, resolution) for a map_server map: the centres of the occupied cells, Nx2.

    The centres are in the map frame. Occupancy follows map_server's rules: with negate 0, a
    pixel's occupancy is (255 - value) / 255, and it is occupied above occupied_thresh.
    """
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image)
    pixels = read_pgm(image).astype(float)
    resolution = float(meta['resolution'])
    ox, oy, oyaw = (float(v) for v in meta['origin'])
    occupancy = pixels / 255.0 if meta.get('negate', 0) else (255.0 - pixels) / 255.0
    rows, cols = np.nonzero(occupancy > float(meta['occupied_thresh']))
    height = pixels.shape[0]
    lx = (cols + 0.5) * resolution
    ly = (height - 1 - rows + 0.5) * resolution
    c, s = math.cos(oyaw), math.sin(oyaw)
    cells = np.column_stack([ox + c * lx - s * ly, oy + s * lx + c * ly])
    return cells, resolution


def transform(points, pose):
    """Apply a planar pose (x, y, yaw) to Nx2 points."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.column_stack([x + c * points[:, 0] - s * points[:, 1],
                            y + s * points[:, 0] + c * points[:, 1]])


def nearest_shape(points, shapes):
    """Return (index of the nearest shape, unsigned distance to its outline) per point."""
    distances = np.abs(np.stack([s.signed_distance(points) for s in shapes]))
    index = np.argmin(distances, axis=0)
    return index, distances[index, np.arange(len(index))]


def face_offsets(points, shapes, band=0.15, min_cells=10, min_length=1.0):
    """
    Return the offset of the map's cells along every box face.

    A cell belongs to the face it is nearest to, among the faces of the nearest box, if it lies
    within ``band`` of the face line and inside the face's extent. Each face gives a dict with
    its label, length, cell count and the signed offsets' median (positive out of the box).
    ``judged`` is true for faces at least ``min_length`` long with at least ``min_cells``.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    boxes = [s for s in shapes if s.kind == 'box']
    results = []
    if len(points) == 0 or not boxes:
        return results
    owner, _ = nearest_shape(points, boxes)
    for b, box in enumerate(boxes):
        mine = points[owner == b]
        faces = box.faces()
        if len(mine) == 0:
            per_face = [np.empty(0)] * len(faces)
        else:
            offsets, inside = [], []
            for _, start, end, normal in faces:
                start = np.asarray(start)
                direction = np.asarray(end) - start
                length = np.linalg.norm(direction)
                along = (mine - start) @ (direction / length)
                offsets.append((mine - start) @ np.asarray(normal))
                inside.append((along >= 0.0) & (along <= length)
                              & (np.abs(offsets[-1]) <= band))
            offsets = np.stack(offsets)
            inside = np.stack(inside)
            masked = np.where(inside, np.abs(offsets), np.inf)
            best = np.argmin(masked, axis=0)
            valid = np.isfinite(masked[best, np.arange(len(mine))])
            per_face = [offsets[f, valid & (best == f)] for f in range(len(faces))]
        for (label, start, end, _), values in zip(faces, per_face):
            length = math.dist(start, end)
            results.append({
                'shape': box.name,
                'face': label,
                'length': length,
                'cells': int(len(values)),
                'median': float(np.median(values)) if len(values) else float('nan'),
                'judged': length >= min_length and len(values) >= min_cells,
            })
    return results


def _closest_on_shapes(points, shapes):
    index, distance = nearest_shape(points, shapes)
    closest = np.empty_like(points)
    for i, shape in enumerate(shapes):
        mask = index == i
        if mask.any():
            closest[mask] = shape.closest_points(points[mask])
    return closest, distance


def best_fit(points, shapes, initial=(0.0, 0.0, 0.0), max_pair=0.2, iterations=30):
    """
    Return the planar pose that best maps the points onto the shapes' outlines.

    Iterative closest point against the outlines, starting at ``initial``; only pairs closer
    than ``max_pair`` count. Returns (pose, residual distances of the moved points).
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    pose = tuple(initial)
    for _ in range(iterations):
        moved = transform(points, pose)
        closest, distance = _closest_on_shapes(moved, shapes)
        use = distance < max_pair
        if use.sum() < 3:
            break
        src, dst = moved[use], closest[use]
        src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
        h = (src - src_mean).T @ (dst - dst_mean)
        dyaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(dyaw), math.sin(dyaw)
        rotation = np.array([[c, -s], [s, c]])
        shift = dst_mean - rotation @ src_mean
        # Compose the increment onto the current pose.
        x, y, yaw = pose
        new_xy = rotation @ np.array([x, y]) + shift
        pose = (float(new_xy[0]), float(new_xy[1]), yaw + dyaw)
        if abs(dyaw) < 1e-7 and np.hypot(*shift) < 1e-6:
            break
    _, residual = nearest_shape(transform(points, pose), shapes)
    return pose, residual


def shape_coverage(points, shapes, radius=0.1):
    """Return the number of cells nearest to each shape and within ``radius`` of it."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    counts = [0] * len(shapes)
    if len(points) == 0:
        return counts
    index, distance = nearest_shape(points, shapes)
    for i in index[distance <= radius]:
        counts[i] += 1
    return counts
