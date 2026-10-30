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

r"""
Convert CAD STL exports (millimetres, CAD assembly frame) into ROS meshes (metres, link frame).

For each point p (CAD units)::

    p_ros = scale * M @ (p - origin)

where ``origin`` is a CAD point that becomes the link-frame origin and ``M`` maps CAD axes to
link axes (``--axes``, e.g. ``y,-x,z`` means link x = CAD y, link y = -CAD x, link z = CAD z).
``M`` must be a proper rotation (det +1) so triangle winding stays outward. Normals are
recomputed. The output is binary STL with a header starting ``ROS-converted:``; files that
already carry it are refused, so a converted file is never transformed twice.

The robot's model (mecanum_description) is built from primitives and has no meshes; use this
tool if meshes are added. Example for the washroom robot's STEP export (Shapr3D, mm), whose
CAD-to-ROS mapping is x = -Y_cad, y = +X_cad, z = Z_cad + 148.9 mm (see the header of
src/mecanum_description/refs/washroom_robot.urdf). This puts a mesh in base_footprint; subtract
the link's own origin for any other link frame::

    tools/cad_stl_to_ros.py --origin=0,0,-148.9 --axes=-y,x,z --scale 0.001 part.stl
"""

import argparse
import math
import struct
import sys

MARKER = b'ROS-converted:'


def read_stl(path):
    """Return (header, triangles) with triangles as lists of three (x, y, z) tuples."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        count = struct.unpack('<I', data[80:84])[0]
        if len(data) == 84 + 50 * count:
            tris = []
            for i in range(count):
                off = 84 + 50 * i + 12
                tris.append([struct.unpack('<3f', data[off + 12 * k:off + 12 * k + 12])
                             for k in range(3)])
            return data[:80], tris
    text = data.decode('ascii', errors='replace').split()
    verts = [tuple(float(v) for v in text[i + 1:i + 4])
             for i, tok in enumerate(text) if tok == 'vertex']
    if not verts or len(verts) % 3:
        raise ValueError(f'{path}: not a valid binary or ASCII STL')
    return b'', [verts[i:i + 3] for i in range(0, len(verts), 3)]


def normal(tri):
    """Return the unit normal of a triangle (right-hand rule)."""
    a = [tri[1][i] - tri[0][i] for i in range(3)]
    b = [tri[2][i] - tri[0][i] for i in range(3)]
    c = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    length = math.sqrt(sum(x * x for x in c))
    return tuple(x / length for x in c) if length > 0 else (0.0, 0.0, 0.0)


def write_stl(path, header, tris):
    """Write triangles as binary STL with recomputed normals."""
    header = header[:80].ljust(80, b' ')
    with open(path, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', len(tris)))
        for tri in tris:
            f.write(struct.pack('<3f', *normal(tri)))
            for v in tri:
                f.write(struct.pack('<3f', *v))
            f.write(b'\0\0')


def parse_axes(spec):
    """Parse 'y,-x,z' into a 3x3 matrix (rows = link axes in CAD coordinates)."""
    index = {'x': 0, 'y': 1, 'z': 2}
    rows = []
    for item in spec.split(','):
        item = item.strip()
        sign = -1.0 if item.startswith('-') else 1.0
        row = [0.0, 0.0, 0.0]
        row[index[item.lstrip('+-')]] = sign
        rows.append(row)
    det = (rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
           - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
           + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]))
    if len(rows) != 3 or abs(det - 1.0) > 1e-9:
        raise ValueError(f'--axes {spec!r} is not a proper rotation (det {det:+.0f})')
    return rows


def bbox(tris):
    """Return the (min, max) corners of the triangles' bounding box."""
    pts = [v for t in tris for v in t]
    return ([min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)])


def main(argv=None):
    """Convert the given STL files in place."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('files', nargs='+')
    parser.add_argument('--origin', required=True, help='CAD point that becomes 0,0,0; use --origin=-1,2,3 for negatives')
    parser.add_argument('--axes', default='x,y,z', help='link axes in CAD axes, e.g. y,-x,z')
    parser.add_argument('--scale', type=float, default=0.001, help='CAD unit -> metres')
    args = parser.parse_args(argv)

    origin = [float(v) for v in args.origin.split(',')]
    rot = parse_axes(args.axes)
    for path in args.files:
        header, tris = read_stl(path)
        if header.startswith(MARKER):
            print(f'{path}: already converted ({header.decode(errors="replace").strip()}), '
                  'skipped', file=sys.stderr)
            continue
        out = []
        for t in tris:
            out.append([tuple(args.scale * sum(rot[r][c] * (v[c] - origin[c]) for c in range(3))
                              for r in range(3)) for v in t])
        new_header = (MARKER + f' m, origin={args.origin}, axes={args.axes}, '
                      f'scale={args.scale:g}'.encode())
        write_stl(path, new_header, out)
        lo, hi = bbox(out)
        print(f'{path}: {len(out)} triangles, bbox (m) '
              f'x {lo[0]:+.4f}..{hi[0]:+.4f}  y {lo[1]:+.4f}..{hi[1]:+.4f}  '
              f'z {lo[2]:+.4f}..{hi[2]:+.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
