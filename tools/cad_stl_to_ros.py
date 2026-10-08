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

L2 mast (mecanum_description/urdf/sensors/unitree_l2.xacro), exported from Shapr3D in mm.
Link frame "l2_mast": origin at the centre of the pole's bottom face, x along the bracket
overhang (away from the pole), z up::

    tools/cad_stl_to_ros.py --origin=-5,-2115,-21 --axes=y,-x,z --scale 0.001 \
        src/mecanum_description/meshes/visual/725mmpole.stl \
        src/mecanum_description/meshes/visual/lidarsetup01.stl

``--report-l2`` additionally locates the Unitree L2 body (75 x 75 x 65 mm block under a
tilted bracket plate) in the converted bracket mesh and prints its base-face centre, tilt
and dome direction, for the xacro constants.
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


def report_l2(tris):
    """Locate the L2 block under the tilted bracket plate (converted mesh, metres)."""
    # Tilt: area-weighted mean of the upward faces that are tilted (the plate top).
    acc = [0.0, 0.0, 0.0]
    for t in tris:
        n = normal(t)
        a = [t[1][i] - t[0][i] for i in range(3)]
        b = [t[2][i] - t[0][i] for i in range(3)]
        area = 0.5 * math.sqrt(sum(x * x for x in (a[1] * b[2] - a[2] * b[1],
                                                   a[2] * b[0] - a[0] * b[2],
                                                   a[0] * b[1] - a[1] * b[0])))
        if n[2] > 0.8 and abs(n[0]) > 0.1:
            acc = [acc[i] + n[i] * area for i in range(3)]
    length = math.sqrt(sum(x * x for x in acc))
    up = [x / length for x in acc]                  # plate normal, pointing up
    along = [up[2], 0.0, -up[0]]                    # in-plane, toward the overhang (+x)
    if along[0] < 0:
        along = [-v for v in along]
    pts = [v for t in tris for v in t]

    def proj(p):
        return (sum(along[i] * p[i] for i in range(3)), p[1],
                sum(up[i] * p[i] for i in range(3)))

    q = [proj(p) for p in pts]
    # Topmost levels along the plate normal: plate top, plate bottom, then the L2 base face.
    plate_bottom = sorted({round(v[2], 5) for v in q})[-2]
    body = [v for v in q if v[2] < plate_bottom - 1e-4]
    face = max(v[2] for v in body)
    top = [v for v in body if v[2] > face - 1e-4]
    cu = (min(v[0] for v in top) + max(v[0] for v in top)) / 2
    cy = (min(v[1] for v in top) + max(v[1] for v in top)) / 2
    centre = [cu * along[i] + face * up[i] for i in range(3)]
    centre[1] = cy
    depth = face - min(v[2] for v in body)
    tilt = math.degrees(math.atan2(-up[0], up[2]))
    print(f'  L2 base-face centre (m): ({centre[0]:.5f}, {centre[1]:.5f}, {centre[2]:.5f})')
    print(f'  plate normal: ({up[0]:+.5f}, {up[1]:+.5f}, {up[2]:+.5f}); '
          f'tilt about y: {tilt:+.3f} deg; L2 body depth {depth * 1000:.1f} mm '
          '(dome points away from the plate, i.e. the L2 hangs inverted)')


def main(argv=None):
    """Convert the given STL files in place."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('files', nargs='+')
    parser.add_argument('--origin', required=True, help='CAD point that becomes 0,0,0; use --origin=-1,2,3 for negatives')
    parser.add_argument('--axes', default='x,y,z', help='link axes in CAD axes, e.g. y,-x,z')
    parser.add_argument('--scale', type=float, default=0.001, help='CAD unit -> metres')
    parser.add_argument('--report-l2', action='store_true',
                        help='print the L2 base face found in each converted mesh')
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
        if args.report_l2:
            try:
                report_l2(out)
            except (ValueError, ZeroDivisionError, IndexError):
                print('  (no tilted bracket / L2 block found)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
