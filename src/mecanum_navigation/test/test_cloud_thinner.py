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

"""Unit tests for cloud_thinner."""

from mecanum_navigation.cloud_thinner import thin, to_cloud, xyz_of
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def test_thin_keeps_one_real_point_per_cube():
    points = np.array([[0.01, 0.01, 1.01], [0.02, 0.04, 1.03],    # one 5 cm cube
                       [0.06, 0.01, 1.01],                         # the next in x
                       [-0.01, 0.01, 1.01]], np.float32)           # the next below 0
    out = thin(points, 0.05)
    assert len(out) == 3
    assert out.tolist() == points[[0, 2, 3]].tolist()


def test_thin_bounds_a_wall_up_close():
    # A full 424 x 240 frame on a wall 0.5 m ahead: about 0.9 x 0.5 m of points.
    u, v = np.meshgrid(np.linspace(-0.45, 0.45, 424), np.linspace(-0.25, 0.25, 240))
    wall = np.column_stack([u.ravel(), v.ravel(), np.full(u.size, 0.5)]).astype(np.float32)
    out = thin(wall, 0.05)
    assert len(out) <= 20 * 12
    assert 12 * len(out) < 100_000       # bytes: far below the 212 kB socket buffer


def test_cloud_round_trip_with_padding_and_nan():
    # The RealSense layout: x, y, z, rgb, 16 bytes a point; NaN points are dropped.
    points = np.array([[1, 2, 3, 0], [np.nan, 0, 0, 0], [4, 5, 6, 0]], np.float32)
    msg = PointCloud2()
    msg.height, msg.width, msg.point_step = 1, 3, 16
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(['x', 'y', 'z', 'rgb'])]
    msg.data = points.tobytes()
    xyz = xyz_of(msg)
    assert xyz.tolist() == [[1, 2, 3], [4, 5, 6]]
    header = Header(frame_id='d435i_optical_frame')
    back = xyz_of(to_cloud(header, xyz))
    assert back.tolist() == xyz.tolist()
    assert to_cloud(header, xyz).header.frame_id == 'd435i_optical_frame'


def test_empty_cloud():
    assert len(thin(np.zeros((0, 3), np.float32), 0.05)) == 0
    assert len(xyz_of(PointCloud2())) == 0
