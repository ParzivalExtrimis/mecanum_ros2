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

"""Check that the driver's geometry defaults match the robot description (URDF)."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from mecanum_base import esp32_driver_node
import pytest
import xacro
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'esp32_driver.yaml')


@pytest.fixture(scope='module')
def urdf_geometry():
    path = os.path.join(get_package_share_directory('mecanum_description'), 'urdf',
                        'mecanum.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path, mappings={'use_l2': 'false'}).toxml())
    origins = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name.endswith('_wheel_joint'):
            xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
            origins[name[:-len('_wheel_joint')]] = xyz
    link = root.find("link[@name='front_left_wheel_link']")
    radius = float(link.find('collision/geometry/cylinder').get('radius'))
    lx = (origins['front_left'][0] - origins['rear_left'][0]) / 2.0
    ly = origins['front_left'][1]
    # The kinematics assume a symmetric layout left/right.
    assert origins['front_right'][1] == pytest.approx(-ly)
    assert origins['rear_right'][1] == pytest.approx(-ly)
    assert origins['rear_left'][1] == pytest.approx(ly)
    return {'wheel_radius': radius, 'lx': lx, 'ly': ly}


def test_yaml_geometry_matches_urdf(urdf_geometry):
    with open(CONFIG) as f:
        params = yaml.safe_load(f)['esp32_driver']['ros__parameters']
    for key, value in urdf_geometry.items():
        assert params[key] == pytest.approx(value, abs=1e-6), key


def test_code_defaults_match_urdf(urdf_geometry):
    assert esp32_driver_node.DEFAULT_WHEEL_RADIUS == pytest.approx(
        urdf_geometry['wheel_radius'], abs=1e-6)
    assert esp32_driver_node.DEFAULT_LX == pytest.approx(urdf_geometry['lx'], abs=1e-6)
    assert esp32_driver_node.DEFAULT_LY == pytest.approx(urdf_geometry['ly'], abs=1e-6)
