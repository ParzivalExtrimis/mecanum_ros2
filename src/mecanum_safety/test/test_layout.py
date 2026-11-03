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

"""Check that the guarded positions in config/safety.yaml match the URDF."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'safety.yaml')


@pytest.fixture(scope='module')
def params():
    with open(CONFIG) as f:
        return yaml.safe_load(f)['safety_supervisor']['ros__parameters']


@pytest.fixture(scope='module')
def joints():
    """Return each fixed joint's xyz relative to base_link (z of base_link above the floor)."""
    path = os.path.join(get_package_share_directory('mecanum_description'), 'urdf',
                        'mecanum.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path, mappings={'use_l2': 'false'}).toxml())
    origins = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        if origin is not None:
            origins[joint.find('child').get('link')] = [
                float(v) for v in origin.get('xyz').split()]
    return origins


def test_cliff_sensors(params, joints):
    base_height = joints['base_link'][2]  # base_footprint -> base_link
    for pos, (sx, sy) in {'front_left': (1, 1), 'front_right': (1, -1),
                          'rear_left': (-1, 1), 'rear_right': (-1, -1)}.items():
        x, y, z = joints[f'cliff_tof_{pos}_link']
        assert x == pytest.approx(sx * params['cliff_x'], abs=1e-4), pos
        assert y == pytest.approx(sy * params['cliff_y'], abs=1e-4), pos
        assert base_height + z == pytest.approx(params['floor_distance'], abs=1e-3), pos


def test_bumper_planks(params, joints):
    assert joints['bumper_front_link'][0] == pytest.approx(params['bumper_x'], abs=1e-4)
    assert joints['bumper_rear_link'][0] == pytest.approx(-params['bumper_x'], abs=1e-4)


def test_ultrasonics(params, joints):
    for pos in ('front_left', 'front_right'):
        assert joints[f'ultrasonic_{pos}_link'][0] == pytest.approx(params['ultrasonic_front_x'])
    for pos in ('rear_left', 'rear_right'):
        assert joints[f'ultrasonic_{pos}_link'][0] == pytest.approx(params['ultrasonic_rear_x'])
