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

"""Check the slam_toolbox mapping config against the frames, the scan and the 2D lidar."""

import os
import re

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'slam_mapping.yaml')
GAZEBO = os.path.join(HERE, '..', '..', 'mecanum_simulation', 'urdf', 'mecanum.gazebo.xacro')
UPSTREAM = '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml'


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)['slam_toolbox']['ros__parameters']


def c1_range():
    """Return (min, max) of the simulated 2D lidar's range from the Gazebo xacro."""
    with open(GAZEBO) as f:
        text = f.read()
    sensor = re.search(r'<sensor name="lidar_2d".*?</sensor>', text, re.S).group(0)
    ray_range = re.search(r'<range>\s*<min>([\d.]+)</min>\s*<max>([\d.]+)</max>', sensor)
    return float(ray_range.group(1)), float(ray_range.group(2))


def test_frames_topic_and_mode():
    params = load(CONFIG)
    assert params['mode'] == 'mapping'
    assert params['scan_topic'] == '/scan'
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_footprint'
    assert params['resolution'] == 0.05


def test_range_is_the_2d_lidar():
    params = load(CONFIG)
    assert (params['min_laser_range'], params['max_laser_range']) == c1_range()


def test_publishes_map_to_odom():
    assert load(CONFIG)['transform_publish_period'] > 0.0


@pytest.mark.skipif(not os.path.exists(UPSTREAM), reason='slam_toolbox not installed')
def test_only_marked_values_differ_from_upstream():
    ours, upstream = load(CONFIG), load(UPSTREAM)
    assert set(ours) == set(upstream)
    changed = {k for k in ours if ours[k] != upstream[k]}
    assert changed == {'min_laser_range', 'max_laser_range', 'map_update_interval',
                       'enable_interactive_mode', 'minimum_travel_distance',
                       'minimum_travel_heading', 'correlation_search_space_smear_deviation'}
    with open(CONFIG) as f:
        lines = f.read().splitlines()
    # Each changed value is under a CHANGED comment (one comment may cover a pair).
    for key in changed:
        index = next(i for i, line in enumerate(lines) if line.strip().startswith(key + ':'))
        above = lines[max(0, index - 4):index]
        assert any('CHANGED' in line for line in above), key
