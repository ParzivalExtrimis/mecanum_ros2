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

"""Check that the sim sensors, the bridge and the worlds agree with each other."""

import glob
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

SHARE = get_package_share_directory('mecanum_simulation')
BRIDGE = os.path.join(SHARE, 'config', 'gz_bridge.yaml')
WORLDS = sorted(glob.glob(os.path.join(SHARE, 'worlds', '*.sdf')))
POSITIONS = ('front_left', 'front_right', 'rear_left', 'rear_right')

# The topics the bridge must publish directly (docs/plan.md, "Sensor contract"). The range,
# bumper and cloud topics come from the sim-only nodes.
CONTRACT_BRIDGED = {
    '/l2/points', '/l2/imu', '/scan', '/d435i/color/image_raw', '/d435i/color/camera_info',
    '/d435i/depth/image_rect_raw', '/d435i/depth/camera_info', '/d435i/imu'}


@pytest.fixture(scope='module')
def bridge():
    with open(BRIDGE) as stream:
        return yaml.safe_load(stream)


@pytest.fixture(scope='module')
def urdf():
    path = os.path.join(SHARE, 'urdf', 'mecanum_sim.urdf.xacro')
    return ET.fromstring(xacro.process_file(path).toxml())


@pytest.fixture(scope='module')
def sensors(urdf):
    return {sensor.get('name'): sensor for sensor in urdf.iter('sensor')}


def sensor_topics(sensor):
    """Return the Gazebo topics a sensor publishes that the stack uses."""
    if sensor.get('type') == 'contact':
        return [sensor.find('contact/topic').text]
    topic = sensor.find('topic').text
    if sensor.get('type') == 'rgbd_camera':
        return [f'{topic}/image', f'{topic}/depth_image', f'{topic}/camera_info']
    if sensor.get('type') == 'gpu_lidar' and sensor.find('lidar/scan/vertical') is not None:
        return [f'{topic}/points']  # a bridged LaserScan keeps the middle row only
    return [topic]


def glass_flags():
    """Return the visibility flags of the visuals that have some, over all worlds."""
    flags = set()
    for world in WORLDS:
        for element in ET.parse(world).getroot().iter('visibility_flags'):
            flags.add(int(element.text, 0))
    return flags


def test_every_sensor_topic_is_bridged(sensors, bridge):
    bridged = {entry['gz_topic_name'] for entry in bridge}
    for name, sensor in sensors.items():
        for topic in sensor_topics(sensor):
            assert topic in bridged, f'{name}: {topic} is not in gz_bridge.yaml'


def test_every_bridged_sensor_topic_has_a_sensor(sensors, bridge):
    published = {topic for sensor in sensors.values() for topic in sensor_topics(sensor)}
    for entry in bridge:
        if entry['ros_topic_name'] != '/clock' and 'ground_truth' not in entry['gz_topic_name']:
            assert entry['gz_topic_name'] in published, entry


def test_contract_topics_are_bridged_once(bridge):
    ros_topics = [entry['ros_topic_name'] for entry in bridge]
    assert len(ros_topics) == len(set(ros_topics))
    assert CONTRACT_BRIDGED <= set(ros_topics)


def test_sensor_names_are_unique(urdf):
    names = [sensor.get('name') for sensor in urdf.iter('sensor')]
    assert len(names) == len(set(names))


def test_sensors_can_be_left_out():
    path = os.path.join(SHARE, 'urdf', 'mecanum_sim.urdf.xacro')
    mappings = {name: 'false' for name in ('use_l2', 'use_d435i', 'use_lidar_2d',
                                           'use_ultrasonics', 'use_cliff_tof', 'use_bumpers')}
    root = ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())
    assert not list(root.iter('sensor'))


def test_only_the_ultrasonics_see_the_glass(sensors):
    flags = glass_flags()
    assert len(flags) == 1
    flag = flags.pop()
    for name, sensor in sensors.items():
        masks = [element.text for element in sensor.iter('visibility_mask')]
        if sensor.get('type') not in ('gpu_lidar', 'rgbd_camera'):
            continue
        assert len(masks) == 1, name
        assert bool(int(masks[0]) & flag) == name.startswith('ultrasonic_'), name


def test_glass_flag_is_above_the_rendering_default():
    # The rgbd_camera's depth pass ignores the mask and renders GZ_VISIBILITY_ALL only.
    assert all(flag > 0x0FFFFFFF for flag in glass_flags())


@pytest.mark.parametrize('world', WORLDS, ids=os.path.basename)
def test_worlds_load_the_sensor_systems(world):
    plugins = {plugin.get('name') for plugin in ET.parse(world).getroot().iter('plugin')}
    assert {'gz::sim::systems::Sensors', 'gz::sim::systems::Imu',
            'gz::sim::systems::Contact'} <= plugins


def test_bumper_planks_survive_the_urdf_conversion(urdf):
    """A preserved fixed joint needs a mass on its child, or the converter deletes the link."""
    for pos in ('front', 'rear'):
        link = urdf.find(f"link[@name='bumper_{pos}_link']")
        assert link.find('inertial/mass') is not None
        references = [gazebo for gazebo in urdf.findall('gazebo')
                      if gazebo.get('reference') == f'bumper_{pos}_joint']
        assert references and references[0].find('preserveFixedJoint').text == 'true'


def test_range_config_matches_the_cones(sensors):
    with open(os.path.join(SHARE, 'config', 'sim_range_sensors.yaml')) as stream:
        config = yaml.safe_load(stream)
    groups = {'sim_ultrasonics': 'ultrasonic_', 'sim_cliff_tof': 'cliff_tof_'}
    for group, prefix in groups.items():
        params = config[group]['ros__parameters']
        for pos in POSITIONS:
            lidar = sensors[prefix + pos].find('lidar')
            assert float(lidar.find('range/min').text) == pytest.approx(params['min_range'])
            assert float(lidar.find('range/max').text) == pytest.approx(params['max_range'])
            width = (float(lidar.find('scan/horizontal/max_angle').text)
                     - float(lidar.find('scan/horizontal/min_angle').text))
            assert width == pytest.approx(params['field_of_view'], abs=1e-4)
