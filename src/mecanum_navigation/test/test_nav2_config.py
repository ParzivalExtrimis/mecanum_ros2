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

"""Check the Nav2 and localization configs against the robot, the drive and the contract."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
SRC = os.path.join(PKG, '..')
NAV2 = os.path.join(PKG, 'config', 'nav2.yaml')
LOCALIZATION = os.path.join(PKG, 'config', 'localization.yaml')
CONTROLLERS = os.path.join(SRC, 'mecanum_controllers', 'config', 'controllers.yaml')
SLAM_LOCALIZATION = os.path.join(SRC, 'mecanum_localization', 'config',
                                 'slam_localization.yaml')
DESCRIPTION = os.path.join(SRC, 'mecanum_description', 'urdf', 'mecanum.urdf.xacro')
BT_DIR = os.path.join(PKG, 'behavior_trees')

# Nodes that publish or take a velocity command (nav2_util TwistPublisher/TwistSubscriber).
VELOCITY_NODES = ['controller_server', 'behavior_server', 'velocity_smoother',
                  'collision_monitor']
DOORWAY_HALF_WIDTH = 0.45   # restroom.sdf: 0.9 m doorway


def params(path, node):
    with open(path) as f:
        data = yaml.safe_load(f)
    if node in ('local_costmap', 'global_costmap'):
        return data[node][node]['ros__parameters']
    return data[node]['ros__parameters']


def footprint(costmap):
    return np.array(yaml.safe_load(params(NAV2, costmap)['footprint']))


@pytest.fixture(scope='module')
def limits():
    with open(CONTROLLERS) as f:
        data = yaml.safe_load(f)
    for node in data.values():
        p = node.get('ros__parameters', {})
        if 'linear' in p:
            return p
    raise AssertionError('no drive controller limits in controllers.yaml')


def test_stamped_everywhere():
    for node in VELOCITY_NODES:
        assert params(NAV2, node).get('enable_stamped_cmd_vel') is True, node


def test_base_frame_is_base_footprint():
    assert params(NAV2, 'bt_navigator')['robot_base_frame'] == 'base_footprint'
    assert params(NAV2, 'behavior_server')['robot_base_frame'] == 'base_footprint'
    assert params(NAV2, 'collision_monitor')['base_frame_id'] == 'base_footprint'
    for costmap in ('local_costmap', 'global_costmap'):
        assert params(NAV2, costmap)['robot_base_frame'] == 'base_footprint'
    assert params(LOCALIZATION, 'amcl')['base_frame_id'] == 'base_footprint'


def test_speeds_within_the_drive_controller(limits):
    vx = limits['linear']['x']
    vy = limits['linear']['y']
    wz = limits['angular']['z']
    mppi = params(NAV2, 'controller_server')['FollowPath']
    assert mppi['motion_model'] == 'Omni'
    assert mppi['vx_max'] <= vx['max_velocity']
    assert mppi['vx_min'] >= vx['min_velocity']
    assert mppi['vy_max'] <= vy['max_velocity']
    assert mppi['wz_max'] <= wz['max_velocity']
    smoother = params(NAV2, 'velocity_smoother')
    top = [vx['max_velocity'], vy['max_velocity'], wz['max_velocity']]
    bottom = [vx['min_velocity'], vy['min_velocity'], wz['min_velocity']]
    assert all(a <= b for a, b in zip(smoother['max_velocity'], top))
    assert all(a >= b for a, b in zip(smoother['min_velocity'], bottom))
    assert smoother['max_velocity'][1] > 0.0, 'vy must be enabled'
    # Reverse is capped below forward (only the rear ultrasonics see behind).
    assert -mppi['vx_min'] < mppi['vx_max']
    assert smoother['min_velocity'][0] == mppi['vx_min']
    assert params(NAV2, 'behavior_server')['max_rotational_vel'] <= wz['max_velocity']


def test_strafing_is_not_filtered():
    # Upstream 0.5 would zero every sideways command below 0.5 m/s.
    assert params(NAV2, 'controller_server')['min_y_velocity_threshold'] < 0.05


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(element):
    origin = element.find('origin') if element is not None else None
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    transform = np.eye(4)
    transform[:3, :3] = _rpy_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def robot_extent(z_max=0.9):
    """Return (x_min, x_max, y_min, y_max) of every collision below z_max, base_footprint."""
    xacro = pytest.importorskip('xacro')
    root = ET.fromstring(xacro.process_file(DESCRIPTION).toxml())
    parent_of = {}
    for joint in root.findall('joint'):
        parent_of[joint.find('child').get('link')] = (joint.find('parent').get('link'),
                                                      _origin(joint))

    def link_pose(link):
        if link == 'base_footprint':
            return np.eye(4)
        parent, transform = parent_of[link]
        return link_pose(parent) @ transform

    corners_all = []
    for link in root.findall('link'):
        pose = link_pose(link.get('name'))
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            if box is not None:
                half = np.array([float(v) for v in box.get('size').split()]) / 2.0
            elif cylinder is not None:
                r, length = float(cylinder.get('radius')), float(cylinder.get('length'))
                half = np.array([r, r, length / 2.0])
            else:
                continue
            signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                              for sz in (-1, 1)])
            local = np.hstack([signs * half, np.ones((8, 1))])
            world = (pose @ _origin(collision) @ local.T).T[:, :3]
            corners_all.append(world[world[:, 2] < z_max])
    corners = np.vstack([c for c in corners_all if len(c)])
    return (corners[:, 0].min(), corners[:, 0].max(), corners[:, 1].min(),
            corners[:, 1].max())


def test_footprint_matches_the_description():
    x_min, x_max, y_min, y_max = robot_extent()
    for costmap in ('local_costmap', 'global_costmap'):
        fp = footprint(costmap)
        assert params(NAV2, costmap)['footprint_padding'] == 0.0
        # The footprint covers the robot, rounded out by at most 5 mm.
        assert x_min - 0.005 <= fp[:, 0].min() <= x_min + 1e-6, costmap
        assert x_max - 1e-6 <= fp[:, 0].max() <= x_max + 0.005, costmap
        assert y_min - 0.005 <= fp[:, 1].min() <= y_min + 1e-6, costmap
        assert y_max - 1e-6 <= fp[:, 1].max() <= y_max + 0.005, costmap


def test_inflation_covers_the_circumscribed_radius():
    fp = footprint('local_costmap')
    circumscribed = np.hypot(fp[:, 0], fp[:, 1]).max()
    for costmap in ('local_costmap', 'global_costmap'):
        inflation = params(NAV2, costmap)['inflation_layer']
        # MPPI's footprint check needs it (its warning otherwise).
        assert inflation['inflation_radius'] >= circumscribed


def _polygons():
    cm = params(NAV2, 'collision_monitor')
    out = {}
    for name in cm['polygons']:
        polygon = cm[name]
        if polygon['type'] == 'velocity_polygon':
            for sub in polygon['velocity_polygons']:
                out[f'{name}/{sub}'] = (polygon, polygon[sub],
                                        np.array(yaml.safe_load(polygon[sub]['points'])))
    return out


def test_collision_polygons():
    cm = params(NAV2, 'collision_monitor')
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert cm['cmd_vel_out_topic'] == 'cmd_vel'
    fp = footprint('local_costmap')
    polygons = _polygons()
    front = polygons['FrontStop/ahead'][2]
    rear = polygons['RearStop/behind'][2]
    # Outside the footprint, on its side.
    assert front[:, 0].min() >= fp[:, 0].max() - 1e-6
    assert rear[:, 0].max() <= fp[:, 0].min() + 1e-6
    # Narrower than the doorway, so its jambs do not stop the robot in it.
    for name, (_, _, points) in polygons.items():
        assert np.abs(points[:, 1]).max() < DOORWAY_HALF_WIDTH - 0.05, name
    # The rear stop acts only when reversing, the front ones never when reversing.
    rear_sub = polygons['RearStop/behind'][1]
    assert rear_sub['linear_min'] > 0.0
    assert rear_sub['direction_start_angle'] > math.pi / 2
    assert rear_sub['direction_end_angle'] < -math.pi / 2
    for name in ('FrontStop/ahead', 'FrontSlowdown/ahead'):
        sub = polygons[name][1]
        assert sub['direction_start_angle'] > -math.pi / 2
        assert sub['direction_end_angle'] < math.pi / 2
    for name in cm['polygons']:
        for source in cm[name].get('sources_names', []):
            assert source in cm['observation_sources'], (name, source)


def test_goal_checkers_and_trees():
    controller = params(NAV2, 'controller_server')
    assert controller['goal_checker_plugins'] == ['staging', 'precise']
    staging, precise = controller['staging'], controller['precise']
    assert (staging['xy_goal_tolerance'], staging['yaw_goal_tolerance']) == (0.10, 0.10)
    assert (precise['xy_goal_tolerance'], precise['yaw_goal_tolerance']) == (0.05, 0.05)
    for tree in ('navigate_to_pose.xml', 'navigate_through_poses.xml'):
        root = ET.parse(os.path.join(BT_DIR, tree)).getroot()
        selectors = root.findall('.//GoalCheckerSelector')
        assert len(selectors) == 1
        assert selectors[0].get('default_goal_checker') == 'staging'
        key = selectors[0].get('selected_goal_checker')
        for follow in root.findall('.//FollowPath'):
            assert follow.get('goal_checker_id') == key


def test_recoveries_keep_what_is_near_the_robot():
    # The sensors look forward: a full clear forgets what is beside and behind the robot. The
    # local costmap is never cleared: a partial clear leaves its range layer not current.
    clearing = ('ClearEntireCostmap', 'ClearCostmapExceptRegion', 'ClearCostmapAroundRobot',
                'ClearCostmapAroundPose')
    for tree in ('navigate_to_pose.xml', 'navigate_through_poses.xml'):
        root = ET.parse(os.path.join(BT_DIR, tree)).getroot()
        clears = [node for tag in clearing for node in root.iter(tag)]
        assert clears and all(node.tag == 'ClearCostmapExceptRegion' for node in clears)
        for clear in clears:
            assert clear.get('service_name') == 'global_costmap/clear_except_global_costmap'
            assert float(clear.get('reset_distance')) >= 2 * 0.494 + 0.5
        # "Start occupied" (205 to pose, 305 through poses) runs the recoveries too.
        checks = root.findall('.//Fallback/AreErrorCodesPresent')
        assert len(checks) == 1
        assert checks[0].get('error_code') == '{compute_path_error_code}'
        assert set(checks[0].get('error_codes_to_check').split(';')) == {'205', '305'}


def test_keepout_layer_wiring():
    # The mask is a static layer before inflation, not a KeepoutFilter (filters run after
    # inflation, and the footprint checks skip uninflated cells).
    topic = params(NAV2, 'keepout_mask_server')['topic_name']
    for costmap in ('local_costmap', 'global_costmap'):
        p = params(NAV2, costmap)
        assert 'filters' not in p
        plugins = p['plugins']
        assert plugins.index('keepout_layer') < plugins.index('inflation_layer')
        assert p['keepout_layer']['plugin'] == 'nav2_costmap_2d::StaticLayer'
        assert p['keepout_layer']['map_topic'] == topic
        assert p['keepout_layer']['enabled'] == '$(var keepout)'
    # Two static layers in the global costmap: the mask must add to the map, not replace it.
    glob = params(NAV2, 'global_costmap')
    assert glob['plugins'][:2] == ['static_layer', 'keepout_layer']
    assert glob['use_maximum'] is True


def test_d435i_consumers_take_the_thinned_cloud():
    # Best-effort subscribers lose the full camera cloud (Phase 2, Phase 6): cloud_thinner.
    thinned = '/costmap/d435i/points'
    for costmap in ('local_costmap', 'global_costmap'):
        voxel = params(NAV2, costmap)['voxel_layer']
        for source in ('d435i_mark', 'd435i_clear'):
            assert voxel[source]['topic'] == thinned
    assert params(NAV2, 'collision_monitor')['d435i']['topic'] == thinned


def test_range_layer_local_only():
    local = params(NAV2, 'local_costmap')
    assert 'range_layer' in local['plugins']
    assert local['range_layer']['clear_on_max_reading'] is True
    assert all(t.startswith('/costmap/ultrasonic/') for t in local['range_layer']['topics'])
    assert 'range_layer' not in params(NAV2, 'global_costmap')['plugins']


def test_voxel_layers_do_not_track_unknown():
    for costmap in ('local_costmap', 'global_costmap'):
        voxel = params(NAV2, costmap)['voxel_layer']
        # true would turn every cell unknown under a max combination.
        assert voxel['track_unknown_space'] is False
        for source in voxel['observation_sources'].split():
            if voxel[source].get('marking'):
                assert voxel[source]['min_obstacle_height'] >= 0.04
                assert voxel[source]['max_obstacle_height'] <= 1.0


def test_one_map_to_odom_publisher_per_mode():
    amcl = params(LOCALIZATION, 'amcl')
    assert amcl['tf_broadcast'] is True
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert amcl['global_frame_id'] == 'map' and amcl['odom_frame_id'] == 'odom'
    with open(SLAM_LOCALIZATION) as f:
        slam = yaml.safe_load(f)['slam_toolbox']['ros__parameters']
    assert slam['mode'] == 'localization'
    assert slam['map_frame'] == 'map' and slam['odom_frame'] == 'odom'
    # The launch file starts AMCL or slam_toolbox, never both.
    with open(os.path.join(PKG, 'launch', 'localization.launch.py')) as f:
        text = f.read()
    assert 'condition=IfCondition(is_amcl)' in text
    assert 'condition=IfCondition(is_slam)' in text
