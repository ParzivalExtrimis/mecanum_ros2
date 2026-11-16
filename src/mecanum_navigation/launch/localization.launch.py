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
Global localization on the 2D lidar (docs/plan.md Phase 6): publishes /map and TF map -> odom.

    localization:=amcl (default)   map_server (map) + AMCL (OmniMotionModel),
                                   config/localization.yaml
    localization:=slam_toolbox     slam_toolbox localization mode on the pose graph
                                   (posegraph), mecanum_localization/slam_localization.launch.py

Exactly one of AMCL and slam_toolbox runs, so map -> odom has one publisher. AMCL starts at
home, the map frame's origin; give another start with initial_x, initial_y and initial_yaw
(the booth demo's map frame is the booth, not home), or with RViz's "2D Pose Estimate"
(/initialpose). amcl_params names an extra YAML file whose amcl: ros__parameters replace the
matching entries of config/localization.yaml (the booth demo's tuning, for example).
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
import yaml


def amcl_node(context, params, log_level):
    """Return AMCL with its start pose and amcl_params from the launch arguments."""
    # Both are written into a copy of the parameter file: a separate /** override would lose
    # to the file's own amcl: entries, which name the node exactly.
    with open(params) as f:
        config = yaml.safe_load(f)
    amcl = config['amcl']['ros__parameters']
    extra = LaunchConfiguration('amcl_params').perform(context)
    if extra:
        with open(extra) as f:
            amcl.update(yaml.safe_load(f)['amcl']['ros__parameters'])
    pose = amcl.setdefault('initial_pose', {})
    for key in ('x', 'y', 'yaw'):
        pose[key] = float(LaunchConfiguration(f'initial_{key}').perform(context))
    with tempfile.NamedTemporaryFile('w', prefix='amcl_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(config, f)
    return [Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 arguments=['--ros-args', '--log-level', log_level], parameters=[f.name])]


def generate_launch_description():
    share = get_package_share_directory('mecanum_navigation')
    loc_share = get_package_share_directory('mecanum_localization')
    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    params = os.path.join(share, 'config', 'localization.yaml')
    is_amcl = PythonExpression(["'", localization, "' == 'amcl'"])
    is_slam = PythonExpression(["'", localization, "' == 'slam_toolbox'"])

    amcl = GroupAction(scoped=True, condition=IfCondition(is_amcl), actions=[
        SetParameter('use_sim_time', use_sim_time),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', arguments=['--ros-args', '--log-level', log_level],
             parameters=[params, {'yaml_filename': LaunchConfiguration('map')}]),
        OpaqueFunction(function=amcl_node, args=[params, log_level]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             arguments=['--ros-args', '--log-level', log_level],
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ])
    slam = GroupAction(scoped=True, condition=IfCondition(is_slam), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'slam_localization.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'map': LaunchConfiguration('posegraph')}.items()),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl',
                              choices=['amcl', 'slam_toolbox'],
                              description='Who publishes map -> odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'restroom.yaml'),
                              description='map_server YAML (AMCL)'),
        DeclareLaunchArgument('initial_x', default_value='0.0',
                              description='AMCL start pose in the map frame, x (m)'),
        DeclareLaunchArgument('initial_y', default_value='0.0',
                              description='AMCL start pose in the map frame, y (m)'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0',
                              description='AMCL start pose in the map frame, yaw (rad)'),
        DeclareLaunchArgument('amcl_params', default_value='',
                              description='Extra YAML whose amcl: parameters replace the '
                                          "defaults' (empty: none)"),
        DeclareLaunchArgument('posegraph', default_value=os.path.join(share, 'maps', 'restroom'),
                              description='Pose graph without .posegraph (slam_toolbox)'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        amcl,
        slam,
    ])
