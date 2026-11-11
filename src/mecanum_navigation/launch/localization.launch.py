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
home, the map frame's origin; give another start with RViz's "2D Pose Estimate" (/initialpose).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


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
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             arguments=['--ros-args', '--log-level', log_level], parameters=[params]),
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
        DeclareLaunchArgument('posegraph', default_value=os.path.join(share, 'maps', 'restroom'),
                              description='Pose graph without .posegraph (slam_toolbox)'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        amcl,
        slam,
    ])
