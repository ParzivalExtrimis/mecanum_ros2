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
Navigate in simulation (docs/plan.md Phase 6): the robot, localization and Nav2.

Starts robot.launch.py (Gazebo, the drive, the EKF, the safety gate) with RViz on
rviz/navigation.rviz, mecanum_navigation/localization.launch.py (AMCL by default, or
localization:=slam_toolbox) and mecanum_navigation/nav2.launch.py. Send goals with RViz's
"Nav2 Goal" tool; set a start pose with "2D Pose Estimate".

    ros2 launch mecanum_bringup navigation.launch.py
    ros2 launch mecanum_bringup navigation.launch.py localization:=slam_toolbox
    ros2 launch mecanum_bringup navigation.launch.py gui:=false rviz:=false   # headless

The map defaults to mecanum_navigation/maps/restroom.yaml and its keepout mask, which match
the default world (restroom.sdf). Every robot.launch.py argument can be given here too
(world, gui, use_* ...); they reach the included file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    nav_share = get_package_share_directory('mecanum_navigation')
    maps = os.path.join(nav_share, 'maps')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'navigation.rviz'),
            description='RViz configuration'),
        DeclareLaunchArgument('localization', default_value='amcl',
                              choices=['amcl', 'slam_toolbox'],
                              description='Who publishes map -> odom'),
        DeclareLaunchArgument('map', default_value=os.path.join(maps, 'restroom.yaml'),
                              description='map_server YAML (AMCL)'),
        DeclareLaunchArgument('posegraph', default_value=os.path.join(maps, 'restroom'),
                              description='Pose graph without .posegraph (slam_toolbox)'),
        DeclareLaunchArgument('keepout', default_value='true',
                              description='Use the keepout mask'),
        DeclareLaunchArgument(
            'keepout_mask', default_value=os.path.join(maps, 'restroom_keepout.yaml'),
            description='Keepout mask (map_server YAML)'),
        DeclareLaunchArgument('mark_max_range', default_value='0.4',
                              description='Largest ultrasonic range the costmap may mark, m '
                                          '(0 = the sensor maximum). 0.4: the D6-a fallback, '
                                          'Phase 6 measured phantoms in the doorway'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'robot.launch.py')),
            launch_arguments={
                'sim': 'true',
                'rviz': LaunchConfiguration('rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'localization.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'localization': LaunchConfiguration('localization'),
                'map': LaunchConfiguration('map'),
                'posegraph': LaunchConfiguration('posegraph'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'keepout': LaunchConfiguration('keepout'),
                'keepout_mask': LaunchConfiguration('keepout_mask'),
                'mark_max_range': LaunchConfiguration('mark_max_range'),
            }.items()),
    ])
