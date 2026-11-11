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
slam_toolbox localization mode on the 2D lidar (docs/plan.md Phase 6), the alternative to AMCL.

    /scan + TF odom -> base_footprint + a saved pose graph --slam_toolbox--> /map, TF map -> odom

Runs the upstream localization launch file (a lifecycle node that configures and activates
itself) with config/slam_localization.yaml. ``map`` (required) is the pose graph without its
extension, such as <mecanum_navigation share>/maps/restroom for restroom.posegraph and
restroom.data, made with tools/map_route.py and tools/save_map.py (not in git). Started by
mecanum_navigation/launch/localization.launch.py with localization:=slam_toolbox.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    share = get_package_share_directory('mecanum_localization')
    slam_share = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share, 'config', 'slam_localization.yaml'),
            description='slam_toolbox parameters'),
        DeclareLaunchArgument(
            'map', description='Pose graph to localize in, without .posegraph'),
        # Scoped, so that map_file_name reaches only this include's node.
        GroupAction(scoped=True, actions=[
            SetParameter('map_file_name', LaunchConfiguration('map')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_share, 'launch', 'localization_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam_params_file': LaunchConfiguration('slam_params_file'),
                    'autostart': 'true',
                    'use_lifecycle_manager': 'false',
                }.items()),
        ]),
    ])
