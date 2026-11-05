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
slam_toolbox online async mapping on the 2D lidar (docs/plan.md Phase 5, decision D3-a).

    /scan + TF odom -> base_footprint --slam_toolbox--> /map and TF map -> odom

Runs the upstream launch file (a lifecycle node that configures and activates itself) with
config/slam_mapping.yaml. Works the same in simulation and on hardware. Save the map with
tools/save_map.py, which calls slam_toolbox's save_map and serialize_map services.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('mecanum_localization')
    slam_share = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share, 'config', 'slam_mapping.yaml'),
            description='slam_toolbox parameters'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items()),
    ])
