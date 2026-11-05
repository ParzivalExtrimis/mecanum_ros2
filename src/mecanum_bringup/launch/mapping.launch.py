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
Map a world with slam_toolbox (docs/plan.md Phase 5), in simulation.

Starts robot.launch.py (Gazebo, the drive, the EKF, the safety gate) with RViz on
rviz/mapping.rviz, and mecanum_localization/slam_mapping.launch.py: slam_toolbox on the 2D
lidar's /scan, publishing /map and map -> odom. Drive with teleop on /cmd_vel (TwistStamped),
then save with tools/save_map.py.

Every robot.launch.py argument can be given here too (world, gui, use_* ...); they reach the
included file. slam_toolbox alone, for example on hardware next to hardware_sensors.launch.py:
    ros2 launch mecanum_localization slam_mapping.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    loc_share = get_package_share_directory('mecanum_localization')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'mapping.rviz'),
            description='RViz configuration'),
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
                os.path.join(loc_share, 'launch', 'slam_mapping.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
