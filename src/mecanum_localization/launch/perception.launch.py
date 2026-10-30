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
L2 perception: robot self-filter, then 3D cloud to 2D scan.

    /l2/points --self_filter (pcl_ros CropBox)--> /l2/points_filtered
               --cloud_to_scan (pointcloud_to_laserscan)--> /scan

Works the same with the real driver (mecanum_lidar) and the simulated L2 (mecanum_simulation).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mecanum_localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud = LaunchConfiguration('cloud_topic')
    filtered = LaunchConfiguration('filtered_cloud_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('cloud_topic', default_value='/l2/points',
                              description='Raw L2 point cloud'),
        DeclareLaunchArgument('filtered_cloud_topic', default_value='/l2/points_filtered',
                              description='Cloud without the robot body'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='2D scan output'),
        Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name='self_filter',
            output='screen',
            parameters=[os.path.join(share, 'config', 'self_filter.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('input', cloud), ('output', filtered)],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='cloud_to_scan',
            output='screen',
            parameters=[os.path.join(share, 'config', 'cloud_to_scan.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('cloud_in', filtered), ('scan', LaunchConfiguration('scan_topic'))],
        ),
    ])
