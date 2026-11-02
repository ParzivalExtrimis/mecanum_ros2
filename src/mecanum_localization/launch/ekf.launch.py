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
The robot_localization EKF: odom -> base_footprint and /odom.

    /wheel/odometry + /l2/imu --ekf_node--> /odom and TF odom -> base_footprint

``config`` selects the candidate (config/ekf.yaml is the default), and ``node_name`` and
``odom_topic`` let several run side by side; only config/ekf.yaml publishes the TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mecanum_localization')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'ekf.yaml'),
                              description='EKF parameters: the candidate to run'),
        DeclareLaunchArgument('node_name', default_value='ekf',
                              description='Node name, for running several candidates'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='Filtered odometry output'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool)},
            ],
            remappings=[('odometry/filtered', LaunchConfiguration('odom_topic'))],
        ),
    ])
