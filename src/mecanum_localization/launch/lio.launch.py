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
LIO odometry: rko_lio plus the planar odometry adapter.

    /l2/points_filtered + /l2/imu --rko_lio--> /lio/raw_odom (lio_odom -> base_footprint, 3D)
        --lio_odom_adapter--> /odom and TF odom -> base_footprint (planar)

rko_lio always broadcasts its own TF; it is remapped to /lio/tf_unused so that the adapter is
the only publisher of odom -> base_footprint. rko_lio is started directly rather than through
its own launch file, which takes no remappings.
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
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'rko_lio.yaml'),
            description='rko_lio parameters'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='Planar odometry output (/lio/odom with an EKF)'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Publish odom -> base_footprint (false with an EKF)'),
        Node(
            package='rko_lio',
            executable='online_node',
            name='rko_lio',
            output='screen',
            parameters=[LaunchConfiguration('config'), {'use_sim_time': use_sim_time}],
            remappings=[('/tf', '/lio/tf_unused')],
        ),
        Node(
            package='mecanum_localization',
            executable='lio_odom_adapter',
            name='lio_odom_adapter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            }],
        ),
    ])
