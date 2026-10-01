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

"""Start the ESP32 drive node with its parameter and calibration files."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mecanum_base')
    transport = LaunchConfiguration('transport')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'transport', default_value='mock',
            description='mock (log only), sim (simulated ESP32) or esp32 (real, not yet '
                        'implemented)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'esp32_driver.yaml'),
            description='Driver parameters'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=os.path.join(share, 'config', 'wheel_calibration.yaml'),
            description='Per-wheel calibration'),
        Node(
            package='mecanum_base',
            executable='esp32_driver',
            name='esp32_driver',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, calibration_file,
                        {'transport': transport, 'use_sim_time': use_sim_time}],
        ),
    ])
