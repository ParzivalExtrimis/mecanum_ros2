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
Run the Unitree L2 driver, optionally with the robot model and RViz.

Connects over Ethernet/UDP by default (factory addresses); connection:=serial for USB-UART.

Bench test (L2 on a desk, default):     ros2 launch mecanum_lidar l2.launch.py
Mounted on the robot, check mount pose: ros2 launch mecanum_lidar l2.launch.py \
                                            robot_model:=true fixed_frame:=base_footprint
From another launch file (no RViz):     include with rviz:=false

robot_model:=true starts robot_state_publisher and joint_state_publisher for the full robot
description, so the l2_lidar and l2_imu frames hang off base_link. Leave it false when another
launch file (for example mecanum_bringup robot.launch.py) already publishes the description.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mecanum_lidar')
    desc_share = get_package_share_directory('mecanum_description')

    robot_model = LaunchConfiguration('robot_model')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_share, 'urdf', 'mecanum.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('connection', default_value='udp',
                              description='udp (Ethernet, default) or serial (USB-UART)'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.62',
                              description='L2 address (udp)'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.2',
                              description='Address of this computer on the L2 link (udp)'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Serial device (serial)'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'l2.yaml'),
            description='Driver parameters'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'robot_model', default_value='false',
            description='Publish the robot description and TF (robot_state_publisher)'),
        DeclareLaunchArgument(
            'fixed_frame', default_value='l2_lidar',
            description='RViz fixed frame; base_footprint needs robot_model:=true'),

        Node(
            package='mecanum_lidar',
            executable='l2_driver',
            name='l2_driver',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'connection': LaunchConfiguration('connection'),
                    'lidar_ip': LaunchConfiguration('lidar_ip'),
                    'local_ip': LaunchConfiguration('local_ip'),
                    'serial_port': LaunchConfiguration('serial_port'),
                },
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(robot_model)),
        # The wheels have no encoders; this publishes zero angles so their TF exists.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(robot_model)),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'l2.rviz'),
                       '-f', LaunchConfiguration('fixed_frame')],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
