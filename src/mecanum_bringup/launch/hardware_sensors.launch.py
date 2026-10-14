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
Real Unitree L2 with the same perception and RViz view as the simulation.

Starts, all on wall-clock time:
  - the L2 driver (mecanum_lidar/l2_driver): /l2/points, /l2/imu
  - robot_state_publisher + joint_state_publisher (zero wheel angles): the URDF TF,
    including the L2 mast mount
  - L2 perception (mecanum_localization/perception.launch.py):
    /l2/points -> /l2/points_filtered -> /scan
  - RViz with rviz/sensors.rviz (robot model, raw and filtered clouds, scan)

The drive node is not started: the ESP32 transport does not exist yet, so the robot only
looks, it does not move. Example:

    ros2 launch mecanum_bringup hardware_sensors.launch.py                      # Ethernet
    ros2 launch mecanum_bringup hardware_sensors.launch.py connection:=serial \
        serial_port:=/dev/ttyACM0                                              # USB-UART
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    lidar_share = get_package_share_directory('mecanum_lidar')
    loc_share = get_package_share_directory('mecanum_localization')

    driver_args = ['connection', 'serial_port', 'lidar_ip', 'local_ip', 'params_file']

    return LaunchDescription([
        DeclareLaunchArgument('connection', default_value='udp',
                              description='udp (Ethernet, default) or serial (USB-UART)'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='L2 serial device (serial)'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.62',
                              description='L2 address (udp)'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.2',
                              description='Address of this computer on the L2 link (udp)'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(lidar_share, 'config', 'l2.yaml'),
            description='L2 driver parameters (work mode, cloud_scan_num, ranges)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'sensors.rviz'),
            description='RViz configuration'),

        # L2 driver + URDF TF. Its own RViz is off; the sensors view below replaces it.
        # Scoped: launch arguments given to an include otherwise leak into this file, and the
        # include's rviz:=false would also switch off the RViz node below.
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_share, 'launch', 'l2.launch.py')),
                launch_arguments=[(name, LaunchConfiguration(name)) for name in driver_args]
                + [('robot_model', 'true'), ('rviz', 'false')]),
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
