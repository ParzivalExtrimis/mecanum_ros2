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
    including the L2 mount
  - L2 perception (mecanum_localization/perception.launch.py):
    /l2/points -> /l2/points_filtered -> /scan
  - with lio:=true, LIO odometry (mecanum_localization/lio.launch.py) with the hardware config
    rko_lio_hardware.yaml: /odom and odom -> base_footprint. The robot must stand still
    while LIO starts.
  - RViz with rviz/sensors.rviz (robot model, raw and filtered clouds, scan)

The drive node is not started: the ESP32 transport does not exist yet, so the robot only
looks, it does not move; for LIO it is pushed by hand (docs/hardware_lio_test.md).

replay:=true leaves out the driver and runs everything on the bag's clock, to replay a
recorded run: ros2 bag play <bag> --clock --topics /l2/points /l2/imu. Examples:

    ros2 launch mecanum_bringup hardware_sensors.launch.py                      # Ethernet
    ros2 launch mecanum_bringup hardware_sensors.launch.py lio:=true            # plus LIO
    ros2 launch mecanum_bringup hardware_sensors.launch.py connection:=serial \
        serial_port:=/dev/ttyACM0                                              # USB-UART
    ros2 launch mecanum_bringup hardware_sensors.launch.py replay:=true lio:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    lidar_share = get_package_share_directory('mecanum_lidar')
    loc_share = get_package_share_directory('mecanum_localization')
    desc_share = get_package_share_directory('mecanum_description')

    driver_args = ['connection', 'serial_port', 'lidar_ip', 'local_ip', 'params_file']
    use_sim_time = LaunchConfiguration('replay')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_share, 'urdf', 'mecanum.urdf.xacro')]),
        value_type=str)

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
        DeclareLaunchArgument('lio', default_value='false',
                              description='Run LIO odometry; start with the robot standing still'),
        DeclareLaunchArgument(
            'lio_config',
            default_value=os.path.join(loc_share, 'config', 'rko_lio_hardware.yaml'),
            description='rko_lio parameters'),
        DeclareLaunchArgument(
            'replay', default_value='false',
            description='No driver, bag clock: replay a recording of /l2/points and /l2/imu'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'sensors.rviz'),
            description='RViz configuration'),

        # L2 driver only; its own RViz is off, the sensors view below replaces it.
        # Scoped: launch arguments given to an include otherwise leak into this file, and the
        # include's rviz:=false would also switch off the RViz node below.
        GroupAction(
            scoped=True,
            condition=UnlessCondition(LaunchConfiguration('replay')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(lidar_share, 'launch', 'l2.launch.py')),
                    launch_arguments=[(name, LaunchConfiguration(name)) for name in driver_args]
                    + [('robot_model', 'false'), ('rviz', 'false')]),
            ]),

        # The other sensors of the robot (D435i, 2D lidar, ultrasonics, cliff ToF, bumpers) have
        # no hardware drivers yet: the current plan (docs/plan.md) is simulation only. Add each
        # driver here next to the L2 driver, wrapped in GroupAction(scoped=True), on the topics
        # and frames of the plan's contract table.

        # URDF TF, including the L2 mount. No drive runs here, so joint_state_publisher
        # publishes zero wheel angles so their TF exists.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc_share, 'launch', 'lio.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'config': LaunchConfiguration('lio_config')}.items(),
            condition=IfCondition(LaunchConfiguration('lio'))),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
