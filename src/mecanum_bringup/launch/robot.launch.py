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
Bring up the robot: simulation (default) or hardware, plus the drive node.

sim:=true   Gazebo world + robot + simulated ESP32 (mecanum_simulation/sim.launch.py) and
            esp32_driver with the sim transport, all on sim time.
sim:=false  robot_state_publisher + joint_state_publisher (zero wheel angles) and
            esp32_driver with the esp32 transport. The real transport is not implemented
            until the rewritten firmware's protocol exists; the driver then exits with an
            explanation. The L2 driver is added in the hardware phase.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    sim_share = get_package_share_directory('mecanum_simulation')
    base_share = get_package_share_directory('mecanum_base')
    desc_share = get_package_share_directory('mecanum_description')

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    transport = PythonExpression(["'sim' if '", sim, "' == 'true' else 'esp32'"])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'use_l2': LaunchConfiguration('use_l2'),
            'plant_file': LaunchConfiguration('plant_file'),
        }.items(),
        condition=IfCondition(sim))

    hardware_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_share, 'urdf', 'mecanum.urdf.xacro'),
                 ' use_l2:=', LaunchConfiguration('use_l2')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='true: Gazebo simulation, false: real hardware'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI (sim only; false = headless)'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'restroom.sdf'),
            description='Gazebo world (sim only)'),
        DeclareLaunchArgument('use_l2', default_value='true',
                              description='Include the Unitree L2 in the robot model'),
        DeclareLaunchArgument(
            'plant_file', default_value=os.path.join(sim_share, 'config', 'sim_plant.yaml'),
            description='Simulated motor plant (sim only)'),

        sim_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': hardware_description}],
            condition=UnlessCondition(sim)),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(sim)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base_share, 'launch', 'base.launch.py')),
            launch_arguments={'transport': transport, 'use_sim_time': sim}.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'robot.rviz')],
            parameters=[{'use_sim_time': sim}],
            condition=IfCondition(rviz)),
    ])
