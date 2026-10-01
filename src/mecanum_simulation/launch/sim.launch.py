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
Start Gazebo with the restroom world and spawn the mecanum base.

Starts: Gazebo (gz sim), robot_state_publisher (sim xacro), joint_state_publisher (zero wheel
angles, no encoders), the spawner, ros_gz_bridge and the simulated ESP32 (sim_motor_plant).
The drive node itself is started by mecanum_bringup/robot.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('mecanum_simulation')
    desc_share = get_package_share_directory('mecanum_description')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                             'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_l2 = LaunchConfiguration('use_l2')
    plant_file = LaunchConfiguration('plant_file')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(sim_share, 'urdf', 'mecanum_sim.urdf.xacro'),
                 ' use_l2:=', use_l2]),
        value_type=str)

    def gazebo(extra_args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [extra_args, world],
                              'on_exit_shutdown': 'true'}.items(),
            condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'restroom.sdf'),
            description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI (false = headless server)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='-1.0', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        DeclareLaunchArgument('use_l2', default_value='true',
                              description='Include the Unitree L2 in the robot model'),
        DeclareLaunchArgument(
            'plant_file', default_value=os.path.join(sim_share, 'config', 'sim_plant.yaml'),
            description='Simulated ESP32/motor plant parameters'),

        # Lets Gazebo resolve package://mecanum_description/... mesh URIs without changing
        # mecanum_description's package.xml.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),

        gazebo('-r -v 2 ', IfCondition(gui)),
        gazebo('-r -s -v 2 ', UnlessCondition(gui)),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        # No encoders: wheel joints are published at zero angle, identically on hardware.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'mecanum',
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                       '-z', '0.01', '-Y', LaunchConfiguration('yaw')],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'config_file': os.path.join(sim_share, 'config', 'gz_bridge.yaml'),
                         'use_sim_time': True}],
        ),
        Node(
            package='mecanum_simulation',
            executable='sim_motor_plant',
            output='screen',
            parameters=[plant_file, {'use_sim_time': True}],
        ),
    ])
