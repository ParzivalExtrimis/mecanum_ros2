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
Start the mecanum base controllers: joint_state_broadcaster and mecanum_drive_controller.

hardware:=sim   Only the spawners. The controller manager runs inside Gazebo (gz_ros2_control
                plugin, mecanum_simulation); include this after the robot is spawned.
hardware:=mock  Also robot_state_publisher on urdf/mecanum_mock.urdf.xacro and
                ros2_control_node on mock hardware, for dry runs without Gazebo.

The drive controller's topics are remapped to the command and odometry contract:
    ~/reference     <- /cmd_vel           (geometry_msgs/TwistStamped)
    ~/odometry      -> /wheel/odometry    (nav_msgs/Odometry, from measured wheel states)
    ~/tf_odometry   -> /tf                (odom -> base_footprint)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, EqualsSubstitution, LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DRIVE_REMAPS = ('-r ~/reference:=/cmd_vel -r ~/odometry:=/wheel/odometry '
                '-r ~/tf_odometry:=/tf')


def generate_launch_description():
    share = get_package_share_directory('mecanum_controllers')
    controllers = os.path.join(share, 'config', 'controllers.yaml')
    # Gazebo runs on the simulation clock; mock hardware on the wall clock.
    use_sim_time = ParameterValue(
        PythonExpression(["'", LaunchConfiguration('hardware'), "' == 'sim'"]), value_type=bool)
    is_mock = IfCondition(EqualsSubstitution(LaunchConfiguration('hardware'), 'mock'))

    mock_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'mecanum_mock.urdf.xacro')]),
        value_type=str)

    def spawner(*args):
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[*args, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='sim',
                              description='sim (Gazebo, gz_ros2_control) or mock'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': mock_description}],
            condition=is_mock,
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers],
            # The controller manager takes the description from robot_state_publisher.
            remappings=[('~/robot_description', '/robot_description')],
            condition=is_mock,
        ),

        spawner('joint_state_broadcaster'),
        spawner('mecanum_drive_controller', '--controller-ros-args', DRIVE_REMAPS),
    ])
