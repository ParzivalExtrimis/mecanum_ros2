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
hardware:=esp32 Also robot_state_publisher on urdf/mecanum_esp32.urdf.xacro and
                ros2_control_node on the real drive (mecanum_hardware/Esp32System on the ESP32
                at port:=...), with config/controllers_hardware.yaml over controllers.yaml.
                The ESP32_ARGS below go to the xacro; docs/hardware_drive_test.md says what
                each test stage sets.

The drive controller's topics are remapped to the command and odometry contract:
    ~/reference     <- /cmd_vel_safe      (geometry_msgs/TwistStamped, the safety gate's output;
                                           reference_topic:=/cmd_vel bypasses the gate, for the
                                           mock dry runs only, which have no sensors to watch)
    ~/odometry      -> /wheel/odometry    (nav_msgs/Odometry, from measured wheel states)
    ~/tf_odometry   -> /tf                (sim and mock: unused, enable_odom_tf is false and the
                                           EKF owns the TF; esp32: odom -> base_footprint)
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


# xacro args of urdf/mecanum_esp32.urdf.xacro, with the same defaults (all STAND-INs).
ESP32_ARGS = {
    'port': ('/dev/ttyUSB0', 'The ESP32 serial port; use the /dev/serial/by-id/ path'),
    'pulses_per_rev': ('1600', 'Step pulses per motor revolution (CS-D808 DIP switches)'),
    'counts_per_wheel_rev': ('4000', 'Encoder counts per wheel revolution (x4 decoding)'),
    'motor_for_wheel': ('0 1 2 3', 'Motor index (0..3) of FL FR RL RR'),
    'dir_invert': ('0 0 0 0', 'Per motor 1..4: 1 inverts its step direction'),
    'enc_invert': ('0 0 0 0', 'Per motor 1..4: 1 inverts its encoder count'),
    'watchdog_ms': ('200', 'The ESP32 stops the wheels when no command arrives this long'),
    'accel_backstop': ('2.5', 'The ESP32 ramp at each wheel, starts and stops (rad/s^2)'),
    'max_wheel_speed': ('3.5', 'The ESP32 clamps wheel commands to this (rad/s)'),
    'following_error_limit': ('0.5', 'Pulses vs counts difference that stops all (wheel rad)'),
}


def drive_remaps(reference_topic):
    return ['-r ~/reference:=', reference_topic,
            ' -r ~/odometry:=/wheel/odometry -r ~/tf_odometry:=/tf']


def generate_launch_description():
    share = get_package_share_directory('mecanum_controllers')
    controllers = os.path.join(share, 'config', 'controllers.yaml')
    controllers_hardware = os.path.join(share, 'config', 'controllers_hardware.yaml')
    # Gazebo runs on the simulation clock; mock and esp32 hardware on the wall clock.
    use_sim_time = ParameterValue(
        PythonExpression(["'", LaunchConfiguration('hardware'), "' == 'sim'"]), value_type=bool)
    is_mock = IfCondition(EqualsSubstitution(LaunchConfiguration('hardware'), 'mock'))
    is_esp32 = IfCondition(EqualsSubstitution(LaunchConfiguration('hardware'), 'esp32'))

    mock_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'mecanum_mock.urdf.xacro')]),
        value_type=str)
    esp32_xacro = ['xacro ', os.path.join(share, 'urdf', 'mecanum_esp32.urdf.xacro')]
    for name in ESP32_ARGS:
        esp32_xacro += [f' {name}:="', LaunchConfiguration(name), '"']
    esp32_description = ParameterValue(Command(esp32_xacro), value_type=str)

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
                              description='sim (Gazebo, gz_ros2_control), mock, or esp32 '
                              '(the real drive)'),
        *[DeclareLaunchArgument(name, default_value=default, description=f'esp32: {text}')
          for name, (default, text) in ESP32_ARGS.items()],
        DeclareLaunchArgument('reference_topic', default_value='/cmd_vel_safe',
                              description='The drive reference: the safety gate output, or '
                              '/cmd_vel to bypass the gate (mock dry runs only)'),

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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': esp32_description}],
            condition=is_esp32,
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers, controllers_hardware],
            remappings=[('~/robot_description', '/robot_description')],
            condition=is_esp32,
        ),

        spawner('joint_state_broadcaster'),
        spawner('mecanum_drive_controller', '--controller-ros-args',
                drive_remaps(LaunchConfiguration('reference_topic'))),
    ])
