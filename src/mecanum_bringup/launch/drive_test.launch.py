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

r"""
Drive the real base: the ESP32 drive, its controllers and the safety gate, for teleop and tuning.

Stage 5H of docs/hardware_transition.md; the procedure is docs/hardware_drive_test.md. Starts:
  - mecanum_controllers controllers.launch.py hardware:=esp32: robot_state_publisher,
    ros2_control_node on mecanum_hardware/Esp32System, joint_state_broadcaster and
    mecanum_drive_controller with config/controllers_hardware.yaml (low STAND-IN limits;
    the controller owns odom -> base_footprint, there is no EKF on the hardware yet);
  - the safety gate, /cmd_vel -> /cmd_vel_safe, watching /joint_states only (no sensor is
    fitted for this stage): when the drive stops reporting, motion is blocked;
  - RViz (rviz:=true), fixed frame odom, with the wheel odometry.

Every argument of controllers.launch.py's esp32 set passes through (port, pulses_per_rev,
counts_per_wheel_rev, motor_for_wheel, dir_invert, enc_invert, watchdog_ms, accel_backstop,
max_wheel_speed, following_error_limit). Drive it from another terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true \\
        -p frame_id:=base_footprint -p speed:=0.1 -p turn:=0.3
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def esp32_args():
    """Read the esp32 argument names and defaults from controllers.launch.py."""
    path = os.path.join(get_package_share_directory('mecanum_controllers'), 'launch',
                        'controllers.launch.py')
    spec = importlib.util.spec_from_file_location('mecanum_controllers_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ESP32_ARGS


def generate_launch_description():
    controllers_share = get_package_share_directory('mecanum_controllers')
    safety_share = get_package_share_directory('mecanum_safety')
    bringup_share = get_package_share_directory('mecanum_bringup')
    args = esp32_args()
    watched = ('lidar_2d', 'l2', 'd435i', 'ultrasonics', 'cliff_tof', 'bumpers', 'odom')

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in args.items()],
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controllers_share, 'launch', 'controllers.launch.py')),
            launch_arguments={
                'hardware': 'esp32',
                'reference_topic': '/cmd_vel_safe',
                **{name: LaunchConfiguration(name) for name in args},
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_share, 'launch', 'safety.launch.py')),
            launch_arguments={
                'watch_joint_states': 'true',
                **{f'watch_{name}': 'false' for name in watched},
                'watchdog_clock': 'steady',
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'drive_test.rviz')],
            output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
