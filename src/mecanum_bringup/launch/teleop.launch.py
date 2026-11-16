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
Keyboard teleop with the stack's defaults: stamped twists in base_footprint on /cmd_vel.

The same as
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true \\
        -p frame_id:=base_footprint -p speed:=0.1 -p turn:=0.3
which is the teleop of docs/hardware_drive_test.md. teleop_twist_keyboard reads the keyboard
from its own terminal, and ros2 launch gives a node none, so it opens in a new terminal window
(`terminal`, default gnome-terminal); type into that window. Closing the window, or q / Ctrl-C
in it, ends the launch; Ctrl-C in the launch terminal closes the window.

    ros2 launch mecanum_bringup teleop.launch.py                    # hardware defaults
    ros2 launch mecanum_bringup teleop.launch.py speed:=0.2 turn:=0.5  # sim mapping speeds

On the hardware the drive limits clamp any higher speed (controllers_hardware.yaml). The twists
go to /cmd_vel, which the safety gate passes on to /cmd_vel_safe.
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def close_teleop(marker):
    """SIGHUP this launch's teleop (no-op when it has already exited)."""
    # '[m]' keeps the pattern from matching pkill's own command line.
    subprocess.run(['pkill', '-HUP', '-f', '--', f'[{marker[0]}]{marker[1:]}'], check=False)
    return []


def generate_launch_description():
    # The terminal runs teleop outside this launch's process tree, so a Ctrl-C here does not
    # reach it. A marker argument (teleop ignores it) lets the shutdown handler close exactly
    # this launch's teleop with SIGHUP, as closing its window would.
    marker = f'mecanum_teleop_launch_{os.getpid()}'
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix=LaunchConfiguration('terminal'),
        output='screen',
        arguments=[marker],
        parameters=[{
            'stamped': True,
            'frame_id': LaunchConfiguration('frame_id'),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'turn': ParameterValue(LaunchConfiguration('turn'), value_type=float),
        }],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.1',
                              description='Linear speed (m/s) at start; w/x change it'),
        DeclareLaunchArgument('turn', default_value='0.3',
                              description='Angular speed (rad/s) at start; e/c change it'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Frame of the stamped twists'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Twist topic (the safety gate input)'),
        DeclareLaunchArgument('terminal', default_value='gnome-terminal --wait --',
                              description='Terminal prefix that gives teleop a keyboard, '
                              'for instance: xterm -e'),
        teleop,
        RegisterEventHandler(OnProcessExit(
            target_action=teleop, on_exit=[EmitEvent(event=Shutdown(reason='teleop closed'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: close_teleop(marker))])),
    ])
