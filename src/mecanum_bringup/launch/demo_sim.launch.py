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
The booth demo in simulation: Gazebo's booth, the robot, AMCL, the demo stack and its pages.

    ros2 launch mecanum_bringup demo_sim.launch.py
    ros2 launch mecanum_bringup demo_sim.launch.py gui:=false browser:=false   # headless
    ros2 launch mecanum_bringup demo_sim.launch.py use_d435i:=false            # lighter

Starts
  - robot.launch.py in mecanum_simulation/worlds/booth.sdf, spawned at home (the drive, the
    EKF, the safety gate and the sensors, as in every simulation);
  - mecanum_navigation/localization.launch.py: map_server on maps/booth.yaml and AMCL, started
    at home (the booth map's frame is the world frame, shortcut S11);
  - mecanum_demo/demo.launch.py on sim time: the supervisor, the segment driver, the health
    monitor, the websocket bridge and the web pages, with the simulation controls shown;
  - booth_visitor (mecanum_simulation), which moves the visitor in Gazebo;
  - after a few seconds, Firefox on the dashboard http://localhost:<http_port>/ (browser:=true).

Every robot.launch.py argument can be given too (use_*, gui, rviz...). On the robot, the same
demo.launch.py runs on the hardware bringup instead; see docs/demo_app.md.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def check_bridge(context):
    """Stop before Gazebo starts if the bridge is wanted but not installed."""
    if LaunchConfiguration('bridge').perform(context).lower() in ('true', '1'):
        try:
            get_package_share_directory('rosbridge_server')
        except PackageNotFoundError:
            raise RuntimeError(
                'The panel needs the rosbridge websocket, which is not installed. Install it '
                'once with:  sudo apt install ros-jazzy-rosbridge-suite   (or launch with '
                'bridge:=false to run without the web pages talking to ROS)') from None
    return []


def generate_launch_description():
    bringup = get_package_share_directory('mecanum_bringup')
    sim = get_package_share_directory('mecanum_simulation')
    nav = get_package_share_directory('mecanum_navigation')
    demo = get_package_share_directory('mecanum_demo')
    with open(os.path.join(demo, 'config', 'booth_demo.yaml')) as f:
        site = yaml.safe_load(f)
    home_x, home_y = site['poses']['home']
    home_yaw = site['headings']['W']
    http_port = LaunchConfiguration('http_port')

    return LaunchDescription([
        DeclareLaunchArgument('browser', default_value='true',
                              description='Open the dashboard in Firefox'),
        DeclareLaunchArgument('http_port', default_value='8080', description='Web page port'),
        DeclareLaunchArgument('ws_port', default_value='9090', description='Bridge port'),
        DeclareLaunchArgument('joy', default_value='false',
                              description='Start joy_node and teleop_twist_joy for a USB '
                                          'controller (STOP button, Manual driving)'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='Start the rosbridge websocket (needs '
                                          'ros-jazzy-rosbridge-suite)'),
        OpaqueFunction(function=check_bridge),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'robot.launch.py')),
            launch_arguments={
                'world': os.path.join(sim, 'worlds', 'booth.sdf'),
                'x': str(home_x), 'y': str(home_y), 'yaw': f'{home_yaw:.6f}',
                'rviz_config': os.path.join(bringup, 'rviz', 'navigation.rviz'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav, 'launch', 'localization.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'map': os.path.join(nav, 'maps', 'booth.yaml'),
                'amcl_params': os.path.join(demo, 'config', 'booth_amcl.yaml'),
                'initial_x': str(home_x), 'initial_y': str(home_y),
                'initial_yaw': f'{math.remainder(home_yaw, 2 * math.pi):.6f}',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(demo, 'launch', 'demo.launch.py')),
            launch_arguments={
                'use_sim_time': 'true', 'sim_controls': 'true', 'joy': LaunchConfiguration('joy'),
                'bridge': LaunchConfiguration('bridge'),
                'http_port': http_port, 'ws_port': LaunchConfiguration('ws_port'),
            }.items()),
        Node(package='mecanum_simulation', executable='booth_visitor', output='screen',
             parameters=[{'use_sim_time': True}]),
        TimerAction(period=8.0, condition=IfCondition(LaunchConfiguration('browser')), actions=[
            ExecuteProcess(cmd=['firefox', '--new-window', ['http://localhost:', http_port, '/']],
                           output='log')]),
    ])
