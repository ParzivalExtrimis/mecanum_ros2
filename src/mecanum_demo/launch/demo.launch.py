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
The robot side of the booth demo: the same on the laptop with Gazebo and on the robot's SBC.

    ros2 launch mecanum_demo demo.launch.py                     # hardware (wall clock)
    ros2 launch mecanum_demo demo.launch.py use_sim_time:=true  # with a simulation

Starts the demo supervisor, the segment driver, the health monitor, the websocket bridge
(rosbridge_server, port ws_port) and the panel's web server (mecanum_panel, port http_port),
and with joy:=true the controller's joy_node and teleop_twist_joy (config/joy_teleop.yaml: STOP,
and Manual driving on /manual/cmd_vel). A keyboard drives Manual with
mecanum_bringup/teleop.launch.py cmd_vel_topic:=/manual/cmd_vel. It expects the robot's own
bringup to run already: the drive behind the safety gate, the sensors, and localization (TF
map -> base_footprint and /map). mecanum_bringup/demo_sim.launch.py starts all of it in Gazebo.

Pages, from any machine on the robot's network:
    http://<robot>:<http_port>/panel/   the touchscreen panel (800 x 480)
    http://<robot>:<http_port>/tv/      the TV
    http://<robot>:<http_port>/         the development dashboard: panel, TV, sim controls
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def bridge_node(context, sim_time):
    """Return the rosbridge websocket, or stop the launch with the install command."""
    if LaunchConfiguration('bridge').perform(context).lower() not in ('true', '1'):
        return []
    try:
        get_package_share_directory('rosbridge_server')
    except PackageNotFoundError:
        raise RuntimeError(
            'The panel needs the rosbridge websocket, which is not installed. Install it once '
            'with:  sudo apt install ros-jazzy-rosbridge-suite   (or launch with bridge:=false '
            'to run without the web pages talking to ROS)') from None
    return [Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
                 parameters=[{'port': ParameterValue(LaunchConfiguration('ws_port'),
                                                     value_type=int),
                              'use_sim_time': sim_time}])]


def generate_launch_description():
    share = get_package_share_directory('mecanum_demo')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    config = LaunchConfiguration('config')
    common = {'use_sim_time': sim_time, 'config': config}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('config',
                              default_value=os.path.join(share, 'config', 'booth_demo.yaml'),
                              description='The booth: poses, features and modes'),
        DeclareLaunchArgument('joy', default_value='false',
                              description='Start joy_node and teleop_twist_joy for a USB '
                                          'controller (STOP button; Manual driving on '
                                          '/manual/cmd_vel)'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='Start the rosbridge websocket'),
        DeclareLaunchArgument('panel', default_value='true',
                              description="Serve the panel's web pages"),
        DeclareLaunchArgument('ws_port', default_value='9090',
                              description='rosbridge websocket port'),
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='Web page port'),
        DeclareLaunchArgument('sim_controls', default_value='false',
                              description='Show the simulation controls on the dashboard'),

        OpaqueFunction(function=bridge_node, args=[sim_time]),
        Node(package='mecanum_demo', executable='demo_supervisor', output='screen',
             parameters=[common]),
        Node(package='mecanum_demo', executable='segment_driver', output='screen',
             parameters=[common]),
        Node(package='mecanum_demo', executable='health_monitor', output='screen',
             parameters=[{'use_sim_time': sim_time}]),
        Node(package='joy', executable='joy_node', output='screen',
             parameters=[{'use_sim_time': sim_time}],
             condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
             output='screen',
             parameters=[os.path.join(share, 'config', 'joy_teleop.yaml'),
                         {'use_sim_time': sim_time}],
             remappings=[('cmd_vel', '/manual/cmd_vel')],
             condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='mecanum_panel', executable='panel_server', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('http_port'),
                                                 value_type=int),
                          'ws_port': ParameterValue(LaunchConfiguration('ws_port'),
                                                    value_type=int),
                          'sim_controls': ParameterValue(LaunchConfiguration('sim_controls'),
                                                         value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('panel'))),
    ])
