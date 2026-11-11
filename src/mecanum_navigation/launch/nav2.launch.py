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
Nav2 for the mecanum base (docs/plan.md Phase 6), without localization.

    /map (map_server or slam_toolbox), TF map -> odom -> base_footprint, the sensors
      --> controller_server (MPPI Omni), planner_server (Smac Lattice, omni),
          behaviors, bt_navigator
      --> /cmd_vel_nav --velocity_smoother--> /cmd_vel_smoothed --collision_monitor--> /cmd_vel

The safety gate (mecanum_safety) takes /cmd_vel from there. Modelled on the upstream
nav2_bringup navigation_launch.py, with config/nav2.yaml, our behavior trees (a goal checker
selector added), the keepout mask's map_server, range_relay for the ultrasonic range layer, and
cloud_thinner (the D435i cloud thinned to 5 cm for the voxel layer and the collision monitor).
Not started, unlike upstream: route_server (it needs a route graph, and a node that fails to
configure stops the whole lifecycle bringup) and docking_server (Phase 7).

Arguments: use_sim_time, params_file, keepout (serve the keepout mask and enable the costmaps'
keepout layers; config/nav2.yaml reads it), keepout_mask (the mask's map_server YAML, on the
map's grid), mark_max_range (range_relay's cap, decision D6-a: default 0.4 m,
0 = off), log_level.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile

# Lifecycle order: the keepout mask first, so the costmaps' keepout layers get it at once.
KEEPOUT_NODES = ['keepout_mask_server']
NAV_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']


def generate_launch_description():
    share = get_package_share_directory('mecanum_navigation')
    # allow_substs: the params file uses $(find-pkg-share ...) for the lattice primitives.
    params = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)
    log_level = LaunchConfiguration('log_level')
    keepout = LaunchConfiguration('keepout')
    common = {'parameters': [params], 'output': 'screen',
              'arguments': ['--ros-args', '--log-level', log_level]}
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    bt_dir = os.path.join(share, 'behavior_trees')

    nodes = [
        Node(package='nav2_controller', executable='controller_server', remappings=to_nav,
             **common),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             remappings=to_nav, **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', arguments=['--ros-args', '--log-level', log_level],
             parameters=[params, {
                 'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_to_pose.xml'),
                 'default_nav_through_poses_bt_xml': os.path.join(
                     bt_dir, 'navigate_through_poses.xml'),
             }]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', remappings=to_nav, **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', **common),
        Node(package='mecanum_navigation', executable='cloud_thinner', name='cloud_thinner',
             output='screen'),
        Node(package='mecanum_navigation', executable='range_relay', name='range_relay',
             output='screen',
             parameters=[{'mark_max_range': LaunchConfiguration('mark_max_range')}]),
    ]
    keepout_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='keepout_mask_server',
             output='screen', arguments=['--ros-args', '--log-level', log_level],
             parameters=[params, {'yaml_filename': LaunchConfiguration('keepout_mask')}]),
    ]

    lifecycle_nodes = PythonExpression([
        str(KEEPOUT_NODES), ' + ', str(NAV_NODES), " if '", keepout, "'.lower() == 'true' else ",
        str(NAV_NODES)])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2.yaml'),
                              description='Nav2 parameters'),
        DeclareLaunchArgument('keepout', default_value='true',
                              description='Serve the keepout mask and use it in both costmaps'),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.join(share, 'maps', 'restroom_keepout.yaml'),
            description='Keepout mask (map_server YAML), made by tools/make_keepout_mask.py'),
        DeclareLaunchArgument('mark_max_range', default_value='0.4',
                              description='Largest ultrasonic range the costmap may mark, m '
                                          '(0 = the sensor maximum). 0.4: the D6-a fallback, '
                                          'Phase 6 measured phantoms in the doorway'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        GroupAction(scoped=True, actions=[
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            *nodes,
            GroupAction(condition=IfCondition(keepout), actions=keepout_nodes),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 arguments=['--ros-args', '--log-level', log_level],
                 parameters=[{'autostart': True, 'node_names': lifecycle_nodes}]),
        ]),
    ])
