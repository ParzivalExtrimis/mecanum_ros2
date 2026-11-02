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
Measure every odometry candidate on the same motion (simulation only, docs/plan.md Phase 3).

Run beside ``robot.launch.py``, which provides E0 (/wheel/odometry), E1 (/odom) and LIO
(/lio/odom). This adds the other filters on /eval/<name>/odom with no TF, one ``gt_compare`` per
candidate, and ``motion_test``. When the motion ends everything shuts down, and each gt_compare
prints its final report and writes ``<output_dir>/<name>.csv``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


# The candidates run here, beside the one robot.launch.py starts.
EXTRA_FILTERS = {'e2': 'ekf_e2.yaml', 'e3a': 'ekf_e3a.yaml', 'e3b': 'ekf_e3b.yaml'}

COMPARED = {
    'e0': '/wheel/odometry',
    'e1': '/odom',
    'lio': '/lio/odom',
    **{name: f'/eval/{name}/odom' for name in EXTRA_FILTERS},
}


def generate_launch_description():
    share = get_package_share_directory('mecanum_localization')
    output_dir = LaunchConfiguration('output_dir')

    filters = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'ekf.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'config': os.path.join(share, 'config', config),
                'node_name': f'ekf_{name}',
                'odom_topic': f'/eval/{name}/odom',
            }.items())
        for name, config in EXTRA_FILTERS.items()
    ]

    comparisons = [
        Node(
            package='mecanum_localization',
            executable='gt_compare',
            name=f'gt_compare_{name}',
            output='screen',
            parameters=[{
                'odom_topic': topic,
                'report_period': 0.0,
                'csv_path': PathJoinSubstitution([output_dir, f'{name}.csv']),
            }])
        for name, topic in COMPARED.items()
    ]

    motion = Node(
        package='mecanum_base',
        executable='motion_test',
        name='motion_test',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'cmd_vel_stamped': True,
            'measure_odom_topic': '/sim/ground_truth/odom',
            'pattern': LaunchConfiguration('pattern'),
            'laps': LaunchConfiguration('laps'),
            'loop_length': LaunchConfiguration('loop_length'),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', description='Existing folder for the CSV files'),
        DeclareLaunchArgument('pattern', default_value='loop',
                              description='motion_test pattern: loop or corridor'),
        DeclareLaunchArgument('laps', default_value='2', description='Number of laps'),
        DeclareLaunchArgument('loop_length', default_value='4.0',
                              description='Forward length of a lap (m)'),
        *filters,
        *comparisons,
        motion,
        RegisterEventHandler(OnProcessExit(target_action=motion,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
