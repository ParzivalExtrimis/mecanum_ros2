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
Start the safety supervisor: /cmd_vel -> gate -> /cmd_vel_safe (docs/plan.md Phase 4).

The node stays on the wall clock, also in simulation (see safety_supervisor.py). Each watch_*
argument is true when that sensor is fitted; robot.launch.py passes its use_*
arguments. A sensor that is not watched gives no protection.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WATCHED = ('lidar_2d', 'l2', 'd435i', 'ultrasonics', 'cliff_tof', 'bumpers', 'joint_states',
           'odom')


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mecanum_safety'), 'config', 'safety.yaml')
    return LaunchDescription([
        *[DeclareLaunchArgument(f'watch_{name}', default_value='true',
                                description=f'Watch the {name} topics') for name in WATCHED],
        Node(
            package='mecanum_safety',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            parameters=[config, {
                **{f'watch_{name}': ParameterValue(LaunchConfiguration(f'watch_{name}'),
                                                   value_type=bool) for name in WATCHED},
            }]),
    ])
