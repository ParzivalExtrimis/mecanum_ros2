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

"""Run the motion test: forward, strafe left, rotate, one at a time."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'use_sim_time': ('false', 'Use the simulation clock'),
        'measure_odom_topic': ('', 'Odometry used to measure each step; empty = no '
                               'measurement. /sim/ground_truth/odom in sim, /odom with LIO'),
        'cmd_vel_stamped': ('false', 'Publish TwistStamped instead of Twist'),
        'linear_speed': ('0.15', 'Forward and strafe speed (m/s)'),
        'angular_speed': ('0.5', 'Rotation speed (rad/s)'),
        'move_duration': ('3.0', 'Seconds per step'),
    }
    declared = [DeclareLaunchArgument(name, default_value=default, description=text)
                for name, (default, text) in args.items()]
    params = {name: LaunchConfiguration(name) for name in args}
    # Keep an empty topic name a string instead of letting YAML coercion turn it into null.
    params['measure_odom_topic'] = ParameterValue(
        LaunchConfiguration('measure_odom_topic'), value_type=str)

    return LaunchDescription(declared + [
        Node(
            package='mecanum_base',
            executable='motion_test',
            name='motion_test',
            output='screen',
            emulate_tty=True,
            parameters=[params],
        ),
    ])
