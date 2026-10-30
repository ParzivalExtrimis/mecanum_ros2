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
Start Gazebo with the restroom world and spawn the mecanum base.

Starts: Gazebo (gz sim), robot_state_publisher (sim xacro), the spawner, ros_gz_bridge and,
once the robot is spawned, the controllers (mecanum_controllers/controllers.launch.py):
joint_state_broadcaster publishes /joint_states and mecanum_drive_controller takes
TwistStamped on /cmd_vel and publishes /wheel/odometry and odom -> base_footprint.

Sensors: each use_<sensor> argument takes a sensor out of the model. Its nodes stay down and its
bridge entries wait for a Gazebo topic that never appears. The ultrasonic, cliff ToF and bumper
topics and the D435i cloud come from the sim-only nodes sim_range_sensor, sim_bumper and
sim_depth_cloud. These three stay off the simulation clock on purpose: they copy the stamps of
their input or have none, and a Python node on /clock (500 Hz) takes half a core.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Xacro arguments of mecanum_description, with what each one leaves out when false.
SENSORS = {
    'use_l2': 'the Unitree L2',
    'use_d435i': 'the RealSense D435i',
    'use_lidar_2d': 'the RPLIDAR C1',
    'use_ultrasonics': 'the four ultrasonics',
    'use_cliff_tof': 'the four cliff ToF sensors',
    'use_bumpers': 'the bumper planks',
}


def generate_launch_description():
    sim_share = get_package_share_directory('mecanum_simulation')
    desc_share = get_package_share_directory('mecanum_description')
    ctrl_share = get_package_share_directory('mecanum_controllers')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                             'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    xacro_command = ['xacro ', os.path.join(sim_share, 'urdf', 'mecanum_sim.urdf.xacro')]
    for sensor in SENSORS:
        xacro_command += [f' {sensor}:=', LaunchConfiguration(sensor)]
    robot_description = ParameterValue(Command(xacro_command), value_type=str)
    range_config = os.path.join(sim_share, 'config', 'sim_range_sensors.yaml')

    def range_sensor(name, sensor):
        return Node(
            package='mecanum_simulation',
            executable='sim_range_sensor',
            name=name,
            output='screen',
            parameters=[range_config],
            condition=IfCondition(LaunchConfiguration(sensor)))

    def gazebo(extra_args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [extra_args, world],
                              'on_exit_shutdown': 'true'}.items(),
            condition=condition)

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'mecanum',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.01', '-Y', LaunchConfiguration('yaw')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'restroom.sdf'),
            description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI (false = headless server)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x (m)'),
        DeclareLaunchArgument('y', default_value='-1.0', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad)'),
        *[DeclareLaunchArgument(sensor, default_value='true',
                                description=f'Include {what} in the robot model')
          for sensor, what in SENSORS.items()],

        # Lets Gazebo resolve package://mecanum_description/... mesh URIs without changing
        # mecanum_description's package.xml.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),

        gazebo('-r -v 2 ', IfCondition(gui)),
        gazebo('-r -s -v 2 ', UnlessCondition(gui)),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        spawn,
        # The controller manager lives in the gz_ros2_control plugin, which loads with the
        # robot: spawn the controllers once the robot exists.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ctrl_share, 'launch', 'controllers.launch.py')),
                launch_arguments={'hardware': 'sim'}.items()),
        ])),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'config_file': os.path.join(sim_share, 'config', 'gz_bridge.yaml'),
                         'use_sim_time': True}],
        ),
        Node(
            package='mecanum_simulation',
            executable='sim_depth_cloud',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_d435i'))),
        range_sensor('sim_ultrasonics', 'use_ultrasonics'),
        range_sensor('sim_cliff_tof', 'use_cliff_tof'),
        Node(
            package='mecanum_simulation',
            executable='sim_bumper',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_bumpers'))),
    ])
