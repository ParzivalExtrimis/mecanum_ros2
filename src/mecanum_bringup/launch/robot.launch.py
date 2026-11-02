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
Bring up the robot: simulation (default) or hardware, plus the drive.

sim:=true   Gazebo world + robot + ros2_control (mecanum_simulation/sim.launch.py), all on
            sim time. mecanum_drive_controller takes TwistStamped on /cmd_vel and owns
            odom -> base_footprint from the measured wheel states (/wheel/odometry).
sim:=false  robot_state_publisher + joint_state_publisher (zero wheel angles) and the
            open-loop esp32_driver with the esp32 transport. The real transport is not
            implemented until the rewritten firmware's protocol exists; the driver then exits
            with an explanation.
Both        L2 perception (mecanum_localization/perception.launch.py): the self-filter,
            /l2/points -> /l2/points_filtered. With use_l2_scan:=true also cloud-to-scan, on
            /l2/scan; /scan is the 2D lidar's.
            LIO odometry (mecanum_localization/lio.launch.py, lio:=true): rko_lio and the
            adapter. In sim it publishes /lio/odom only, with no TF (wheel odometry owns it);
            on hardware, with no wheel odometry, /odom and odom -> base_footprint. Its config
            is for simulation; the hardware LIO config is in hardware_sensors.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, Command, LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# The sensors with no hardware driver yet, with what each one leaves out when false.
OTHER_SENSORS = {
    'use_d435i': 'the RealSense D435i',
    'use_lidar_2d': 'the RPLIDAR C1',
    'use_ultrasonics': 'the four ultrasonics',
    'use_cliff_tof': 'the four cliff ToF sensors',
    'use_bumpers': 'the bumper planks',
}


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    sim_share = get_package_share_directory('mecanum_simulation')
    base_share = get_package_share_directory('mecanum_base')
    desc_share = get_package_share_directory('mecanum_description')
    loc_share = get_package_share_directory('mecanum_localization')

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    # Wheel odometry owns odom -> base_footprint in sim; LIO only on hardware (no encoders
    # read by the open-loop driver).
    lio_odom_topic = PythonExpression(["'/lio/odom' if '", sim, "' == 'true' else '/odom'"])
    lio_publish_tf = PythonExpression(["'false' if '", sim, "' == 'true' else 'true'"])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            **{sensor: LaunchConfiguration(sensor) for sensor in OTHER_SENSORS},
            'use_l2': LaunchConfiguration('use_l2'),
        }.items(),
        condition=IfCondition(sim))

    hardware_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_share, 'urdf', 'mecanum.urdf.xacro'),
                 ' use_l2:=', LaunchConfiguration('use_l2')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='true: Gazebo simulation, false: real hardware'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup_share, 'rviz', 'sensors.rviz'),
            description='RViz configuration (robot.rviz: model and TF only)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI (sim only; false = headless)'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'restroom.sdf'),
            description='Gazebo world (sim only)'),
        DeclareLaunchArgument('use_l2', default_value='true',
                              description='Include the Unitree L2 in the robot model'),
        *[DeclareLaunchArgument(sensor, default_value='true',
                                description=f'Include {what} in the robot model (sim only)')
          for sensor, what in OTHER_SENSORS.items()],
        DeclareLaunchArgument('use_l2_scan', default_value='false',
                              description='Also make a 2D scan from the L2 cloud, on /l2/scan'),
        DeclareLaunchArgument('lio', default_value='true',
                              description='Run LIO odometry (needs use_l2)'),

        sim_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': hardware_description}],
            condition=UnlessCondition(sim)),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(sim)),

        # L2 perception, identical in sim and on hardware.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': sim,
                              'use_l2_scan': LaunchConfiguration('use_l2_scan')}.items(),
            condition=IfCondition(LaunchConfiguration('use_l2'))),

        # LIO odometry. In sim an evaluation input on /lio/odom (an EKF input from
        # docs/plan.md Phase 3); on hardware the source of odom -> base_footprint.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc_share, 'launch', 'lio.launch.py')),
            launch_arguments={'use_sim_time': sim, 'odom_topic': lio_odom_topic,
                              'publish_tf': lio_publish_tf}.items(),
            condition=IfCondition(AndSubstitution(LaunchConfiguration('use_l2'),
                                                  LaunchConfiguration('lio')))),

        # The open-loop ESP32 driver, hardware only; in sim, ros2_control drives the wheels.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base_share, 'launch', 'base.launch.py')),
            launch_arguments={'transport': 'esp32', 'use_sim_time': 'false'}.items(),
            condition=UnlessCondition(sim)),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': sim}],
            condition=IfCondition(rviz)),
    ])
