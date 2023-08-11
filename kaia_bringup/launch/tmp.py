#!/usr/bin/env python3
#
# Copyright 2023 REMAKE.AI
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
#
# ACKNOWLEDGEMENTS: This code is based on ROS2 urdf_tutorial

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    default_model_name = os.getenv('KAIA_BOT_MODEL', default='kaia_snoopy')
    default_package_name = os.getenv('KAIA_BOT_PACKAGE', default='kaia_description')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('kaia_bringup'),
        'rviz',
        'inspect_urdf.rviz')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_name,
                                      description='Robot model name, overrides KAIA_BOT_MODEL')
    package_arg = DeclareLaunchArgument(name='package', default_value=default_package_name,
                                      description='Robot description package name, overrides KAIA_BOT_PACKAGE')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')

    model_urdf_path = os.path.join(
        get_package_share_directory(LaunchConfiguration('package')),
        'urdf',
        LaunchConfiguration('model') + '.urdf')

    print()
    print(model_urdf_path)
    print()

    robot_description = ParameterValue(Command(['xacro ', model_urdf_path]),
                                       value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
