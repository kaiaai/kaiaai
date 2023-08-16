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
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def make_nodes(context: LaunchContext, description):
    description_str = context.perform_substitution(description)

    urdf_path_name = os.path.join(
      get_package_share_path(description_str),
      'urdf',
      'robot.urdf')

    print("URDF file name : {}".format(urdf_path_name))
    robot_description = ParameterValue(Command(['xacro ', urdf_path_name]), value_type=str)

    rviz_config_path = os.path.join(
        get_package_share_path(description_str),
        'rviz',
        'inspect_urdf.rviz')
    print("Rviz2 config file name : {}".format(rviz_config_path))

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )
    ]


def generate_launch_description():
    default_description_name = os.getenv('KAIA_ROBOT_DESCRIPTION', default='kaia_snoopy_description')

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='description',
            default_value=default_description_name,
            description='Robot description package name, overrides KAIA_ROBOT_DESCRIPTION'
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('description')
        ]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    ])
