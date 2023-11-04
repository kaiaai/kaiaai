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

import os, re
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def make_node(context: LaunchContext, urdf_path):
    urdf_path_str = context.perform_substitution(urdf_path)

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    print("URDF file : {}".format(urdf_path))

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
    ]


def generate_launch_description():

    default_urdf_path = os.path.join(
        get_package_share_path('makerspet_snoopy'),
        'urdf',
#        default_robot_model + '.urdf.xacro')
        'robot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            # default_value='false',
            default_value='true',
            choices=['true', 'false'],
            description='Enable joint state publisher GUI'
        ),
        DeclareLaunchArgument(
            name='urdf_path',
            default_value=default_urdf_path,
            description='A full pathname to the robot description .urdf.xacro file'
        ),
        OpaqueFunction(function=make_node, args=[
            LaunchConfiguration('urdf_path'),
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
