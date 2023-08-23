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
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def make_nodes(context: LaunchContext, description, model, gui):
    description_str = context.perform_substitution(description)
    model_str = context.perform_substitution(model)
    gui_str = context.perform_substitution(gui)
    description_package_path = get_package_share_path(description_str)

    if model_str == '':
       model_str = re.sub(r'_description$', '', description_str) + '.urdf.xacro'

    urdf_path_name = os.path.join(
      description_package_path,
      'urdf',
      model_str)

    rviz_config_path = os.path.join(
        description_package_path,
        'rviz',
        'inspect_urdf.rviz')

    print("Rviz2 config : {}".format(rviz_config_path))
    print("URDF  config : {}".format(urdf_path_name))

    return [
        ExecuteProcess(
            cmd=[[
                'ros2 run kaia_bringup watch_urdf.sh ',
                urdf_path_name,
                ' ',
                gui_str,
            ]],
            output='screen',
            shell=True
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

    return LaunchDescription([
        DeclareLaunchArgument(
            name='description',
            default_value=default_description_name,
            description='Robot description package name, overrides KAIA_ROBOT_DESCRIPTION'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value='',
            description='URDF model file name'
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='false',
            choices=['true', 'false'],
            description='Enable joint state publisher GUI'
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('description'),
            LaunchConfiguration('model'),
            LaunchConfiguration('gui'),
        ])
    ])
