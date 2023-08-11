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
# ACKNOWLEDGEMENTS: This code is based on ROBOTIS Turtlebot3

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def make_robot_description_node(context: LaunchContext, model_name, package_name):
    model_name_str = context.perform_substitution(model_name)
    package_name_str = context.perform_substitution(package_name)

    rviz_config_path_str = os.path.join(
        get_package_share_path(package_name_str),
        'rviz',
        model_name_str + '.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path_str],
        output='screen')

    return [rviz2_node]


def generate_launch_description():
    default_model_name = os.getenv('KAIA_BOT_MODEL', default='kaia_snoopy')
    default_package_name = os.getenv('KAIA_BOT_PACKAGE', default='kaia_description')

    package_name_arg = DeclareLaunchArgument(name='package', default_value=str(default_package_name),
                                             description='Robot description package name, overrides KAIA_BOT_PACKAGE')
    model_name_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_name),
                                           description='Robot model name, overrides KAIA_BOT_MODEL')

    return LaunchDescription([
        model_name_arg,
        package_name_arg,
        OpaqueFunction(function=make_robot_description_node, args=[
            LaunchConfiguration('model'),
            LaunchConfiguration('package')
        ])
    ])
