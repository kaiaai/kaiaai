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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def make_rviz2_node(context: LaunchContext, description):
    description_str = context.perform_substitution(description)

    model_name = re.sub(r'_description$', '', description_str)
    rviz_config_path = os.path.join(
        get_package_share_path(description_str),
        'rviz',
        model_name + '.rviz')
    print("Rviz2 config file name : {}".format(rviz_config_path))

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
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
        OpaqueFunction(function=make_rviz2_node, args=[
            LaunchConfiguration('description')
        ])
    ])
