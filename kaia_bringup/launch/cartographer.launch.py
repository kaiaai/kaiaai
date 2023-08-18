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
from launch.substitutions import Command, LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def make_nodes(context: LaunchContext, description, use_sim_time, config_lua):
    description_str = context.perform_substitution(description)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    config_lua_str = context.perform_substitution(config_lua)
    description_package_path = get_package_share_path(description_str)

    model_name = re.sub(r'_description$', '', description_str)
    urdf_path_name = os.path.join(
      description_package_path,
      'urdf',
      model_name + '.urdf')

    robot_description = ParameterValue(Command(['xacro ', urdf_path_name]), value_type=str)

    cartographer_config_path = os.path.join(
        description_package_path,
        'config')

    rviz_config_path = os.path.join(
        description_package_path,
        'rviz',
        'cartographer.rviz')

    print('URDF file           : {}'.format(urdf_path_name))
    print('Cartographer config : {}/{}'.format(config_lua_str, cartographer_config_path))
    print('Rviz2 config        : {}'.format(rviz_config_path))

    return [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_str}],
            arguments=['-configuration_directory', cartographer_config_path,
                       '-config_lua', config_lua_str]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time_str}],
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
            'config_lua',
            default_value='cartographer_lds_2d.lua',
            description='Name of Lua configuration file for cartographer'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': LaunchConfiguration('resolution'),
                'publish_period_sec': LaunchConfiguration('publish_period_sec')
            }.items(),
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('description'),
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('config_lua')
        ]),
    ])
