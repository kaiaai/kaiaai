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

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_robot_description_node(context: LaunchContext, description, use_sim_time):
    description_str = context.perform_substitution(description)
    use_sim_time_str = context.perform_substitution(use_sim_time)

    urdf_path_name = os.path.join(
      get_package_share_path(description_str),
      'urdf',
      'robot.urdf')
    print('URDF file name : {}'.format(urdf_path_name))

    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
    robot_description = ParameterValue(Command(['xacro ', urdf_path_name]), value_type=str)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_str,
                'robot_description': robot_description
            }]
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
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package="kaia_telemetry",
            executable="telem",
            output="screen"
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output="screen",
            arguments=["udp4", "-p", "8888"]  # , "-v6"
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            output="screen",
            arguments = ["--frame-id", "map", "--child-frame-id", "lds"]
            # arguments = ["0", "0", "0", "0", "0", "0", "map", "lds"]
        ),
        OpaqueFunction(function=make_robot_description_node, args=[
            LaunchConfiguration('description'),
            LaunchConfiguration('use_sim_time')
        ]),
    ])
