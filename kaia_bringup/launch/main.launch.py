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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    urdf_file_name = os.getenv('KAIA_BOT_MODEL', default='snoopy') + '.urdf'
    package_name = os.getenv('KAIA_BOT_PACKAGE', default='kaia_description')
    # print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_path(package_name),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # print (robot_desc) # Printing urdf information.
    rsp_params = {'robot_description': robot_desc}
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        Node(
            package="kaia_telemetry",
            executable="telem",
            # name="kaia_telem_node",
            output="screen" #,
            # emulate_tty=True,
            # parameters=[
            #     {"my_parameter": "earth"}
            # ]
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    ])
