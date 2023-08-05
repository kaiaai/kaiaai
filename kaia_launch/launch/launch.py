from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kaia_telem",
            executable="telem",
            # name="kaia_telem_node",
            output="screen",
            # emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
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
        )
    ])
