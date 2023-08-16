# [Kaia.ai](https://kaia.ai) ROS2 pet robots

- 3D printable models of [Kaia.ai](https://kaia.ai) DIY pet robots [repo](https://github.com/kaiaai/3d_printables)
- Arduino ESP32 robot firmware [repo](https://github.com/kaiaai/arduino_fw/)
- Micro-ROS Arduino library for Kaia.ai robots [repo](https://github.com/kaiaai/micro_ros_arduino_kaia/)
- end-user and development ROS2 Docker images [repo](https://github.com/kaiaai/docker/)
- robot simulation ROS2 packages [repo](https://github.com/kaiaai/kaia_simulations/)
- ROS2 messages package for Kaia.ai robots [repo](https://github.com/kaiaai/kaia_msgs/)
- DIY electronic hardware, PCB designs for Kaia.ai robots [repo](https://github.com/kaiaai/electronics/)

## Command cheat sheet
```
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_snoopy_description
ros2 launch kaia_bringup inspect_urdf.launch.py description:=kaia_snoopy_description
ros2 launch kaia_bringup main.launch.py description:=kaia_snoopy_description

ros2 launch kaia_gazebo kaia_world.launch.py description:=kaia_snoopy_description
ros2 run kaia_teleop teleop_keyboard
ros2 run kaia_gazebo kaia_self_drive
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_snoopy_description

ros2 launch kaia_gazebo kaia_world.launch.py
ros2 launch kaia_cartographer cartographer.launch.py use_sim_time:=True
ros2 run kaia_gazebo kaia_self_drive
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

ros2 launch kaia_gazebo kaia_world.launch.py
ros2 launch kaia_navigation nav2.launch.py use_sim_time:=True map:=$HOME/my_map.yaml
```

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS
[Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3)
