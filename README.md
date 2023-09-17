# [Kaia.ai](https://kaia.ai) ROS2 pet robots

[Kaia.ai](https://kaia.ai) is a platform for 3D-printable pet robots. Please sign up for an early launch invite [here](https://remake.ai).

Kaia.ai DIY 3D-printable pet robots platform consists of these parts:
- 3D printable models of [Kaia.ai](https://kaia.ai) DIY pet robots [repo](https://github.com/makerspet/3d_printables)
- Arduino ESP32 robot firmware [repo](https://github.com/kaiaai/arduino_fw/)
- Micro-ROS Arduino library for Kaia.ai robots [repo](https://github.com/kaiaai/micro_ros_arduino_kaia/)
- End-user and development ROS2 Docker images [repo](https://github.com/kaiaai/docker/)
- Robot simulation ROS2 packages [repo](https://github.com/kaiaai/kaia_simulations/)
- Robot operation ROS2 packages in this repo - including SLAM mapping, navigation, etc.
- DIY electronic hardware, PCB designs for Kaia.ai robots [repo](https://github.com/makerspet/electronics/)


## Kaia.ai compatible robots
Here is a [list of robots](https://github.com/topics/kaia-ai-robot) compatible with Kaia.ai software platform.

## Command cheat sheets
<details>
<summary>Operate the default Maker's Pet Snoopy pet robot</summary>

```
# Launch the physical robot
ros2 launch kaia_bringup main.launch.py

# Monitor robot's sensors
ros2 launch kaia_bringup rviz2.launch.py

# Launch the robot in a simulation - drive manually or let it self-drive
ros2 launch kaia_gazebo world.launch.py
ros2 run kaia_teleop teleop_keyboard
ros2 launch kaia_gazebo self_drive_gazebo.launch.py
ros2 launch kaia_bringup rviz2.launch.py

# Launch the robot in a simulation - create, save a map
ros2 launch kaia_gazebo world.launch.py
ros2 launch kaia_bringup cartographer.launch.py use_sim_time:=true
ros2 launch kaia_gazebo self_drive_gazebo.launch.py
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaia_gazebo world.launch.py
ros2 launch kaia_bringup navigation.launch.py use_sim_time:=true map:=$HOME/my_map.yaml

# Inspect or edit robot's URDF model - useful when modding a robot
ros2 launch kaia_bringup inspect_urdf.launch.py
ros2 launch kaia_bringup edit_urdf.launch.py
```

</details>

<details>
<summary>Operate a modded pet robot residing `awesome_droid` repo:</summary>

```
# Launch the physical robot
ros2 launch kaia_bringup main.launch.py description:=awesome_droid

# Monitor robot's sensors
ros2 launch kaia_bringup rviz2.launch.py description:=awesome_droid

# Launch the robot in a simulation - drive manually or let it self-drive
ros2 launch kaia_gazebo world.launch.py description:=awesome_droid
ros2 run kaia_teleop teleop_keyboard description:=awesome_droid
ros2 launch kaia_gazebo self_drive_gazebo.launch.py description:=awesome_droid
ros2 launch kaia_bringup rviz2.launch.py description:=awesome_droid

# Launch the robot in a simulation - create, save a map
ros2 launch kaia_gazebo world.launch.py description:=awesome_droid
ros2 launch kaia_bringup cartographer.launch.py use_sim_time:=true description:=awesome_droid
ros2 launch kaia_gazebo self_drive_gazebo.launch.py description:=awesome_droid
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaia_gazebo world.launch.py description:=awesome_droid
ros2 launch kaia_bringup navigation.launch.py use_sim_time:=true map:=$HOME/my_map.yaml description:=awesome_droid

# Inspect or edit robot's URDF model - useful when modding a robot
ros2 launch kaia_bringup inspect_urdf.launch.py description:=awesome_droid model:=my_model
ros2 launch kaia_bringup edit_urdf.launch.py description:=awesome_droid model:=my_model

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaia_gazebo urdf2sdf.sh /ros_ws/src/awesome_droid/urdf/ r2d2
cd /ros_ws && colcon build --symlink-install --packages-select awesome_droid
```

</details>

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS
[Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3)
