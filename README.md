# [Kaia.ai](https://kaia.ai) ROS2 home robots

[Kaia.ai](https://kaia.ai) is a platform for DIY home robots. Please sign up for an early launch invite [here](https://remake.ai).

Kaia.ai robotics software platform consists of these parts:
- Micro-ROS Arduino library for Kaia.ai robots [repo](https://github.com/kaiaai/micro_ros_arduino_kaia)
- End-user and development ROS2 Docker images [repo](https://github.com/kaiaai/docker)
- Robot simulation ROS2 packages [repo](https://github.com/kaiaai/kaiaai_simulations)
- Robot operation ROS2 packages [repo](https://github.com/kaiaai/kaiaai), including SLAM mapping, navigation, etc.
- Cloud software infrastructure

## Kaia.ai compatible robots
Here is a [list of robots](https://github.com/topics/kaiaai-robot) compatible with Kaia.ai software platform.

In particular, these two models are supported:
- Maker's Pet [Loki](https://github.com/makerspet/makerspet_loki) 200mm 3D-printable pet robot
- Maker's Pet [Fido](https://github.com/makerspet/makerspet_fido) 250mm 3D-printable pet robot
- Maker's Pet [Snoopy](https://github.com/makerspet/makerspet_snoopy) 300mm 3D-printable pet robot

## Setup
- Set up your development PC following these
[instructions](https://github.com/kaiaai/kaiaai_simulations#your-pc-setup).
- [Launch](https://github.com/kaiaai/kaiaai_simulations/blob/main/README.md#launch-the-development-docker-image)
the Kaia.ai development Docker image.
- [Power up](https://github.com/makerspet/makerspet_snoopy/tree/main/firmware) your robot.
- [Connect](https://github.com/makerspet/makerspet_snoopy/tree/main/firmware) your robot to your WiFi.


## Command cheat sheets

Example: operate [makerspet_loki](https://github.com/makerspet/makerspet_loki) robot using commands below.

Operate other compatible robot models by making these changes:
- operate Maker's Pet [Fido](https://github.com/makerspet/makerspet_fido) robot by replacing `makerspet_loki`
with `makerspet_fido`
- operate Maker's Pet [Snoopy](https://github.com/makerspet/makerspet_fido) robot by replacing `makerspet_loki`
with `makerspet_snoopy` or simply omitting `description:=...`

## Operate a physical robot

```
# Launch the physical robot and drive it manually
ros2 launch kaiaai_bringup main.launch.py description:=makerspet_loki
ros2 run kaiaai_teleop teleop_keyboard description:=makerspet_loki

# Launch the physical robot and monitor its sensors
ros2 launch kaiaai_bringup main.launch.py description:=makerspet_loki
ros2 launch kaiaai_bringup rviz2.launch.py description:=makerspet_loki

# Launch the physical robot, monitor its sensors and drive it manually to create and save a map
ros2 launch kaiaai_bringup main.launch.py description:=makerspet_loki
ros2 launch kaiaai_bringup rviz2.launch.py description:=makerspet_loki
ros2 run kaiaai_teleop teleop_keyboard description:=makerspet_loki
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true description:=makerspet_loki
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map
```

### Operate a simulated robot

```
# Launch the robot in a simulation - drive manually or let it self-drive
ros2 launch kaiaai_gazebo world.launch.py description:=makerspet_loki
ros2 run kaiaai_teleop teleop_keyboard description:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py description:=makerspet_loki
ros2 launch kaiaai_bringup rviz2.launch.py description:=makerspet_loki

# Launch the robot in a simulation - create, save a map
ros2 launch kaiaai_gazebo world.launch.py description:=makerspet_loki
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true description:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py description:=makerspet_loki
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py description:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true map:=$HOME/my_map.yaml description:=makerspet_loki
```

### Add your own modifications to an existing robot

```
# Inspect, edit robot's URDF model
ros2 launch kaiaai_bringup inspect_urdf.launch.py description:=makerspet_loki
ros2 launch kaiaai_bringup edit_urdf.launch.py description:=makerspet_loki

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/makerspet_loki/urdf/makerspet_loki.urdf.xacro
cd /ros_ws && colcon build --symlink-install --packages-select makerspet_loki
```

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS
[Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3)
