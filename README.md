# Kaia.ai ROS2 home robots

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
with `makerspet_snoopy` or simply omitting `robot_model:=...`

### Launch Docker image to operate your robot

The Docker image contains ROS2 and micro-ROS pre-configured with additional Kaia.ai ROS2 packages.

Open a Windows command shell or Windows PowerShell window and type the command below. This should give you a bash prompt.
```
docker pull kaiaai/kaiaai-ros-dev:humble
docker run --name makerspet -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaiaai-ros-dev:humble
```

To get an aditional bash prompt, open another Windows command shell or Windows PowerShell window and type:
```
docker exec -it makerspet bash
```

### Operate a physical robot
```
# Launch the physical robot and drive it manually
ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki

# Monitor robot sensors
ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_loki

# Drive robot manually to create and save a map
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki

# Create and save a map
ros2 launch kaiaai_bringup cartographer.launch.py robot_model:=makerspet_loki
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0

# Robot self-drives using an existing map
ros2 launch kaiaai_bringup navigation.launch.py robot_model:=makerspet_loki map:=$HOME/my_map
```

### View, set physical robot's parameters
```
# View parameters
ros2 node list
ros2 node info /MAKERSPET_LOKI
ros2 param list /MAKERSPET_LOKI
ros2 param dump /MAKERSPET_LOKI

# Turn the robot's laser scanner motor off
ros2 param set /MAKERSPET_LOKI lds.motor_speed 0.0

# Turn the robot's laser scanner motor on, default speed
ros2 param set /MAKERSPET_LOKI lds.motor_speed -1.0

# Get laser scanner motor current speed
ros2 param get /MAKERSPET_LOKI lds.motor_speed
```

### Operate a simulated robot

```
# Launch the robot in a simulation - drive manually
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_loki

# Launch the robot in a simulation - robot self-drives around
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_loki

# Launch the robot in a simulation - create, save a map; robot self-drives around
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true robot_model:=makerspet_loki
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=makerspet_loki
ros2 run nav2_map_server map_saver_cli -f ~/living_room_map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki \
  map:=/ros_ws/src/kaiaai_simulations/kaiaai_gazebo/map/living_room.yaml

# Launch the robot in a simulation - navigate and create a map simultaneously; save the map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki slam:=True
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0

# Launch the robot in a simulation - navigate and create a map simultaneously
# Robot seeks out, self-drives to unknown locations to complete the mapping
ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true robot_model:=makerspet_loki slam:=True
ros2 launch explore_lite explore.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p save_map_timeout:=60.0
```

### Add your own modifications to an existing robot

```
# Inspect, edit robot's URDF model
ros2 launch kaiaai_bringup inspect_urdf.launch.py robot_model:=makerspet_loki
ros2 launch kaiaai_bringup edit_urdf.launch.py robot_model:=makerspet_loki

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/makerspet_loki
cd /ros_ws && colcon build --symlink-install --packages-select makerspet_loki
```

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS
[Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3)
