# [Kaia.ai](https://kaia.ai/) ROS2 telemetry package
[Kaia.ai](https://kaia.ai/) ROS2 kaia_telemetry package communicates with the Kaia.ai ESP32-based robot,
receives raw sensor telemetry data over WiFi using Micro-ROS and re-publishes the telemetry to standard ROS2 topics:
- robot laser scan data on the /scan topic
- robot wheels and servos position on the /joint_states topic
- robot wheel odometry on the /odom topic
- robot odometry transform on the /tf topic

The [telemetry message](https://github.com/kaiaai/kaia_msgs) is a ROS2 custom message designed to be as compact as possible in terms of its size in order to reduce communication latency and minimize dropped packets, thus keeping the robot's navigation responsive and agile.

The [kaiaai-ros](https://github.com/kaiaai/docker/tree/main/kaiaai-ros) and
[kaiaai-ros-dev](https://github.com/kaiaai/docker/tree/main/kaiaai-ros-dev) Docker images
already include a pre-compiled kaiaai_telemetry package. In case you'd like to modify and/or
build this package for development purposes, please follow these steps:
- on your PC run this command (e.g. in a Windows shell) to launch the Kaia.ai development
Docker image. The image should launch and give you a bash prompt.
```
docker run --name kaiaai-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai-ros-dev:iron
```
- navigate to the development workspace, manually build, install and launch Kaia.ai
packages including kaiaai_telemetry
```
cd /ros_ws
colcon build
. install/setup.bash
ros2 launch kaiaai_bringup main.launch.py robot_model:=makerspet_loki
```
- open one more bash prompt by running this command (e.g. in a Windows shell)
```
docker exec -it kaiaai-ros-dev-iron
```
- at the newly opened bash prompt, run the telemetry node that subscribes to the raw telemetry
data on /telemetry topic, converts the raw telemetry data to proper ROS2 messages re-publishes those on
/scan, /joint_states, /odom and /tf topics.
```
ros2 run kaiaai_telemetry telem
```
- open yet another bash prompt and inspect the raw telemetry data going "in" on the /telemetry topic
and the converted telemetry data published on the /scan, /joint_states, /odom and /tf topics, etc.:
```
ros2 topic list
ros2 topic echo /telemetry
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /joint_states
ros2 topic echo /tf
```
If you need to recreate the ROS2 workspace and re-populate the package source code, run
[commands from this file](https://github.com/kaiaai/docker/blob/main/kaiaai-ros-dev/Dockerfile)
before proceeding with the build, install and launch (see above):

If you don't have a robot handy, you can also run the test node that publishes some fake telemetry
data directly to /telemetry, bypassing Micro-ROS
```
ros2 run kaiaai_telemetry test_pub
```
Once you are ready to test and deploy your modifications into the end user Docker image, follow and
modify the instructions on rebuilding the
[end user Docker image](https://github.com/kaiaai/kaia_docker/tree/main/kaia-ros) to your liking to
build an end user Docker image for your own robot design.

## Modding the default robot
The telemetry launch commands described above default to `makerspet_snoopy` robot, which defined in the
`makerspet_snoopy` robot description package. To create a new robot named `jack45_waldo`, start
by cloning an existing robot description package `makerspet_snoopy` into `jack45_waldo`
and proceed with modding `jack45_waldo` files. The file containing telemetry parameters
for `jack45_waldo` is `/ros_ws/src/jack45_waldo/config/telem.yaml`
```
cp -r /ros_ws/src/makerspet_loki /ros_ws/src/jack45_waldo
```
Modify files in `/ros_ws/src/jack45_waldo/` as needed, including `/ros_ws/src/jack45_waldo/config/telem.yaml`

Now you can run telemetry on `jack45_waldo` with `jack45_waldo`-specific telemetry parameters as follows:
```
ros2 run kaiaai_telemetry telem robot_model:=jack45_waldo
```
