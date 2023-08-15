# [Kaia.ai](https://kaia.ai/) ROS2 bringup package

Before launching Kaia.ai ROS2 stack on your PC, please make sure you have installed
Docker for Windows or Docker for Linux

## Launching a Kaia.ai robot
Run the [end user](https://hub.docker.com/r/kaiaai/kaia-ros) Docker image by typing this command
at your Windows or Linux prompt. The `launch` option will make the [Kaia.ai](https://kaia.ai)
ROS2 stack will launch automatically. Make sure that you have installed Docker for Windows or Docker for Linux.
```
docker run --name kaia-ros-iron -it --rm -p 8888:8888/udp kaiaai/kaia-ros:iron launch
```

Now you can power up your Kaia.ai robot. If this is the first time you have powered up
your robot:
- run the Docker image using the `launch` option as described above
- use your smartphone or PC to search for WiFi networks
- find and connect to "KAIA-WIFI" network
- using your smartphone or PC browser to open [http://192.168.4.1](http://192.168.4.1)
- enter your WiFi network name (SSID), password and click Submit
- your robot will attempt to connect to your WiFi network. If unsuccessful, the robot will re-start
its KAIA-WIFI network, so you can retry entering the correct WIFI name and password

Once connected to your WiFi, the robot will then attempt connecting to your to your PC that is running
the Kaia.ai ROS2 stack you have launched using the Docker image.

## Development
If you are a developer adapting the Kaia.ai platform to your own robot, use this command to
launch the [development](https://hub.docker.com/r/kaiaai/kaia-ros-dev) - as opposed to the
[end user](https://hub.docker.com/r/kaiaai/kaia-ros)) - version of the Kaia.ai ROS2 stack:
```
docker run --name kaia-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:iron launch
```

The command below loads the Kaia.ai development Docker image, but does not automatically launch the ROS2 stack:
```
docker run --name kaia-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:iron launch
```

Now you can launch the Kaia.ai ROS2 stack manually by typing
```
ros2 launch kaia_bringup main.launch.py description:=kaia_snoopy_snoopy_description
```

### Inspect robot model - URDF
```
ros2 launch kaia_bringup inspect_urdf.launch.py description:=kaia_snoopy_description
```

### Monitor your robot in action
```
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_snoopy_description
```
