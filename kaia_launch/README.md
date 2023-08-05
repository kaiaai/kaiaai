# [Kaia.ai](https://kaia.ai/) ROS2 launch package

Before launching Kaia.ai ROS2 stack on your PC, please make sure you have installed Docker for Windows or Docker for Linux

Launch the *end user* Docker image by typing this command at your Windows or Linux prompt. The `launch` option will make the [Kaia.ai](https://kaia.ai) ROS2 stack will launch automatically. Make sure that you have installed Docker for Windows or Docker for Linux.
```
docker run --name kaia-ros-iron -it --rm -p 8888:8888/udp kaiaai/kaia-ros:iron launch
```

Now you can turn your Kaia.ai robot on. The robot will then attempt connecting to your WiFi network and, subsequently, to your PC ROS2 stack you have just launched.

If you are a developer adapting the Kaia.ai platform to your own robot, use this command to launch the *development* (as opposed to the *end user*) version of the Kaia.ai ROS2 stack:
```
docker run --name kaia-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:iron launch
```

Lastly, the command below loads the Kaia.ai development Docker image, but does not automatically launch the ROS2 stack:
```
docker run --name kaia-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:iron launch
```

Now you can launch the Kaia.ai ROS2 stack manually by typing
```
ros2 launch kaia_launch launch.py
```
