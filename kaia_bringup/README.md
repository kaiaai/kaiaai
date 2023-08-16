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
  - The WiFi network ESP32 is connecting to must be the same network where your Kaia.ai ROS2 PC
is connected to
  - The network also must not block its WiFi clients from connecting to each
other. For example, your home WiFi network will likely work fine, but WiFi at a Starbucks
will probably not work
  - Important - your WiFi network must provide a strong and fast connection throughout
your residence (or place of use). If your WiFi signal is not strong enough, consider
using a WiFi repeater
  - For security purposes, make sure your WiFi connection is encrypted using a strong
password and a strong WiFi encryption protocol, e.g. WPA3 or WPA2.
  - For best security, consider creating a dedicated WiFi network for your robot and
your Kaia.ai ROS2 PC only, so that no other WiFi device (except the WiFi router) will
receive robot network traffic - which, currently, runs unencrypted in ROS2 applications
- your robot will attempt to connect to your WiFi  
If unsuccessful, the robot will re-start its KAIA-WIFI network, so you can retry entering
the correct WIFI name and password. Monitor your robot's ESP32 board LED blinking pattern
to detect WiFi connection success or failure
- Once connected to your WiFi, the robot will connect automatically to your to your PC
running the Kaia.ai ROS2 stack that you have launched using the Docker image  
If the connection fails, your EPS32 board will indicating the connection error using
a corresponding LED blinking pattern and reboot after a delay. Please continue reading
to learn how to debug ESP32 connection problems.

## Development
If you are a developer adapting the Kaia.ai platform to your own robot, use this command to
launch the [development](https://hub.docker.com/r/kaiaai/kaia-ros-dev) - as opposed to the
[end user](https://hub.docker.com/r/kaiaai/kaia-ros) - version of the Kaia.ai ROS2 stack:
```
docker run --name kaia-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:iron launch
```

The command below loads the Kaia.ai development Docker image, but does not automatically launch the ROS2 stack:
```
docker run --name kaia-ros-dev-iron -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:iron launch
```

Launch physical robot manually
- launch the Kaia.ai ROS2 stack using the command below. Set `description` below to the
robot model description of your physical robot
- turn on your robot's power
- make sure your robot connects to same WiFi where your Kaia.ai ROS2 stack PC is connected
- at this point your robot should connect to the Kaia.ai ROS2 stack automatically  
You can verify the successful connection by monitoring ESP32 board LED blinking pattern.
If the LED blinking pattern indicates an error, connect your PC to your robot's ESP32 board
using a USB cable, launch Arduino IDE and open the Arduino IDE Serial Monitor by
selecting Tools -> Serial Monitor menu items
- once the WiFi and ROS2 PC connections have been established, your robot is ready for use
```
ros2 launch kaia_bringup main.launch.py description:=kaia_snoopy_description
```

### Monitor your robot links, sensors
```
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_snoopy_description
```

### Optional: Inspect robot model - URDF
```
ros2 launch kaia_bringup inspect_urdf.launch.py description:=kaia_snoopy_description
```
