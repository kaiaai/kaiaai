# Control [Kaia.ai](https://kaia.ai) robots using a keyboard

## Run keyboard teleop with default settings
The default settings assume you are using `makerspet_snoopy` robot.
```
ros2 run kaiaai_teleop teleop_keyboard
```

## Run keyboard teleop with another robot
Let's say you are using a `makerspet_loki` robot. File `makerspet_loki/config/teleop_keyboard.yaml` is expected to contain parameters for
keyboard teleoperation, including the maximum linear and angular speeds.

The command below loads runs the `teleop_keyboard` node and loads `makerspet_loki`-specific settings.
```
ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_loki
```
If you take updesigning your own robot hardware that uses [Kaia.ai](https://kaia.ai) platform as its software,
you have to update `teleop_keyboard.yaml` file to match your particular robot hardware. Thankfully,
updating that file is just a matter of editing a few values, e.g. the maximum speeds.

## Tips
- `teleop_keyboard` node cannot be launched from a `launch` file, including `.launch.py`.
If you try doing that, you will get an error (in Linux) `termios.error: (25, 'Inappropriate ioctl for device')`.
This error appears because `teleop_keyboard.py` needs direct access to TTY STDIN device in order to
capture keyboard strokes.
- you can also specify an arbitrary path and file name of the YAML configuration file when launching
`teleop_keyboard` as shown in the example below. This can be useful during development.
```
ros2 run kaiaai_teleop teleop_keyboard
ros2 run kaiaai_teleop teleop_keyboard --ros-args --params-file /path/to/teleop_keyboard.yaml
ros2 run kaiaai_teleop teleop_keyboard --ros-args --params-file `ros2 pkg prefix --share makerspet_loki`/config/teleop_keyboard.yaml
```
