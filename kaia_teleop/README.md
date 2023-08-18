# Control [Kaia.ai](https://kaia.ai) robots using a keyboard

## Run keyboard teleop with default settings
```
ros2 run kaia_teleop teleop_keyboard
```

## Run keyboard teleop with a robot description package
Let's say you are using a `waldo` robot defined in a `waldo_description` robot description package.
File `waldo_package/config/teleop_keyboard.yaml` is expected to contain `waldo`'s parameters for
keyboard teleoperation, including `waldo`'s maximum linear and angular speeds.

The command below loads runs the `teleop_keyboard` node and loads `waldo`-specific settings.
```
ros2 run kaia_teleop teleop_keyboard description:=kaia_loki_description
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
ros2 run kaia_teleop teleop_keyboard --ros-args --params-file `ros2 pkg prefix --share waldo_description`/config/teleop_keyboard_test.yaml
```
