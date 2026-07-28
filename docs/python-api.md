# kaiaai Python API reference

The `kaiaai.config` module is the stable contract that **launch files** use to
read the configuration the [`kaia` CLI](cli.md) manages. If you write a launch
file for a Kaia.ai robot, this is the API you call.

```python
from kaiaai import config
```

## The override convention

A launch file should read a setting with `config.get_var(name, default)`, and
let an explicit launch argument override it. The precedence is:

1. an explicit `name:=value` launch argument (highest),
2. the value stored for the active scope (`config.get_var`),
3. the built-in default (lowest).

This is exactly how `robot_model` and `lidar_model` already behave — e.g.
`ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_loki`
overrides the stored `robot.model`.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from kaiaai import config

def generate_launch_description():
    # stored value (or a fallback) becomes the argument's default;
    # a v_cruise:=... on the command line still wins.
    v_cruise = float(config.get_var('clean.v_cruise', 0.35))
    return LaunchDescription([
        DeclareLaunchArgument('v_cruise', default_value=str(v_cruise)),
        # ... nodes read LaunchConfiguration('v_cruise') ...
    ])
```

**Stored values are strings.** `get_var` returns the stored string, or the
`default` you pass unchanged. Wrap the call in the type you need — `float(...)`,
`int(...)` — which works whether the value came from the file (a string) or your
numeric default. Always pass a real default; don't rely on `None`.

## Reading and writing variables

```python
config.current_model()                  # -> 'oomwoo_one'  (active robot.model)
config.current_instance()               # -> 'fast'        (active robot.instance)

config.get_var(name, default=None, model=None, instance=None)
config.set_var(name, value, model=None, instance=None)
config.unset_var(name, model=None, instance=None)   # -> True if it existed
```

- `name` — a variable name. `robot.model` and `robot.instance` are **global**
  (stored at the top level); every other name is stored in the scope.
- `model` / `instance` — target another scope. Left as `None`, they resolve to
  the active `robot.model` / `robot.instance`.

```python
scope = config.scope_vars(model=None, instance=None)   # dict of the scope's vars
names = config.instances(model=None)                   # sorted instance names
```

## Switching the active robot

```python
config.use_robot(model=None, instance=None)   # -> (model, instance)
```

Sets the global `robot.model` / `robot.instance`. A `None` instance resets to
`default`. (The CLI's `kaia use` also re-renders config files; `use_robot`
itself only updates the stored selection.)

## Sharing configurations

```python
config.copy_scope(src_model, src_instance, dst_model, dst_instance)  # -> copied dict
config.export_config(model=None, instance=None)   # -> dict shaped like the config file
config.import_config(payload)                     # deep-merges a payload back in
```

`export_config(None)` returns the whole config; a `model` returns that model's
subtree; `model` + `instance` returns one scope. `import_config` ignores
`robot.model` / `robot.instance` in the payload so importing never changes which
robot the caller is on.

Helpers: `config.dumps(data)` (YAML text) and `config.load_file(path)`.

## Constants

| Name               | Value                          |
|--------------------|--------------------------------|
| `CONFIG_FILE_NAME` | `.kaiaai.yaml`                 |
| `DEFAULT_MODEL`    | `makerspet_mini`               |
| `DEFAULT_INSTANCE` | `default`                      |
| `GLOBAL_VARS`      | `('robot.model', 'robot.instance')` |

## `kaiaai.yamledit` — advanced, unstable

`kaiaai.yamledit` performs the in-place, comment-preserving edits behind
`kaia set FILE.yaml/...`. It is **not** part of the stable API — signatures may
change — but is available for tools that need to edit a package's config files
directly:

```python
from kaiaai import yamledit
path = yamledit.config_file_path(model, 'navigation.yaml')  # via the ROS package index
yamledit.ensure_writable(path)                              # raises if missing/read-only
was = yamledit.apply(path, 'amcl.ros__parameters.alpha1', '0.2')  # stash prior value
yamledit.read_value(path, 'amcl.ros__parameters.alpha1')    # current on-disk value
yamledit.revert(path, 'amcl.ros__parameters.alpha1')        # restore from # kaia-was:
```

Prefer `kaiaai.config` for anything a launch file needs.
