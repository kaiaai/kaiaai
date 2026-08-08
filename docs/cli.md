# kaia CLI reference

`kaia` stores per-robot configuration on your machine and lets you tune a
robot's settings — including values inside its ROS 2 config files — from the
command line. It is a thin wrapper around `ros2 run kaiaai cli`.

The configuration lives in a single file, `~/.kaiaai.yaml`.

## The model

Every setting belongs to a **scope** identified by two things:

- `robot.model` — the robot description package, e.g. `makerspet_mini`,
  `oomwoo_one`. This is the default (`makerspet_mini`) selected package.
- `robot.instance` — a named variant of that model, e.g. `default`, `fast`,
  `slow`. Instances let you keep several tunings of the *same* robot side by
  side and switch between them.

Together they read as `robot = model.instance` (e.g. `oomwoo_one.fast`). The
active `robot.model`/`robot.instance` pair is **global**; every *other*
variable is stored under `models/<model>/<instance>/`, so a value set for one
robot (or one instance) never leaks into another.

## Quick start

```bash
kaia use oomwoo_one                 # select the robot (instance defaults to "default")
kaia set clean.v_cruise 0.35        # set a variable for oomwoo_one.default
kaia list                           # show the active scope
kaia use oomwoo_one.fast            # switch to a "fast" instance of the same robot
kaia set clean.v_cruise 0.6         # this value belongs to oomwoo_one.fast only
```

## Commands

### `kaia use [MODEL[.INSTANCE]]`

With no argument, print the robot currently in use (as `model.instance`).
With an argument, switch the active robot, and optionally the instance: a bare
`MODEL` selects its `default` instance; `.INSTANCE` alone keeps the current model.

```bash
kaia use                       # -> oomwoo_one.default   (which robot am I on?)
kaia use oomwoo_one            # oomwoo_one, default instance
kaia use oomwoo_one.fast       # oomwoo_one, fast instance
kaia use .slow                 # current model, slow instance
```

Switching **re-renders** any config files this scope edits (see
[Editing config files](#editing-config-files)): the scope you leave is reverted
to package defaults, then the scope you enter is written on top.

### `kaia set VAR VALUE [--robot M[.I]]`

Set a variable in the active scope (or another scope with `--robot`).

```bash
kaia set clean.v_cruise 0.35
kaia set nav.v_cruise 0.5
```

Variable names are opaque keys; dotted prefixes like `clean.` / `nav.` are just
a convention so different launches can read their own group.

#### Live push to a running node

Setting a simple variable also updates it **live**, so you don't have to switch
to `ros2 param set` once a node is up. After persisting the value, `kaia set`
looks for any running node that declares a parameter named after the variable's
leaf (`clean.arc_omega` → parameter `arc_omega`) and sets it there:

```bash
kaia set clean.arc_omega 0.1
# set clean.arc_omega = 0.1  [makerspet_mini/default]
#   live: /wall_clean arc_omega updated
```

This works because the launch convention seeds each node parameter from the
variable's leaf name (see `wall_clean.launch.py`), so no node↔section registry
is needed. It is **best-effort**: it stays silent when ROS isn't sourced, no
node is running, or nothing declares that parameter — so setting values before
launch behaves exactly as before. The value is coerced to the parameter's
declared type; if more than one running node declares the leaf, each is updated
and reported. It does not apply to `FILE.yaml/...` keys (those edit files on
disk) or to `robot.model` / `robot.instance`.

Because probing the ROS graph adds ~1 s per `set` when your shell is sourced,
set **`KAIA_NO_LIVE_PARAMS=1`** to skip the probe (handy when scripting many
sets before anything is launched).

### `kaia get VAR [--robot M[.I]]`

Print a variable's value.

### `kaia unset VAR [--robot M[.I]]`

Remove a variable, reverting it to its default. For a config-file edit, this
restores the original value (see below).

### `kaia list [PREFIX] [--robot M[.I]]`

Show a scope's variables, and any other instances of the model. A `PREFIX`
narrows the list to that namespace — `kaia list clean` shows `clean` and every
`clean.*` variable.

```bash
kaia list          # everything in the active scope
kaia list clean    # only clean / clean.* variables
```

### `kaia copy SRC DST`

Copy one scope's variables into another (a merge — collisions are overwritten).
Handy to port settings between separate description packages, or to seed a new
instance:

```bash
kaia copy oomwoo_one oomwoo_one_with_dock    # port to another robot package
kaia copy oomwoo_one oomwoo_one.fast         # seed a new instance from default
```

### `kaia export [--robot M[.I]] [FILE]` / `kaia import FILE`

Share a complete configuration. `export` writes YAML (the whole config, one
model, or one scope) to `FILE` or stdout; `import` merges it back.

```bash
kaia export --robot oomwoo_one team-tuning.yaml
kaia import team-tuning.yaml
```

`import` never changes which robot *you* are on — it drops `robot.model` /
`robot.instance` from the incoming file and merges the rest.

## Targeting another scope: `--robot`

`set`, `get`, `unset`, and `list` accept `--robot` to operate on a scope other
than the active one, without switching to it:

| `--robot` value  | Targets                                   |
|------------------|-------------------------------------------|
| *(omitted)*      | the active scope                          |
| `MODEL`          | `MODEL`, its **default** instance         |
| `MODEL.INSTANCE` | `MODEL`, `INSTANCE`                        |
| `.INSTANCE`      | the **current** model, `INSTANCE`         |

A bare `MODEL` resolves to `default` (not the active instance), so naming
another robot never leaks your current instance across models.

## Editing config files

A variable whose name contains `/` addresses a ROS 2 config file shipped by the
robot package: `FILE.yaml/dotted.path.to.key`.

```bash
kaia set navigation.yaml/amcl.ros__parameters.alpha1 0.2
```

This edits `<robot_package>/config/navigation.yaml` **in place**, changing only
the single target line so the rest of a hand-tuned file stays intact. The
previous value is stashed inline as a comment:

```yaml
    alpha1: 0.2  # kaia-was: 0.25
```

- **`kaia get navigation.yaml/...`** prints the file's current value.
- **`kaia unset navigation.yaml/...`** restores the value from `# kaia-was:`
  and removes the comment.
- Repeated `set`s keep the **original** default in `# kaia-was:`, so `unset`
  always returns to where you started.
- Nav2 (and other nodes) load the real package file — kaia does not generate a
  separate merged file.

The file is located via the ROS package index, so the package must be built and
sourced. Under `colcon build --symlink-install` (the dev images) the package's
share path points at your source tree, so edits persist and are what the robot
loads. On a robot where the package was installed **read-only** (e.g. from a
Debian package), the edit fails with a clear message — config-file tuning is a
dev-workspace activity; the tuned values ship *with* the package.

## The config file

`~/.kaiaai.yaml`:

```yaml
robot.model: oomwoo_one
robot.instance: fast
models:
  oomwoo_one:
    default:
      clean.v_cruise: '0.35'
      navigation.yaml/amcl.ros__parameters.alpha1: '0.2'
    fast:
      clean.v_cruise: '0.6'
```

Values are stored as strings. Editing the file by hand is fine; the CLI is just
a convenience over it.

## Legacy syntax

The older `kaia config` form still works and maps onto the new verbs:

```bash
kaia config                    # like: kaia list
kaia config VAR                # like: kaia get VAR
kaia config VAR VALUE          # like: kaia set VAR VALUE
kaia config robot.model NAME   # sets the active model (kaia use NAME also
                               #   resets the instance and re-renders files)
```

For the programmatic side — how launch files read these values — see the
[Python API reference](python-api.md).
