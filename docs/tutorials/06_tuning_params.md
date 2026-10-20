# 6. Tuning parameters

The `console_*` launch files take their parameters from one full file per platform, optionally
changed by a small file of yours. You never edit the installed copy and never need a rebuild to try
a value. Needs only [tutorial 1](01_install_and_build.md).

## 1. The platform files

| File | Sections | Used by |
|---|---|---|
| `config/slam_<platform>.yaml` | `slam_toolbox` (mapping), `map_server` and `amcl` (localization) | `console_slam.launch.py` |
| `config/nav2_<platform>.yaml` | the Nav2 servers (`bt_navigator`, `controller_server`, `local_costmap`, ...) | `console_nav.launch.py` |
| `config/keepout_filter.yaml` | the two keepout-filter servers; the same for every platform | `console_nav.launch.py` |

`<platform>` is `create3` or `rover` (`platform:=...`). There is no shared base file: each platform
file is complete, and its comments say which values differ from slam_toolbox's stock
`mapper_params_online_async.yaml` or from Nav2's defaults, and why. Typical knobs:
`max_laser_range`, `minimum_travel_distance` and `map_update_interval` (slam_toolbox);
`max_particles` and `update_min_d` (AMCL); `FollowPath.vx_max`, `robot_radius` and
`inflation_radius` (Nav2).

`params_file:=/full/path/file.yaml` replaces the installed platform file, for example your own
edited copy; if it is empty, the launch uses the installed `config/<stem>_<platform>.yaml`. A
platform with no file fails at launch, naming the missing file.

## 2. Overrides

`overrides_file:=` is merged over the platform file. It has the same shape, and may only change
keys the platform file **already has**; a misspelt key fails the launch instead of being ignored.
An empty or missing file means no overrides.

```yaml
# ~/maps/robot_overrides.yaml, for console_slam.launch.py
slam_toolbox:
  ros__parameters:
    max_laser_range: 4.0
```

```bash
ros2 launch cpsl_nav console_slam.launch.py namespace:=cpsl_ugv_1 overrides_file:=$HOME/maps/robot_overrides.yaml
```

For Nav2, a costmap section is one level deeper:

```yaml
controller_server:
  ros__parameters:
    FollowPath: {vx_max: 0.2}
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer: {inflation_radius: 0.25}
```

## 3. What you cannot set in a file

Frames, the scan, map, odometry and command topics, `mode`, and each root key's namespace come from
the launch arguments, not from the file; the launch overwrites them. Change them with the
arguments (`base_frame`, `scan_topic`, ...). The full list is in [DEVELOPING](../DEVELOPING.md#paramspy-rules).

## 4. See the result before you launch

`python3 -m cpsl_nav.params` prints exactly what a launch would hand its nodes. It needs only
Python and PyYAML, plus the sourced workspace so the module is found. From the repo root:

```bash
python3 -m cpsl_nav.params src/cpsl_nav/config/slam_create3.yaml --namespace cpsl_ugv_1 \
  --overrides ~/maps/robot_overrides.yaml | grep max_laser_range         # max_laser_range: 4.0
python3 -m cpsl_nav.params src/cpsl_nav/config/nav2_create3.yaml --mode nav2 --namespace cpsl_ugv_1
python3 -m cpsl_nav.params src/cpsl_nav/config/slam_create3.yaml --mode localization \
  --map-yaml ~/maps/room.yaml
```

Options: `--overrides`, `--namespace`, `--mode {mapping,localization,nav2}`, `--scan-topic`,
`--base-frame`, `--odom-frame`, `--map-yaml`. A bad override ends with `error: override '...' is
not a parameter of the platform file`.

To support a new kind of robot, see [DEVELOPING.md](../DEVELOPING.md#adding-a-platform-kind).
