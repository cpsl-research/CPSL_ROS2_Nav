# 4. Navigate with Nav2 and RViz

Drive the robot to a goal on the map. Nav2 does the planning; it needs a map, a localized robot and
the robot's scan. Start from [tutorial 3](03_localization.md) (or a mapping run from
[tutorial 2](02_mapping.md)): **one** of them must be running, because Nav2 reads `map -> odom`
from it and `console_nav.launch.py` starts neither slam nor AMCL.

## 1. Start Nav2

In a new terminal (sourced as usual):

```bash
ros2 launch cpsl_nav console_nav.launch.py namespace:=cpsl_ugv_1 platform:=create3
```

This runs the stock `nav2_bringup` `bringup_launch.py` inside the namespace, with the parameters of
`config/nav2_create3.yaml` rewritten for your frames and topics, plus the keepout filter
([tutorial 5](05_explore_and_keepout.md)). You should see the `[console_nav] namespace=/cpsl_ugv_1
tf=/cpsl_ugv_1/tf ...` line, then the Nav2 lifecycle manager reporting `Managed nodes are active`.
If the costmaps complain about a missing transform, localization is not running or its initial
pose has not been given.

| Argument | Default | Meaning |
|---|---|---|
| `platform` | `create3` | picks `config/nav2_<platform>.yaml` |
| `scan_topic` | `scan` | `LaserScan` for both costmaps and the collision monitor |
| `map_topic` | `map` | the map the static layers read (`/<ns>/map`, from slam or map_server) |
| `odom_topic`, `cmd_vel_topic` | empty | empty keeps the platform file's (`odom`, `cmd_vel`) |
| `base_frame`, `odom_frame`, `map_frame` | `base_link`, `odom`, `map` | frames as published |
| `keepout_mask` | empty | a no-go-zone mask (tutorial 5) |
| `tf_static_relay` | `false` | `true` copies global `/tf_static` onto `/<ns>/tf_static` if your robot publishes its sensor mounts only there |
| `params_file`, `overrides_file` | empty | tutorial 6 |

Velocity commands leave Nav2 as plain `Twist` on `/cpsl_ugv_1/cmd_vel` (from the collision monitor).
The robot must accept them there.

## 2. Send a goal

With RViz open as in tutorial 3 (`ros2 launch cpsl_nav rviz_nav.launch.py namespace:=cpsl_ugv_1`,
Fixed Frame `map`), add the costmaps (`/cpsl_ugv_1/global_costmap/costmap`,
`/cpsl_ugv_1/local_costmap/costmap`) and the global path (`/cpsl_ugv_1/plan`, type `Path`). Click
the **2D Goal Pose** tool and drag on the map. `rviz_nav.launch.py` remaps RViz's `/goal_pose`
into the namespace, where Nav2's `bt_navigator` listens. You should see a path appear, and the
robot should start following it.

Or without RViz, a goal one metre ahead of the map origin:

```bash
ros2 action send_goal /cpsl_ugv_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Cancel with Ctrl-C on that command, or stop Nav2 with Ctrl-C in its terminal. Keep a hand on the
robot's own stop the first time.

## 3. The older launch files

`slam_sync.launch.py`, `localization.launch.py` and `nav2.launch.py` are the lab's first launch
files, kept because they work for the robots they were written for. They differ from the
`console_*` ones in a way that decides whether they work for you:

- They **prefix frames with the namespace**: with `namespace:=cpsl_ugv_1`, slam and AMCL look for
  `cpsl_ugv_1/odom` and `cpsl_ugv_1/base_footprint`, and they read the **global** `/tf`.
- `nav2.launch.py` loads `config/nav2_ugv.yaml`, which hard-codes `cpsl_ugv_1/odom`,
  `cpsl_ugv_1/base_footprint` and `/cpsl_ugv_1/radar_combined/scan`.

A robot that follows the [tf protocol](https://github.com/cpsl-research/console/blob/main/docs/TF_AND_TOPICS.md)
(unprefixed frames, `/<ns>/tf`) therefore needs the `console_*` launch files of tutorials 2-4.
Use the older ones for a simulation or robot built with tf prefixes, after editing a copy of the
frame and topic keys in `config/nav2_ugv.yaml` (every `global_frame`, `robot_base_frame`,
`odom_topic`, scan `topic` and `enable_stamped_cmd_vel`). Each lists its arguments with
`--show-args`.

Next: [keepout zones and exploration](05_explore_and_keepout.md).
