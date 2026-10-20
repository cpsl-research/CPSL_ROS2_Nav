# 2. Make a map

Run slam_toolbox on your robot's scan, drive around, and save the map. You need the install from
[tutorial 1](01_install_and_build.md) and a robot publishing the topics listed there, in namespace
`cpsl_ugv_1` (use yours throughout). Remember `source install/setup.bash` in every terminal.

## 1. Start mapping

```bash
ros2 launch cpsl_nav console_slam.launch.py mode:=mapping namespace:=cpsl_ugv_1 platform:=create3
```

You should see these lines (the launch file prints them), ending with slam_toolbox active:

```
[console_slam] mode=mapping namespace=/cpsl_ugv_1 tf=tf tf_static=tf_static scan=/cpsl_ugv_1/scan frames=map->odom->base_link
[console_slam] slam_toolbox configured; activating
[console_slam] mapping: slam_toolbox active
```

Every node runs in the namespace and reads tf from `/cpsl_ugv_1/tf`; `map -> odom` is published
there too. Frame names are used as given, with no prefix. Useful arguments (all in
`--show-args`):

| Argument | Default | Meaning |
|---|---|---|
| `platform` | `create3` | picks `config/slam_<platform>.yaml` (`create3`, `rover`) |
| `scan_topic` | `scan` | `LaserScan`; a relative name resolves in the namespace |
| `cloud_topic` | empty | a `PointCloud2`; if set, `pointcloud_to_laserscan` flattens it onto `scan_topic` (slice z = -0.1 to 0.1 m, range 0.25-5 m, in `base_frame`) |
| `base_frame`, `odom_frame`, `map_frame` | `base_link`, `odom`, `map` | frames as the robot publishes them |
| `params_file`, `overrides_file` | empty | your own parameters ([tutorial 6](06_tuning_params.md)) |
| `use_sim_time` | `false` | `true` when a simulator publishes `/clock` |

## 2. Look at the map

In a second terminal:

```bash
ros2 topic list | grep map                 # /cpsl_ugv_1/map
ros2 launch cpsl_nav rviz_nav.launch.py namespace:=cpsl_ugv_1
```

`rviz_nav.launch.py` starts RViz with `/tf`, `/tf_static`, `/initialpose` and `/goal_pose`
remapped into the namespace. In RViz set **Fixed Frame** to `map`, then **Add > By topic** for
`/cpsl_ugv_1/map` and `/cpsl_ugv_1/scan`.

## 3. Drive and save

Drive slowly and close loops by returning to places you have been. For example:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cpsl_ugv_1/cmd_vel
```

(`sudo apt install ros-jazzy-teleop-twist-keyboard` if it is missing; or use your robot's own controller.)

Save the occupancy map (a `.yaml` and a `.pgm`, what tutorials 3-5 load):

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/room -t /cpsl_ugv_1/map
```

The tool logs that the map was saved, and `~/maps/room.yaml` and `room.pgm` exist. To also keep
the pose graph, so you can continue the map later, ask slam_toolbox (absolute path, no extension):

```bash
ros2 service call /cpsl_ugv_1/slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/room'}"
```

This writes `room.posegraph` and `room.data`. Stop mapping with Ctrl-C in the launch terminal.

Next: [localize on the saved map](03_localization.md).
