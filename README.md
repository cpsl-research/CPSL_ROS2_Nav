# CPSL_ROS2_Nav

One ROS 2 package, `cpsl_nav`: launch files and parameter sets that give a namespaced ground robot
mapping ([slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)), localization (Nav2 AMCL),
navigation (Nav2) with no-go zones, and two explorers (frontier, wander). Everything here works
without any other CPSL repository, on any robot that provides the topics listed below.

- Host: **Ubuntu 24.04**, **ROS 2 Jazzy** ([official install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html));
  Gazebo Harmonic if you simulate.
- Platform parameter files: `create3` (the reference) and `rover` (its header says it has not been run on a Rover).
  The UAV files (`nav2_uav*.yaml`) are old lab starting points and are not covered here.

## What your robot must provide

Inside one non-empty namespace `<ns>` (these docs use `cpsl_ugv_1`), following
[TF_AND_TOPICS.md](https://github.com/cpsl-research/console/blob/main/docs/TF_AND_TOPICS.md):

| Topic | Type | Content |
|---|---|---|
| `/<ns>/tf` | `tf2_msgs/TFMessage` | `odom -> base_link` (one publisher) |
| `/<ns>/tf_static` | `tf2_msgs/TFMessage` | `base_link -> <laser frame>`, latched |
| `/<ns>/scan` | `sensor_msgs/LaserScan` | the lidar (or a `PointCloud2`, see tutorial 02) |
| `/<ns>/odom` | `nav_msgs/Odometry` | for Nav2 |
| `/<ns>/cmd_vel` | `geometry_msgs/Twist` | accepted by the robot (not `TwistStamped`) |

Frames are never prefixed: `map`, `odom`, `base_link`. The launch files add `map -> odom`.

## Quick start

```bash
git clone https://github.com/cpsl-research/CPSL_ROS2_Nav
cd CPSL_ROS2_Nav
sudo bash scripts/deps/slam.sh && sudo bash scripts/deps/nav2.sh   # apt dependencies
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro jazzy
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch cpsl_nav console_slam.launch.py namespace:=cpsl_ugv_1 platform:=create3   # map
```

## Tutorials (no console needed)

1. [Install and build](docs/tutorials/01_install_and_build.md)
2. [Make a map](docs/tutorials/02_mapping.md): `console_slam.launch.py mode:=mapping`, saving, continuing a map
3. [Localize on a saved map](docs/tutorials/03_localization.md): `mode:=localization`, AMCL, initial pose
4. [Navigate with Nav2 and RViz](docs/tutorials/04_nav2_and_rviz.md): `console_nav.launch.py`, goals; the older launch files
5. [Keepout zones and exploration](docs/tutorials/05_explore_and_keepout.md): `console_explore.launch.py`
6. [Tuning parameters](docs/tutorials/06_tuning_params.md): platform files, overrides, `python3 -m cpsl_nav.params`

Developers: [docs/DEVELOPING.md](docs/DEVELOPING.md) (layout, parameter rules, adding a platform, tests, maps).

## Launch files (`src/cpsl_nav/launch/`)

| File | Runs |
|---|---|
| `console_slam.launch.py` | `mode:=mapping` slam_toolbox, or `mode:=localization` map_server + AMCL |
| `console_nav.launch.py` | Nav2 (navigation only) plus the keepout filter |
| `console_explore.launch.py` | `mode:=frontier` or `mode:=wander` |
| `slam_sync`, `localization`, `nav2`, `rviz_nav` | the older launch files; they prefix frames with the namespace (tutorial 04) |

Each lists its arguments with `ros2 launch cpsl_nav <file> --show-args`. Maps live in
`src/cpsl_nav/maps/` (lab rooms; make your own with tutorial 02).

Optional: the lab's console app runs the `console_*` launch files; nothing here needs it.
