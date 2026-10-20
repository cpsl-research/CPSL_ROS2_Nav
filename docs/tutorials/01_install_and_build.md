# 1. Install and build

Install `cpsl_nav` and its dependencies on a fresh Ubuntu 24.04 host, without any other CPSL
repository. Tested target: Ubuntu 24.04, ROS 2 Jazzy.

## 1. Install ROS 2 Jazzy

Follow the official
[ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Install `ros-jazzy-desktop` (it includes RViz, used in tutorials 3-4) and `ros-dev-tools` (colcon and
rosdep). Then confirm:

```bash
source /opt/ros/jazzy/setup.bash && ros2 --help | head -2
```

## 2. Get the code and the system dependencies

```bash
git clone https://github.com/cpsl-research/CPSL_ROS2_Nav
cd CPSL_ROS2_Nav
sudo bash scripts/deps/slam.sh
sudo bash scripts/deps/nav2.sh
```

- `slam.sh` installs `slam_toolbox`, `pointcloud_to_laserscan`, `nav2_map_server`, `nav2_amcl` and
  `nav2_lifecycle_manager`: everything mapping and localization need.
- `nav2.sh` installs `nav2_bringup` (which pulls in Nav2), the MPPI controller, the collision
  monitor, `opennav_docking` and `diagnostic_updater`.

Both need root, change nothing in your workspace or shell, and are quick when everything is already
installed. They read `ROS_DISTRO` and default to `jazzy`.

## 3. Resolve the rest and build

```bash
sudo rosdep init                         # once per machine; ignore "already exists"
rosdep update
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro jazzy   # from the repo root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`rosdep` reads `src/cpsl_nav/package.xml` and installs whatever the two scripts did not. You should
see `Summary: 1 package finished`. Every new terminal needs both `source` lines.

Check the install:

```bash
ros2 pkg executables cpsl_nav
ros2 launch cpsl_nav console_slam.launch.py --show-args | head -5
```

The first lists `frontier_explore`, `tf_static_relay` and `wander`; the second prints the launch
file's arguments (`mode`, `namespace`, `platform`, ...). Starting no node is needed for either.

## 4. Check that your robot provides what the stack needs

All the launch files in these tutorials expect a robot inside one namespace (here `cpsl_ugv_1`; use
yours) publishing the topics in the [README](../../README.md#what-your-robot-must-provide). With the
robot running and on the same `ROS_DOMAIN_ID` as this host:

```bash
ros2 topic list | grep cpsl_ugv_1
ros2 topic echo /cpsl_ugv_1/tf --once            # a transform with frame_id odom, child base_link
ros2 topic hz /cpsl_ugv_1/scan                   # a steady rate
ros2 topic echo /cpsl_ugv_1/tf_static --once     # base_link -> your lidar's frame
```

If your lidar publishes a `PointCloud2` instead of a `LaserScan`, tutorial 2 shows `cloud_topic:=`.
If the transform or the scan is missing, mapping cannot start; fix the robot side first. Robots
that follow this layout: [CPSL_ROS2_Create3](https://github.com/cpsl-research/CPSL_ROS2_Create3),
[CPSL_ROS2_Rover](https://github.com/cpsl-research/CPSL_ROS2_Rover).

Next: [make a map](02_mapping.md).
