# 1. Install and build

Install `cpsl_nav` and its dependencies on a fresh Ubuntu 24.04 host, without any other CPSL
repository.

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

For driving the robot by keyboard in tutorials 2-3, also `sudo apt install ros-jazzy-teleop-twist-keyboard`.

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

## 4. Is my robot ready?

`platform:=create3` (the iRobot Create 3) is the **only tested profile**; `rover` exists but has
not been run. Any robot works if it meets the contract in the
[README](../../README.md#what-your-robot-must-provide). With the robot running, on the same
`ROS_DOMAIN_ID` as this host (`<ns>` is its namespace, `cpsl_ugv_1` in these tutorials), five
checks; each should answer as shown:

```bash
TF="--ros-args -r /tf:=/<ns>/tf -r /tf_static:=/<ns>/tf_static"
ros2 run tf2_ros tf2_echo odom base_link $TF     # 1. repeated "At time ..." / "Translation: [...]"
ros2 run tf2_ros tf2_echo base_link <lidar_frame> $TF   # 2. the same, from the latched tf_static
ros2 topic hz /<ns>/scan                         # 3. "average rate:" lines, steady
ros2 topic info /<ns>/odom                       # 4. "Type: nav_msgs/msg/Odometry"
ros2 topic info /<ns>/cmd_vel                    # 5. "Type: geometry_msgs/msg/Twist" (not TwistStamped)
```

Stop 1-3 with Ctrl-C. `<lidar_frame>` is the scan's `frame_id`
(`ros2 topic echo /<ns>/scan --once --field header`). A check that prints "Invalid frame" or
nothing has failed:

- **Check 1 fails, but `ros2 topic echo /tf --once` shows transforms** (global `/tf`): for
  mapping, add `tf_topic:=/tf tf_static_topic:=/tf_static` to the `console_slam` command. `console_nav` always
  reads `/<ns>/tf`, so with a global `/tf` run it with `namespace:=` empty (then the scan, odom
  and `cmd_vel` must be at the root too).
- **Check 2 fails but check 1 passes** (sensor mounts on global `/tf_static`): add
  `tf_static_relay:=true` to `console_nav`, which copies `/tf_static` onto `/<ns>/tf_static`.
- **No namespace at all** is fine: leave `namespace` empty and drop `/<ns>` above.
- **A `PointCloud2` instead of a scan**: `cloud_topic:=` ([tutorial 2](02_mapping.md)).
- **Another chassis**: copy `config/slam_create3.yaml` and `config/nav2_create3.yaml` and change
  the robot-specific keys (frames as published, `robot_radius` or footprint, `vx_max` and the
  velocity limits, `odom_topic`, the scan topic). Steps and the full list:
  [DEVELOPING](../DEVELOPING.md#adding-a-platform-kind) and [tutorial 6](06_tuning_params.md).

Next: [make a map](02_mapping.md).
