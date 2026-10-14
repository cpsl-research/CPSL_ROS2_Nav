# CPSL_ROS2_Nav
A set of nodes for performing navigation and localization on CPSL Agents in ROS2

## Software Notes

The following code was used to implement a SLAM and navigation stack for the CPSL's UAVs (coming soon) and UGV platforms. The specific software versions used are as follows:

- Gazebo Version (if simulating): GZ Harmonic
- ROS Version: ROS2 Jazzy
- Ubuntu Version: Ubuntu 24.04

## Installation
Follow the below instructions to install the ROS2 Nav2 stack
### Pre-requisites
Prior to installing the CPSL's ROS2 Nav2 stack, perfororm the following steps to make sure that everything is installed correctly.

1. [backup] Just in case, we have found that the ros diagnostic-updater may not be installed using the below instructions. As such, we recommend running the following command to ensure that it is installed. Note, that this isn't required for SLAM, but is required for any nav2 operations:
```
sudo apt install ros-jazzy-diagnostic-updater
```
* **note**: In the above command, replace jazzy with your actual distro if you're using a different ros distro
### Installing CPSL_ROS2_nav
1. First, clone the repository
    - If no submodules are contained in this repo (the current configuration), use the following command to clone the repo
    ```
    git clone https://github.com/cpsl-research/CPSL_ROS2_Nav
    ```
    - Otherwise, use the following command to recursively clone the repository with its submodules
    ```
    git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_Nav
    ```

    - If you forgot to clone the submodules as well, you can use the following command:

    ```
    cd CPSL_ROS2_Nav
    git submodule update --init --recursive
    ```

2. Once the repository has been cloned, install all required dependencies for the navigation and slam packages
    - `scripts/deps/slam.sh` installs the system packages mapping and localization need (`slam_toolbox`, `pointcloud_to_laserscan`, `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager`); run it as root: `sudo bash scripts/deps/slam.sh`
    - `scripts/deps/nav2.sh` installs the system packages Nav2 needs (`nav2_bringup`, MPPI controller, collision monitor, docking, `diagnostic_updater`); run it as root: `sudo bash scripts/deps/nav2.sh`
    - The rosdep command below installs everything else (including the Nav2 packages)
    ```
    cd CPSL_ROS2_nav
    rosdep install -q -y -r --from-paths src --ignore-src --rosdistro jazzy
    ```
3. Finally, build and install the codebase for future use
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```

## Console: mapping and localization

[console](https://github.com/cpsl-research/console) runs mapping and localization from this repo with one launch file, `launch/console_slam.launch.py`. It can be used without console too.

```
ros2 launch cpsl_nav console_slam.launch.py namespace:=robot1 scan_topic:=/livox/scan
ros2 launch cpsl_nav console_slam.launch.py mode:=localization namespace:=robot1 \
    scan_topic:=/livox/scan map_yaml:=/full/path/room.yaml
```

- `mode:=mapping` runs slam_toolbox (online async). The launch file configures and activates it. The map is published on `/<namespace>/map`, and `map -> odom` on the tf topic.
- `mode:=localization` runs map_server, serving `map_yaml` on `/<namespace>/map`, and AMCL, both started by a nav2 lifecycle manager. AMCL publishes nothing until it gets an initial pose on `/<namespace>/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, frame `map`). After that it publishes `map -> odom` on the tf topic and its estimate on `/<namespace>/amcl_pose`.
- **tf:** every node reads and writes tf on `tf_topic`. The default is `/<namespace>/tf` (or `/tf` with no namespace). Static tf is read from `tf_static_topic` (default `/tf_static`). Frame names are used as given, with no prefix added. The Create 3 publishes `odom -> base_link` unprefixed on `/<namespace>/tf`, so the defaults fit it.
- `cloud_topic:=<PointCloud2>` adds `pointcloud_to_laserscan`, which turns the cloud into a scan on `scan_topic` (frame `base_frame`).
- `ros2 launch cpsl_nav console_slam.launch.py --show-args` lists all arguments.

### TF and topics

Console runs everything on the robot's own `/<namespace>/tf` and `/<namespace>/tf_static` with unprefixed frame names. `console_slam.launch.py` defaults `tf_topic` / `tf_static_topic` to the relative `tf` / `tf_static`, which resolve in the namespace. The protocol is in console's [docs/TF_AND_TOPICS.md](https://github.com/cpsl-research/console/blob/feature/gui-redesign/docs/TF_AND_TOPICS.md).

### Parameters for console

- **One full file per console platform kind:** `config/slam_<kind>.yaml`, where `<kind>` is console's platform kind (`create3`, `rover`). There is no shared base file. Each file holds the `slam_toolbox` section (mapping) and the `map_server` and `amcl` sections (localization). The comments in each file say which values differ from slam_toolbox's stock `mapper_params_online_async.yaml` and why. `slam_rover.yaml` has not been tested on a Rover yet.
- `params_file:=` takes the full path of a platform file. If it is empty, the launch uses the installed `config/slam_<platform>.yaml`. console passes the source-tree path, so an edit takes effect without a rebuild. A platform with no file fails at launch, naming the missing file.
- `overrides_file:=` (optional) is merged over the platform file. It has the same shape as the platform file (`slam_toolbox: ros__parameters: {max_laser_range: 4.0}`) and may only change keys the platform file already has. A misspelt key fails the launch. If the file is empty or missing, no overrides are applied. console keeps one overrides file per robot.
- `cpsl_nav/params.py` does the merge. It then sets everything that comes from launch arguments rather than tuning: frames, `scan_topic`, slam_toolbox's `map_name` (stock is the absolute `/map`), map_server's `yaml_filename`, and `use_sim_time`. It also rewrites each root key to `/<namespace>/<node>`. The result is written to a temporary file that the launch deletes on exit. To print what a launch would use: `python3 -m cpsl_nav.params config/slam_create3.yaml --namespace robot1 [--overrides f.yaml]`.
- **Nav2 has its own platform files:** `config/nav2_<kind>.yaml` (`nav2_create3.yaml` from the lab's `nav2_ugv.yaml`, `nav2_rover.yaml` untested on a Rover). Overrides work the same way, e.g. `controller_server: ros__parameters: {FollowPath: {vx_max: 0.2}}` or `local_costmap: local_costmap: ros__parameters: {inflation_layer: {inflation_radius: 0.25}}`. Nav2's root keys are not rewritten (stock `bringup_launch.py` nests the file under the namespace itself). `params.nav2_rewrite` sets every frame key by its last segment (`map`, `odom`, `base_link`/`base_footprint` -> `map_frame`, `odom_frame`, `base_frame`), every scan source to `scan_topic`, every static layer to `map_topic`, and the keepout filters to `/<namespace>/costmap_filter_info`, all as absolute topics. `odom_topic`/`cmd_vel_topic` replace the file's only if given. `python3 -m cpsl_nav.params config/nav2_create3.yaml --mode nav2 --namespace robot1` prints the result.
- **Adding a platform:** add `config/slam_<kind>.yaml` and `config/nav2_<kind>.yaml`. Copy the closest existing file, then change what differs. Keep `filters: ["keepout_filter"]` in both costmaps. The `config/*` glob in `setup.py` installs it.

### Navigation (Nav2) for console

```
ros2 launch cpsl_nav console_nav.launch.py namespace:=robot1 platform:=create3 \
    scan_topic:=/robot1/scan [overrides_file:=f.yaml] [keepout_mask:=/full/path/room.keepout.yaml]
```

- Runs stock `nav2_bringup` `bringup_launch.py` in the namespace, navigation only (no slam, no localization: `map -> odom` comes from `console_slam.launch.py`, either mode). Its nodes read tf on `/<namespace>/tf` and `/<namespace>/tf_static`. `tf_static_relay` (default off) can copy `/tf_static` onto `/<namespace>/tf_static` when nothing publishes it there.
- **Keepout (no-go zones):** `costmap_filter_info_server`, `filter_mask_server` and `lifecycle_manager_costmap_filters` run in the namespace. The mask is a map_server map in the map frame. Black (occupied) cells are keepout and everything else is ignored. Without `keepout_mask` the launch serves a one-cell free mask, which means no zones. To replace the mask without restarting Nav2: `ros2 service call /<ns>/filter_mask_server/load_map nav2_msgs/srv/LoadMap "{map_url: /full/path/x.keepout.yaml}"`. Nav2 applies costmap filters after the inflation layer, so a zone is not inflated. Rasterize each zone grown by at least the robot's radius plus a margin, or the planner will route the robot's centre along the zone's edge.
- `ros2 launch cpsl_nav console_nav.launch.py --show-args` lists all arguments.

### Exploration: frontiers or wander

```
ros2 launch cpsl_nav console_explore.launch.py mode:=frontier namespace:=robot1
ros2 launch cpsl_nav console_explore.launch.py mode:=wander namespace:=robot1
```

Both need `console_nav.launch.py` and a map (`console_slam.launch.py`, either mode) in the same namespace. Each sends its own `NavigateToPose` goals, one at a time, and avoids keepout zones (it reads the mask on `/<ns>/keepout_filter_mask`).

- `mode:=frontier` (`/<ns>/frontier_explore`) drives to the nearest large frontier, where known free space meets unknown space. It cancels a goal once its frontier has been mapped, blacklists goals that fail, and reports `done` / `no frontiers` when nothing is left.
- `mode:=wander` (`/<ns>/wander`) drives to random free goals at least 1 m away. Each goal must keep clear of obstacles and unknown space, and is checked for a path with Nav2's `compute_path_to_pose` before it is sent. Wander needs enough of the map to be known: it reports `no free goal far enough away` until there is.
- Status: JSON on `/<ns>/<node>/status` (`std_msgs/String`, reliable and transient local), every second: `mode`, `state` (`waiting`, `searching`, `navigating`, `done`, `stopped`), `detail`, `goal`, `goals_sent`, `goals_succeeded`, `goals_failed`, `known_m2`, plus `frontiers`/`blacklisted` or `rejected`.
- Stop with SIGINT. The node cancels its active goal, waits up to 1.5 s for Nav2 to confirm, publishes `stopped` and exits 0.
- The detection, clustering and sampling code is `cpsl_nav/grid.py` (numpy, no ROS) and is tested in `test/test_grid.py`.

### Map storage convention

A map is `<name>.yaml` + `<name>.pgm` (the map_server format), optionally with `<name>.keepout.yaml` + `<name>.keepout.pgm` (a keepout mask) and `<name>.zones.json`. console saves to its own `data/maps/` by default. It can also save to this repo's `src/cpsl_nav/maps/`, but only into the working tree: whether to commit a map is the user's decision.

## Tutorials

### 1. Generate a map using the slam_toolbox [single agent]:
Complete the following steps to generate a map using the slam_toolbox

#### Pre-Requesites:
In order for the slam_toolbox to function correctly, the following topics must be published. Note that the topic name and namespace of the published topics can be dynamically changed when running the launch commands

| **Topic** | **Type** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `/scan`   | `sensor_msgs/LaserScan`  | The input scan from your laser to utilize |  
| `/tf`    | N/A                      | A valid transform from your configured `odom_frame` to `base_frame` |  

#### Note: Gazebo simulation
If you are using a gazebo simulation, I recommend starting the slam_toolbox before starting the gazebo simulation. Performance was much more consistent when starting the toolbox first followed by the gazebo simulation compared to the other way around. 

#### Note: tf frame naming scheme
For multi-robot operations, it is common to give each agent its own namespace (e.g. `/agent_1/scan`). If you are using this approach, I've also designed the following stacks to assume that the tf frames also have the same tf prefix applied (e.g. the lidar frame would be `lidar_frame/scan`). 

#### Steps to follow:
Once the pre-requisites are fullfilled, you should now be able to run the slam_toolbox as follows:
1. Source the CPSL_ROS2_Nav package
    ```
    cd CPSL_ROS2_Nav
    source install/setup.bash
    ```

2. Run the slam_toolbox launch file
    ```
    ros2 launch cpsl_nav slam_sync.launch.py
    ```
    When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
    | **Parameter** | **Default** | **Description** |
    |----------------|--------------|------------------------------------------------------|
    |`use_sim_time`|false|Use the time from a Gazebo simulation|
    |`namespace`|''|The robot's namespace (will also be applied as a prefix to base_frame_id)|
    |`scan_topic`|'/scan'|The LaserScan topic to use for slam (`/radar_combined/scan` for radar, `/livox/scan/` for lidar)|
    |`base_frame_id`|'base_link'|The frame ID of the base_link frame (without tf pre-fix)|
    |`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
    |`use_lifecycle_manager`| false| Enable bond connection during node activation| 
    |`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
    |`rviz`|false|Display an RViz window with navigation|

3. Drive the vehicle around to generate the map
4. To save the serialized map to a file (for continuation of mapping), use the following code (replace NAMESPACE with the namespace of the robot and MAP_NAME with the desired file name you wish to use to save the map): 
    ```
    ros2 service call /NAMESPACE/slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph "{filename: 'MAP_NAME'}"
    ```
5. To save a .pgm and .yaml file of the map, use the following commands instead (replace NAMESPACE with the namespace of the robot and MAP_NAME with the desired file name you wish to use to save the map):
    ```
    ros2 service call /NAMESPACE/slam_toolbox/save_map slam_toolbox/SaveMap "{name: 'file_name'}"
    ```

### 2. Generate a map using a previously generated map [multi agent]:
If you have a previous map of a space and would like to extend or continue to update the pose graph, use the following steps 

#### Pre-Requesites:
1. In order for the slam_toolbox to function correctly, the following topics must be published. Note that the topic name and namespace of the published topics can be dynamically changed when running the launch commands

| **Topic** | **Type** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `/scan`   | `sensor_msgs/LaserScan`  | The input scan from your laser to utilize |  
| `/tf`    | N/A                      | A valid transform from your configured `odom_frame` to `base_frame` |  

2. The previous map must also be saved as a .posegraph and .data file (see the above tutorial for guidance on how to do this). 

#### Note: Gazebo simulation
If you are using a gazebo simulation, I recommend starting the slam_toolbox before starting the gazebo simulation. Performance was much more consistent when starting the toolbox first followed by the gazebo simulation compared to the other way around. 

#### Note: tf frame naming scheme
For multi-robot operations, it is common to give each agent its own namespace (e.g. `/agent_1/scan`). If you are using this approach, I've also designed the following stacks to assume that the tf frames also have the same tf prefix applied (e.g. the lidar frame would be `lidar_frame/scan`). 

#### Steps to follow:
Once the pre-requisites are fullfilled, you should now be able to run the slam_toolbox as follows:
1. In the `cpsl_nav/config` directory, open the slam_from_previous.yaml configuration file. In there, modify the `map_file_name` parameter to be the absolute path to the .posegraph and .data files without the extension (e.g.; "/home/cpsl/Documents/uav_map"). Additionally, update the `map_start_pose` to the approximate location of the new agent in the original map's frame. 

2. Source the CPSL_ROS2_Nav package
    ```
    cd CPSL_ROS2_Nav
    source install/setup.bash
    ```

3. Run the slam_toolbox launch file (update other parameters as needed)
    ```
    ros2 launch cpsl_nav slam_sync.launch.py slam_params_file:=slam_from_previous.yaml
    ```
    When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
    | **Parameter** | **Default** | **Description** |
    |----------------|--------------|------------------------------------------------------|
    |`use_sim_time`|false|Use the time from a Gazebo simulation|
    |`namespace`|''|The robot's namespace (will also be applied as a prefix to base_frame_id)|
    |`scan_topic`|'/scan'|The LaserScan topic to use for slam (`/radar_combined/scan` for radar, `/livox/scan/` for lidar)|
    |`base_frame_id`|'base_link'|The frame ID of the base_link frame (without tf pre-fix)|
    |`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
    |`use_lifecycle_manager`| false| Enable bond connection during node activation| 
    |`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
    |`rviz`|false|Display an RViz window with navigation|

3. Drive the vehicle around to generate the map
4. To save an updated serialized map to a file (for continuation of mapping), use the following code (replace NAMESPACE with the namespace of the robot and MAP_NAME with the desired file name you wish to use to save the map): 
    ```
    ros2 service call /NAMESPACE/slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph "{filename: 'MAP_NAME'}"
    ```
5. To save an updated .pgm and .yaml file of the map, use the following commands instead (replace NAMESPACE with the namespace of the robot and MAP_NAME with the desired file name you wish to use to save the map):
    ```
    ros2 service call /NAMESPACE/slam_toolbox/save_map slam_toolbox/SaveMap "{name: 'file_name'}"
    ```
### 3. Run localization in a mapped environment [single agent]:

Complete the following steps to localize a vehicle using the nav2 ROS2 package

```
ros2 launch cpsl_nav localization.launch.py scan_topic:=/scan map:=cpsl.yaml
```

#### Pre-Requesites:
In order for the slam_toolbox to function correctly, the following topics must be published. Note that the topic name and namespace of the published topics can be dynamically changed when running the launch commands.

| **Topic** | **Type** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `/scan`   | `sensor_msgs/LaserScan`  | The input scan from your laser to utilize |  
| `/tf`    | N/A                      | A valid transform from your configured `odom_frame` to `base_frame` | 

## Helpful tips, tricks, and notes:
### 1. slam_toolbox number of threads: 
Note that as installed, the slam_toolbox ceres_solver (found ing slam_toolbox/solvers/ceres_solver.cpp) is set to use 50 threads. If this continues to be an issue, take the following steps:
1. Reduce the number of threads to be more reasonable for the system (ex: 2-3 on RPi)
2. Create a fork of the slam_toolbox just for the CPSL, modify the file, and then re-compile it for future purposes. **Note**: The pre-compiled installation has some optimizations made already, but this should help to reduce the computational load

#### Steps to follow:
Once the pre-requisites are fullfilled, you should now be able to run the ROS2 nav2 localization as follows:

1. Source the CPSL_ROS2_Nav package. Do this for each of the following commands
    ```
    cd CPSL_ROS2_Nav
    source install/setup.bash
    ```

2. [terminal 1] Start the rviz window for localization
    ```
    ros2 launch cpsl_nav rviz_nav.launch.py namespace:=/cpslCreate3 config:=localization_config.rviz
    ```
    When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
    | **Parameter** | **Default** | **Description** |
    |----------------|--------------|------------------------------------------------------|
    |`use_sim_time`|false|Use the time from a Gazebo simulation|
    |`namespace`|''|The robot's namespace|
    |`config`|'localization_config.rviz'|rviz config file in the rviz_cfgs directory|

3. [terminal 2] Next, run the localization launch file
    ```
    ros2 launch cpsl_nav localization.launch.py namespace:=/cpslCreate3 scan_topic:=/livox/scan map:=cpsl.yaml
    ```
    When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
    | **Parameter** | **Default** | **Description** |
    |----------------|--------------|------------------------------------------------------|
    |`use_sim_time`|false|Use the time from a Gazebo simulation|
    |`namespace`|''|The robot's namespace|
    |`scan_topic`|'/scan'|The LaserScan topic to use for slam|
    |`params_file`| 'localization.yaml'|Path to the localization configuration file|
    |`map`|'cpsl.yaml'|yaml file in the cpsl_nav/maps folder with map information|

4. Finally, return to the RVIZ window. At the top, click on the "2D Pose Estimate", and use it to select the location and direction in the map where the robot is currently placed. Localization should now be up and running. Before running the navigation stack, I recommend driving the vehicle around a bit so that the particle filter can better estimate the vehicle's location.

### 3. Run the navigation stack

If you have a map, a method of localizing your agent in the map, and want to autonomously navigate the vehicle in the environment, complete the following steps.

#### Pre-Requesites:
1. In order for the nav2 stack to function correctly, the following topics must be published. Note that the topic name and namespace of the published topics can be dynamically changed when configuring the stack

| **Topic** | **Type** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `/scan`  | `sensor_msgs/LaserScan`  | The input scan from your laser to utilize |  
| `/tf`    | N/A                      | A valid transform from your configured `odom_frame` to `base_frame` (usually published by the agend) and `map` to `odom` (published by slam_toolbox or nav2 localization) | 

2. Additionally, the nav2 stack will publish a `cmd_vel_nav` message indicating the desired velocity from the nav2 stack. Your platform must be able to accept these commands and move accordingly. Note the namespace can be applied dynamically as documented later on.
| **Topic** | **Type** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `/cmd_vel_nav`  | `geometry_msgs/Twist` or `geometry_msgs/TwistStamped`  | output command from the nav2 stack for the agent to follow|

3. Finally, complete the following steps to create a custom .yaml file configuration for your agent:
    - Create a new .yaml file config for your agent, by first copying either the `nav2_ugv.yaml` (for differential drive agents) or `nav2_uav.yaml` (for omni-directional control).
    - In the `bt_navigator`, update the `global_frame`, `robot_base_frame`, and `odom_topic` to match your platform. Note that when specifying a topic `/topic_name` implies absolute topic name while `topic_name` implies relative topic name (within the namespace). we recommend using the second option as it allows for custom namespaces. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `controller_server`, update the `motion_model` to your vehicle. We recommend visiting the ROS2 nav2 wiki page to learn how to customize this if needed. Note that the current configuraitons use the MPPIController plugin though. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `local_costmap`, update the `global_frame`, `robot_base_frame`, and `scan/topic` parameters to correspond to your platform. Note that when specifying a topic `/topic_name` implies absolute topic name while `topic_name` implies relative topic name (within the namespace). we recommend using the second option as it allows for custom namespaces. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `global_costmap`, update the `global_frame`, `robot_base_frame`, and `scan/topic` parameters to correspond to your platform. Note that when specifying a topic `/topic_name` implies absolute topic name while `topic_name` implies relative topic name (within the namespace). we recommend using the second option as it allows for custom namespaces. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `planner_server`, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `smoother_server`, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `behavior_server`, adjust the `local_frame` and `robot_base_frame` to be the corresponding frames in your tf tree. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `waypoint_follower`, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - In the `velocity_smoother`, update the `odom_topic`. Note that when specifying a topic `/topic_name` implies absolute topic name while `topic_name` implies relative topic name (within the namespace). we recommend using the second option as it allows for custom namespaces. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
    - Finally, in the `collision_monitor`, update the `base_frame_id`, `odom_frame_id`, and `scan/topic` parameters to match your agent's configuration. Note that when specifying a topic `/topic_name` implies absolute topic name while `topic_name` implies relative topic name (within the namespace). we recommend using the second option as it allows for custom namespaces. Finally, adjust the `enable_stamped_cmd_vel` to match the type of twist (Twist vs TwistStamped) message your platform expects
4. Finally, for a new platform, it may be helpful to have ChatGPT refine the .yaml configuration file for additional tuning as needed.

#### Steps to follow:
Once the pre-requisites are fullfilled, you should now be able to run the navigation stack as follows (note localization or SLAM must already be running for navigation to work):

1. Build the navigation stack
```
cd CPSL_ROS2_Nav
colcon build --symlink-install
source install/setup.bash
```

2. Then launch the navigation stack as follows:
```
ros2 launch cpsl_nav nav2.launch.py
```

When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
| **Parameter** | **Default** | **Description** |
|----------------|--------------|------------------------------------------------------|
|`use_sim_time`|false|Use the time from a Gazebo simulation|
|`namespace`|''|The robot's namespace|
|`params_file`| "nav2_ugv.yaml" | the .yaml config to use in the config folder
|`autostart`|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
|`use_lifecycle_manager`| false| Enable bond connection during node activation| 
|`slam_params_file`| 'slam.yaml'|Path to the SLAM Toolbox configuration file|

3. Next, start rviz using the following command. A pre-setup configuration for viewing/controlling navigation can be found in the `cpsl_nav/rviz_cfgs` directory (try the nav_config_uav.rviz or nav_config.rviz) files. Note that you may have to change some of the subscribed topics for things to work correctly.
```
rviz2
```
4. Finally, you can navigate the vehicle accordingly by using the "Goal pose" button in RVIZ
### 2. View the TF Tree

While settup up the navigation stack, it may be helpful to view the full tf tree. To do so, perform the following steps:

1. Install tf tools (replace jazzy with your distribution if you're using another distribution):
```
sudo apt-get install ros-jazzy-rviz2  ros-jazzy-tf2-ros ros-jazzy-tf2-tools
```
2. Run the following command to generate a pdf of the tf tree
```
ros2 run tf2_tools view_frames
```
3. Alternatively, you can open rviz using the following commands, and then add a tf view
```
rviz2
```
## Helpful Documentation:
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox): Github page for SLAM toolbox 