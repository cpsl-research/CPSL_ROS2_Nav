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
    ```
    cd CPSL_ROS2_nav
    rosdep install -q -y -r --from-paths src --ignore-src --rosdistro jazzy
    ```
3. Finally, build and install the codebase for future use
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```

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