# CPSL_ROS2_Nav
A set of nodes for performing navigation and localization on CPSL Agents in ROS2

## Software Notes

The following code was used to implement a SLAM and navigation stack for the CPSL's UAVs (coming soon) and UGV platforms. The specific software versions used are as follows:

- Gazebo Version (if simulating): GZ Harmonic
- ROS Version: ROS2 Jazzy
- Ubuntu Version: Ubuntu 24.04

## Installation
Follow the below instructions to install the ROS2 Nav2 stack

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
In order for the slam_toolbox to function correctly, the following topics must be published

| **Topic** | **Type** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `/scan`   | `sensor_msgs/LaserScan`  | The input scan from your laser to utilize |  
| `/tf`    | N/A                      | A valid transform from your configured `odom_frame` to `base_frame` |  

#### Note: Gazebo simulation
If you are using a gazebo simulation, I recommend starting the slam_toolbox before starting the gazebo simulation. Performance was much more consistent when starting the toolbox first followed by the gazebo simulation compared to the other way around. 

#### Steps to follow:
Once the pre-requisites are fullfilled, you should now be able to run the slam_toolbox as follows:
1. Source the CPSL_ROS2_Nav package
    ```
    cd CPSL_ROS2_Nav
    source install/setup.bash
    ```

2. Run the slam_toolbox launch file
    ```
    ros2 launch cpsl_nav slam.launch.py
    ```
    When launching, the following parameters can also be set by using the `parameter:=value` notation after the name of the launch file:
    | **Parameter** | **Default** | **Description** |
    |----------------|--------------|------------------------------------------------------|
    |use_sim_time|false|Use the time from a Gazebo simulation|
    |sync|true|use synchronous SLAM (slower than asyncrhonous SLAM)|
    |namespace|''|The robot's namespace|
    |scan_topic|'/scan'|The LaserScan topic to use for slam|
    |autostart|true| Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.|
    |use_lifecycle_manager| false| Enable bond connection during node activation| 
    |slam_params_file| 'slam.yaml'|Path to the SLAM Toolbox configuration file|
    |rviz|false|Display an RViz window with navigation|

3. Drive the vehicle around to generate the map
4. Once finished, use one of the following two methods to save the generated map
    - If rviz is displayed, go into the SlamToolboxPlugin Window, specify the file name (e.g.;"building_1") without the .yaml/.pgm. and slick the "Save Map" button. The file will be saved in the current directory
    - If you aren't using RViz, use the following command to save the map (doesn't currently work):
    ```
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{'name': 'file_name'}"
    ```


## Helpful tips, tricks, and notes:
### 1. slam_toolbox number of threads: 
Note that as installed, the slam_toolbox ceres_solver (found ing slam_toolbox/solvers/ceres_solver.cpp) is set to use 50 threads. If this continues to be an issue, take the following steps:
1. Reduce the number of threads to be more reasonable for the system (ex: 2-3 on RPi)
2. Create a fork of the slam_toolbox just for the CPSL, modify the file, and then re-compile it for future purposes. **Note**: The pre-compiled installation has some optimizations made already, but this should help to reduce the computational load


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