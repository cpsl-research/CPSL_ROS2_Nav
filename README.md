# CPSL_ROS2_Nav
A set of nodes for performing navigation and localization on CPSL Agents in ROS2

## Installation
Follow the below instructions to install the ROS2 Nav2 stack

```
git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_Nav
```

If you forgot to clone the submodules as well, you can use the following command:

```
git submodule update --init --recursive
```

Once, cloned, the following commands can be used to build/install the necessary packages

```
cd CPSL_ROS2_nav
rosdep install -q -y -r --from-paths src --ignore-src
```

```
colcon build --symlink-install
source install/setup.bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

Alternative for installing rosdep per Nav2
```
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro jazzy
```

## Tutorials

### 1. SLAM Pipeline:

Pre-requisites:
1. TBD


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