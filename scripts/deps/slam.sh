#!/usr/bin/env bash
# SLAM: the system dependencies for mapping with slam_toolbox.
#
# Used by console (which runs it inside its own container before starting a
# mapping job) and by anyone setting up a host; on a host run it with
# `sudo bash scripts/deps/slam.sh`. Contract: see _common.sh.
#
#   slam_toolbox            - online async mapping
#   pointcloud_to_laserscan - turns a lidar point cloud into the scan
#                             slam_toolbox consumes
#   nav2_map_server, nav2_amcl, nav2_lifecycle_manager
#                           - localization on a saved map
#                             (console_slam.launch.py mode:=localization)
#
# The Nav2 packages (nav2_bringup etc.) are not here; they are installed by
# rosdep as the README describes.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

D="${ROS_DISTRO:-jazzy}"
apt_ensure "ros-$D-slam-toolbox" "ros-$D-pointcloud-to-laserscan" \
    "ros-$D-nav2-map-server" "ros-$D-nav2-amcl" "ros-$D-nav2-lifecycle-manager"
