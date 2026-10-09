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
#
# The Nav2 packages (nav2_bringup etc.) are not here; they are installed by
# rosdep as the README describes.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure "ros-${ROS_DISTRO:-jazzy}-slam-toolbox" "ros-${ROS_DISTRO:-jazzy}-pointcloud-to-laserscan"
