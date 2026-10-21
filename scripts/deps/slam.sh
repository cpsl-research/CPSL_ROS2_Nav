#!/usr/bin/env bash
# SLAM: the system dependencies for mapping with slam_toolbox.
#
# Used by console (which runs it inside its own container before starting a
# mapping job) and by anyone setting up a host; on a host run it with
# `sudo bash scripts/deps/slam.sh`. Contract: see _common.sh.
#
# Usage:
#   sudo bash slam.sh           install whatever is missing
#   bash slam.sh --check        no root, installs nothing: exit 0 if every package
#                                below is installed, else exit 1 and list the missing
#                                ones on stderr. Works on plain ROS 2, without console.
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
D="${ROS_DISTRO:-jazzy}"

PACKAGES=(
    "ros-$D-slam-toolbox"
    "ros-$D-pointcloud-to-laserscan"
    "ros-$D-nav2-map-server"
    "ros-$D-nav2-amcl"
    "ros-$D-nav2-lifecycle-manager"
)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure "${PACKAGES[@]}"
