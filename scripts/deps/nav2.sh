#!/usr/bin/env bash
# Navigation: the system dependencies for running Nav2 with config/nav2_ugv.yaml.
#
# Used by console (which runs it inside its own container before starting a
# navigation job) and by anyone setting up a host; on a host run it with
# `sudo bash scripts/deps/nav2.sh`. Contract: see _common.sh.
#
#   nav2_bringup       - navigation_launch.py and, through ros-<distro>-navigation2,
#                        the rest of Nav2 (bt_navigator, planner/controller servers,
#                        behaviors, smoothers, waypoint_follower, ...)
#   nav2_mppi_controller, nav2_collision_monitor, opennav_docking
#                      - the plugins/servers nav2_ugv.yaml configures; already
#                        pulled in by navigation2, named here so a change to that
#                        dependency chain cannot silently drop them
#   diagnostic_updater - needed by Nav2 nodes; the README notes it has been
#                        seen missing, so it is ensured explicitly
#
# Mapping (slam_toolbox) is separate: scripts/deps/slam.sh.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

D="${ROS_DISTRO:-jazzy}"
apt_ensure \
    "ros-${D}-nav2-bringup" \
    "ros-${D}-nav2-mppi-controller" \
    "ros-${D}-nav2-collision-monitor" \
    "ros-${D}-opennav-docking" \
    "ros-${D}-diagnostic-updater"
