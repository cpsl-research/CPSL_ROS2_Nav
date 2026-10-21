#!/usr/bin/env bash
# Navigation: the system dependencies for running Nav2 with config/nav2_ugv.yaml.
#
# Used by console (which runs it inside its own container before starting a
# navigation job) and by anyone setting up a host; on a host run it with
# `sudo bash scripts/deps/nav2.sh`. Contract: see _common.sh.
#
# Usage:
#   sudo bash nav2.sh           install whatever is missing
#   bash nav2.sh --check        no root, installs nothing: exit 0 if every package
#                                below is installed, else exit 1 and list the missing
#                                ones on stderr. Works on plain ROS 2, without console.
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
D="${ROS_DISTRO:-jazzy}"

PACKAGES=(
    "ros-${D}-nav2-bringup"
    "ros-${D}-nav2-mppi-controller"
    "ros-${D}-nav2-collision-monitor"
    "ros-${D}-opennav-docking"
    "ros-${D}-diagnostic-updater"
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
