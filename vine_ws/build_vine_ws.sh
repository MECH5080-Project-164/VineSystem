#!/bin/bash

# Script to build the vine_ws ROS workspace

set -e

# --- Functions ---
function error_exit() {
    echo "ERROR: $1" >&2
    exit 1
}

function check_command() {
    command -v "$1" >/dev/null 2>&1 || error_exit "'$1' is not installed. Please install it and try again."
}

# --- Source ROS environment ---
if [ -f /opt/ros/"$ROS_DISTRO"/setup.bash ]; then
    source /opt/ros/"$ROS_DISTRO"/setup.bash
else
    error_exit "ROS setup file not found. Please ensure ROS is installed and sourced correctly."
fi

# --- Check for required tools ---
check_command colcon
check_command rosdep

# --- Install dependencies ---
echo "Installing dependencies with rosdep..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# --- Build the workspace ---
echo "Building the workspace with colcon..."
colcon build --symlink-install

echo "Build complete. To use the workspace, run:"
echo "  source install/setup.bash"
