#!/bin/bash

# --- Functions ---
function usage() {
    echo "Usage: $0 [camera_index]"
    echo "  camera_index: The index of the camera to start (optional)."
}

# --- Main Script ---

# Verify that we're in the Vine Container
if [ -z "$VINE_CONTAINER" ]; then
    echo "This script must be run inside the Vine Container."
    exit 1
fi

source /opt/ros/"$ROS_DISTRO"/setup.bash
source /picam/install/setup.bash

COMMAND="ros2 run camera_ros camera_node"
CAMERA_INDEX=${1}

# if no camera index is provided, we hand over to the camera node by running the stem command
if [ -z "$CAMERA_INDEX" ]; then
    echo "No camera index provided. Starting camera node with default index."
    $COMMAND
else
    # if a camera index is provided, we run the command with the camera index
    echo "Starting camera node with index $CAMERA_INDEX."
    $COMMAND --ros-args -p camera:=$CAMERA_INDEX
fi
