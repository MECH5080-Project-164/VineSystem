#!/bin/bash

# Verify that we're in the Vine Container
if [ -z "$VINE_CONTAINER" ]; then
    echo "This script must be run inside the Vine Container."
    exit 1
fi

source /opt/ros/"$ROS_DISTRO"/setup.bash
source /picam/install/setup.bash

ros2 run camera_ros camera_node
