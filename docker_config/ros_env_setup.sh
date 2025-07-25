#!/bin/bash

# Source the main ROS environment setup script
if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "ROS environment setup script not found for distribution: $ROS_DISTRO"
    exit 1
fi

# Source the micro-ROS environment setup script
if [ -f /microros_ws/install/setup.bash ]; then
    source /microros_ws/install/setup.bash
else
    echo "Micro-ROS environment setup script not found"
fi

# Source the PiCamera environment setup script
if [ -f /picam/install/setup.bash ]; then
    source /picam/install/setup.bash
else
    echo "PiCamera environment setup script not found"
fi
