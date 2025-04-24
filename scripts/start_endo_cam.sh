#!/bin/bash

# Verify that we're in the Vine Container
if [ -z "$VINE_CONTAINER" ]; then
    echo "This script must be run inside the Vine Container."
    exit 1
fi

PARAM_FILE="/home/workspace/students/acutiy_code/pharos_ws/src/pharos_ros/config/params_1.yaml"

if [ ! -f "$PARAM_FILE" ]; then
    echo "Default parameter file not found: $PARAM_FILE"
    exit 1
fi

ros2 run usb_cam usb_cam_node_exe --ros-args --params-file "$PARAM_FILE"
