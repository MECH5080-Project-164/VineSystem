#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /picam/install/setup.bash

exec "$@"
