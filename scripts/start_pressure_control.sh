#!/bin/bash

# Simple script to start the pressure control node
# Usage: ./start_pressure_control.sh [target_pressure]

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Default target pressure
TARGET_PRESSURE=100.0

# Check if target pressure is provided
if [ $# -eq 1 ]; then
    TARGET_PRESSURE=$1
fi

# Check workspace exists
WORKSPACE_DIR="/home/workspace/students/VineSystem/vine_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}Error: Workspace not found. Make sure you're in the Docker container.${NC}" >&2
    exit 1
fi

# Check ROS 2
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS 2 not found.${NC}" >&2
    exit 1
fi

# Build if needed
cd "$WORKSPACE_DIR"
if [ ! -d "install" ]; then
    echo -e "${YELLOW}Building workspace...${NC}"
    ./build_vine_ws.sh
fi

# Source workspace
source install/setup.bash

# Check package
if ! ros2 pkg list 2>/dev/null | grep -q pressure_control 2>/dev/null; then
    echo -e "${RED}Error: pressure_control package not found.${NC}" >&2
    exit 1
fi

# Start the node
echo -e "${GREEN}Starting pressure control (target: ${TARGET_PRESSURE} kPa)${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

exec ros2 run pressure_control pressure_control_node --ros-args -p target_pressure:=$TARGET_PRESSURE
