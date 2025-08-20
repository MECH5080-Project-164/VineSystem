#!/bin/bash

# Simple script to start the chassis LED control node
# Usage: ./start_chassis_led.sh [brightness]
#   brightness: integer 0-100 (default: 100)

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

DEFAULT_BRIGHTNESS=100

# Read argument (use default if not provided)
BRIGHTNESS=${1:-$DEFAULT_BRIGHTNESS}

# Validate number
if ! [[ "$BRIGHTNESS" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Error: brightness must be an integer between 0 and 100.${NC}" >&2
    echo "Usage: $0 [brightness]  # brightness: 0-100" >&2
    exit 1
fi

# Validate range
if [ "$BRIGHTNESS" -lt 0 ] || [ "$BRIGHTNESS" -gt 100 ]; then
    echo -e "${RED}Error: brightness must be between 0 and 100.${NC}" >&2
    exit 1
fi

# Workspace directory (matches other scripts)
WORKSPACE_DIR="/home/workspace/students/VineSystem/vine_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}Error: Workspace not found at $WORKSPACE_DIR. Make sure you're in the Vine container or update WORKSPACE_DIR.${NC}" >&2
    exit 1
fi

# Check for ros2
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS 2 not found.${NC}" >&2
    exit 1
fi

# Build workspace if needed
cd "$WORKSPACE_DIR"
if [ ! -d "install" ]; then
    echo -e "${YELLOW}Building workspace...${NC}"
    ./build_vine_ws.sh
fi

# Source workspace
source install/setup.bash

# Check package exists
if ! ros2 pkg list 2>/dev/null | grep -q led_control_chassis 2>/dev/null; then
    echo -e "${RED}Error: led_control_chassis package not found in workspace.${NC}" >&2
    exit 1
fi

echo -e "${GREEN}Starting chassis LED node (brightness: ${BRIGHTNESS})${NC}"
echo ""

# If the node is already running, inform and exit — this script's job is to start the node
if ros2 node list 2>/dev/null | grep -q "led_control_chassis"; then
    echo -e "${YELLOW}Node 'led_control_chassis' is already running. Exiting.${NC}"
    exit 0
fi

# Start the node in the foreground and pass the brightness as a parameter
if [ -n "$BRIGHTNESS" ]; then
    exec ros2 run led_control_chassis led_control_chassis_node --ros-args -p brightness:=$BRIGHTNESS
else
    exec ros2 run led_control_chassis led_control_chassis_node
fi
