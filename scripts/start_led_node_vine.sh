#!/bin/bash

# Script to send a vine LED command
# Usage: ./start_vine_led.sh <brightness> [duration_ms] [--force]
#   brightness: integer 0-100
#   duration_ms: integer milliseconds (default: 0 -> remain on until new command)
#   --force: optional flag to bypass safety limits

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

usage() {
  echo "Usage: $0 <brightness> [duration_ms] [--force]"
  echo "  brightness: 0-100"
  echo "  duration_ms: duration in milliseconds (default 0 = remain on)"
  echo "  --force: override safety duration limits"
}

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

BRIGHTNESS=$1
DURATION=${2:-0}
FORCE=false

# If third arg is provided and equals --force, set FORCE; allow either position 3 or 2 if duration omitted
if [ "$2" = "--force" ] || [ "$3" = "--force" ]; then
  FORCE=true
  # if duration was actually the flag, set duration to default 0
  if [ "$2" = "--force" ]; then
    DURATION=0
  fi
fi

# Validate brightness integer
if ! [[ "$BRIGHTNESS" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Error: brightness must be an integer between 0 and 100.${NC}" >&2
  usage
  exit 1
fi

# Range check
if [ "$BRIGHTNESS" -lt 0 ] || [ "$BRIGHTNESS" -gt 100 ]; then
  echo -e "${RED}Error: brightness must be between 0 and 100.${NC}" >&2
  exit 1
fi

# Validate duration
if ! [[ "$DURATION" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Error: duration must be a non-negative integer (milliseconds).${NC}" >&2
  usage
  exit 1
fi

# Workspace path (match other scripts)
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
if ! ros2 pkg list 2>/dev/null | grep -q led_control_vine 2>/dev/null; then
  echo -e "${RED}Error: led_control_vine package not found in workspace.${NC}" >&2
  exit 1
fi

echo -e "${GREEN}Starting vine LED node (brightness: ${BRIGHTNESS})${NC}"
echo ""

# If the node is already running, inform and exit — this script's job is to start the node
if ros2 node list 2>/dev/null | grep -q "led_control_vine"; then
  echo -e "${YELLOW}Node 'led_control_vine' is already running. Exiting.${NC}"
  exit 0
fi

# Start the node in the foreground; pass brightness and duration as parameters
exec ros2 run led_control_vine led_control_vine_node --ros-args -p brightness:=$BRIGHTNESS -p duration:=$DURATION
