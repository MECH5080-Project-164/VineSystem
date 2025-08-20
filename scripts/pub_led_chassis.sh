#!/bin/bash

# Publish a single chassis brightness message to /led_control/chassis
# Usage: ./pub_chassis_brightness.sh <brightness>
#   brightness: integer 0-100

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

if [ $# -lt 1 ]; then
  echo -e "${RED}Error: brightness argument required.${NC}" >&2
  echo "Usage: $0 <brightness>"
  exit 1
fi

BRIGHTNESS=$1

if ! [[ "$BRIGHTNESS" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Error: brightness must be an integer between 0 and 100.${NC}" >&2
  exit 1
fi

if [ "$BRIGHTNESS" -lt 0 ] || [ "$BRIGHTNESS" -gt 100 ]; then
  echo -e "${RED}Error: brightness must be between 0 and 100.${NC}" >&2
  exit 1
fi

# Ensure ros2 is available
if ! command -v ros2 &> /dev/null; then
  echo -e "${RED}Error: ros2 not found. Run inside the Vine container and source the workspace.${NC}" >&2
  exit 1
fi

# Publish the message
echo -e "${GREEN}Publishing brightness ${BRIGHTNESS} to /led_control/chassis${NC}"
exec ros2 topic pub /led_control/chassis vine_interfaces/msg/ChassisLed "{brightness: $BRIGHTNESS}" -1
