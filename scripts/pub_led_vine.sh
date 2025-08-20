#!/bin/bash

# Publish a single VineLed message to /led_control/vine
# Usage: ./pub_vine_led.sh <brightness> [duration_ms] [--force]
#   brightness: integer 0-100
#   duration_ms: integer milliseconds (default: 0 = remain on)
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

# If second or third arg is --force, set FORCE and adjust DURATION if needed
if [ "$2" = "--force" ] || [ "$3" = "--force" ]; then
  FORCE=true
  if [ "$2" = "--force" ]; then
    DURATION=0
  fi
fi

# Validate brightness
if ! [[ "$BRIGHTNESS" =~ ^[0-9]+$ ]]; then
  echo -e "${RED}Error: brightness must be an integer between 0 and 100.${NC}" >&2
  usage
  exit 1
fi
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

# Ensure ros2 is available
if ! command -v ros2 &> /dev/null; then
  echo -e "${RED}Error: ros2 not found. Run inside the Vine container and source the workspace.${NC}" >&2
  exit 1
fi

# Prepare force string as lower-case true/false
FORCE_STR="false"
if [ "$FORCE" = true ]; then
  FORCE_STR="true"
fi

echo -e "${GREEN}Publishing VineLed -> brightness: ${BRIGHTNESS}, duration: ${DURATION}, force: ${FORCE_STR}${NC}"

exec ros2 topic pub /led_control/vine vine_interfaces/msg/VineLed "{brightness: $BRIGHTNESS, duration: $DURATION, force: $FORCE_STR}" -1
