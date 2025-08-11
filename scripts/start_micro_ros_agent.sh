#!/bin/bash

# Script to automatically find and start micro_ros_agent with Pico device
# Usage: ./start_micro_ros_agent.sh [OPTIONS]

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIND_PICO_SCRIPT="$SCRIPT_DIR/find_pico.sh"

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Automatically find and start micro_ros_agent with Raspberry Pi Pico"
    echo ""
    echo "OPTIONS:"
    echo "  --device, -d DEVICE    Use specific device (e.g., /dev/ttyACM0)"
    echo "  --list, -l             List available Pico devices and exit"
    echo "  --verbose, -v          Show verbose output"
    echo "  --help, -h             Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                     # Auto-find and start with first Pico"
    echo "  $0 --list             # List available Pico devices"
    echo "  $0 -d /dev/ttyACM0     # Use specific device"
    echo "  $0 --verbose          # Show verbose output during startup"
}

# Parse command line arguments
DEVICE=""
LIST_ONLY=false
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --device|-d)
            DEVICE="$2"
            shift 2
            ;;
        --list|-l)
            LIST_ONLY=true
            shift
            ;;
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Check if find_pico.sh exists
if [ ! -f "$FIND_PICO_SCRIPT" ]; then
    echo -e "${RED}Error: find_pico.sh not found at $FIND_PICO_SCRIPT${NC}" >&2
    echo "Make sure find_pico.sh is in the same directory as this script."
    exit 1
fi

# Make sure find_pico.sh is executable
if [ ! -x "$FIND_PICO_SCRIPT" ]; then
    if [ "$VERBOSE" = true ]; then
        echo "Making find_pico.sh executable..."
    fi
    chmod +x "$FIND_PICO_SCRIPT"
fi

# If list only, show devices and exit
if [ "$LIST_ONLY" = true ]; then
    echo "Available Raspberry Pi Pico devices:"
    "$FIND_PICO_SCRIPT"
    exit 0
fi

# Determine which device to use
if [ -n "$DEVICE" ]; then
    # User specified a device
    if [ ! -e "$DEVICE" ]; then
        echo -e "${RED}Error: Device $DEVICE does not exist${NC}" >&2
        exit 1
    fi

    if [ "$VERBOSE" = true ]; then
        echo -e "${BLUE}Using specified device: $DEVICE${NC}"
    fi

    PICO_DEVICE="$DEVICE"
else
    # Auto-find Pico device
    if [ "$VERBOSE" = true ]; then
        echo "Searching for Raspberry Pi Pico devices..."
        "$FIND_PICO_SCRIPT"
        echo ""
    fi

    PICO_DEVICE=$("$FIND_PICO_SCRIPT" --first)

    if [ -z "$PICO_DEVICE" ]; then
        echo -e "${RED}No Raspberry Pi Pico devices found!${NC}" >&2
        echo ""
        echo "Troubleshooting:"
        echo "1. Make sure the Pico is connected via USB"
        echo "2. Check if the device appears with: ls /dev/tty*"
        echo "3. Try running: $0 --list"
        echo "4. Manually specify device with: $0 -d /dev/ttyACM0"
        exit 1
    fi

    echo -e "${GREEN}Found Pico device: $PICO_DEVICE${NC}"
fi

# Check if micro_ros_agent is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS 2 not found. Make sure ROS 2 is installed and sourced.${NC}" >&2
    exit 1
fi

# Check if micro_ros_agent package is available
if ! ros2 pkg list 2>/dev/null | grep -q micro_ros_agent 2>/dev/null; then
    echo -e "${RED}Error: micro_ros_agent package not found.${NC}" >&2
    echo "Install it with: sudo apt install ros-${ROS_DISTRO}-micro-ros-agent"
    exit 1
fi

# Start micro_ros_agent
echo -e "${BLUE}Starting micro_ros_agent with device: $PICO_DEVICE${NC}"
echo ""

if [ "$VERBOSE" = true ]; then
    echo "Command: ros2 run micro_ros_agent micro_ros_agent serial --dev $PICO_DEVICE"
    echo ""
fi

echo -e "${YELLOW}Press Ctrl+C to stop the agent${NC}"
echo ""

# Execute the micro_ros_agent
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$PICO_DEVICE"
