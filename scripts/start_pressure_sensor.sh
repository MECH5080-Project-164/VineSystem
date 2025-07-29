#!/bin/bash

# Script to start the pressure sensor reading node
# Usage: ./start_pressure_sensor.sh [OPTIONS]

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Start the pressure sensor reading node"
    echo ""
    echo "OPTIONS:"
    echo "  --verbose, -v    Show verbose output"
    echo "  --debug, -d      Enable debug logging"
    echo "  --help, -h       Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0               # Start with default settings"
    echo "  $0 --verbose     # Start with verbose output"
    echo "  $0 --debug       # Start with debug logging"
}

# Parse command line arguments
VERBOSE=false
DEBUG=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        --debug|-d)
            DEBUG=true
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

# Check if we're in the right workspace
WORKSPACE_DIR="/home/workspace/students/VineSystem/vine_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}Error: Workspace not found at $WORKSPACE_DIR${NC}" >&2
    echo "Make sure you're running this from inside the Docker container."
    exit 1
fi

# Check if ROS 2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS 2 not found. Make sure ROS 2 is installed and sourced.${NC}" >&2
    exit 1
fi

# Check if workspace is built
INSTALL_DIR="$WORKSPACE_DIR/install"
if [ ! -d "$INSTALL_DIR" ]; then
    echo -e "${YELLOW}Warning: Workspace not built. Building now...${NC}"
    cd "$WORKSPACE_DIR"
    ./build_vine_ws.sh
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error: Failed to build workspace${NC}" >&2
        exit 1
    fi
fi

# Source the workspace
cd "$WORKSPACE_DIR"
source install/setup.bash

# Check if the package exists
if ! ros2 pkg list 2>/dev/null | grep -q gravity_pressure_sensor 2>/dev/null; then
    echo -e "${RED}Error: gravity_pressure_sensor package not found.${NC}" >&2
    echo "Make sure the workspace is properly built."
    exit 1
fi

# Prepare launch command
LOG_LEVEL="INFO"
if [ "$DEBUG" = true ]; then
    LOG_LEVEL="DEBUG"
fi

# Show startup info
echo -e "${GREEN}Starting pressure sensor reading node...${NC}"
if [ "$VERBOSE" = true ]; then
    echo "Workspace: $WORKSPACE_DIR"
    echo "Log level: $LOG_LEVEL"
    echo "Command: ros2 run gravity_pressure_sensor pressure_reading_node"
    echo ""
fi

echo -e "${YELLOW}Press Ctrl+C to stop the sensor${NC}"
echo ""

# Start the pressure sensor node
if [ "$DEBUG" = true ]; then
    exec ros2 run gravity_pressure_sensor pressure_reading_node --ros-args --log-level DEBUG
else
    exec ros2 run gravity_pressure_sensor pressure_reading_node
fi
