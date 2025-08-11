#!/bin/bash

# Script to find Raspberry Pi Pico serial devices for micro_ros_agent
# Usage: ./find_pico.sh [--first] [--all] [--agent]

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Find Raspberry Pi Pico devices for micro_ros_agent"
    echo ""
    echo "OPTIONS:"
    echo "  --first, -f     Return only the first Pico device found"
    echo "  --all, -a       Show all USB serial devices (not just Picos)"
    echo "  --agent, -g     Output micro_ros_agent command directly"
    echo "  --help, -h      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0              # List all Pico devices"
    echo "  $0 --first     # Get first Pico device"
    echo "  $0 --agent     # Show ready-to-run micro_ros_agent command"
}

find_pico_devices() {
    local devices=()

    # Look for devices in /dev/ttyACM* and /dev/ttyUSB*
    for device in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$device" ]; then
            # Get device info using udevadm
            local device_info=$(udevadm info --name="$device" --query=property 2>/dev/null)

            if [ $? -eq 0 ]; then
                local vendor_name=$(echo "$device_info" | grep "ID_VENDOR=" | cut -d'=' -f2)
                local model_name=$(echo "$device_info" | grep "ID_MODEL=" | cut -d'=' -f2)
                local serial=$(echo "$device_info" | grep "ID_SERIAL_SHORT=" | cut -d'=' -f2)

                # Check if it's a Pico device by looking for "Pico" in the name
                local is_pico=false
                if [[ "$model_name" =~ [Pp]ico ]] || [[ "$vendor_name" =~ [Pp]ico ]] || [[ "$vendor_name" =~ "Raspberry_Pi" ]]; then
                    is_pico=true
                fi

                if [ "$1" = "all" ] || [ "$is_pico" = true ]; then
                    devices+=("$device|$vendor_name|$model_name|$serial|$is_pico")
                fi
            fi
        fi
    done

    echo "${devices[@]}"
}

# Parse command line arguments
SHOW_FIRST=false
SHOW_ALL=false
SHOW_AGENT=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --first|-f)
            SHOW_FIRST=true
            shift
            ;;
        --all|-a)
            SHOW_ALL=true
            shift
            ;;
        --agent|-g)
            SHOW_AGENT=true
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

# Find devices
if [ "$SHOW_ALL" = true ]; then
    devices=($(find_pico_devices "all"))
else
    devices=($(find_pico_devices "pico"))
fi

# Check if any devices found
if [ ${#devices[@]} -eq 0 ]; then
    echo -e "${RED}No Raspberry Pi Pico devices found!${NC}" >&2
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure the Pico is connected via USB"
    echo "2. Check if the device appears in 'lsusb' output"
    echo "3. Verify the Pico is running micro-ROS firmware"
    echo "4. Try running with --all to see all USB serial devices"
    exit 1
fi

# Process results
pico_devices=()
for device_info in "${devices[@]}"; do
    IFS='|' read -r device vendor model serial is_pico <<< "$device_info"

    if [ "$SHOW_ALL" = true ] || [ "$is_pico" = "true" ]; then
        pico_devices+=("$device")

        if [ "$SHOW_FIRST" = false ] && [ "$SHOW_AGENT" = false ]; then
            echo -e "${GREEN}Found device:${NC} $device"
            echo "  Vendor: $vendor"
            echo "  Model: $model"
            if [ -n "$serial" ]; then
                echo "  Serial: $serial"
            fi
            if [ "$is_pico" = "true" ]; then
                echo -e "  ${BLUE}Type: Raspberry Pi Pico${NC}"
            else
                echo "  Type: Other USB Serial Device"
            fi
            echo ""
        fi
    fi
done

# Output based on options
if [ "$SHOW_AGENT" = true ]; then
    if [ ${#pico_devices[@]} -gt 0 ]; then
        echo "ros2 run micro_ros_agent micro_ros_agent serial --dev ${pico_devices[0]}"
    fi
elif [ "$SHOW_FIRST" = true ]; then
    if [ ${#pico_devices[@]} -gt 0 ]; then
        echo "${pico_devices[0]}"
    fi
else
    if [ ${#pico_devices[@]} -gt 1 ]; then
        echo -e "${YELLOW}Multiple devices found. Use --first to get the first one.${NC}"
        echo ""
        echo "To start micro_ros_agent with the first device:"
        echo -e "${BLUE}ros2 run micro_ros_agent micro_ros_agent serial --dev ${pico_devices[0]}${NC}"
    elif [ ${#pico_devices[@]} -eq 1 ]; then
        echo "To start micro_ros_agent:"
        echo -e "${BLUE}ros2 run micro_ros_agent micro_ros_agent serial --dev ${pico_devices[0]}${NC}"
    fi
fi
