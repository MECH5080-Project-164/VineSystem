#!/bin/bash

# Script to find Raspberry Pi Pico serial devices for micro_ros_agent
# Supports both USB (ttyACM*/ttyUSB*) and UART (ttyAMA*/ttyS*/serial*) connections
# Usage: ./find_pico.sh [--first] [--all] [--agent] [--prefer-uart] [--no-prefer-uart]

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Find Raspberry Pi Pico devices for micro_ros_agent (USB or direct UART)"
    echo ""
    echo "OPTIONS:"
    echo "  --first, -f          Return only the first Pico device found"
    echo "  --all, -a            Show all candidate serial devices"
    echo "  --agent, -g          Output micro_ros_agent command directly"
    echo "  --prefer-uart        Prefer on-board UART devices first (default)"
    echo "  --no-prefer-uart     Do not prioritize UART devices"
    echo "  --help, -h           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                   # List Pico candidates (UART first if present)"
    echo "  $0 --first           # Output the best candidate path"
    echo "  $0 --agent           # Show ready-to-run micro_ros_agent command"
    echo "  $0 --no-prefer-uart  # Treat USB and UART equally"
}

# Collect candidate devices (USB + UART)
find_pico_devices() {
    local devices=()
    for device in /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* /dev/ttyS* /dev/serial*; do
        if [ -e "$device" ]; then
            local device_info
            device_info=$(udevadm info --name="$device" --query=property 2>/dev/null)
            if [ $? -eq 0 ]; then
                local vendor_name model_name serial
                vendor_name=$(echo "$device_info" | grep "ID_VENDOR=" | cut -d'=' -f2)
                model_name=$(echo "$device_info" | grep "ID_MODEL=" | cut -d'=' -f2)
                serial=$(echo "$device_info" | grep "ID_SERIAL_SHORT=" | cut -d'=' -f2)
                local is_pico=false
                # Heuristics for Pico via USB
                if [[ "$model_name" =~ [Pp]ico ]] || [[ "$vendor_name" =~ [Pp]ico ]] || [[ "$vendor_name" =~ Raspberry_Pi ]]; then
                    is_pico=true
                fi
                # Heuristics for UART (assume candidate if core UART names)
                if [[ "$device" =~ /dev/(ttyAMA[0-9]+|serial0|serial1) ]]; then
                    is_pico=true
                    [ -z "$vendor_name" ] && vendor_name="UART"
                    [ -z "$model_name" ] && model_name="Serial_Device"
                fi
                if [ "$1" = "all" ] || [ "$is_pico" = true ]; then
                    # Store: device|vendor|model|serial|is_pico
                    devices+=("$device|$vendor_name|$model_name|$serial|$is_pico")
                fi
            fi
        fi
    done
    echo "${devices[@]}"
}

SHOW_FIRST=false
SHOW_ALL=false
SHOW_AGENT=false
PREFER_UART=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --first|-f) SHOW_FIRST=true; shift ;;
        --all|-a) SHOW_ALL=true; shift ;;
        --agent|-g) SHOW_AGENT=true; shift ;;
        --prefer-uart) PREFER_UART=true; shift ;;
        --no-prefer-uart) PREFER_UART=false; shift ;;
        --help|-h) show_help; exit 0 ;;
        *) echo "Unknown option: $1"; show_help; exit 1 ;;
    esac
done

# Fast path: if the primary UART /dev/ttyAMA0 exists, use it directly unless --all requested
if [ -e /dev/ttyAMA0 ] && [ "$SHOW_ALL" = false ]; then
    if [ "$SHOW_AGENT" = true ]; then
        echo "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0"
        exit 0
    fi
    if [ "$SHOW_FIRST" = true ]; then
        echo "/dev/ttyAMA0"
        exit 0
    fi
    echo -e "${GREEN}Found device:${NC} /dev/ttyAMA0"
    echo "  Vendor: UART"
    echo "  Model: Onboard_Primary_UART"
    echo -e "  ${BLUE}Type: UART (On-board preferred)${NC}"
    echo ""
fi

# Gather devices
if [ "$SHOW_ALL" = true ]; then
    devices=( $(find_pico_devices "all") )
else
    devices=( $(find_pico_devices "pico") )
fi

if [ ${#devices[@]} -eq 0 ]; then
    echo -e "${RED}No Raspberry Pi Pico (or candidate UART) devices found!${NC}" >&2
    echo ""; echo "Troubleshooting:";
    echo "1. Check wiring (Pi GPIO14->Pico RX, GPIO15->Pico TX, shared GND)";
    echo "2. Enable UART: ensure enable_uart=1 in /boot/firmware/config.txt";
    echo "3. Disable serial console (raspi-config)";
    echo "4. For USB mode, confirm dmesg shows ttyACM* assignment";
    exit 1
fi

# Reorder if preferring UART
if [ "$PREFER_UART" = true ]; then
    ordered=()
    others=()
    for entry in "${devices[@]}"; do
        dev_path="${entry%%|*}"
        if [[ $dev_path =~ /dev/(ttyAMA[0-9]+|serial0|serial1) ]]; then
            ordered+=("$entry")
        else
            others+=("$entry")
        fi
    done
    devices=("${ordered[@]}" "${others[@]}")
fi

pico_devices=()
for device_info in "${devices[@]}"; do
    IFS='|' read -r device vendor model serial is_pico <<< "$device_info"
    pico_devices+=("$device")
    if [ "$SHOW_FIRST" = false ] && [ "$SHOW_AGENT" = false ]; then
        echo -e "${GREEN}Found device:${NC} $device"
        echo "  Vendor: ${vendor:-N/A}"
        echo "  Model: ${model:-N/A}"
        [ -n "$serial" ] && echo "  Serial: $serial"
        if [[ $device =~ /dev/(ttyAMA[0-9]+|serial0|serial1) ]]; then
            echo -e "  ${BLUE}Type: UART (On-board)${NC}"
        elif [[ $device =~ /dev/ttyACM|/dev/ttyUSB ]]; then
            echo -e "  ${BLUE}Type: USB CDC${NC}"
        else
            echo "  Type: Serial"
        fi
        echo ""
    fi
    [ "$SHOW_ALL" = false ] && [ "$is_pico" != true ] && continue
done

# Output modes
if [ "$SHOW_AGENT" = true ]; then
    [ ${#pico_devices[@]} -gt 0 ] && echo "ros2 run micro_ros_agent micro_ros_agent serial --dev ${pico_devices[0]}"
    exit 0
fi

if [ "$SHOW_FIRST" = true ]; then
    [ ${#pico_devices[@]} -gt 0 ] && echo "${pico_devices[0]}"
    exit 0
fi

if [ ${#pico_devices[@]} -gt 1 ]; then
    echo -e "${YELLOW}Multiple candidates found. First chosen (respecting UART preference). Use --first to script retrieval.${NC}"
    echo "To start micro_ros_agent with first:"
    echo -e "${BLUE}ros2 run micro_ros_agent micro_ros_agent serial --dev ${pico_devices[0]}${NC}"
elif [ ${#pico_devices[@]} -eq 1 ]; then
    echo "To start micro_ros_agent:"
    echo -e "${BLUE}ros2 run micro_ros_agent micro_ros_agent serial --dev ${pico_devices[0]}${NC}"
fi
