#!/bin/bash

set -e

# --- Configuration ---
CAMERA_NAME_PATTERN="USB Camera:.*Medical"
DEFAULT_MOUNT_POINT="/dev/generic_video_mount"
PARAMETER_FILE="$MOUNT_POINT/students/VineSystem/resources/endo_cam_params.yaml"

# --- Functions ---
error() {
    echo "ERROR: $1" >&2
}

show_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help        Show this help message"
    echo "  -m, --modify      Modify the parameter yaml file"
    echo "  -p, --param FILE  Specify the parameter yaml file (default: $PARAMETER_FILE)"
    echo "  -d, --device PATH Specify the device path (default: $DEFAULT_MOUNT_POINT)"
    echo "  -y, --yes         Assume yes to confirmation prompt when modifying"
    echo
}

check_dependencies() {
    # Check if v4l2-ctl is installed
    if ! command -v v4l2-ctl &> /dev/null; then
        error "v4l-utils (v4l2-ctl command) not found."
        error "Please install it, e.g., 'sudo apt update && sudo apt install v4l-utils'"
        exit 1
    fi

    # Check if sed is installed
    if ! command -v sed &> /dev/null; then
        error "sed command not found"
    fi
}

container_verification() {
    if [ -z "$VINE_CONTAINER" ]; then
        error "This script must be run inside the Vine Container."
        exit 1
    fi
}

modify_yaml() {
    local param_file="$1"
    local device_path="$2"
    local original_device_path

    if [ ! -f "$param_file" ]; then
        error "Parameter file $param_file does not exist." >&2
        exit 1
    fi

    # Create a backup of the original parameter file
    local backup_file="${param_file}.bak.$(date +%Y%m%d_%H%M%S)"
    cp "$param_file" "$backup_file" || error "Failed to create backup of parameter file"

    # Extract the original device path
    original_device_path=$(grep -oP 'video_device: \K.*' "$param_file")
    if [ -z "$original_device_path" ]; then
        error "Could not find video_device parameter in $param_file"
    fi

    # Modify the parameter file with the new device path
    sed -i "s|video_device: .*|video_device: \"$device_path\"|" "$param_file" || error "Failed to modify parameter file"

    # Validate the modification
    if ! grep -q "video_device: \"$device_path\"" "$param_file"; then
        cp "$backup_file" "$param_file" || error "Failed to restore the original parameter file"
        error "Failed to modify the parameter file. Restored the original file."
    fi

    echo "-----------------------------------------"
    echo ""
    echo "Modified parameter file $param_file:"
    echo "  Original device path: $original_device_path"
    echo "  New device path: $device_path"
    echo "  Backup created: $backup_file"
    echo ""
    echo "-----------------------------------------"
}

camera_found_instructions() {
    echo "----------------------------------------"
    echo ""
    echo Endoscope Cameras found:
    echo ""
    echo "$DEVICE_PATHS"
    echo ""
    echo "----------------------------------------"
    echo ""
    echo "To configure the parameter file, use the -m option:"
    echo "  $0 -m"
    echo ""
    echo "Options:"
    echo " -p, --param FILE  Specify the parameter yaml file (default: $PARAMETER_FILE)"
    echo " -d, --device PATH Specify the device path (default: $DEFAULT_MOUNT_POINT)"
    echo ""
    echo "Example:"
    echo "  $0 -m -p /path/to/params.yaml -d /dev/videoX"
    echo ""
    echo "----------------------------------------"
}

user_confirmation() {
    echo "-----------------------------------------"
    echo ""
    echo "About to modify parameter file $PARAMETER_FILE with device path $DEVICE_PATH"
    if [ "$FORCE_YES" = true ]; then
        echo "Force yes enabled; skipping prompt."
        return 0
    fi
    while true; do
        read -r -p "Are you sure you want to continue? (Y/n): " REPLY
        case $REPLY in
            [Yy]* ) break ;;
            [Nn]* ) echo "Operation cancelled."; exit 0 ;;
            * ) break ;;
        esac
    done
}

# --- Arguments ---
MODIFY=false
FORCE_YES=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -m|--modify)
            MODIFY=true
            shift
            ;;
        -p|--param)
            PARAMETER_FILE="$2"
            shift 2
            ;;
        -d|--device)
            DEVICE_PATH="$2"
            shift 2
            ;;
        -y|--yes)
            FORCE_YES=true
            shift
            ;;
        *)
            echo "Unknown option: $1" >&2
            show_help
            exit 1
            ;;
    esac
done

# --- Script ---
container_verification
check_dependencies

# Get the list of video devices
DEVICE_INFO=$(v4l2-ctl --list-devices) || error "Failed to list video devices"
if [ $? -ne 0 ]; then
    error "Failed to list video devices"
    exit 1
fi


# Find the device
DEVICE_PATHS=$(echo "$DEVICE_INFO" | awk -v pattern="$CAMERA_NAME_PATTERN" '
    $0 ~ pattern { found = 1; print $0; next }
    found && $0 ~ "/dev/.*" { print }
    found && $0 !~ "/dev/.*" { found = 0 }
')

DEFAULT_MOUNT_POINT=$(echo "$DEVICE_PATHS" | grep -o "/dev/video[0-9]*" | head -n 1)

# If the modify flag is set, modify the parameter file
if [ "$MODIFY" = true ]; then
    if [ -z "$DEVICE_PATH" ]; then
        DEVICE_PATH="$DEFAULT_MOUNT_POINT"
    fi

    user_confirmation

    modify_yaml "$PARAMETER_FILE" "$DEVICE_PATH"
    exit 0
fi

if [ -z "$DEVICE_PATHS" ]; then
    error "No Endoscope Cameras found."
fi
if [ -z "$DEFAULT_MOUNT_POINT" ]; then
    error "No Endoscope Cameras found."
fi

# Print the found devices
camera_found_instructions
