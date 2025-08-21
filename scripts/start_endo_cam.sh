#!/usr/bin/env bash
set -euo pipefail

# start_endo_cam.sh
# Usage: start_endo_cam.sh [-p|--param-file PATH]

# Ensure we're running inside the Vine Container
if [ -z "${VINE_CONTAINER:-}" ]; then
  echo "This script must be run inside the Vine Container." >&2
  exit 1
fi

PARAM_FILE="/home/workspace/students/VineSystem/resources/endo_cam_params.yaml"

usage() {
  cat <<EOF
Usage: ${0##*/} [options]

Options:
  -p, --param-file PATH   Use PATH as the ros2 params file (overrides default)
  -h, --help              Show this help

Examples:
  ${0##*/} --param-file /path/to/params.yaml
EOF
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--param-file)
      if [[ -z "${2:-}" ]]; then
        echo "Error: --param-file requires a path" >&2
        usage
        exit 2
      fi
      PARAM_FILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -* )
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
    * )
      echo "Unexpected argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

# Validate parameter file exists
if [ ! -f "$PARAM_FILE" ]; then
  echo "Parameter file not found: $PARAM_FILE" >&2
  exit 1
fi

exec ros2 run usb_cam usb_cam_node_exe --ros-args --params-file "$PARAM_FILE"
