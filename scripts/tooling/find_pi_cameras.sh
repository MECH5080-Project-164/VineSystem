#!/bin/bash

# Find connected Pi cameras and print their indexes (/dev/videoN -> index N)
# Intended to run inside the Vine Docker container with libcamera installed.
# Usage: ./find_pi_cameras.sh

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

usage() {
  echo "Usage: $0"
  echo "Prints detected camera devices and their corresponding indexes (from /dev/videoN)."
}

# Prefer running inside the Vine container (matches other scripts)
if [ -z "$VINE_CONTAINER" ]; then
  echo -e "${YELLOW}Warning:${NC} This script is intended to run inside the Vine Container. Continuing anyway..."
fi

found_any=false

# 1) If libcamera-hello is available, show its camera listing (helpful info)
if command -v libcamera-hello &> /dev/null; then
  echo -e "${GREEN}libcamera listing:${NC}"
  # Try common flags; some libcamera builds use --list-cameras or -l
  LIBCAM_OUT=""
  if libcamera-hello --list-cameras &> /tmp/libcam_list.$$; then
    LIBCAM_OUT=$(cat /tmp/libcam_list.$$)
  elif libcamera-hello -l &> /tmp/libcam_list.$$; then
    LIBCAM_OUT=$(cat /tmp/libcam_list.$$)
  else
    LIBCAM_OUT=$(libcamera-hello --help 2>/dev/null | head -n 20)
  fi
  # Print captured output
  echo "$LIBCAM_OUT"
  echo ""
  rm -f /tmp/libcam_list.$$ 2>/dev/null || true
  found_any=true
fi

# 1.5) Check kernel messages for camera driver hints (imx708, imx, media, video)
if command -v dmesg &> /dev/null; then
  echo -e "${GREEN}Kernel/dmesg camera hints (last 500 lines):${NC}"
  # Search for imx708 specifically, then fallback to generic imx/video/media keywords
  if dmesg | tail -n 500 | grep -i --color=never -E "imx708" >/tmp/dmesg_cam.$$; then
    echo "-- imx708 messages --"
    cat /tmp/dmesg_cam.$$
  else
    dmesg | tail -n 500 | grep -i --color=never -E "imx|imx7|media|video|uvc|v4l2|cam" >/tmp/dmesg_cam.$$ || true
    if [ -s /tmp/dmesg_cam.$$ ]; then
      echo "-- kernel messages matching imx/media/video/uvc --"
      cat /tmp/dmesg_cam.$$
    else
      echo "(no relevant dmesg lines found in last 500 lines)"
    fi
  fi
  echo ""
  rm -f /tmp/dmesg_cam.$$ 2>/dev/null || true
  found_any=true
fi

# 2) Try v4l2-ctl to get device names and mapping
declare -A dev_name_map
if command -v v4l2-ctl &> /dev/null; then
  V4L_RAW=$(v4l2-ctl --list-devices 2>/dev/null)
  if [ -n "$V4L_RAW" ]; then
    echo -e "${GREEN}v4l2 devices:${NC}"
    # Parse blocks: non-indented line is device name, indented lines are device nodes
    current_name=""
    while IFS= read -r line; do
      if [[ -z "$line" ]]; then
        current_name=""
        continue
      fi
      if [[ ! "$line" =~ ^[[:space:]] ]]; then
        current_name="$line"
      else
        dev=$(echo "$line" | awk '{print $1}')
        dev_name_map["$dev"]="$current_name"
      fi
    done <<< "$V4L_RAW"
    # Print raw v4l2 output for visibility
    echo "$V4L_RAW"
    echo ""
    found_any=true
  fi
fi

# 3) Map /dev/videoN to indexes (N) and print friendly table
echo -e "${GREEN}Detected /dev/video devices and indexes:${NC}"
shopt -s nullglob
devices=(/dev/video*)
if [ ${#devices[@]} -eq 0 ]; then
  echo -e "${RED}No /dev/video devices found.${NC}"
  if ! $found_any; then
    echo "Ensure libcamera and v4l2-utils are installed inside the container, and that the cameras are passed through to the container." >&2
  fi
  exit 1
fi

for dev in "${devices[@]}"; do
  # Extract numeric index from /dev/videoN
  base=${dev##*/}
  idx=${base#video}
  name=${dev_name_map["$dev"]:-N/A}
  printf "  Index %s -> %s" "$idx" "$dev"
  if [ "$name" != "N/A" ]; then
    printf "  (name: %s)" "$name"
  fi
  printf "\n"
done

# Helpful hint for camera index usage
echo ""
echo -e "${YELLOW}Hint:${NC} If a node/script expects a camera index (0,1,...), use the numeric index shown above (the N in /dev/videoN)."

exit 0
