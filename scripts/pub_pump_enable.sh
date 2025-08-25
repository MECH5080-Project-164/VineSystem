#!/usr/bin/env bash
# Simple helper to enable/disable the pressure control pump by setting the ROS 2 parameter
# The /pump/enable topic has been removed; this now only manipulates the parameter
#
# Usage:
#   pub_pump_enable.sh on        # set pump_enabled true
#   pub_pump_enable.sh off       # set pump_enabled false
#   pub_pump_enable.sh toggle    # invert current value
#   pub_pump_enable.sh status    # show current value
#   pub_pump_enable.sh help      # show help
#
# Notes:
# - Assumes the environment already has ROS 2 sourced (e.g., via your container entrypoint)
# - Fails gracefully if node or parameter not available yet

set -euo pipefail

NODE="/pressure_control_node"
PARAM="pump_enabled"

color() { # $1=color $2=message
  local c="$1"; shift
  local msg="$*"
  local code
  case "$c" in
    red) code='\033[31m';;
    green) code='\033[32m';;
    yellow) code='\033[33m';;
    blue) code='\033[34m';;
    *) code='';;
  esac
  printf "%b%s\033[0m\n" "$code" "$msg"
}

get_current() {
  # Output: true/false/unknown
  local out
  if ! out=$(ros2 param get "$NODE" "$PARAM" 2>/dev/null); then
    echo unknown
    return 1
  fi
  if grep -qi 'True' <<<"$out"; then echo true; return 0; fi
  if grep -qi 'False' <<<"$out"; then echo false; return 0; fi
  echo unknown; return 1
}

set_param() { # $1=true|false
  local val="$1"
  if ros2 param set "$NODE" "$PARAM" "$val" >/dev/null 2>&1; then
    color green "Set $PARAM -> $val"
  else
    color red "Failed to set $PARAM (is the node running?)"
    exit 1
  fi
}

print_status() {
  local cur
  cur=$(get_current || true)
  if [[ $cur == unknown ]]; then
    color yellow "Status: unknown (node or parameter not available)"
  else
    if [[ $cur == true ]]; then
      color green "Status: $PARAM = true"
    else
      color red "Status: $PARAM = false"
    fi
  fi
}

usage() {
  cat <<EOF
Usage: $0 <on|off|toggle|status|help>

Controls the '$PARAM' parameter on node '$NODE'.
EOF
}

cmd="${1:-status}"
case "$cmd" in
  on)
    set_param true
    ;;
  off)
    set_param false
    ;;
  toggle)
    cur=$(get_current || true)
    case "$cur" in
      true) set_param false ;;
      false) set_param true ;;
      *) color red "Cannot toggle: current value unknown"; exit 1 ;;
    esac
    ;;
  status)
    print_status
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    color red "Unknown command: $cmd"
    usage
    exit 1
    ;;
 esac
