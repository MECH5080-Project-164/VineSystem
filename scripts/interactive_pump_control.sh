#!/usr/bin/env bash
# Interactive pump control UI (terminal based)
# Adjusts the pressure controller's target_pressure parameter with simple key presses.
# Optionally toggles pump_enabled.
#
# Requirements:
#   - ROS 2 environment already sourced (ros2 CLI available)
#   - pressure_control_node running
#
# Controls:
#   Up / Down / Left / Right OR w/s/a/d adjust by STEP (default 1)
#   + / - also adjust by STEP
#   Live pressure from /pressure displayed (polled asynchronously)
#   p             Toggle pump_enabled
#   e             Enable pump (pump_enabled=true)
#   x             Disable pump (pump_enabled=false)
#   r             Refresh display from parameters
#   q             Quit
#
# Options:
#   --step N        Step size (kPa) (default 1)
#   --min N         Minimum target (default 0)
#   --max N         Maximum target (default 150)
#   --no-color      Disable ANSI colors
#   --pressure-topic NAME  Topic name for pressure (default /pressure)
#
# Notes:
#   - Each keypress writes the parameter immediately (no buffering) except 'r'.
#   - If the node is not yet available, the script waits until parameters can be read.
#   - Press 'q' to exit cleanly (terminal settings restored automatically on exit or error).

set -euo pipefail

NODE="/pressure_control_node"
PARAM="target_pressure"
PUMP_PARAM="pump_enabled"
STEP=1          # single step size
MIN_TARGET=0
MAX_TARGET=150
COLOR=1
PRESSURE_TOPIC="/pressure"

# Parse CLI args
while [[ $# -gt 0 ]]; do
  case "$1" in
  --step) STEP="$2"; shift 2;;
    --pressure-topic) PRESSURE_TOPIC="$2"; shift 2;;
    --min) MIN_TARGET="$2"; shift 2;;
    --max) MAX_TARGET="$2"; shift 2;;
    --no-color) COLOR=0; shift;;
    -h|--help)
      grep '^# ' "$0" | sed 's/^# //'
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS 2 environment first." >&2
  exit 1
fi

# Colors
if [[ $COLOR -eq 1 ]]; then
  C_RESET='\033[0m'
  C_GREEN='\033[32m'
  C_YELLOW='\033[33m'
  C_RED='\033[31m'
  C_BLUE='\033[34m'
else
  C_RESET=''; C_GREEN=''; C_YELLOW=''; C_RED=''; C_BLUE='';
fi

cleanup() {
  stty echo -icanon time 0 min 1 2>/dev/null || true
  tput cnorm 2>/dev/null || true
  echo -e "\n${C_RESET}Exiting."
}
trap cleanup EXIT INT TERM

# Wait for node parameters
spinner='|/-\\'
i=0
printf "Waiting for %s parameters" "$NODE"
while true; do
  if ros2 param get "$NODE" "$PARAM" >/dev/null 2>&1; then
    break
  fi
  printf "\rWaiting for %s parameters %c" "$NODE" "${spinner:i++%${#spinner}:1}"
  sleep 0.2
done
printf "\r%-70s\n" "Parameters ready." # clear line

get_target() {
  local line
  line=$(ros2 param get "$NODE" "$PARAM" 2>/dev/null || true)
  # Expected format: target_pressure: 123.0
  awk '{print $2}' <<<"$line" | tr -d '\r' || echo "0"
}

get_pump() {
  local line
  line=$(ros2 param get "$NODE" "$PUMP_PARAM" 2>/dev/null || true)
  if grep -qi 'True' <<<"$line"; then echo true; else echo false; fi
}

set_target() { # $1 = new target
  local val="$1"
  ros2 param set "$NODE" "$PARAM" "$val" >/dev/null 2>&1 || return 1
  CUR_TARGET="$val"
  return 0
}

set_pump() { # $1=true|false
  ros2 param set "$NODE" "$PUMP_PARAM" "$1" >/dev/null 2>&1 || return 1
  return 0
}

read_pressure_latest() {
  # Single sample (best-effort). Using timeout to avoid blocking if topic inactive.
  local line
  line=$(timeout 0.2 ros2 topic echo -n1 "$PRESSURE_TOPIC" --field data 2>/dev/null | tr -d '\r' || true)
  if [[ -z "$line" || ! $line =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
    echo "--"
  else
    echo "$line"
  fi
}

redraw() {
  CUR_TARGET=$(get_target)
  PUMP_STATE=$(get_pump)
  CURRENT_PRESSURE=$(read_pressure_latest)
  printf "\r${C_BLUE}Target:${C_RESET} %7.2f kPa  ${C_BLUE}Current:${C_RESET} %7s kPa  ${C_BLUE}Pump:${C_RESET} %s  ${C_BLUE}Step:${C_RESET} %s  (q to quit)    " \
    "$CUR_TARGET" "$CURRENT_PRESSURE" "$( [[ $PUMP_STATE == true ]] && echo "${C_GREEN}ON${C_RESET}" || echo "${C_RED}OFF${C_RESET}" )" "$STEP"
}

# Initial state
CUR_TARGET=$(get_target)
PUMP_STATE=$(get_pump)

# Switch terminal to raw-ish mode
stty -echo -icanon time 0 min 1
printf "${C_YELLOW}Interactive pump control started. Press 'q' to quit.${C_RESET}\n"
redraw

adjust_and_set() { # $1 = delta
  local delta="$1"
  local new
  new=$(awk -v a="$CUR_TARGET" -v b="$delta" 'BEGIN{printf "%.3f", a+b}')
  # Clamp
  awk -v x="$new" -v min="$MIN_TARGET" -v max="$MAX_TARGET" 'BEGIN{if (x<min) x=min; if (x>max) x=max; printf "%.3f", x}' | {
    read clamped
    if set_target "$clamped"; then
      CUR_TARGET="$clamped"
    fi
  }
}

# Hide cursor
tput civis 2>/dev/null || true

while true; do
  if ! IFS= read -rsn1 key; then
    redraw
    sleep 0.25
    continue
  fi
  if [[ $key == $'\e' ]]; then
    # Possible escape sequence - read two more bytes if available
    read -rsn2 -t 0.001 rest || true
    seq="$key$rest"
    case "$seq" in
  $'\e[A') adjust_and_set "$STEP";;      # Up
  $'\e[B') adjust_and_set "-$STEP";;     # Down
  $'\e[C') adjust_and_set "$STEP";;     # Right
  $'\e[D') adjust_and_set "-$STEP";;    # Left
    esac
    redraw
    continue
  fi
  case "$key" in
    q) break ;;
    w) adjust_and_set "$STEP" ;;
    s) adjust_and_set "-$STEP" ;;
  d) adjust_and_set "$STEP" ;;
  a) adjust_and_set "-$STEP" ;;
  '+') adjust_and_set "$STEP" ;;
  '-') adjust_and_set "-$STEP" ;;
    p)
      PUMP_STATE=$(get_pump)
      if [[ $PUMP_STATE == true ]]; then set_pump false; else set_pump true; fi
      ;;
    e) set_pump true ;;
    x) set_pump false ;;
    r) : ;; # force redraw only
  esac
  redraw
done

echo
