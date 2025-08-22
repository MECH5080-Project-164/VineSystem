#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# # Try to source workspace if present (convenience)
# if [[ -f "$SCRIPT_DIR/../vine_ws/install/setup.bash" ]]; then
#   # shellcheck disable=SC1090
#   source "$SCRIPT_DIR/../vine_ws/install/setup.bash"
# fi

usage() {
  cat <<EOF
Usage: ${0##*/} <on|off|toggle|status> [--persist]

Commands:
  on        Publish /pump/enable true
  off       Publish /pump/enable false
  toggle    Toggle current state (reads parameter or topic)
  status    Show pump_enabled parameter and last /pump/enable message (if available)

Options:
  --persist   Also set the persistent parameter /pressure_control_node pump_enabled

Examples:
  ${0##*/} on
  ${0##*/} toggle --persist
  ${0##*/} status
EOF
}

if [[ ${#@} -lt 1 ]]; then
  usage
  exit 2
fi

CMD="${1:-}"; shift || true
PERSIST=0
while [[ ${#@} -gt 0 ]]; do
  case "$1" in
    --persist) PERSIST=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage; exit 2;;
  esac
done

publish() {
  local val=$1
  echo "Publishing /pump/enable -> $val"
  ros2 topic pub -1 /pump/enable std_msgs/msg/Bool "{data: $val}"
  if [[ $PERSIST -eq 1 ]]; then
    echo "Setting persistent parameter /pressure_control_node pump_enabled -> $val"
    ros2 param set /pressure_control_node pump_enabled "$val" || true
  fi
}

read_current() {
  # Prefer parameter (fast). If missing, attempt to read one message from topic with timeout.
  local param_val
  if param_val=$(ros2 param get /pressure_control_node pump_enabled 2>/dev/null || true); then
    if [[ -n "$param_val" ]]; then
      # param output looks like: 'pump_enabled: True'
      if echo "$param_val" | grep -iq "true"; then
        echo true
        return
      elif echo "$param_val" | grep -iq "false"; then
        echo false
        return
      fi
    fi
  fi

  # fallback: try reading last published message (non-blocking with timeout)
  if command -v timeout >/dev/null 2>&1; then
    out=$(timeout 2 ros2 topic echo -n1 /pump/enable 2>/dev/null || true)
  else
    out=$(ros2 topic echo -n1 /pump/enable 2>/dev/null || true)
  fi
  if echo "$out" | grep -iq "data:\s*true"; then
    echo true
  elif echo "$out" | grep -iq "data:\s*false"; then
    echo false
  else
    echo unknown
  fi
}

case "$CMD" in
  on)
    publish true
    ;;
  off)
    publish false
    ;;
  toggle)
    cur=$(read_current)
    if [[ "$cur" == "true" ]]; then
      publish false
    elif [[ "$cur" == "false" ]]; then
      publish true
    else
      echo "Current state unknown; defaulting to enabling pump" >&2
      publish true
    fi
    ;;
  status)
    echo "Persistent parameter (/pressure_control_node pump_enabled):"
    ros2 param get /pressure_control_node pump_enabled || echo "(not set or not available)"
    echo "Last /pump/enable topic message (attempt):"
    if command -v timeout >/dev/null 2>&1; then
      timeout 2 ros2 topic echo -n1 /pump/enable || true
    else
      ros2 topic echo -n1 /pump/enable || true
    fi
    ;;
  *)
    echo "Unknown command: $CMD" >&2
    usage
    exit 2
    ;;
esac

exit 0
