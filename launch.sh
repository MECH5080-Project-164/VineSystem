#!/usr/bin/env bash
set -euo pipefail

# launch.sh
# Launches an interactive session for controlling the Vine System in a tmux session.
#
# Requirements:
#   - tmux installed on the host (Raspberry Pi)
#   - Vine System Container running or started (see docker-compose.yml)
#
# Note that tmux will always be run on the host, not inside the container.
# Therefore, all ROS processes are launched using 'docker exec'. This enables
# local (host) panes to be added manually as desired.

SESSION_NAME=vine
ATTACH_AUTO=true
FORCE_RECREATE=false

# Which nodes/components to start (defaults)
DO_MICRO_ROS=true
DO_PRESSURE_SENSOR=true
DO_PRESSURE_CONTROL=false
DO_LEDS=true
DO_CAMERAS=true
DO_ENDOSCOPE=true     # subset of cameras; allows disabling just endoscope
DO_PI_CAM=true        # subset of cameras; allows disabling pi cam

# Endoscope auto configuration
AUTO_CONFIG_ENDO=true
ASSUME_YES=false

CONTAINER_NAME="vine-docker"
START_CONTAINER=false

# Container workspace root (mount point in docker-compose)
CONTAINER_WORKSPACE="/home/workspace"

# Workspace overlay (built packages) setup script relative to workspace root
OVERLAY_SETUP_REL="$CONTAINER_WORKSPACE/students/VineSystem/vine_ws/install/setup.bash"
SOURCE_OVERLAY=true

# Path to helper function file (host & container view share path under mounted workspace)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_FN_FILE="$SCRIPT_DIR/scripts/exit_vine_fn.sh"
CONTAINER_FN_FILE="$CONTAINER_WORKSPACE/students/VineSystem/scripts/exit_vine_fn.sh"

# Derived paths inside container (avoid stale hard-coded host paths)
FIND_ENDO_SCRIPT="$CONTAINER_WORKSPACE/students/VineSystem/scripts/tooling/find_endoscope.sh"
FIND_PICO_SCRIPT="$CONTAINER_WORKSPACE/students/VineSystem/scripts/tooling/find_pico.sh"
# Path to Endoscope Camera parameters file
ENDO_PARAMS="$CONTAINER_WORKSPACE/students/VineSystem/resources/endo_cam_params.yaml"

error() { echo "[vine_tmux] ERROR: $*" >&2; }
info()  { echo "[vine_tmux] $*" >&2; }

usage() {
  cat <<EOF
Usage: $0 [options]

Launches an interactive session for controlling the Vine System in a tmux session.

General Options:
  -h, --help              Show this help message and exit.
  -s, --session NAME      Set the tmux session name (default: $SESSION_NAME).
  -a, --attach            Automatically attach to the tmux session (default).
  -d, --detach            Do not attach to the session after creation.
  --force-recreate        Kill any existing tmux session with the same name before starting.

Container Options:
  --container NAME        Specify the target container name (default: $CONTAINER_NAME).
  --start-container       Attempt to start the container using 'docker compose up -d'.

Component Toggles:
  --no-overlay-source     Do not source the workspace overlay setup file.
  --no-micro-ros          Disable the micro-ROS agent window.
  --no-pressure-sensor    Disable the pressure sensor pane.
  --pressure-control      Enable the pressure control pane (disabled by default).
  --no-leds               Disable the LEDs window.
  --no-cameras            Disable the cameras window entirely.
  --no-endoscope          Disable only the endoscope pane within the cameras window.
  --no-pi-cam             Disable only the Pi camera panes within the cameras window.

Endoscope Configuration:
  --endo-params FILE      Path to the endoscope parameters file inside the container.
  --no-auto-config-endo   Disable automatic endoscope configuration script.
  -y, --yes               Assume 'yes' for any interactive prompts (e.g., endoscope config).

EOF
  exit 0
}

# Check if tmux is installed
command -v tmux >/dev/null || { error "tmux not installed on host."; exit 1; }

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -s|--session) SESSION_NAME="$2"; shift 2;;
            -a|--attach) ATTACH_AUTO=true; shift;;
            -d|--detach|--no-attach) ATTACH_AUTO=false; shift;;
            --force-recreate) FORCE_RECREATE=true; shift;;
            --container) CONTAINER_NAME="$2"; shift 2;;
            --start-container) START_CONTAINER=true; shift;;
            --no-overlay-source) SOURCE_OVERLAY=false; shift;;
            --no-micro-ros) DO_MICRO_ROS=false; shift;;
            --no-pressure-sensor) DO_PRESSURE_SENSOR=false; shift;;
            --pressure-control) DO_PRESSURE_CONTROL=true; shift;;
            --no-pressure-control) DO_PRESSURE_CONTROL=false; shift;;
            --no-leds) DO_LEDS=false; shift;;
            --no-cameras) DO_CAMERAS=false; shift;;
            --no-endoscope) DO_ENDOSCOPE=false; shift;;
            --no-pi-cam) DO_PI_CAM=false; shift;;
            --endo-params) ENDO_PARAMS="$2"; shift 2;;
            --no-auto-config-endo) AUTO_CONFIG_ENDO=false; shift;;
            --yes|-y) ASSUME_YES=true; shift;;
            -h|--help) usage;;
            *) error "Unknown option: $1"; usage;;
        esac
    done
}

# Parse arguments now (previously missing call)
parse_args "$@"

info "Using container: $CONTAINER_NAME"
info "Component selection:" \
    " micro_ros=$DO_MICRO_ROS" \
    " pressure_sensor=$DO_PRESSURE_SENSOR" \
    " pressure_control=$DO_PRESSURE_CONTROL" \
    " leds=$DO_LEDS" \
    " cameras=$DO_CAMERAS(pi=$DO_PI_CAM, endoscope=$DO_ENDOSCOPE)" \
    " endo_autoconfig=$AUTO_CONFIG_ENDO"
info "Overlay sourcing: $SOURCE_OVERLAY (file: $OVERLAY_SETUP_REL)"
if $START_CONTAINER; then
    if command -v docker >/dev/null; then
        info "Starting container via docker compose ..."
        docker compose up -d || { error "Failed to start container"; exit 1; }
    else
        error "docker not available to start container"; exit 1
    fi
fi

# Resolve actual container name (allow for compose project prefixes)
resolve_container_name() {
    local exact
    if exact=$(docker ps --format '{{.Names}}' | grep -Fx "$CONTAINER_NAME" || true); then
        if [[ -n "$exact" ]]; then echo "$exact"; return 0; fi
    fi
    # Try prefix/suffix patterns (compose may prepend project hash or directory)
    local match
    match=$(docker ps --format '{{.Names}}' | grep -E "(^|_)${CONTAINER_NAME}$" | head -n1 || true)
    if [[ -n "$match" ]]; then echo "$match"; return 0; fi
    return 1
}

ACTUAL_CONTAINER_NAME=$(resolve_container_name) || { error "Container matching '$CONTAINER_NAME' not running. Use --start-container or specify --container <name>."; exit 1; }
if [[ "$ACTUAL_CONTAINER_NAME" != "$CONTAINER_NAME" ]]; then
    info "Resolved container name: $ACTUAL_CONTAINER_NAME (from requested $CONTAINER_NAME)"
fi
CONTAINER_NAME="$ACTUAL_CONTAINER_NAME"

# Kill existing session if requested
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    if $FORCE_RECREATE; then
        info "Removing existing tmux session '$SESSION_NAME'"
        tmux kill-session -t "$SESSION_NAME"
    else
        error "Session '$SESSION_NAME' already exists. Use --force-recreate to kill it or attach manually."
        exit 1
    fi
fi

# Command wrapper
if $SOURCE_OVERLAY; then
    ENV_PREFIX='source /ros_env_setup.sh 2>/dev/null || true; if [ -f '"$OVERLAY_SETUP_REL"' ]; then source '"$OVERLAY_SETUP_REL"'; else echo "[overlay] missing $OVERLAY_SETUP_REL" >&2; fi; '
else
    ENV_PREFIX='source /ros_env_setup.sh 2>/dev/null || true; '
fi
# Always try to source exit_vine helper inside container panes
ENV_PREFIX+="[ -f $CONTAINER_FN_FILE ] && source $CONTAINER_FN_FILE; "
run_in_container() {
    local cmd="$1"
    # Always cd into workspace so relative paths (like ENDO_PARAMS) are valid
    printf "docker exec -it %q bash -lc %q" "$CONTAINER_NAME" "cd $CONTAINER_WORKSPACE; ${ENV_PREFIX}${cmd}"
}


create_session() {
    local core_cmd
    if [[ -f "$HOST_FN_FILE" ]]; then
        core_cmd="bash -lc 'source $HOST_FN_FILE 2>/dev/null || true; echo Hello from vine core; export SESSION_NAME=$SESSION_NAME CONTAINER_NAME=$CONTAINER_NAME; exec bash'"
    else
        core_cmd="bash -lc 'echo Hello from vine core; export SESSION_NAME=$SESSION_NAME CONTAINER_NAME=$CONTAINER_NAME; exec bash'"
    fi
    tmux new-session -d -s "$SESSION_NAME" -n core "$core_cmd"
    tmux set-environment -t "$SESSION_NAME" SESSION_NAME "$SESSION_NAME"
    tmux set-environment -t "$SESSION_NAME" CONTAINER_NAME "$CONTAINER_NAME"
}

pane_cmd() {
    local target="$1"; shift
    tmux send-keys -t "$target" "$*" C-m
}

# Functions for creating windows
create_micro_ros_window() {
    if $DO_MICRO_ROS; then
        tmux new-window -t "$SESSION_NAME" -n micro_ros
        CMD=$(cat <<EOF
FIND="$FIND_PICO_SCRIPT"
if [[ -x \$FIND ]]; then
    DEV=\$(\$FIND --first || true)
    if [[ -n \$DEV ]]; then
        echo "[micro_ros] Starting on \$DEV"
        ros2 run micro_ros_agent micro_ros_agent serial --dev \$DEV || echo "[micro_ros] agent exited"
    else
        echo "[micro_ros] Pico not found"
    fi
else
    echo "[micro_ros] Finder script missing at \$FIND"
fi
echo "[micro_ros] pane idle"; exec bash
EOF
        )
        tmux send-keys -t "$SESSION_NAME:micro_ros" "$(run_in_container "$CMD")" C-m
    fi
}

create_pressure_window() {
    if $DO_PRESSURE_SENSOR || $DO_PRESSURE_CONTROL; then
        tmux new-window -t "$SESSION_NAME" -n pressure
        if $DO_PRESSURE_SENSOR; then
            CMD=$(cat <<'EOF'
echo '[pressure_sensor] starting'
ros2 run gravity_pressure_sensor pressure_reading_node || echo '[pressure_sensor] exited'
echo '[pressure_sensor] pane idle'; exec bash
EOF
            )
            pane_cmd "$SESSION_NAME:pressure" "$(run_in_container "$CMD")"
        else
            pane_cmd "$SESSION_NAME:pressure" "echo 'Pressure sensor disabled'; exec bash"
        fi
        if $DO_PRESSURE_CONTROL; then
            CMD=$(cat <<'EOF'
echo '[pressure_control] starting'
ros2 run pressure_control pressure_control_node || echo '[pressure_control] exited'
echo '[pressure_control] pane idle'; exec bash
EOF
            )
            tmux split-window -h -t "$SESSION_NAME:pressure" "$(run_in_container "$CMD")"
        else
            tmux split-window -h -t "$SESSION_NAME:pressure" "bash -lc 'echo Pressure control disabled; exec bash'"
        fi
    else
        tmux new-window -t "$SESSION_NAME" -n pressure "bash -lc 'echo Pressure sensing and control disabled; exec bash'"
    fi
}

create_led_window() {
    if $DO_LEDS; then
        tmux new-window -t "$SESSION_NAME" -n leds

        tmux split-window -h -t "$SESSION_NAME:leds"
        tmux split-window -v -t "$SESSION_NAME:leds.1"

        tmux select-pane -t "$SESSION_NAME:leds.1"
        pane_cmd "$SESSION_NAME:leds" "$(run_in_container "echo [vine_led] starting; ros2 run led_control_vine led_control_vine_node || echo [vine_led] exited; echo '[vine_led] pane idle'; exec bash")"
        tmux select-pane -t "$SESSION_NAME:leds.2"
        pane_cmd "$SESSION_NAME:leds" "$(run_in_container "echo '[chassis_led] starting'; ros2 run led_control_chassis led_control_chassis_node || echo '[chassis_led] exited'; echo '[chassis_led] pane idle'; exec bash")"
    fi
}

create_camera_window() {
    if $DO_CAMERAS; then
        tmux new-window -t "$SESSION_NAME" -n cameras

        # Create a 2x2 layout
        tmux split-window -h -t "$SESSION_NAME:cameras"
        tmux split-window -v -t "$SESSION_NAME:cameras.1"
        tmux split-window -v -t "$SESSION_NAME:cameras.2"

        # Top-left: Pi cam 0
        tmux select-pane -t "$SESSION_NAME:cameras.1"
        if $DO_PI_CAM; then
            CMD_PI0=$(cat <<'EOF'
echo '[pi_cam0] starting camera index 0'
ros2 run camera_ros camera_node --ros-args -p camera:=0 || echo '[pi_cam0] exited'
echo '[pi_cam0] pane idle'; exec bash
EOF
            )
            pane_cmd "$SESSION_NAME:cameras" "$(run_in_container "$CMD_PI0")"
        else
            pane_cmd "$SESSION_NAME:cameras" "echo 'Pi cam 0 disabled'; exec bash"
        fi

        # Top-right: Pi cam 1
        tmux select-pane -t "$SESSION_NAME:cameras.3"
        if $DO_PI_CAM; then
            CMD_PI1=$(cat <<'EOF'
echo '[pi_cam1] starting camera index 1'
ros2 run camera_ros camera_node --ros-args -p camera:=1 || echo '[pi_cam1] exited'
echo '[pi_cam1] pane idle'; exec bash
EOF
            )
            pane_cmd "$SESSION_NAME:cameras" "$(run_in_container "$CMD_PI1")"
        else
            pane_cmd "$SESSION_NAME:cameras" "bash -lc 'echo Pi cam 1 disabled'; exec bash'"
        fi

        # Bottom-left: viewer pane (split from top-left)
        tmux select-pane -t "$SESSION_NAME:cameras.2"
        pane_cmd "$SESSION_NAME:cameras" "$(run_in_container "echo '[viewer] rqt_image_view pane ready'; exec bash")"
        # Pre-type the command into the new active pane without executing
        tmux send-keys -t "$SESSION_NAME:cameras" "ros2 run rqt_image_view rqt_image_view"

        # Bottom-right: endoscope (split from top-right)
        tmux select-pane -t "$SESSION_NAME:cameras.4"
        if $DO_ENDOSCOPE; then
            CMD_ENDO=$(cat <<EOF
echo '[endoscope] pre-config'
if $AUTO_CONFIG_ENDO; then
    if [[ -x "$FIND_ENDO_SCRIPT" ]]; then
        ARGS='-m'
        $ASSUME_YES && ARGS+=" -y"
        [[ -f "$ENDO_PARAMS" ]] && ARGS+=" -p $ENDO_PARAMS"
        bash "$FIND_ENDO_SCRIPT" \${ARGS:-} || echo '[endoscope] finder script failed'
    else
        echo '[endoscope] finder script not executable: $FIND_ENDO_SCRIPT'
    fi
fi
echo '[endoscope] starting'
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file "$ENDO_PARAMS" || echo '[endoscope] exited'
echo '[endoscope] pane idle'; exec bash
EOF
            )
            pane_cmd "$SESSION_NAME:cameras" "$(run_in_container "$CMD_ENDO")"
        else
            pane_cmd "$SESSION_NAME:cameras" "bash -lc 'echo Endoscope disabled; exec bash'"
        fi

        # Select a default layout to balance the panes
        tmux select-layout -t "$SESSION_NAME:cameras" tiled
    fi
}

# Create initial tmux session and panes
create_session

# Window 0 (core)
# Leave as is

# Window 1 (micro-ROS agent)
create_micro_ros_window

# Window 2 (pressure sensing and control)
create_pressure_window

# Window 3 (LED control)
create_led_window

# Window 4 (Cameras)
create_camera_window

# Switch to core window
tmux select-window -t "$SESSION_NAME:core"

info "Created tmux session '$SESSION_NAME'."

if $ATTACH_AUTO && [[ -t 1 ]]; then
  exec tmux attach -t "$SESSION_NAME"
else
  info "Attach with: tmux attach -t $SESSION_NAME"
fi
