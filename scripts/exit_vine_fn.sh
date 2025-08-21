#!/usr/bin/env bash
# exit_vine_fn.sh
# Helper that provides the exit_vine function so any pane in the Vine tmux
# session can cleanly tear everything down.

# Prevent double sourcing
if [[ -n "${VINE_EXIT_FUNCTIONS_LOADED:-}" ]]; then
  return 0
fi
VINE_EXIT_FUNCTIONS_LOADED=1

exit_vine() {
  local session_name container_name stop_container=false force=false ask=true
  session_name="${SESSION_NAME:-vine}"
  container_name="${CONTAINER_NAME:-vine-docker}"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --stop-container) stop_container=true; shift ;;
      --force|-f|--no-ask|-y) force=true; ask=false; shift ;;
      -h|--help)
        cat <<EOF
Usage: exit_vine [options]

Gracefully tear down the Vine tmux session and (optionally) the Docker container.

Options:
  --stop-container    Also stop the docker container (default: leave running)
  -f, --force, -y     Do not prompt for confirmation (immediate)
  --no-ask            Alias for --force
  -h, --help          Show this help message

Examples:
  exit_vine
  exit_vine --stop-container
  exit_vine -f --stop-container
EOF
        return 0 ;;
      *) echo "[exit_vine] Unknown option: $1" >&2; return 2 ;;
    esac
  done

  if ! command -v tmux >/dev/null 2>&1; then
    echo "[exit_vine] tmux not available in this shell. Run from a host pane (core)." >&2
    return 1
  fi

  if $ask && ! $force; then
    echo "[exit_vine] Session=$session_name Container=$container_name StopContainer=$stop_container"
    read -r -p "Proceed with shutdown? [y/N] " answer
    [[ $answer =~ ^[Yy]$ ]] || { echo "[exit_vine] Aborted"; return 1; }
  fi

  echo "[exit_vine] Attempting graceful ROS process stop inside container" >&2
  if command -v docker >/dev/null 2>&1; then
    docker exec -it "$container_name" bash -lc 'pkill -f ros2 || true; pkill -f micro_ros_agent || true' >/dev/null 2>&1 || true
  fi

  if $stop_container; then
    if command -v docker >/dev/null 2>&1; then
      echo "[exit_vine] Stopping container $container_name" >&2
      docker stop "$container_name" >/dev/null 2>&1 || echo "[exit_vine] Container already stopped" >&2
    else
      echo "[exit_vine] docker command unavailable; cannot stop container" >&2
    fi
  fi

  echo "[exit_vine] Killing tmux session $session_name" >&2
  tmux kill-session -t "$session_name" 2>/dev/null || echo "[exit_vine] Session already gone" >&2
}

# Provide a shorter alias
alias vine-exit=exit_vine
