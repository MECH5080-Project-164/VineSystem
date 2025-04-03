#!/bin/bash

# --- Configuration ---
IMAGE_NAME="vine_system:latest"     # Docker image name
CONTAINER_NAME="ros2_container"     # Container name
MOUNT_POINT="/home/workspace"       # Where to mount inside the container
BIND_DIR="/home/$(whoami)"          # Host directory to bind mount

# --- Functions ---

show_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help        Show this help message"
    echo "  -n, --name NAME   Set the container name (default: $CONTAINER_NAME)"
    echo "  -c, --clean       Remove existing container before starting a new one"
    echo
}

check_docker() {
    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        echo "Docker is not installed. Please install Docker first."
        exit 1
    fi

    # Check if Docker daemon is running
    if ! docker info &> /dev/null; then
        echo "Error: Docker daemon is not running. Please start Docker service."
        exit 1
    fi
}

setup_x11() {
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding for GUI applications."

        # Allow local connections to the X server
        if command -v xhost &> /dev/null; then
            xhost +local:docker &> /dev/null || true
        fi

        X11_ARGS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
        return 0
    else
        echo "NOTE: DISPLAY environment variable not set. Skipping X11 forwarding setup."
        echo "      Run 'xhost +local:docker' on the host if you need GUI support later."
        return 1
    fi
}

container_start() {
    echo "Starting a new container with name $CONTAINER_NAME."

    # Prepare X11 for GUI applications
    setup_x11
    if [ $? -ne 0 ]; then
        echo "X11 setup failed. GUI applications may not work."
    fi

    # Check if user is in the docker group to avoid sudo
    if groups | grep -q '\bdocker\b'; then
        DOCKER_CMD="docker"
    else
        echo "Note: You're not in the docker group. Using sudo for docker commands."
        DOCKER_CMD="sudo docker"
    fi

    # Run the Docker container
    echo "-> Mounting host home '$BIND_DIR' to '$MOUNT_POINT' inside container."
    echo "-> Using --privileged for device access (USB, GPIO)."
    echo "-> Using --net=host for ROS 2 networking."

    CONTAINER_ID=$($DOCKER_CMD run \
        -dit \
        --name "$CONTAINER_NAME" \
        --privileged \
        --net=host \
        -v "${BIND_DIR}:${MOUNT_POINT}:rw" \
        -v /dev:/dev \
        -v /run/udev/:/run/udev/ \
        -v /dev:/dev \
        $X11_ARGS \
        -e TERM=xterm-256color \
        -e VINE_CONTAINER=1 \
        --hostname vine-docker \
        --add-host vine-docker:127.0.1.1 \
        "$IMAGE_NAME")

    if [ $? -ne 0 ]; then
        echo "Failed to start the container. Please check the Docker image and configuration."
        exit 1
    fi

    if [ -z "$CONTAINER_ID" ]; then
        echo "Failed to retrieve container ID."
        exit 1
    fi

    SHORT_CONTAINER_ID=${CONTAINER_ID:0:12}
    echo "Container started successfully with ID: $SHORT_CONTAINER_ID"
    echo "Attaching to the container"
    $DOCKER_CMD exec -it "$CONTAINER_ID" bash

}

container_remove() {
    if [ -n "$1" ]; then
        echo "Removing container $1"
        docker rm -f "$1" > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "Container $1 removed successfully."
        else
            echo "Failed to remove container $1."
        fi
    fi
}

# --- Arguments ---
CLEAN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_help
            exit 0
            ;;
        -n|--name)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "Error: --name requires a non-empty argument."
                show_help
                exit 1
            fi

            CONTAINER_NAME="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN=1
            shift
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
        *)
            break
            ;;
    esac
done

# --- Script ---
check_docker

if [ $CLEAN -eq 1 ]; then
    EXISTING_CONTAINER=$(docker ps -aq -f name=^/${CONTAINER_NAME}$)
    if [ -n "$EXISTING_CONTAINER" ]; then
        container_remove "$EXISTING_CONTAINER"
    fi
fi

# Check if a container with the same name already exists
RUNNING_CONTAINER_ID=$(docker ps -q -f name=^/${CONTAINER_NAME}$)

if [ -n "$RUNNING_CONTAINER_ID" ]; then
    echo "Attaching to existing container $CONTAINER_NAME (ID: $RUNNING_CONTAINER_ID)..."
    docker exec -it $RUNNING_CONTAINER_ID bash
else
    echo "No running container found with the name $CONTAINER_NAME."
    # Check if a stopped container with the same name exists
    STOPPED_CONTAINER_ID=$(docker ps -aq -f name=^/${CONTAINER_NAME}$)
    if [ -n "$STOPPED_CONTAINER_ID" ] && [ $CLEAN -eq 0 ]; then
        echo "Starting stopped container $CONTAINER_NAME (ID: $STOPPED_CONTAINER_ID)..."
        docker start $STOPPED_CONTAINER_ID
        docker exec -it $STOPPED_CONTAINER_ID bash
        exit 0
    else
        # No container found, start a new one
        container_start
    fi
fi

echo "Script finished. Exited container shell."
