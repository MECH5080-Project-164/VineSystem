#!/bin/bash

set -euo pipefail

# Script to start the Docker container for Vine System
# Usage: ./start_docker.sh [OPTIONS]

CONTAINER_NAME=${CONTAINER_NAME:-vine-docker}

start_container() {
    docker exec -it "$CONTAINER_NAME" bash
}

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Start the Vine System Docker container"
    echo ""
    echo "OPTIONS:"
    echo "  --name, -n NAME     Set the name of the Docker container (default: vine-docker)"
    echo "  --help, -h          Show this help message"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --name|-n)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "Error: --name requires a non-empty argument."
                show_help
                exit 1
            fi
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker to run this script."
    exit 1
fi

start_container
