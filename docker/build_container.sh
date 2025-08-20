#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Robust Docker buildx wrapper for this repo.
# Features:
# - checks that `docker` is available and usable
# - checks for `buildx` and warns if missing
# - supports configurable tag, platform, context
# - supports --no-push (load locally) and --no-cache
# - will create a temporary buildx builder if none is available

TAG="ninjapig40/vine_system:latest"
PLATFORM="linux/arm64"
CONTEXT="."
PUSH=1
NO_CACHE=0
BUILDER_NAME=""

print_usage() {
	cat <<EOF
Usage: ${0##*/} [options]

Options:
	-t TAG           Image tag (default: $TAG)
	-p PLATFORM      Build platform (default: $PLATFORM)
	-c CONTEXT       Build context (default: $CONTEXT)
	--no-push        Do not push the resulting image (loads into local docker)
	--no-cache       Disable build cache
	-b BUILDER       Use or create a named buildx builder
	-h|--help        Show this help

Examples:
	${0##*/} -t myname/vine:dev -p linux/arm64 --no-push
EOF
}

while [[ ${#@} -gt 0 ]]; do
	case "$1" in
		-t) TAG="$2"; shift 2;;
		-p) PLATFORM="$2"; shift 2;;
		-c) CONTEXT="$2"; shift 2;;
		--no-push) PUSH=0; shift;;
		--no-cache) NO_CACHE=1; shift;;
		-b) BUILDER_NAME="$2"; shift 2;;
		-h|--help) print_usage; exit 0;;
		--) shift; break;;
		-*) echo "Unknown option: $1" >&2; print_usage; exit 2;;
		*) break;;
	esac
done

log() { printf '%s\n' "$*"; }

# Check docker command
if ! command -v docker >/dev/null 2>&1; then
	echo "Error: docker is not installed or not on PATH." >&2
	exit 2
fi

# Determine if we need sudo to run docker
SUDO=""
if ! docker info >/dev/null 2>&1; then
	if sudo -n docker info >/dev/null 2>&1; then
		SUDO="sudo"
		log "docker requires sudo; commands will be prefixed with sudo"
	else
		echo "Error: docker is installed but not usable by this user (and sudo is not available without a password)." >&2
		echo "Either add your user to the docker group or run this script with privileges." >&2
		exit 3
	fi
fi

# Check buildx
if ! docker buildx version >/dev/null 2>&1; then
	log "Warning: docker buildx not found or not functional. You may need to enable buildx or upgrade Docker."
fi

CLEANUP_BUILDER=0
if [[ -z "$BUILDER_NAME" ]]; then
	# try to use the default builder if it exists
	if $SUDO docker buildx inspect default >/dev/null 2>&1; then
		BUILDER_NAME=default
	else
		# create a temporary builder for multi-platform builds
		BUILDER_NAME="vine_builder_$(id -un)_$(date +%s)"
		log "Creating temporary buildx builder: $BUILDER_NAME"
		$SUDO docker buildx create --name "$BUILDER_NAME" --use >/dev/null
		CLEANUP_BUILDER=1
	fi
else
	# ensure requested builder exists (create if not)
	if ! $SUDO docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
		log "Builder '$BUILDER_NAME' not found; creating and using it"
		$SUDO docker buildx create --name "$BUILDER_NAME" --use >/dev/null
		CLEANUP_BUILDER=1
	else
		$SUDO docker buildx use "$BUILDER_NAME"
	fi
fi

cleanup() {
	if [[ $CLEANUP_BUILDER -eq 1 ]]; then
		log "Removing temporary buildx builder: $BUILDER_NAME"
		$SUDO docker buildx rm "$BUILDER_NAME" || true
	fi
}
trap cleanup EXIT

log "Building tag=$TAG platform=$PLATFORM context=$CONTEXT push=$PUSH no_cache=$NO_CACHE builder=$BUILDER_NAME"

BUILD_CMD=(docker buildx build --platform "$PLATFORM" -t "$TAG")
if [[ $NO_CACHE -eq 1 ]]; then
	BUILD_CMD+=(--no-cache)
fi
if [[ $PUSH -eq 1 ]]; then
	BUILD_CMD+=(--push)
else
	# load the resulting image into local docker if not pushing
	BUILD_CMD+=(--load)
fi
BUILD_CMD+=("$CONTEXT")

if [[ -n "$SUDO" ]]; then
	# run with sudo as a single command
	log "Running: sudo ${BUILD_CMD[*]}"
	sudo "${BUILD_CMD[@]}"
else
	log "Running: ${BUILD_CMD[*]}"
	"${BUILD_CMD[@]}"
fi

log "Build finished."
