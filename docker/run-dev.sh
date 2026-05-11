#!/usr/bin/env bash
# Build (once) and enter the wfb-ng dev container.
#
# Usage:
#   docker/run-dev.sh               # interactive shell
#   docker/run-dev.sh make test     # run a command inside the container
#   REBUILD=1 docker/run-dev.sh     # force rebuild the image
#
# The project root is bind-mounted at /workspace. Files created inside the
# container are owned by the host invoker's UID/GID (see docker/entrypoint.sh).
# --privileged is used because the project's test suite exercises TUN/TAP
# and sysctl.

set -euo pipefail

IMAGE=${IMAGE:-wfb-ng-dev:latest}
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [[ -n "${REBUILD:-}" ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker build --network host -t "$IMAGE" -f "$REPO_ROOT/docker/Dockerfile.dev" "$REPO_ROOT/docker"
fi

# Persist Claude Code auth + history in ./.claude under the repo root, so the
# same state is visible to the host and to the container.
CLAUDE_STATE_DIR=${CLAUDE_STATE_DIR:-$REPO_ROOT/.claude}
mkdir -p "$CLAUDE_STATE_DIR"

exec docker run --rm -it \
    --privileged \
    --network host \
    -e HOST_UID="$(id -u)" \
    -e HOST_GID="$(id -g)" \
    -e HOST_USER="$(id -un)" \
    -v "$REPO_ROOT":/workspace \
    -v "$CLAUDE_STATE_DIR":/claude-state \
    -w /workspace \
    "$IMAGE" "$@"
