#!/usr/bin/env bash
# Entrypoint for the wfb-ng dev container.
#
# Creates (on first start) a user whose UID/GID match the host user passed in
# via HOST_UID / HOST_GID, so that files produced inside /workspace appear on
# the host with the invoker's ownership instead of root's.

set -euo pipefail

HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
HOST_USER=${HOST_USER:-dev}

# --- group ---------------------------------------------------------------
if ! getent group "$HOST_GID" >/dev/null; then
    groupadd -g "$HOST_GID" "$HOST_USER" 2>/dev/null || groupadd -g "$HOST_GID" "${HOST_USER}grp"
fi
GROUP_NAME=$(getent group "$HOST_GID" | cut -d: -f1)

# --- user ----------------------------------------------------------------
if ! getent passwd "$HOST_UID" >/dev/null; then
    # Pick a name that's not already taken (e.g. "ubuntu" exists on some bases)
    NAME="$HOST_USER"
    if getent passwd "$NAME" >/dev/null; then NAME="${HOST_USER}$HOST_UID"; fi
    useradd -m -u "$HOST_UID" -g "$HOST_GID" -s /bin/bash "$NAME"
fi
USER_NAME=$(getent passwd "$HOST_UID" | cut -d: -f1)
USER_HOME=$(getent passwd "$HOST_UID" | cut -d: -f6)

# passwordless sudo — this is a dev container, convenience over hardening
if [ ! -f /etc/sudoers.d/99-dev ]; then
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/99-dev
    chmod 0440 /etc/sudoers.d/99-dev
fi

# Persist Claude Code state (auth, history, projects, todos) across container
# runs by routing both ~/.claude (settings/history) and ~/.claude.json
# (OAuth credentials + per-project config) into the bind-mounted /claude-state.
# Seed settings.json on first use; never clobber existing data.
if [ -d /claude-state ]; then
    chown "$HOST_UID:$HOST_GID" /claude-state 2>/dev/null || true

    # ~/.claude  ->  /claude-state
    rm -rf "$USER_HOME/.claude"
    ln -s /claude-state "$USER_HOME/.claude"
    chown -h "$HOST_UID:$HOST_GID" "$USER_HOME/.claude"

    # ~/.claude.json  ->  /claude-state/.claude.json   (this is where login tokens live)
    if [ ! -e /claude-state/.claude.json ]; then
        install -m 0600 -o "$HOST_UID" -g "$HOST_GID" /dev/null /claude-state/.claude.json
    fi
    rm -f "$USER_HOME/.claude.json"
    ln -s /claude-state/.claude.json "$USER_HOME/.claude.json"
    chown -h "$HOST_UID:$HOST_GID" "$USER_HOME/.claude.json"

    if [ ! -f /claude-state/settings.json ]; then
        install -m 0644 -o "$HOST_UID" -g "$HOST_GID" \
            /etc/claude-settings.json /claude-state/settings.json
    fi
else
    # No persistence requested — fall back to an ephemeral in-container dir
    install -d -o "$HOST_UID" -g "$HOST_GID" "$USER_HOME/.claude"
    if [ ! -f "$USER_HOME/.claude/settings.json" ]; then
        install -m 0644 -o "$HOST_UID" -g "$HOST_GID" \
            /etc/claude-settings.json "$USER_HOME/.claude/settings.json"
    fi
fi

export HOME="$USER_HOME"
cd /workspace 2>/dev/null || true

if [ "$#" -eq 0 ]; then
    exec gosu "$USER_NAME" /bin/bash
else
    exec gosu "$USER_NAME" "$@"
fi
