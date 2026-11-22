#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ ! -x build/auto_aim_test ]]; then
    printf '%s\n' 'Missing build/auto_aim_test. Build the local-demo preset first.' >&2
    exit 1
fi

exec env -i \
    HOME="$HOME" USER="${USER:-liu}" PATH=/usr/bin:/bin \
    DISPLAY="${DISPLAY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
    XAUTHORITY="${XAUTHORITY:-}" QT_QPA_PLATFORM=xcb \
    ./build/auto_aim_test "$@"