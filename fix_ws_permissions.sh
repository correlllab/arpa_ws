#!/bin/bash
# Fix workspace ownership so you can build without sudo.
# Run once: sudo ./fix_ws_permissions.sh

set -e
WS="${1:-/home/the2xman/arpa_ws}"
USER="${SUDO_USER:-$USER}"
if [ -z "$USER" ] || [ "$USER" = root ]; then
  echo "Run as: sudo -E ./fix_ws_permissions.sh"
  exit 1
fi
echo "Chown $WS/build $WS/install $WS/log to $USER"
chown -R "$USER:$USER" "$WS/build" "$WS/install" "$WS/log" 2>/dev/null || true
echo "Done. You can now run: colcon build --symlink-install --packages-select arpa_control arpa_behavior_trees"
