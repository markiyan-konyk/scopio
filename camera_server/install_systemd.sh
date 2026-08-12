#!/bin/bash
# Install the SCOPIO camera server as a systemd unit on the Pi host.
# FALLBACK path -- use only if the `camera` docker compose service fails
# (libcamera/kernel mismatch inside the container). Run ON THE PI:
#
#   cd camera_server && sudo ./install_systemd.sh
#
# Then disable the compose service:  docker compose stop camera  (and comment
# it out of ros2_ws/docker-compose.yml so `up` doesn't restart it).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
UNIT=/etc/systemd/system/scopio-camera.service

sed "s|__REPO__|$REPO|" "$REPO/camera_server/scopio-camera.service" > "$UNIT"
systemctl daemon-reload
systemctl enable --now scopio-camera.service

echo "Installed + started. Check:"
echo "  systemctl status scopio-camera"
echo "  curl http://127.0.0.1:8081/controls"
