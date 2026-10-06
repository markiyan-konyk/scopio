#!/bin/bash
# Install the camera server as a systemd unit, the fallback when libcamera fails in the container (see README.md).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
UNIT=/etc/systemd/system/scopio-camera.service

sed "s|__REPO__|$REPO|" "$REPO/camera_server/scopio-camera.service" > "$UNIT"
systemctl daemon-reload
systemctl enable --now scopio-camera.service

echo "Installed + started. Check:"
echo "  systemctl status scopio-camera"
echo "  curl http://127.0.0.1:8081/controls"
