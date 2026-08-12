# SCOPIO camera server container (the `camera` service in ros2_ws/docker-compose.yml).
#
# picamera2/libcamera cannot run in the Ubuntu ROS container, so the camera
# gets its own tiny container: Debian bookworm (matching Raspberry Pi OS
# Bookworm's userland) + the Raspberry Pi apt archive, which ships
# python3-picamera2 with a libcamera built for the Pi's camera stack
# (vc4/unicam on Pi 4, pisp on Pi 5 -- the archive covers both).
#
# VALIDATE ON THE PI before trusting it (the one high-risk piece):
#   docker compose run --rm camera \
#     python3 -c "from picamera2 import Picamera2; print(Picamera2.global_camera_info())"
# If libcamera misbehaves in-container on your Pi, fall back to the systemd
# unit (./install_systemd.sh) -- identical HTTP surface, nothing else changes.
FROM debian:bookworm-slim

# Raspberry Pi apt archive (source of picamera2 + a Pi-compatible libcamera).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
        | gpg --dearmor -o /usr/share/keyrings/raspberrypi.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/raspberrypi.gpg] http://archive.raspberrypi.com/debian bookworm main" \
        > /etc/apt/sources.list.d/raspberrypi.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3-picamera2 \
    && apt-get purge -y curl gnupg && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

COPY pi_camera_server.py /app/pi_camera_server.py

# Loopback-only by default; the API gateway is the authenticated front door.
ENV CAM_HOST=127.0.0.1 CAM_PORT=8081

CMD ["python3", "/app/pi_camera_server.py"]
