# Camera server image: Debian bookworm + the Raspberry Pi apt archive, the only source of picamera2 (see README.md).
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
