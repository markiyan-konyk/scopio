# SCOPIO microscope ROS 2 node, all-in-one container.
#
# Base: ROS 2 Jazzy (Ubuntu 24.04 Noble). The container owns the camera
# (picamera2/libcamera), the galvo (USB) and the stage (serial), so it runs
# --privileged with host networking and device passthrough (see compose).
#
# The libcamera/picamera2 apt packages must match the Pi's kernel/firmware --
# this is the one piece to verify on the Pi (see README "Camera caveat"). If the
# apt package is unavailable on your image, the camera node degrades gracefully
# (the rest of the graph still comes up) until it's sorted.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-opencv \
        python3-gpiozero \
        python3-lgpio \
        ffmpeg \
        libusb-1.0-0 \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-srvs \
    && rm -rf /var/lib/apt/lists/*
# libusb-1.0-0 is REQUIRED for pyvisa-py/pyusb to enumerate USB-TMC instruments
# (the Rigol AWG that steers the galvo, and the TC LAB temperature controller
# when it is on USB rather than Ethernet). Without it the container sees only
# serial ports and those nodes report unavailable -- the host has libusb so the
# standalone bench scripts work there, but the Ubuntu container needs it
# installed explicitly.

# NOTE: picamera2 / libcamera are NOT in Ubuntu's ARM repos -- they ship from
# Raspberry Pi OS (Debian). So they are intentionally NOT installed here. The
# camera is owned by the separate `camera` compose service (camera_server/,
# Debian bookworm + the Raspberry Pi apt archive) and camera_node falls back to
# BRIDGE MODE: it ingests the camera server's MJPEG stream over loopback HTTP,
# republishes frames on image/compressed, and forwards the camera services to
# the camera server's HTTP API. See camera_server/README.md.

# Python deps for the hardware drivers. --break-system-packages because Noble's
# Python is PEP-668 "externally managed"; fine inside a container.
# NOTE: no Flask here -- this image is the headless BACKEND. The web UI is a
# separate app (../ui) that talks to the gateway over plain HTTP.
# NO image-analysis stack either (no trackpy/pandas/scipy): the backend senses,
# streams and effectuates -- it never analyses frames. Bead detection/tracking
# runs on the CLIENT side, off the Pi (viscosity/, viscosity_agent/). Keeping it
# out also keeps the image off the apt-numpy-vs-pip-numpy collision entirely.
RUN pip3 install --no-cache-dir --break-system-packages \
        simplejpeg \
        pyvisa \
        pyvisa-py \
        pyusb \
        pyserial \
        libusb-package \
        sangaboard \
        fastapi \
        "uvicorn[standard]" \
        httpx
# fastapi/uvicorn/httpx power the scopio_gateway package (the HTTP/WebSocket
# API that external clients use instead of joining the DDS graph). They are
# pure-python and light -- fine on the Pi.
# pyserial is listed EXPLICITLY even though sangaboard pulls it in: pyvisa-py
# cannot enumerate or open a single ASRL (serial / USB virtual-COM) instrument
# without it, and inheriting that from another package's dependency tree is how
# you end up debugging an instrument that "VISA can't see" for an afternoon.

# Build the workspace (interfaces + nodes) into the image.
#
# THE NODES RUN FROM THIS IMAGE, not from the repo mounted at /workspace. The
# mount only supplies the working directory (calibration.json). So editing a
# node on the Pi and running `docker compose up -d` changes NOTHING -- you need
# `--build`. That failure is silent and looks exactly like a broken fix, so the
# build is stamped and the stamp is printed at launch. Docker caches this layer
# with the COPY above it: same source, same stamp; changed source, new stamp.
WORKDIR /ros2_ws
COPY . /ros2_ws/
RUN date -u +'%Y-%m-%dT%H:%M:%SZ' > /ros2_ws/BUILD_STAMP
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# The nodes import nothing from outside this workspace. The repo is still
# mounted at /workspace at runtime and used as the working directory, so
# relative paths (calibration.json) land on the Pi's real disk.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "scopio_microscope", "microscope.launch.py"]
