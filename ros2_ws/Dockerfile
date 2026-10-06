# SCOPIO nodes and gateway: ROS 2 Jazzy on Ubuntu 24.04. The build layers are explained in README.md.
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
# libusb-1.0-0: without it pyvisa-py cannot enumerate USB instruments (the AWG and the TC10).
# No picamera2/libcamera: they exist only for Raspberry Pi OS, so the camera has its own container.

# Not yet version-pinned. --break-system-packages: Noble's Python is PEP-668 managed; fine in a container.
RUN pip3 install --no-cache-dir --break-system-packages \
        pyvisa \
        pyvisa-py \
        pyusb \
        pyserial \
        libusb-package \
        sangaboard \
        fastapi \
        "uvicorn[standard]" \
        httpx
# pyserial is explicit: pyvisa-py needs it for serial instruments, not only sangaboard.

# ---- The workspace, in layers ordered by how rarely they change ----------------
# The nodes run from this image, never from the repo: after an edit, `docker compose up -d --build`.
WORKDIR /ros2_ws
# 1. Interfaces alone: by far the slowest compile; -j2 so the Pi does not swap to the SD card.
COPY src/scopio_interfaces /ros2_ws/src/scopio_interfaces
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS=-j2 colcon build --symlink-install --packages-select scopio_interfaces

# 2. Nodes and gateway; --packages-skip scopio_interfaces reuses layer 1. The stamp is printed at launch.
COPY src /ros2_ws/src
RUN date -u +'%Y-%m-%dT%H:%M:%SZ' > /ros2_ws/BUILD_STAMP
RUN source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash \
    && colcon build --symlink-install --packages-skip scopio_interfaces

# 3. Scripts last; in the container run them by absolute path (the working directory is /data).
COPY scripts /ros2_ws/scripts

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "scopio_microscope", "microscope.launch.py"]
