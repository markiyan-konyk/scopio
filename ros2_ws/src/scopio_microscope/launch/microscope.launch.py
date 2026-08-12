"""Bring up the full SCOPIO microscope sensor/effectuator graph.

    ros2 launch scopio_microscope microscope.launch.py

All driver nodes start under the /scopio namespace and load their parameters
from config/params.yaml. Each node degrades gracefully if its hardware is
absent, so the graph comes up even on an incomplete rig.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def build_stamp():
    """When the image running these nodes was built (see Dockerfile).

    Printed first, every launch, because the nodes run from the IMAGE and not
    from the mounted repo: without this, `docker compose up -d` after an edit
    silently reruns the old code and the log is indistinguishable from a fix
    that did not work. If this timestamp predates your edit, rebuild.
    """
    try:
        with open("/ros2_ws/BUILD_STAMP", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return "unknown (not running from the built image)"


def generate_launch_description():
    pkg_share = get_package_share_directory("scopio_microscope")
    params = os.path.join(pkg_share, "config", "params.yaml")

    common = dict(package="scopio_microscope", namespace="scopio", output="screen",
                  parameters=[params])

    return LaunchDescription([
        LogInfo(msg=f"SCOPIO image built {build_stamp()} "
                    "-- older than your last edit? `docker compose up -d --build`"),
        Node(executable="calibration_node", name="calibration_node", **common),
        Node(executable="camera_node", name="camera_node", **common),
        Node(executable="stage_node", name="stage_node", **common),
        Node(executable="galvo_node", name="galvo_node", **common),
        Node(executable="temperature_node", name="temperature_node", **common),
        Node(executable="relay_node", name="relay_node", **common),
    ])
