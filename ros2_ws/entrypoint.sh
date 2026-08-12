#!/bin/bash
# Source ROS 2 and the SCOPIO workspace, then run the command.
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi
exec "$@"
