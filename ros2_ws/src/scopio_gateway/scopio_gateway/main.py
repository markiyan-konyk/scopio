"""Entry point (`ros2 run scopio_gateway gateway`): rclpy in a thread, uvicorn on the main thread."""

import os
import traceback

import uvicorn

from .app import app
from .ros_bridge import bridge


def main():
    host = os.environ.get("SCOPIO_GATEWAY_HOST", "0.0.0.0")
    port = int(os.environ.get("SCOPIO_GATEWAY_PORT", "8000"))

    # Serve even without ROS: raising would crash-loop the container, and /health could not say why.
    try:
        bridge.start()
    except Exception:
        traceback.print_exc()
        print("Gateway starting WITHOUT ROS: /health will report ros_ok=false.",
              flush=True)
    try:
        uvicorn.run(app, host=host, port=port, log_level="info")
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
