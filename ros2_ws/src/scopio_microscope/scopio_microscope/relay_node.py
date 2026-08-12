#!/usr/bin/env python3
"""ROS 2 node controlling the laser relay on BCM GPIO17."""

import threading

import rclpy
from gpiozero import OutputDevice
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class RelayNode(Node):
    def __init__(self):
        super().__init__("relay_node")

        self.declare_parameter("gpio_pin", 17) # Do not use 14,15, 23-25. they are used by sangaboard
        self.declare_parameter("active_high", True)

        gpio_pin = int(self.get_parameter("gpio_pin").value)
        active_high = bool(self.get_parameter("active_high").value)

        self._lock = threading.Lock()
        self._relay = None
        self._state = False

        # Retain the latest state for clients that subscribe later.
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._state_pub = self.create_publisher(
            Bool,
            "relay/state",
            state_qos,
        )

        self._set_service = self.create_service(
            SetBool,
            "relay/set",
            self._set_relay,
        )

        try:
            # gpiozero uses BCM numbering:
            self._relay = OutputDevice(
                gpio_pin,
                active_high=active_high,
                initial_value=False,
            )

            self._publish_state()
            self.get_logger().info(
                f"Relay ready on BCM GPIO{gpio_pin}; initial state OFF"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Could not initialize relay on BCM GPIO{gpio_pin}: {exc}"
            )

    def _publish_state(self):
        message = Bool()
        message.data = self._state
        self._state_pub.publish(message)

    def _set_relay(self, request, response):
        """Set an explicit relay state using std_srvs/SetBool."""

        with self._lock:
            if self._relay is None:
                response.success = False
                response.message = "Relay GPIO is unavailable"
                return response

            try:
                if request.data:
                    self._relay.on()
                else:
                    self._relay.off()

                self._state = bool(request.data)
                self._publish_state()

                response.success = True
                response.message = (
                    "Relay ON" if self._state else "Relay OFF"
                )

                self.get_logger().info(response.message)

            except Exception as exc:
                # Attempt a safe shutdown if GPIO control fails.
                try:
                    self._relay.off()
                except Exception:
                    pass

                self._state = False
                self._publish_state()

                response.success = False
                response.message = f"Relay operation failed: {exc}"
                self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        """Always leave the relay switched off."""

        with self._lock:
            if self._relay is not None:
                try:
                    self._relay.off()
                    self._state = False
                    self._publish_state()
                    self._relay.close()
                    self.get_logger().info("Relay switched OFF during shutdown")
                except Exception as exc:
                    self.get_logger().error(
                        f"Relay shutdown failed: {exc}"
                    )
                finally:
                    self._relay = None

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
