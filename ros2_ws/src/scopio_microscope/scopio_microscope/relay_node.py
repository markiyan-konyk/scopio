#!/usr/bin/env python3
"""relay_node - the laser relay, a single GPIO pin on the Pi itself.

  srv  relay/set    std_srvs/SetBool   data=true -> laser ON
  pub  relay/state  std_msgs/Bool      latched, so late subscribers get it

Same shape as every other driver node here: the pin is not a precondition for
running. If gpiozero cannot claim it (the line is held by a leftover process,
/dev/gpiochip* not visible in the container yet) the node still comes up,
relay/set answers with the reason, and a retry timer keeps trying.

THE ONE RULE: NEVER PUBLISH `false` FOR A STATE WE DID NOT REACH. If a GPIO
call throws, the relay's real position is unknown, and "unknown" reported as
"off" is a green button next to a live laser. Unknown is published as ON until
an off() actually succeeds. Re-opening the device drives it off (initial_value
is the OFF state for either polarity), so recovery and the safe action are the
same operation.
"""

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

        # BCM numbering. Do NOT use 14, 15 or 23-25: the sangaboard has them.
        self.declare_parameter("gpio_pin", 17)
        self.declare_parameter("active_high", True)
        self.declare_parameter("reconnect_period", 10.0)

        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.active_high = bool(self.get_parameter("active_high").value)

        # RLock: _open/_release are reached from both the service callback and
        # the retry timer, and _set_relay calls _release while already holding it.
        self._lock = threading.RLock()
        self._relay = None
        self._state = False
        self._last_error = ""

        # Latched (TRANSIENT_LOCAL, depth 1): a UI that starts after this node
        # still learns whether the laser is on. Created BEFORE the pin is
        # claimed, so the topic exists in the graph even when the GPIO does not
        # -- clients subscribe to it unconditionally.
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(Bool, "relay/state", state_qos)

        self.create_service(SetBool, "relay/set", self._set_relay)

        self._open()
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_open)

    def _publish_state(self):
        self.state_pub.publish(Bool(data=self._state))

    def _open(self):
        """Claim the pin, OFF. Idempotent; returns whether we hold it."""
        with self._lock:
            if self._relay is not None:
                return True
            try:
                self._relay = OutputDevice(self.gpio_pin,
                                           active_high=self.active_high,
                                           initial_value=False)
            except Exception as exc:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(
                    f"Relay unavailable on BCM GPIO{self.gpio_pin}; node runs, "
                    f"relay/set reports the failure.\n  error: {exc}",
                    throttle_duration_sec=60.0)
                return False
            self._state = False
            self._last_error = ""
            self._publish_state()
        self.get_logger().info(f"Relay ready on BCM GPIO{self.gpio_pin}; OFF")
        return True

    def _retry_open(self):
        if self._relay is None:
            self._open()

    def _release(self):
        """Drop the device so the retry timer re-opens it (and drives it OFF)."""
        with self._lock:
            relay, self._relay = self._relay, None
            if relay is not None:
                try:
                    relay.close()
                except Exception:
                    pass

    def _set_relay(self, request, response):
        with self._lock:
            if self._relay is None:
                response.success = False
                response.message = (f"Relay GPIO{self.gpio_pin} is unavailable"
                                    + (f" ({self._last_error})" if self._last_error else ""))
                return response

            try:
                self._relay.on() if request.data else self._relay.off()
                self._state = bool(request.data)
                self._publish_state()
                response.success = True
                response.message = "Relay ON" if self._state else "Relay OFF"
                self.get_logger().info(response.message)
            except Exception as exc:
                # The pin threw: the relay could be in either position. Force it
                # off -- and report OFF only if that call itself succeeded.
                try:
                    self._relay.off()
                    self._state = False
                except Exception:
                    self._state = True   # unknown => assume energized
                    self._release()      # re-open on the retry timer forces OFF
                    self.get_logger().error(
                        "Relay is stuck; TREAT THE LASER AS ON until it recovers.")
                self._publish_state()
                response.success = False
                response.message = f"Relay operation failed: {exc}"
                self.get_logger().error(response.message)
            return response

    def destroy_node(self):
        """Always leave the relay switched off."""
        with self._lock:
            relay, self._relay = self._relay, None
            if relay is not None:
                # Hardware first, each step in its own try. Under `ros2 launch`,
                # SIGINT tears the rclpy context down before this runs, so the
                # publish below throws InvalidHandle -- sharing one try with
                # off()/close() would leave a laser energized over a failure
                # that has nothing to do with the laser.
                try:
                    relay.off()
                    self._state = False
                except Exception as exc:
                    self.get_logger().error(f"Relay would NOT switch off: {exc}")
                try:
                    relay.close()
                except Exception:
                    pass
                try:
                    self._publish_state()
                except Exception:
                    pass
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
