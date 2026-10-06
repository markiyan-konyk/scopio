"""temperature_node - the Wavelength TC10 LAB temperature controller, exposed whole (see ../README.md)."""

import math
import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scopio_interfaces.msg import TemperatureStatus
from scopio_interfaces.srv import InstrumentCall

from .connect_guard import ConnectGuard
from .drivers import dispatch
from .drivers.TC10LAB import TC10LAB

MAX_FAILURES = 3

META_METHODS = (
    {"name": "list_methods", "signature": "()",
     "doc": "List every method callable through this node."},
    {"name": "reconnect", "signature": "()",
     "doc": "Re-open the session to the controller; returns True on success."},
    {"name": "connected", "signature": "()",
     "doc": "Whether the node currently holds a live session."},
)


class TemperatureNode(Node):
    def __init__(self):
        super().__init__("temperature_node")
        self.declare_parameter("resource", "")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("timeout_ms", 5000)
        self.declare_parameter("reconnect_period", 15.0)
        self.declare_parameter("units", "C")

        # Held across _connect/_release: a service reconnect must not race the timer into two sessions.
        self._lock = threading.RLock()
        self.tc = None
        self.idn = ""
        self.last_error = ""
        self.state = {}
        self._failures = 0
        # A hard deadline per connect: a VISA open can hang in libusb (see connect_guard.py).
        timeout_s = int(self.get_parameter("timeout_ms").value) / 1000.0
        self._guard = ConnectGuard(3 * timeout_s + 5.0)

        self._connect()

        self.status_pub = self.create_publisher(TemperatureStatus, "temperature/status", 5)
        cb = ReentrantCallbackGroup()
        self.create_service(InstrumentCall, "temperature/call", self._on_call,
                            callback_group=cb)
        # Default (exclusive) group: a slow instrument cannot stack polls.
        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_status)
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_connect)

    def _connect(self):
        # .strip(): a trailing \r from a Windows editor makes a path that does not exist.
        resource = (os.environ.get("TCLAB_RESOURCE")
                    or self.get_parameter("resource").value or "").strip()
        timeout_ms = int(self.get_parameter("timeout_ms").value)
        units = str(self.get_parameter("units").value).strip()

        def attempt():
            tc = TC10LAB(resource, timeout_ms=timeout_ms)
            try:
                tc._open()     # picks the transport, clears, verifies *IDN?
            except BaseException:
                tc._close()
                raise
            # Units are only a label: best-effort, never a reason to fail a working link.
            try:
                tc.set_units(units) if units else tc.get_units()
            except Exception as exc:
                self.get_logger().warning(f"TC10 LAB units unavailable ({exc}).")
            return tc

        with self._lock:
            if self.tc is not None:
                return True
            try:
                tc = self._guard.run(attempt, discard=lambda t: t._close())
                self.tc, self.idn, self.last_error = tc, tc.identity, ""
                self._failures = 0
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                asked = repr(resource) if resource else "<auto: kernel usbtmc or VISA>"
                self.get_logger().warning(
                    f"TC10 LAB unavailable; node runs, reports connected=false.\n"
                    f"  tried:  {asked}\n"
                    f"  error:  {type(exc).__name__}: {exc}",
                    throttle_duration_sec=60.0)
                return False
        if tc.probe_note:
            self.get_logger().warning(f"TCLAB_RESOURCE {tc.probe_note}")
        self.get_logger().info(f"TC10 LAB connected on {tc.resource}: {tc.identity}")
        return True

    def _retry_connect(self):
        if self.tc is None:
            self._connect()

    def _release(self):
        with self._lock:
            tc, self.tc = self.tc, None
            if tc is None:
                return False
            tc._close()
            self._failures = 0
            return True

    def _note_failure(self, exc):
        """Record an I/O failure; drop the session only after MAX_FAILURES in a row."""
        self.last_error = str(exc)
        self._failures += 1
        if self._failures >= MAX_FAILURES and self._release():
            self.get_logger().warning(
                f"TC10 LAB dropped after {MAX_FAILURES} failures ({exc}); "
                "will reconnect.")

    def _publish_status(self):
        tc = self.tc
        if tc is not None:
            try:
                self.state = tc.status()
                self.last_error = ""
                self._failures = 0
            except Exception as exc:
                self.state = {}
                self._note_failure(exc)

        s = self.state
        msg = TemperatureStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self.tc is not None
        msg.idn = self.idn
        msg.temperature = float(s.get("temperature", math.nan))
        msg.setpoint = float(s.get("setpoint", math.nan))
        msg.current = float(s.get("current", math.nan))
        msg.voltage = float(s.get("voltage", math.nan))
        msg.output = bool(s.get("output", False))
        msg.in_tolerance = bool(s.get("in_tolerance", False))
        msg.units = str(s.get("units", ""))
        msg.condition = int(s.get("condition", 0))
        msg.faults = list(s.get("faults", []))
        msg.last_error = self.last_error
        self.status_pub.publish(msg)

    def _on_call(self, request, response):
        method = (request.method or "").strip()
        response.result = "null"

        if method == "list_methods":
            response.success = True
            response.result = dispatch.to_json(dispatch.describe(TC10LAB, META_METHODS))
            response.error = ""
            return response
        if method == "connected":
            response.success = True
            response.result = dispatch.to_json(self.tc is not None)
            response.error = ""
            return response
        if method == "reconnect":
            self._release()
            ok = self._connect()
            response.success = ok
            response.result = dispatch.to_json(ok)
            response.error = "" if ok else (self.last_error or "reconnect failed")
            return response

        tc = self.tc
        if tc is None:
            response.success = False
            response.error = f"TC10 LAB unavailable ({self.last_error or 'not connected'})"
            return response

        try:
            response.result = dispatch.call(tc, method, request.args, request.kwargs)
            self.last_error = ""
            self._failures = 0
            response.success = True
            response.error = ""
        except dispatch.DispatchError as exc:   # bad request: the link is fine
            response.success = False
            response.error = str(exc)
        except (ValueError, TypeError) as exc:
            # A refused argument or an unparseable reply: the link is alive, so no strike.
            response.success = False
            response.error = f"{type(exc).__name__}: {exc}"
        except Exception as exc:                # instrument or method fault
            self._note_failure(exc)
            response.success = False
            response.error = f"{type(exc).__name__}: {exc}"
        return response

    def destroy_node(self):
        # The TEC output is left as-is: a sample held at temperature survives a restart.
        tc = self.tc
        if tc is not None:
            try:
                tc.local()
            except Exception:
                pass
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
