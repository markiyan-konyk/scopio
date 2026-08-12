"""temperature_node - the sample temperature controller, exposed whole.

Same pattern as galvo_node: the node owns a TC10LAB driver object and offers
every public method of it over one service.

  srv  temperature/call    scopio_interfaces/InstrumentCall
  pub  temperature/status  scopio_interfaces/TemperatureStatus  (polled)

Raw SCPI comes free -- the driver's command/query are public methods:
method="query", args='["TEC:ACT?"]'.

A FAILED CALL DOES NOT DROP THE SESSION. It takes MAX_FAILURES consecutive
failures. One slow reply is normal on a USB-TMC instrument being polled every
second, and tearing the session down for it produced the worst possible
behaviour: connected, one good reading, disconnected, silent until the retry
timer came round, repeat. The status topic carries `last_error` the whole time,
so a real fault is visible immediately without the link being thrown away.
"""

import math
import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scopio_interfaces.msg import TemperatureStatus
from scopio_interfaces.srv import InstrumentCall

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

        # Held for the whole of _connect/_release, so a reconnect asked for over
        # the service cannot race the retry timer into two sessions on one device.
        self._lock = threading.RLock()
        self.tc = None
        self.idn = ""
        self.last_error = ""
        self.state = {}
        self._failures = 0

        self._connect()

        self.status_pub = self.create_publisher(TemperatureStatus, "temperature/status", 5)
        cb = ReentrantCallbackGroup()
        self.create_service(InstrumentCall, "temperature/call", self._on_call,
                            callback_group=cb)
        # The timers stay in the node's default (mutually exclusive) group, so a
        # slow instrument cannot stack polls on top of each other.
        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_status)
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_connect)

    def _connect(self):
        # .strip(): a value pasted into .env from a Windows editor carries a
        # trailing \r, and "/dev/usbtmc0\r" is a different path than the one on
        # disk -- ENOENT, which looks exactly like a missing instrument.
        resource = (os.environ.get("TCLAB_RESOURCE")
                    or self.get_parameter("resource").value or "").strip()
        timeout_ms = int(self.get_parameter("timeout_ms").value)
        units = str(self.get_parameter("units").value).strip()
        with self._lock:
            if self.tc is not None:
                return True
            tc = TC10LAB(resource, timeout_ms=timeout_ms)
            try:
                tc._open()
                idn = tc.idn()
                # Units are a LABEL on the status topic. Best-effort, like the
                # galvo's dcinit: a link that answers *IDN? is a working link,
                # and throwing it away because one cosmetic reply came back in
                # an unexpected format is how a connected instrument reads as
                # absent. (This firmware answers TEC:UNITS? with 'CELSIUS'.)
                try:
                    tc.set_units(units) if units else tc.get_units()
                except Exception as exc:
                    self.get_logger().warning(f"TC10 LAB units unavailable ({exc}).")
                self.tc, self.idn, self.last_error = tc, idn, ""
                self._failures = 0
            except Exception as exc:
                tc._close()
                self.last_error = str(exc)
                asked = repr(resource) if resource else "<auto-discover Wavelength on USB>"
                self.get_logger().warning(
                    f"TC10 LAB unavailable; node runs, reports connected=false.\n"
                    f"  tried:  {asked}\n"
                    f"  error:  {type(exc).__name__}: {exc}",
                    throttle_duration_sec=60.0)
                return False
        if tc.probe_note:
            self.get_logger().warning(f"TCLAB_RESOURCE {tc.probe_note}")
        self.get_logger().info(f"TC10 LAB connected on {tc.resource}: {idn}")
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
        """Record an instrument failure; drop the session only once they pile up.
        See the module docstring -- a single timeout is not a dead link."""
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
        except Exception as exc:                # instrument or method fault
            self._note_failure(exc)
            response.success = False
            response.error = f"{type(exc).__name__}: {exc}"
        return response

    def destroy_node(self):
        # The TEC output is deliberately left as-is: a sample being held at
        # temperature should survive a backend restart.
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
