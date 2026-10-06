"""galvo_node - the Rigol DG1022Z AWG that drives the galvo mirrors, exposed whole (see ../README.md)."""

import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scopio_interfaces.msg import AwgStatus
from scopio_interfaces.srv import AwgQuery, AwgWrite, InstrumentCall

from .connect_guard import ConnectGuard
from .drivers import dispatch
from .drivers.dg1022z import DG1022Z

MAX_FAILURES = 3

META_METHODS = (
    {"name": "list_methods", "signature": "()",
     "doc": "List every method callable through this node."},
    {"name": "reconnect", "signature": "()",
     "doc": "Re-open the VISA session to the instrument; returns True on success."},
    {"name": "connected", "signature": "()",
     "doc": "Whether the node currently holds a live session."},
)


class GalvoNode(Node):
    def __init__(self):
        super().__init__("galvo_node")
        self.declare_parameter("resource", "")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("timeout_ms", 15000)
        self.declare_parameter("reconnect_period", 15.0)
        self.declare_parameter("init_on_connect", True)

        # Held across _connect/_release: a service reconnect must not race the timer into two sessions.
        self._lock = threading.RLock()
        self.gen = None
        self.idn = ""
        self.last_command = ""
        self.last_error = ""
        self._failures = 0
        # A hard deadline per connect: a VISA open can hang in libusb (see connect_guard.py).
        timeout_s = int(self.get_parameter("timeout_ms").value) / 1000.0
        self._guard = ConnectGuard(3 * timeout_s + 5.0)

        self._connect()

        self.status_pub = self.create_publisher(AwgStatus, "awg/status", 5)
        cb = ReentrantCallbackGroup()
        self.create_service(InstrumentCall, "awg/call", self._on_call, callback_group=cb)
        self.create_service(AwgWrite, "awg/write", self._on_write, callback_group=cb)
        self.create_service(AwgQuery, "awg/query", self._on_query, callback_group=cb)

        rate = max(0.5, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_status, callback_group=cb)
        # Default (exclusive) group: a slow connect must never stack on the previous one.
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_connect)

    # ------------------------------------------------------------------ #
    #  Connection
    # ------------------------------------------------------------------ #
    def _connect(self):
        """Open a session: GALVO_RESOURCE env > `resource` param > the driver's Rigol vendor-id discovery."""
        # .strip(): a trailing \r from a Windows editor makes the name fail to parse (INV_RSRC_NAME).
        resource = (os.environ.get("GALVO_RESOURCE")
                    or self.get_parameter("resource").value or "").strip()
        timeout_ms = int(self.get_parameter("timeout_ms").value)
        init = bool(self.get_parameter("init_on_connect").value)

        def attempt():
            gen = DG1022Z(resource, timeout_ms=timeout_ms)
            try:
                gen._open()            # also clears the session, verifies *IDN?
            except BaseException:
                gen._drop()            # a half-open session still holds the USB device
                raise
            # Best-effort: DC at the offsets, outputs on, so update(ch, val) moves the galvo at once.
            if init:
                try:
                    gen.dcinit()
                except Exception as exc:
                    self.get_logger().warning(f"AWG dcinit failed ({exc}).")
            return gen

        with self._lock:
            if self.gen is not None:
                return True
            try:
                gen = self._guard.run(attempt, discard=lambda g: g._drop())
                self.gen, self.idn, self.last_error = gen, gen.identity, ""
                self._failures = 0
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                # Echo the resource: a name that fails to parse is config, one not found is a cable.
                asked = repr(resource) if resource else "<auto-discover Rigol on USB>"
                self.get_logger().warning(
                    f"AWG unavailable; node runs, reports connected=false.\n"
                    f"  tried:  {asked}\n"
                    f"  error:  {type(exc).__name__}: {exc}\n"
                    f"  INV_RSRC_NAME/parsing => the STRING is wrong (check "
                    f"GALVO_RESOURCE in ros2_ws/.env); RSRC_NFOUND => the "
                    f"address names nothing that is plugged in.",
                    throttle_duration_sec=60.0)
                return False
        self.get_logger().info(f"AWG connected on {gen.resource}: {gen.identity}")
        return True

    def _retry_connect(self):
        if self.gen is None:
            self._connect()

    def _release(self):
        """Drop the session without sending SCPI (the link may be dead), so the timer rebuilds it."""
        with self._lock:
            gen, self.gen = self.gen, None
            if gen is None:
                return False
            gen._drop()
            self._failures = 0
            return True

    def _fail(self, exc, response):
        """Record a failed call; MAX_FAILURES in a row cost the session (bad requests never count)."""
        self.last_error = str(exc)
        self._failures += 1
        if self._failures >= MAX_FAILURES and self._release():
            self.get_logger().warning(
                f"AWG dropped after {MAX_FAILURES} failures ({exc}); will reconnect.")
        response.success = False
        response.error = f"{type(exc).__name__}: {exc}"
        return response

    def _publish_status(self):
        # Cached state only: polling would share the session with multi-second waveform uploads.
        msg = AwgStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self.gen is not None
        msg.idn = self.idn
        msg.last_command = self.last_command
        msg.last_error = self.last_error
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  The API: call any public method on the driver
    # ------------------------------------------------------------------ #
    def _on_call(self, request, response):
        method = (request.method or "").strip()
        response.result = "null"

        if method == "list_methods":
            response.success = True
            response.result = dispatch.to_json(dispatch.describe(DG1022Z, META_METHODS))
            response.error = ""
            return response
        if method == "connected":
            response.success = True
            response.result = dispatch.to_json(self.gen is not None)
            response.error = ""
            return response
        if method == "reconnect":
            self._release()
            ok = self._connect()
            response.success = ok
            response.result = dispatch.to_json(ok)
            response.error = "" if ok else (self.last_error or "reconnect failed")
            return response

        gen = self.gen
        if gen is None:
            response.success = False
            response.error = f"AWG unavailable ({self.last_error or 'not connected'})"
            return response

        try:
            response.result = dispatch.call(gen, method, request.args, request.kwargs)
            self.last_command = f"{method}()"
            self.last_error = ""
            self._failures = 0
            response.success = True
            response.error = ""
        except dispatch.DispatchError as exc:      # bad request: link is fine
            response.success = False
            response.error = str(exc)
        except (ValueError, TypeError) as exc:
            # An argument refused before the wire: the link is fine, so no strike.
            response.success = False
            response.error = f"{type(exc).__name__}: {exc}"
        except Exception as exc:                   # instrument or method fault
            self._fail(exc, response)
        return response

    # ------------------------------------------------------------------ #
    #  Raw SCPI passthrough, via the driver's locked command()/query()
    # ------------------------------------------------------------------ #
    def _on_write(self, request, response):
        gen = self.gen
        if gen is None:
            response.success = False
            response.error = "AWG unavailable"
            return response
        try:
            gen.command(request.command)
            self.last_command = request.command
            self.last_error = ""
            self._failures = 0
            response.success = True
            response.error = ""
        except Exception as exc:
            self._fail(exc, response)
        return response

    def _on_query(self, request, response):
        gen = self.gen
        if gen is None:
            response.success = False
            response.response = ""
            response.error = "AWG unavailable"
            return response
        try:
            response.response = gen.query(request.command)
            self.last_command = request.command
            self.last_error = ""
            self._failures = 0
            response.success = True
            response.error = ""
        except Exception as exc:
            response.response = ""
            self._fail(exc, response)
        return response

    def destroy_node(self):
        # The driver's _close turns the outputs off first; best-effort.
        gen = self.gen
        if gen is not None:
            try:
                gen._close()
            except Exception:
                pass
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GalvoNode()
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
