"""galvo_node - the galvo/laser AWG, exposed whole.

The node owns the VISA session to the arbitrary-waveform generator (a Rigol
DG1022Z: CH1 = X mirror, CH2 = Y mirror) through the vendored, EDITABLE
`drivers/dg1022z.DG1022Z` class, and offers it to clients two ways:

  1. `awg/call`  -- ANY public method of the driver class, by name, with JSON
     args. The galvo ones are `dcinit`, `update`, `move`, `sininit`,
     `sinupdate`, `offsets`, `position`; the rest of the class is the plain
     instrument. Add a method to dg1022z.py and it is reachable here the same
     instant -- no new .srv, no gateway and no client change. Example: a UI
     slider calls `awg/call` with method="update", args=[1, 0.20].
     Call `list_methods` for the live list with signatures.
  2. `awg/write` / `awg/query` -- raw SCPI passthrough for clients that compose
     their own SCPI; they route through the driver's locked command()/query().

Connection is the NODE's job, not the client's: DG1022Z() does not open on
construction, so `_connect` builds the object and calls its `_open()`. `_open`
(and every other private, underscore-prefixed method) is therefore NOT callable
over `awg/call` -- re-opening the link is exposed as the `reconnect` meta-method
instead. The mirror geometry (volts<->pixels<->micrometres) stays in client
code; this node takes no view of what a command means.

Topics / services (under /scopio):
  pub  awg/status   scopio_interfaces/AwgStatus       (connected, idn, last cmd/err)
  srv  awg/call     scopio_interfaces/InstrumentCall  (any public driver method)
  srv  awg/write    scopio_interfaces/AwgWrite        (raw SCPI command)
  srv  awg/query    scopio_interfaces/AwgQuery        (raw SCPI query -> reply)

Meta-methods handled by the NODE, not the driver:
  list_methods()  -> [{name, signature, doc}] of everything callable (works
                     even while disconnected -- it introspects the class)
  reconnect()     -> re-open the VISA session (rebuild + _open); True on success
  connected()     -> bool

Degrades gracefully: with no resource / no instrument it reports connected=false
and calls fail with a readable error; the rest of the graph still comes up.
"""

import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scopio_interfaces.msg import AwgStatus
from scopio_interfaces.srv import AwgQuery, AwgWrite, InstrumentCall

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

        # Held for the whole of _connect/_release, so a reconnect asked for over
        # the service cannot race the retry timer into two sessions on one USB
        # device. Opening a VISA session can take longer than the retry period.
        self._lock = threading.RLock()
        self.gen = None
        self.idn = ""
        self.last_command = ""
        self.last_error = ""
        self._failures = 0

        self._connect()

        self.status_pub = self.create_publisher(AwgStatus, "awg/status", 5)
        cb = ReentrantCallbackGroup()
        self.create_service(InstrumentCall, "awg/call", self._on_call, callback_group=cb)
        self.create_service(AwgWrite, "awg/write", self._on_write, callback_group=cb)
        self.create_service(AwgQuery, "awg/query", self._on_query, callback_group=cb)

        rate = max(0.5, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_status, callback_group=cb)
        # The retry timer stays in the node's default (mutually exclusive)
        # group: opening a VISA session can outlast the retry period, and a
        # connect attempt must never stack on top of the previous one.
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_connect)

    # ------------------------------------------------------------------ #
    #  Connection
    # ------------------------------------------------------------------ #
    def _connect(self):
        """Open a session. GALVO_RESOURCE env > `resource` param > the driver's
        own Rigol-vendor-id discovery (which never opens another vendor's box)."""
        # .strip(): a value pasted into .env from a Windows editor arrives with a
        # trailing \r, and a VISA resource name with a stray byte on the end fails
        # to PARSE (VI_ERROR_INV_RSRC_NAME) -- which reads nothing like "your
        # instrument has whitespace in it".
        resource = (os.environ.get("GALVO_RESOURCE")
                    or self.get_parameter("resource").value or "").strip()
        timeout_ms = int(self.get_parameter("timeout_ms").value)
        with self._lock:
            if self.gen is not None:
                return True
            # DG1022Z() does NOT open on construction -- the node opens it.
            gen = DG1022Z(resource, timeout_ms=timeout_ms)
            try:
                gen._open()
                idn = gen.query("*IDN?")
                # Put both channels in DC mode at their offsets with outputs ON,
                # so a client's update(ch, val) positions the galvo immediately.
                # Best-effort: a connected-but-uninitable AWG still counts as
                # connected. (Set init_on_connect:false to skip.)
                if self.get_parameter("init_on_connect").value:
                    try:
                        gen.dcinit()
                    except Exception as exc:
                        self.get_logger().warning(f"AWG dcinit failed ({exc}).")
                self.gen, self.idn, self.last_error = gen, idn, ""
                self._failures = 0
            except Exception as exc:
                # A session that opened and then failed still HOLDS the USB
                # device; leaving it claimed makes every later retry fail too.
                gen._drop()
                self.last_error = str(exc)
                # ECHO THE RESOURCE. Without it "Parsing error" is unreadable:
                # a name that fails to parse is a config bug, a name that is not
                # found is a cable, and the message must tell them apart.
                asked = repr(resource) if resource else "<auto-discover Rigol on USB>"
                self.get_logger().warning(
                    f"AWG unavailable; node runs, reports connected=false.\n"
                    f"  tried:  {asked}\n"
                    f"  error:  {type(exc).__name__}: {exc}\n"
                    f"  INV_RSRC_NAME/parsing => the STRING is wrong (check "
                    f"GALVO_RESOURCE in ros2_ws/.env); RSRC_NFOUND/no Rigol => "
                    f"the instrument is not on the bus.",
                    throttle_duration_sec=60.0)
                return False
        self.get_logger().info(f"AWG connected on {gen.resource}: {idn}")
        return True

    def _retry_connect(self):
        if self.gen is None:
            self._connect()

    def _release(self):
        """Give up the session so the retry timer can rebuild it. Drops the VISA
        handles WITHOUT sending SCPI (the link may already be dead, and a write
        would just burn a full timeout)."""
        with self._lock:
            gen, self.gen = self.gen, None
            if gen is None:
                return False
            gen._drop()
            self._failures = 0
            return True

    def _fail(self, exc, response):
        """Record a failed call. It takes MAX_FAILURES consecutive failures to
        cost the session, so one slow reply mid-waveform-upload does not knock
        the AWG offline for everybody. A bad command or argument never counts --
        those raise DispatchError and are answered without coming through here."""
        self.last_error = str(exc)
        self._failures += 1
        if self._failures >= MAX_FAILURES and self._release():
            self.get_logger().warning(
                f"AWG dropped after {MAX_FAILURES} failures ({exc}); will reconnect.")
        response.success = False
        response.error = f"{type(exc).__name__}: {exc}"
        return response

    def _publish_status(self):
        # Cached state only -- deliberately no polling: a status query at 5 Hz
        # would share the USBTMC session with multi-second arbitrary-waveform
        # uploads. The driver reports link health through the calls themselves.
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
        except Exception as exc:                   # instrument or method fault
            self._fail(exc, response)
        return response

    # ------------------------------------------------------------------ #
    #  Raw SCPI passthrough -- the driver's own locked command()/query(), so
    #  it serializes against every other client of the same session.
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
        # Clean shutdown: the driver's own close turns the outputs off first.
        # Best-effort -- a dead link must not block shutdown.
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
