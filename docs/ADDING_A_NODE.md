# Adding a node

How to give SCOPIO a new device or capability. Adding nodes is how this repo grows. A node that follows the conventions below appears in the API, the SDK and the MCP with no change to any of them.

The worked example is a hypothetical chamber pressure gauge, `pressure_node`. It is not in the repo.

## 1. Choose the pattern

| Pattern | Use it for | Templates |
|---|---|---|
| **Plain node** | A simple device with a few fixed operations: a GPIO pin, a file, a sensor with one reading. You write the services yourself. | `relay_node.py`, `calibration_node.py`, `stage_node.py` |
| **Instrument node** | An instrument with a command language (SCPI over VISA, or similar) and many features. You write a driver class; the node exposes **all** of its public methods over one generic `InstrumentCall` service. | `galvo_node.py`, `temperature_node.py` |

An instrument node is made of four parts, all already in the repo:

- a **driver class** in `scopio_microscope/drivers/`, following the [driver contract](../ros2_ws/src/scopio_microscope/scopio_microscope/drivers/README.md);
- **`drivers/dispatch.py`**, which turns the driver into the `<name>/call` service;
- **`connect_guard.py`**, which gives each connect attempt a deadline, because a VISA open can hang inside libusb;
- the **3-strikes rule**: a node drops its session only after 3 consecutive I/O failures, so one slow reply does not knock the instrument offline.

## 2. The files to touch

| File | Change | Rebuild cost |
|---|---|---|
| `ros2_ws/src/scopio_microscope/scopio_microscope/drivers/<driver>.py` | new driver class (instrument node only) | about a minute |
| `ros2_ws/src/scopio_microscope/scopio_microscope/<name>_node.py` | the node | about a minute |
| `ros2_ws/src/scopio_microscope/setup.py` | a `console_scripts` entry point | about a minute |
| `ros2_ws/src/scopio_microscope/launch/microscope.launch.py` | a `Node(...)` line | about a minute |
| `ros2_ws/src/scopio_microscope/config/params.yaml` | a `/scopio/<name>_node:` section | about a minute |
| `ros2_ws/src/scopio_interfaces/` | new `.msg`/`.srv`/`.action` files, listed in `CMakeLists.txt` (only if needed) | **the long build** |
| `ros2_ws/docker-compose.yml` | an `environment:` line (only if the node reads a variable from `.env`) | none |
| `ros2_ws/scripts/test_drivers.py` | offline tests | seconds |

Prefer the existing types. Any change in `scopio_interfaces` recompiles every message on the Pi. A device's status topic usually does need one new `.msg`, though, and that is a fair price once.

Compose passes only the variables listed under `environment:` into the container, so a new `.env` variable must be added there too. Document it in `.env.example`.

## 3. Conventions that make it appear everywhere

| Convention | What it buys you |
|---|---|
| Publish state on `<name>/status` (or `<name>/state`) under `/scopio`, with a `connected` bool. | The gateway finds any `*/status` or `*/state` topic and adds it to `GET /api/v1/status`. The UI and the MCP show it. |
| Publish only on change? Make the topic latched (TRANSIENT_LOCAL, depth 1). | A client that connects later still gets the current value. The gateway mirrors the publisher's QoS. |
| Services reply with `success` and `message` (or `error`). | Every client handles failures the same way. |
| Float request fields use NaN for "leave unchanged". | The gateway turns an omitted float into NaN, so partial requests are safe. See [the NaN rule](API.md#the-nan-rule). |
| An instrument's service is `<name>/call` of type `InstrumentCall`. | The SDK and the MCP list it as an instrument named `<name>` (the service prefix, not the node name). |
| A latched or polled status, never a blocking call in a timer, for anything slow. | One slow device cannot stall the node's executor. |

## 4. Degrade gracefully

Every node in SCOPIO must:

- start without its hardware, publish `connected=false`, and say why in its log and its status;
- keep a retry timer (`reconnect_period`), so a device plugged in later is picked up without a restart;
- answer every service call with `success=false` and a reason while disconnected, never raise;
- never take the graph down. Each node is its own process, so a crash loses only that node, but a node that crashes on import is simply absent from `ros2 node list`.

## 5. The driver (instrument node only)

The full contract is in [drivers/README.md](../ros2_ws/src/scopio_microscope/scopio_microscope/drivers/README.md). In short:

- The constructor never opens the instrument. The node calls `_open()`.
- All I/O goes through `command()` and `query()`. They hold `_lock` and pace the wire to at most 50 commands per second.
- `_open()` clears the session, then checks that `*IDN?` names the right instrument.
- Public methods are the API. The first line of each docstring is what `list_methods` shows to clients and AI agents, so write it for them.
- Import only the standard library and `pyvisa`.

`drivers/pressure_gauge.py`:

```python
"""Hypothetical SCPI pressure gauge."""

import threading
import time

import pyvisa


class PressureGauge:
    MIN_INTERVAL_S = 1.0 / 50          # the instrument lags past ~60 commands/s

    def __init__(self, resource="", timeout_ms=5000):
        self.resource = resource
        self.timeout_ms = timeout_ms
        self._lock = threading.RLock()
        self._last_io = 0.0
        self._desynced = False
        self.identity = ""
        self.rm = None
        self.device = None

    def _open(self):
        self.rm = pyvisa.ResourceManager("@py")
        self.device = self.rm.open_resource(self.resource)
        self.device.read_termination = "\n"
        self.device.write_termination = "\n"
        self.device.timeout = self.timeout_ms
        self._resync()                 # a dead session may have left a reply queued
        idn = self.query("*IDN?")
        if "PG100" not in idn.upper():
            raise RuntimeError(f"{self.resource} answered *IDN? with {idn!r}")
        self.identity = idn

    def _resync(self):
        with self._lock:
            self._desynced = False
            try:
                self.device.clear()
            except Exception:
                pass
            try:
                self.command("*CLS")
            except Exception:
                pass

    def _close(self):
        with self._lock:
            for handle in (self.device, self.rm):
                try:
                    if handle is not None:
                        handle.close()
                except Exception:
                    pass
            self.device = self.rm = None

    def _pace(self):
        wait = self._last_io + self.MIN_INTERVAL_S - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    def command(self, cmd):
        """Write a raw SCPI command."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("gauge session is closed")
            self._pace()
            try:
                self.device.write(cmd)
            finally:
                self._last_io = time.monotonic()

    def query(self, cmd):
        """Write a raw SCPI query and return the reply, stripped."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("gauge session is closed")
            if self._desynced:
                self._resync()
            self._pace()
            try:
                return self.device.query(cmd).strip()
            except Exception:
                self._desynced = True  # its reply may still arrive; clear first
                raise
            finally:
                self._last_io = time.monotonic()

    def pressure(self):
        """Chamber pressure in mbar."""
        return float(self.query("MEAS:PRES?"))

    def status(self):
        """Everything the status topic needs, in one call."""
        return {"pressure": self.pressure()}
```

## 6. The status message (only if no existing type fits)

`ros2_ws/src/scopio_interfaces/msg/PressureStatus.msg`:

```
std_msgs/Header header
bool connected
string idn
float32 pressure      # mbar; NaN when disconnected
string last_error
```

Add `"msg/PressureStatus.msg"` to the `rosidl_generate_interfaces(...)` list in `ros2_ws/src/scopio_interfaces/CMakeLists.txt`.

## 7. The node

`scopio_microscope/pressure_node.py`, a trimmed copy of `temperature_node.py`:

```python
"""pressure_node - the chamber pressure gauge, exposed whole."""

import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scopio_interfaces.msg import PressureStatus
from scopio_interfaces.srv import InstrumentCall

from .connect_guard import ConnectGuard
from .drivers import dispatch
from .drivers.pressure_gauge import PressureGauge

MAX_FAILURES = 3

META_METHODS = (
    {"name": "list_methods", "signature": "()",
     "doc": "List every method callable through this node."},
    {"name": "reconnect", "signature": "()",
     "doc": "Re-open the session to the gauge; returns True on success."},
    {"name": "connected", "signature": "()",
     "doc": "Whether the node currently holds a live session."},
)


class PressureNode(Node):
    def __init__(self):
        super().__init__("pressure_node")
        self.declare_parameter("resource", "")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("timeout_ms", 5000)
        self.declare_parameter("reconnect_period", 10.0)

        self._lock = threading.RLock()     # one session, even if reconnect races the timer
        self.gauge = None
        self.idn = ""
        self.last_error = ""
        self.state = {}
        self._failures = 0
        timeout_s = int(self.get_parameter("timeout_ms").value) / 1000.0
        self._guard = ConnectGuard(3 * timeout_s + 5.0)

        self._connect()

        self.status_pub = self.create_publisher(PressureStatus, "pressure/status", 5)
        cb = ReentrantCallbackGroup()
        self.create_service(InstrumentCall, "pressure/call", self._on_call,
                            callback_group=cb)
        # Timers stay in the default (mutually exclusive) group: no stacked polls.
        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_status)
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_connect)

    def _connect(self):
        resource = (self.get_parameter("resource").value or "").strip()
        timeout_ms = int(self.get_parameter("timeout_ms").value)

        def attempt():
            gauge = PressureGauge(resource, timeout_ms=timeout_ms)
            try:
                gauge._open()
            except BaseException:
                gauge._close()
                raise
            return gauge

        with self._lock:
            if self.gauge is not None:
                return True
            try:
                gauge = self._guard.run(attempt, discard=lambda g: g._close())
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().warning(
                    f"Gauge unavailable; node runs, reports connected=false.\n"
                    f"  tried: {resource!r}\n  error: {self.last_error}",
                    throttle_duration_sec=60.0)
                return False
            self.gauge, self.idn = gauge, gauge.identity
            self.last_error, self._failures = "", 0
        self.get_logger().info(f"Gauge connected on {resource}: {self.idn}")
        return True

    def _retry_connect(self):
        if self.gauge is None:
            self._connect()

    def _release(self):
        with self._lock:
            gauge, self.gauge = self.gauge, None
            if gauge is None:
                return False
            gauge._close()
            self._failures = 0
            return True

    def _note_failure(self, exc):
        self.last_error = str(exc)
        self._failures += 1
        if self._failures >= MAX_FAILURES and self._release():
            self.get_logger().warning(
                f"Gauge dropped after {MAX_FAILURES} failures ({exc}); will reconnect.")

    def _publish_status(self):
        gauge = self.gauge
        if gauge is not None:
            try:
                self.state = gauge.status()
                self.last_error, self._failures = "", 0
            except Exception as exc:
                self.state = {}
                self._note_failure(exc)
        msg = PressureStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self.gauge is not None
        msg.idn = self.idn
        msg.pressure = float(self.state.get("pressure", math.nan))
        msg.last_error = self.last_error
        self.status_pub.publish(msg)

    def _on_call(self, request, response):
        method = (request.method or "").strip()
        response.result, response.error = "null", ""
        if method == "list_methods":
            response.success = True
            response.result = dispatch.to_json(
                dispatch.describe(PressureGauge, META_METHODS))
            return response
        if method == "connected":
            response.success = True
            response.result = dispatch.to_json(self.gauge is not None)
            return response
        if method == "reconnect":
            self._release()
            ok = self._connect()
            response.success, response.result = ok, dispatch.to_json(ok)
            response.error = "" if ok else self.last_error
            return response

        gauge = self.gauge
        if gauge is None:
            response.success = False
            response.error = f"gauge unavailable ({self.last_error or 'not connected'})"
            return response
        try:
            response.result = dispatch.call(gauge, method, request.args, request.kwargs)
            response.success = True
            self.last_error, self._failures = "", 0
        except (dispatch.DispatchError, ValueError, TypeError) as exc:
            # A bad request or argument: the link is fine, so it is no strike.
            response.success = False
            response.error = f"{type(exc).__name__}: {exc}"
        except Exception as exc:
            self._note_failure(exc)
            response.success = False
            response.error = f"{type(exc).__name__}: {exc}"
        return response

    def destroy_node(self):
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PressureNode()
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
```

## 8. Wire it in

`ros2_ws/src/scopio_microscope/setup.py`, in `console_scripts`:

```python
"pressure_node = scopio_microscope.pressure_node:main",
```

`ros2_ws/src/scopio_microscope/launch/microscope.launch.py`, in the `LaunchDescription` list:

```python
Node(executable="pressure_node", name="pressure_node", **common),
```

`ros2_ws/src/scopio_microscope/config/params.yaml`. The key must be the node's full name:

```yaml
/scopio/pressure_node:
  ros__parameters:
    resource: "USB0::0x1234::0x0001::SN0001::INSTR"
    publish_rate: 1.0
```

Optionally, add `pressure_node` to the node list in `ros2_ws/scripts/smoke_test.sh`.

## 9. Test it offline

`ros2_ws/scripts/test_drivers.py` runs anywhere, with no ROS and no hardware: it stubs `pyvisa` and drives the driver against `FakeDevice`, which records every write and answers queries from a table. Every module-level function named `test_*` runs.

Import the driver next to the others at the top of the file:

```python
from scopio_microscope.drivers.pressure_gauge import PressureGauge  # noqa: E402
```

Add a test:

```python
def test_pressure_gauge_reads_pressure():
    gauge = PressureGauge("USB0::0x1234::0x0001::FAKE::INSTR")
    gauge.device = FakeDevice({"MEAS:PRES?": "1.2e-3"})
    assert gauge.pressure() == 1.2e-3
```

Add the node to the node-and-driver contract test, `test_the_nodes_and_their_drivers_still_agree`. It reads the node's source and fails if the node calls a method the driver does not have. Extend its tuple:

```python
("pressure_node", PressureGauge, "gauge"),
```

Run it on your laptop or the Pi:

```bash
python3 ros2_ws/scripts/test_drivers.py
```

## 10. Build and verify

On the Pi, in `scopio/ros2_ws`. With a new `.msg` this is the long build; without one it takes about a minute:

```bash
docker compose up -d --build
```

Then check, from a laptop:

- `GET /api/v1/interfaces` lists `/scopio/pressure/call` and `/scopio/pressure/status`.
- `GET /api/v1/status` has a `pressure/status` entry within 5 s of the node publishing.
- `POST /api/v1/service/pressure/call` with `{"method": "list_methods"}` lists `pressure` and `status`.
- In the MCP, `describe_instrument()` lists `pressure`, and `describe_instrument('pressure')` shows its methods.

## Why no gateway, SDK or MCP change is needed

- The gateway looks up every service, topic and action in the live graph on demand. Nothing is hard-coded except six telemetry names, kept so clients can always rely on them.
- `GET /api/v1/status` subscribes to every `*/status` and `*/state` topic it finds, rescanning every 5 s.
- `dispatch.describe()` lists the driver's public methods from the class itself, so `list_methods` is always the code that is running.
- The SDK and the MCP find instruments by looking for `*/call` services in `GET /api/v1/interfaces` (the MCP also checks the type is `InstrumentCall`).

## Next

- [drivers/README.md](../ros2_ws/src/scopio_microscope/scopio_microscope/drivers/README.md): the driver contract in full
- [scopio_microscope/README.md](../ros2_ws/src/scopio_microscope/README.md): the existing nodes
- [CONFIGURATION.md](CONFIGURATION.md#what-a-change-costs): what each change costs to rebuild
