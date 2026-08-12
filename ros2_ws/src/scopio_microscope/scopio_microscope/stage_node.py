"""stage_node - owns the Sangaboard XYZ stage.

Publishes the microscope's global (open-loop) position and executes long-horizon
motion goals. It re-implements the small amount of stage logic leanly for ROS
rather than importing the Flask app's controls.py (which carries app-global
state).

Topics / services / actions (under /scopio):
  pub     stage/position    scopio_interfaces/StagePosition  (steps + micrometres)
  sub     calibration       scopio_interfaces/Calibration (latched; steps_per_um)
  srv     stage/jog         scopio_interfaces/StageJog    (relative jog, low latency)
  srv     stage/move_abs    scopio_interfaces/MoveAbs     (single absolute move)
  action  stage/move_path   scopio_interfaces/MoveStagePath
  action  scan_region       scopio_interfaces/ScanRegion

Degrades gracefully: with no board present it publishes connected=false and
move goals/services abort cleanly.
"""

import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from scopio_interfaces.msg import Calibration, StagePosition
from scopio_interfaces.srv import MoveAbs, StageJog
from scopio_interfaces.action import MoveStagePath, ScanRegion


# The Sangaboard v0.5 is an RP2040 HAT: it sits on the 40-pin header and talks
# over the Pi's UART, so it has NO USB vendor/product id. The library's
# auto-detection scans USB serial devices, which means a header-mounted board is
# powered, wired and working yet can never be found -- naming the port is the
# only way. /dev/serial0 is the Pi's alias for whichever UART is on pins 8/10.
GPIO_UART_PORTS = ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0")


def serial_ports():
    """Every serial port visible here. This list IS the diagnosis: empty means
    the board is not reaching this process at all; a port that is present means
    auto-detection did not recognise it, and naming it fixes that."""
    try:
        from serial.tools import list_ports
        found = [f"{p.device} [{p.vid:04x}:{p.pid:04x}] {p.description}"
                 if p.vid else f"{p.device} [no USB id] {p.description}"
                 for p in list_ports.comports()]
    except Exception as exc:
        found = [f"<pyserial unavailable: {type(exc).__name__}: {exc}>"]
    # comports() reports USB serial devices; the header UART may not appear
    # there at all, so check it directly -- that is where a HAT lives.
    header = [p for p in GPIO_UART_PORTS if os.path.exists(p)]
    if header:
        found.append(f"GPIO header UART present: {header}")
    return found or ["(none visible to this process)"]


class StageNode(Node):
    def __init__(self):
        super().__init__("stage_node")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("port", "")
        self.declare_parameter("reconnect_period", 10.0)
        self.declare_parameter("step_x", 40)
        self.declare_parameter("step_y", 40)
        self.declare_parameter("step_z", 40)

        # One lock for both the session and the moves: never swap the board out
        # from under a move in progress.
        self._lock = threading.RLock()
        self.sb = None
        self.position = {"x": 0, "y": 0, "z": 0}   # open-loop, origin at startup
        self.steps_per_um = {"x": 1.0, "y": 1.0, "z": 1.0}  # from calibration topic

        self._connect()
        # Every other instrument node retries; this one used to open the board
        # exactly once, so a stage plugged in after launch stayed dead forever.
        period = max(2.0, float(self.get_parameter("reconnect_period").value))
        self.create_timer(period, self._retry_connect)

        self.pos_pub = self.create_publisher(StagePosition, "stage/position", 5)
        # Latched calibration: match the publisher's transient-local durability.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Calibration, "calibration", self._on_calibration, latched)

        rate = max(0.5, self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._publish_position)

        cb = ReentrantCallbackGroup()
        self.create_service(StageJog, "stage/jog", self._on_jog, callback_group=cb)
        self.create_service(MoveAbs, "stage/move_abs", self._on_move_abs, callback_group=cb)
        self._move_server = ActionServer(
            self, MoveStagePath, "stage/move_path",
            execute_callback=self._execute_move_path,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda c: CancelResponse.ACCEPT,
            callback_group=cb)
        self._scan_server = ActionServer(
            self, ScanRegion, "scan_region",
            execute_callback=self._execute_scan_region,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda c: CancelResponse.ACCEPT,
            callback_group=cb)

    # ------------------------------------------------------------------ #
    def _connect(self):
        """Open the board. SANGABOARD_PORT env > `port` param > auto-detection.

        Auto-detection only ever finds a board on USB. A v0.5 HAT on the 40-pin
        header speaks the Pi's UART and has no USB identity at all, so it is
        invisible to it -- hence the explicit port, and hence the header UARTs
        being tried before giving up.
        """
        port = (os.environ.get("SANGABOARD_PORT")
                or self.get_parameter("port").value or "").strip()
        with self._lock:
            if self.sb is not None:
                return True
            attempts = []
            for candidate in ([port] if port else
                              [None] + [p for p in GPIO_UART_PORTS
                                        if os.path.exists(p)]):
                try:
                    from sangaboard import Sangaboard
                    self.sb = Sangaboard(candidate) if candidate else Sangaboard()
                    opened = candidate or "<auto-detected>"
                    break
                except Exception as exc:
                    attempts.append(f"{candidate or '<auto-detection>'}: "
                                    f"{type(exc).__name__}: {exc}")
            else:
                self.get_logger().warning(
                    "Sangaboard unavailable; node runs, reports connected=false.\n"
                    + "".join(f"  tried  {a}\n" for a in attempts)
                    + f"  serial ports here: {serial_ports()}\n"
                    "  A v0.5 HAT on the 40-pin header is a UART device, not "
                    "USB: set SANGABOARD_PORT=/dev/serial0 in ros2_ws/.env, and "
                    "on the Pi enable the UART with the serial LOGIN CONSOLE "
                    "off (raspi-config > Interface Options > Serial Port), or "
                    "a getty holds the port and talks over you.",
                    throttle_duration_sec=60.0)
                return False
        self.get_logger().info(f"Sangaboard connected on {opened}.")
        return True

    def _retry_connect(self):
        if self.sb is None:
            self._connect()

    def _on_calibration(self, msg):
        self.steps_per_um = {"x": msg.steps_per_um_x or 1.0,
                             "y": msg.steps_per_um_y or 1.0,
                             "z": msg.steps_per_um_z or 1.0}

    def _publish_position(self):
        msg = StagePosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self.sb is not None
        msg.x = int(self.position["x"])
        msg.y = int(self.position["y"])
        msg.z = int(self.position["z"])
        msg.x_um = float(self.position["x"]) / self.steps_per_um["x"]
        msg.y_um = float(self.position["y"]) / self.steps_per_um["y"]
        msg.z_um = float(self.position["z"]) / self.steps_per_um["z"]
        self.pos_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Services: relative jog + single absolute move
    # ------------------------------------------------------------------ #
    def _on_jog(self, request, response):
        try:
            self._move_rel(request.dx, request.dy, request.dz)
            response.success = True
            response.message = "ok"
        except Exception as e:
            response.success = False
            response.message = str(e)
        response.x = int(self.position["x"])
        response.y = int(self.position["y"])
        response.z = int(self.position["z"])
        return response

    def _on_move_abs(self, request, response):
        try:
            self._move_abs(request.x, request.y, request.z)
            response.success = True
            response.message = "ok"
        except Exception as e:
            response.success = False
            response.message = str(e)
        response.x = int(self.position["x"])
        response.y = int(self.position["y"])
        response.z = int(self.position["z"])
        return response

    def _move_rel(self, dx, dy, dz):
        """Move by a relative displacement (steps) and track absolute position."""
        if self.sb is None:
            raise RuntimeError("Sangaboard unavailable")
        with self._lock:
            self.sb.move_rel([int(dx), int(dy), int(dz)])
            self.position["x"] += int(dx)
            self.position["y"] += int(dy)
            self.position["z"] += int(dz)

    def _move_abs(self, x, y, z):
        self._move_rel(x - self.position["x"], y - self.position["y"],
                       z - self.position["z"])

    # ------------------------------------------------------------------ #
    #  Action: move along a path of absolute targets
    # ------------------------------------------------------------------ #
    def _execute_move_path(self, goal_handle):
        req = goal_handle.request
        result = MoveStagePath.Result()
        reached = 0
        for i, pt in enumerate(req.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.points_reached = reached
                return result
            try:
                self._move_abs(pt.x, pt.y, pt.z)
            except Exception as e:
                self.get_logger().error(f"move_path failed at {i}: {e}")
                goal_handle.abort()
                result.success = False
                result.points_reached = reached
                return result
            reached += 1
            fb = MoveStagePath.Feedback()
            fb.current_index = i
            fb.x = self.position["x"]
            fb.y = self.position["y"]
            fb.z = self.position["z"]
            goal_handle.publish_feedback(fb)
            if req.settle_s > 0:
                time.sleep(req.settle_s)
        goal_handle.succeed()
        result.success = True
        result.points_reached = reached
        return result

    # ------------------------------------------------------------------ #
    #  Action: jump-scan a rectangular region (boustrophedon grid)
    # ------------------------------------------------------------------ #
    def _execute_scan_region(self, goal_handle):
        req = goal_handle.request
        result = ScanRegion.Result()
        step = max(1, req.step)
        xs = list(range(req.x_min, req.x_max + 1, step))
        ys = list(range(req.y_min, req.y_max + 1, step))
        visited = 0
        for j, y in enumerate(ys):
            row = xs if j % 2 == 0 else list(reversed(xs))   # snake to cut travel
            for x in row:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.frames_visited = visited
                    return result
                try:
                    self._move_abs(x, y, self.position["z"])
                except Exception as e:
                    self.get_logger().error(f"scan failed: {e}")
                    goal_handle.abort()
                    result.success = False
                    result.frames_visited = visited
                    return result
                if req.settle_s > 0:
                    time.sleep(req.settle_s)
                visited += 1
                fb = ScanRegion.Feedback()
                fb.frames_visited = visited
                fb.x = self.position["x"]
                fb.y = self.position["y"]
                goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        result.success = True
        result.frames_visited = visited
        return result

    def destroy_node(self):
        try:
            if self.sb is not None and hasattr(self.sb, "close"):
                self.sb.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StageNode()
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
