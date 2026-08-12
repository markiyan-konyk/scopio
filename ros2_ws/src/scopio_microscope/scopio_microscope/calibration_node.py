"""calibration_node - owns the microscope's spatial calibration.

The single source of truth for how pixels and steps map to micrometres:
  * um_per_px      - image scale (micrometres per native camera pixel)
  * steps_per_um_* - stage conversion (Sangaboard steps per micrometre per axis)

It publishes the calibration on a LATCHED (transient-local) topic so any client
-- including the stage_node, which needs steps_per_um to report its position in
micrometres -- gets the current value immediately on join.

PERSISTENCE. The file lives on the Pi's real disk, not in the container: compose
bind-mounts the repo at /workspace and runs the graph there, so the default
relative path resolves onto the host and survives `docker compose down`, image
rebuilds and reboots. The node logs the ABSOLUTE path it resolved at startup --
if that ever reads as a path inside the container, the mount is what broke, not
this node. Writes are atomic (temp file + os.replace), so power going out
mid-write cannot leave a truncated file that silently reads back as "no
calibration".

Topics / services (under /scopio):
  pub  calibration       scopio_interfaces/Calibration   (latched)
  srv  calibration/set   scopio_interfaces/CalibrationSet
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from scopio_interfaces.msg import Calibration
from scopio_interfaces.srv import CalibrationSet


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")
        self.declare_parameter("calibration_file", "calibration.json")
        self.path = os.path.abspath(self.get_parameter("calibration_file").value)

        self.data = {
            "um_per_px": None,
            "steps_per_um": {"x": 1.0, "y": 1.0, "z": 1.0},
        }
        self._load()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Calibration, "calibration", latched)
        self.create_service(CalibrationSet, "calibration/set", self._on_set)
        self._publish()
        self.get_logger().info(f"Calibration in {self.path}: {self.data}")

    def _load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                d = json.load(f)
        except FileNotFoundError:
            self.get_logger().info("No calibration file yet; using defaults.")
            return
        except (OSError, ValueError) as exc:
            # Loud on purpose: an unreadable file looks exactly like "the
            # calibration did not persist", and silence sends you hunting the
            # volume mount instead of the one bad file.
            self.get_logger().error(
                f"Calibration file {self.path} is unreadable ({exc}); using "
                "defaults. Fix or delete it -- the next calibration/set "
                "overwrites it.")
            return
        try:
            if d.get("um_per_px") is not None:
                self.data["um_per_px"] = float(d["um_per_px"])
            spu = d.get("steps_per_um") or {}
            for ax in ("x", "y", "z"):
                if ax in spu:
                    self.data["steps_per_um"][ax] = float(spu[ax])
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Calibration file {self.path} has bad values ({exc}); "
                "using defaults for those fields.")

    def _save(self):
        """Atomic: a half-written file must never replace a good calibration."""
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.data, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
            return True
        except OSError as exc:
            self.get_logger().error(f"Could NOT persist calibration to {self.path}: {exc}")
            try:
                os.unlink(tmp)
            except OSError:
                pass
            return False

    def _publish(self):
        msg = Calibration()
        msg.header.stamp = self.get_clock().now().to_msg()
        upp = self.data["um_per_px"]
        msg.has_um_per_px = upp is not None
        msg.um_per_px = float(upp) if upp is not None else 0.0
        msg.steps_per_um_x = float(self.data["steps_per_um"]["x"])
        msg.steps_per_um_y = float(self.data["steps_per_um"]["y"])
        msg.steps_per_um_z = float(self.data["steps_per_um"]["z"])
        self.pub.publish(msg)

    def _on_set(self, request, response):
        """Every field is a positive scale factor, so NaN *or* 0 (an omitted
        field on a partially-filled request) means "leave this one alone"."""
        fields = {"um_per_px": request.um_per_px,
                  "x": request.steps_per_um_x,
                  "y": request.steps_per_um_y,
                  "z": request.steps_per_um_z}
        given = {k: float(v) for k, v in fields.items()
                 if not math.isnan(v) and v != 0.0}
        bad = [k for k, v in given.items() if v < 0]
        if bad:
            response.success = False
            response.message = f"must be > 0: {', '.join(sorted(bad))}"
            return response
        if not given:
            response.success = False
            response.message = "nothing to set"
            return response

        if "um_per_px" in given:
            self.data["um_per_px"] = given["um_per_px"]
        for ax in ("x", "y", "z"):
            if ax in given:
                self.data["steps_per_um"][ax] = given[ax]

        response.success = self._save()
        self._publish()      # publish either way: it IS the live value now
        response.message = "ok" if response.success else (
            "applied in memory but NOT persisted -- see the node log")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
