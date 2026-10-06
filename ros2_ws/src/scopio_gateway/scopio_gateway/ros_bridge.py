"""The gateway's node in the ROS graph; rclpy never touches asyncio except via call_soon_threadsafe (see README.md)."""

import threading
import time

import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message, get_service

from .conversion import BULKY_TYPES, build_msg, msg_to_jsonable, normalize_msg_type

NAMESPACE = "/scopio"

# Always in GET /api/v1/status, null until published: clients key on these names.
TELEMETRY_TOPICS = [
    "stage/position",
    "camera/state",
    "awg/status",
    "temperature/status",
    "relay/state",
    "calibration",
]

# The convention for new nodes: any <name>/status or <name>/state topic joins /status by itself.
TELEMETRY_SUFFIXES = ("status", "state")
DISCOVERY_PERIOD_S = 5.0


def resolve(path):
    """'stage/jog' -> '/scopio/stage/jog'; '/other/thing' passes through."""
    path = path.strip()
    if path.startswith("/"):
        return path
    return f"{NAMESPACE}/{path}"


class UnknownInterface(Exception):
    pass


class RosBridge:
    def __init__(self):
        self.node = None
        self.executor = None
        self.thread = None
        self.loop = None  # asyncio loop, set from the FastAPI startup hook
        self._lock = threading.Lock()
        self._service_types = {}
        self._topic_types = {}
        self._action_types = {}
        self._clients = {}
        self._telemetry = {}  # full topic name -> {"msg": jsonable, "stamp": float}
        self._telemetry_subs = {}  # full topic name -> rclpy subscription
        self._telemetry_lock = threading.Lock()
        self._started = time.time()

    # ---------------------------------------------------------- lifecycle
    def start(self):
        rclpy.init()
        self.node = Node("gateway", namespace=NAMESPACE)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True,
                                       name="rclpy-executor")
        self.thread.start()
        self._discover_telemetry()
        self.node.create_timer(DISCOVERY_PERIOD_S, self._discover_telemetry)

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    @property
    def ok(self):
        return self.node is not None and rclpy.ok()

    @property
    def uptime_s(self):
        return time.time() - self._started

    # ------------------------------------------------------ type discovery
    def refresh_types(self):
        if self.node is None:
            # The gateway serves without rclpy (see main.py); every graph lookup fails here, clearly.
            raise RuntimeError("gateway is not attached to a ROS graph "
                               "(see /api/v1/health: ros_ok)")
        with self._lock:
            self._service_types = {
                name: types[0]
                for name, types in self.node.get_service_names_and_types()
                if types
            }
            self._topic_types = {
                name: types[0]
                for name, types in self.node.get_topic_names_and_types()
                if types
            }
            self._action_types = {
                name: types[0]
                for name, types in get_action_names_and_types(self.node)
                if types
            }

    def _lookup(self, table_name, full_name):
        table = getattr(self, table_name)
        if full_name not in table:
            self.refresh_types()
            table = getattr(self, table_name)
        if full_name not in table:
            raise UnknownInterface(full_name)
        return table[full_name]

    def service_type(self, full_name):
        return self._lookup("_service_types", full_name)

    def topic_type(self, full_name):
        return self._lookup("_topic_types", full_name)

    def action_type(self, full_name):
        return self._lookup("_action_types", full_name)

    def tables(self):
        """Snapshot of all discovered interfaces (for /api/v1/interfaces)."""
        self.refresh_types()
        with self._lock:
            return (dict(self._service_types), dict(self._topic_types),
                    dict(self._action_types))

    # ------------------------------------------------------- asyncio bridge
    async def await_ros_future(self, fut, timeout):
        """Await an rclpy Future from the asyncio loop, with timeout."""
        import asyncio

        aio = self.loop.create_future()

        def _done(f):
            def _transfer():
                if aio.done():
                    return
                exc = f.exception()
                if exc is not None:
                    aio.set_exception(exc)
                else:
                    aio.set_result(f.result())
            self.loop.call_soon_threadsafe(_transfer)

        fut.add_done_callback(_done)
        return await asyncio.wait_for(aio, timeout)

    # ------------------------------------------------------- service calls
    def _client_for(self, full_name, type_str):
        key = (full_name, type_str)
        with self._lock:
            client = self._clients.get(key)
            if client is None:
                srv_cls = get_service(type_str)
                client = self.node.create_client(srv_cls, full_name)
                self._clients[key] = client
        return client

    async def call_service(self, path, body, timeout=10.0):
        """Call a service with a JSON body; raises UnknownInterface, ValueError or asyncio.TimeoutError."""
        full_name = resolve(path)
        type_str = self.service_type(full_name)
        srv_cls = get_service(type_str)
        # nan_for_missing: on a service, an omitted float means "leave it alone".
        request = build_msg(srv_cls.Request, body or {}, nan_for_missing=True)
        client = self._client_for(full_name, type_str)
        fut = client.call_async(request)
        try:
            response = await self.await_ros_future(fut, timeout)
        except Exception:
            client.remove_pending_request(fut)
            raise
        return msg_to_jsonable(response)

    # ------------------------------------------------------- subscriptions
    def qos_for_topic(self, full_name, depth=10):
        """Mirror the live publisher's QoS, so latched topics latch and best-effort ones stay best-effort."""
        infos = self.node.get_publishers_info_by_topic(full_name)
        durability = DurabilityPolicy.VOLATILE
        reliability = ReliabilityPolicy.RELIABLE
        if infos:
            if any(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                   for i in infos):
                durability = DurabilityPolicy.TRANSIENT_LOCAL
            if all(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                   for i in infos):
                reliability = ReliabilityPolicy.BEST_EFFORT
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=durability,
            reliability=reliability,
        )

    def create_subscription(self, full_name, callback):
        """Subscribe to any topic; callback(jsonable_msg) runs on the executor thread."""
        type_str = self.topic_type(full_name)
        msg_cls = get_message(normalize_msg_type(type_str))

        def _cb(msg):
            callback(msg_to_jsonable(msg))

        return self.node.create_subscription(
            msg_cls, full_name, _cb, self.qos_for_topic(full_name))

    def create_publisher(self, full_name, type_str):
        msg_cls = get_message(normalize_msg_type(type_str))
        return self.node.create_publisher(msg_cls, full_name, 10), msg_cls

    # ---------------------------------------------------------- telemetry
    @staticmethod
    def is_telemetry(full_name):
        """Whether a topic belongs in /status: the fixed list, or a */status or */state name."""
        prefix = NAMESPACE + "/"
        if not full_name.startswith(prefix):
            return False
        rel = full_name[len(prefix):]
        return rel in TELEMETRY_TOPICS or rel.rsplit("/", 1)[-1] in TELEMETRY_SUFFIXES

    def _discover_telemetry(self):
        """Subscribe to new telemetry topics once they have a publisher, whose QoS (latching) is mirrored."""
        try:
            topics = self.node.get_topic_names_and_types()
        except Exception:
            return
        for name, types in topics:
            if not types or not self.is_telemetry(name):
                continue
            with self._telemetry_lock:
                if name in self._telemetry_subs:
                    continue
            type_str = normalize_msg_type(types[0])
            if type_str in BULKY_TYPES:
                continue
            try:
                if not self.node.get_publishers_info_by_topic(name):
                    continue
                msg_cls = get_message(type_str)
                sub = self.node.create_subscription(
                    msg_cls, name, self._telemetry_cb(name),
                    self.qos_for_topic(name, depth=1))
            except Exception:
                continue           # an unimportable type: skip, retry next scan
            with self._telemetry_lock:
                self._telemetry_subs[name] = sub

    def _telemetry_cb(self, full_name):
        def _cb(msg):
            self._telemetry[full_name] = {"msg": msg_to_jsonable(msg),
                                          "stamp": time.time()}
        return _cb

    def telemetry_snapshot(self):
        """The fixed names (null until published), then every discovered topic, relative to /scopio."""
        out = {rel: self._telemetry.get(resolve(rel)) for rel in TELEMETRY_TOPICS}
        prefix = NAMESPACE + "/"
        for full in sorted(list(self._telemetry)):
            rel = full[len(prefix):] if full.startswith(prefix) else full
            out.setdefault(rel, self._telemetry[full])
        return out


bridge = RosBridge()
