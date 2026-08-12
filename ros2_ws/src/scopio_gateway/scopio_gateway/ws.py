"""WebSocket endpoint: live topic streams + long-running actions.

Endpoint:  WS /api/v1/ws?api_key=<key>     (header X-API-Key also accepted)

Every frame in both directions is a JSON object with an "op". The client picks
an "id" per request; all server frames about that request echo the same id.

  client -> server
    {"op":"subscribe",  "id":"s1", "topic":"stage/position", "rate_hz":10}
    {"op":"unsubscribe","id":"s1"}
    {"op":"publish",    "id":"p1", "topic":"some/topic",
                        "type":"pkg/msg/Type",   # optional if topic is live
                        "msg":{...}}
    {"op":"action_send_goal", "id":"g1", "action":"camera/autofocus",
                              "goal":{"z_range":2000,"steps":15,"settle_s":0.2}}
    {"op":"action_cancel",    "id":"g1"}

  server -> client
    {"op":"ok",    "id":"s1"}
    {"op":"error", "id":"s1", "code":"...", "detail":"..."}
    {"op":"message","id":"s1","topic":"stage/position","stamp":..., "msg":{...}}
    {"op":"action_ack",     "id":"g1", "accepted":true}
    {"op":"action_feedback","id":"g1", "feedback":{...}}
    {"op":"action_result",  "id":"g1", "status":"succeeded|aborted|canceled",
                            "result":{...}}

error codes: unauthorized | bad_request | unknown_topic | unknown_action |
             use_mjpeg | bad_fields | timeout

Notes:
  * Video topics (CompressedImage/Image) are refused with use_mjpeg -- fetch
    /api/v1/stream.mjpg instead. Everything else streams fine as JSON.
  * rate_hz decimates server-side (drop, not queue) so slow clients never
    backpressure the ROS executor. The outbound queue also drops on overflow.
  * Closing the socket destroys subscriptions but does NOT cancel running
    action goals (ROS semantics: a goal outlives its caller unless canceled).
"""

import asyncio
import time

from fastapi import WebSocket, WebSocketDisconnect
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action

from .auth import keystore
from .conversion import BULKY_TYPES, build_msg, msg_to_jsonable, normalize_msg_type
from .ros_bridge import UnknownInterface, bridge, resolve

# GoalStatus values (action_msgs/msg/GoalStatus)
_STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}

OUTBOUND_QUEUE_SIZE = 256


class Connection:
    def __init__(self, websocket: WebSocket):
        self.ws = websocket
        self.queue = asyncio.Queue(maxsize=OUTBOUND_QUEUE_SIZE)
        self.subs = {}      # id -> rclpy subscription
        self.pubs = {}      # (topic, type) -> publisher
        self.pubs_cls = {}  # (topic, type) -> message class
        self.goals = {}     # id -> {"client": ActionClient, "handle": goal_handle}

    # Called from BOTH threads; loop.call_soon_threadsafe from executor side.
    def push(self, envelope):
        try:
            self.queue.put_nowait(envelope)
        except asyncio.QueueFull:
            pass  # drop -- telemetry must never block the graph

    def push_threadsafe(self, envelope):
        bridge.loop.call_soon_threadsafe(self.push, envelope)

    def cleanup(self):
        for sub in self.subs.values():
            try:
                bridge.node.destroy_subscription(sub)
            except Exception:
                pass
        for pub in self.pubs.values():
            try:
                bridge.node.destroy_publisher(pub)
            except Exception:
                pass
        for goal in self.goals.values():
            try:
                goal["client"].destroy()
            except Exception:
                pass
        self.subs.clear()
        self.pubs.clear()
        self.goals.clear()


def _err(conn, msg_id, code, detail=""):
    conn.push({"op": "error", "id": msg_id, "code": code, "detail": detail})


def _handle_subscribe(conn, req):
    msg_id, topic = req.get("id"), req.get("topic")
    if not msg_id or not topic:
        return _err(conn, msg_id, "bad_request", "subscribe needs 'id' and 'topic'")
    if msg_id in conn.subs:
        return _err(conn, msg_id, "bad_request", f"id '{msg_id}' already subscribed")
    full = resolve(topic)
    try:
        type_str = bridge.topic_type(full)
    except UnknownInterface:
        return _err(conn, msg_id, "unknown_topic", f"No topic {full} in the graph")
    if normalize_msg_type(type_str) in BULKY_TYPES:
        return _err(conn, msg_id, "use_mjpeg",
                    "Video frames are not served as JSON -- "
                    "stream GET /api/v1/stream.mjpg instead")

    rate_hz = req.get("rate_hz")
    min_interval = (1.0 / float(rate_hz)) if rate_hz else 0.0
    last_sent = [0.0]

    def _cb(jsonable):  # executor thread
        now = time.monotonic()
        if min_interval and (now - last_sent[0]) < min_interval:
            return
        last_sent[0] = now
        conn.push_threadsafe({"op": "message", "id": msg_id, "topic": topic,
                              "stamp": time.time(), "msg": jsonable})

    conn.subs[msg_id] = bridge.create_subscription(full, _cb)
    conn.push({"op": "ok", "id": msg_id})


def _handle_unsubscribe(conn, req):
    msg_id = req.get("id")
    sub = conn.subs.pop(msg_id, None)
    if sub is None:
        return _err(conn, msg_id, "bad_request", f"no subscription with id '{msg_id}'")
    try:
        bridge.node.destroy_subscription(sub)
    except Exception:
        pass
    conn.push({"op": "ok", "id": msg_id})


def _handle_publish(conn, req):
    msg_id, topic = req.get("id"), req.get("topic")
    if not topic:
        return _err(conn, msg_id, "bad_request", "publish needs 'topic'")
    full = resolve(topic)
    type_str = req.get("type")
    if not type_str:
        try:
            type_str = bridge.topic_type(full)
        except UnknownInterface:
            return _err(conn, msg_id, "unknown_topic",
                        f"{full} has no live publisher -- supply 'type'")
    key = (full, type_str)
    if key not in conn.pubs:
        pub, msg_cls = bridge.create_publisher(full, type_str)
        conn.pubs[key] = pub
        conn.pubs_cls[key] = msg_cls
    try:
        msg = build_msg(conn.pubs_cls[key], req.get("msg") or {})
    except ValueError as exc:
        return _err(conn, msg_id, "bad_fields", str(exc))
    conn.pubs[key].publish(msg)
    conn.push({"op": "ok", "id": msg_id})


async def _handle_action_send_goal(conn, req):
    msg_id, action = req.get("id"), req.get("action")
    if not msg_id or not action:
        return _err(conn, msg_id, "bad_request",
                    "action_send_goal needs 'id' and 'action'")
    if msg_id in conn.goals:
        return _err(conn, msg_id, "bad_request", f"goal id '{msg_id}' already in use")
    full = resolve(action)
    try:
        type_str = bridge.action_type(full)
    except UnknownInterface:
        return _err(conn, msg_id, "unknown_action", f"No action {full} in the graph")
    act_cls = get_action(type_str)
    try:
        goal = build_msg(act_cls.Goal, req.get("goal") or {})
    except ValueError as exc:
        return _err(conn, msg_id, "bad_fields", str(exc))

    client = ActionClient(bridge.node, act_cls, full)
    for _ in range(20):  # ~2 s for the server to appear (it should already)
        if client.server_is_ready():
            break
        await asyncio.sleep(0.1)
    else:
        client.destroy()
        return _err(conn, msg_id, "timeout", f"Action server {full} not ready")

    def _feedback(fb):  # executor thread
        conn.push_threadsafe({"op": "action_feedback", "id": msg_id,
                              "feedback": msg_to_jsonable(fb.feedback)})

    try:
        handle = await bridge.await_ros_future(
            client.send_goal_async(goal, feedback_callback=_feedback), timeout=10.0)
    except asyncio.TimeoutError:
        client.destroy()
        return _err(conn, msg_id, "timeout", "Goal was not acknowledged in time")

    if not handle.accepted:
        client.destroy()
        conn.push({"op": "action_ack", "id": msg_id, "accepted": False})
        return

    conn.goals[msg_id] = {"client": client, "handle": handle}
    conn.push({"op": "action_ack", "id": msg_id, "accepted": True})

    def _result_done(fut):  # executor thread
        try:
            wrapped = fut.result()
            envelope = {
                "op": "action_result", "id": msg_id,
                "status": _STATUS.get(wrapped.status, str(wrapped.status)),
                "result": msg_to_jsonable(wrapped.result),
            }
        except Exception as exc:
            envelope = {"op": "error", "id": msg_id, "code": "timeout",
                        "detail": f"result error: {exc}"}
        conn.push_threadsafe(envelope)
        # Retire the goal + its client once finished.
        def _retire():
            entry = conn.goals.pop(msg_id, None)
            if entry:
                try:
                    entry["client"].destroy()
                except Exception:
                    pass
        bridge.loop.call_soon_threadsafe(_retire)

    handle.get_result_async().add_done_callback(_result_done)


async def _handle_action_cancel(conn, req):
    msg_id = req.get("id")
    entry = conn.goals.get(msg_id)
    if entry is None:
        return _err(conn, msg_id, "bad_request", f"no running goal with id '{msg_id}'")
    try:
        await bridge.await_ros_future(entry["handle"].cancel_goal_async(), timeout=5.0)
        conn.push({"op": "ok", "id": msg_id})
    except asyncio.TimeoutError:
        _err(conn, msg_id, "timeout", "cancel not acknowledged")


async def websocket_endpoint(websocket: WebSocket):
    presented = (websocket.query_params.get("api_key")
                 or websocket.headers.get("x-api-key"))
    await websocket.accept()
    if keystore.check(presented) is None:
        await websocket.send_json({"op": "error", "id": None, "code": "unauthorized",
                                   "detail": "Missing or invalid api_key"})
        await websocket.close(code=4401)
        return

    conn = Connection(websocket)

    async def _sender():
        while True:
            envelope = await conn.queue.get()
            await websocket.send_json(envelope)

    sender_task = asyncio.create_task(_sender())
    try:
        while True:
            try:
                req = await websocket.receive_json()
            except ValueError:
                conn.push({"op": "error", "id": None, "code": "bad_request",
                           "detail": "frames must be JSON objects"})
                continue
            op = req.get("op")
            if op == "subscribe":
                _handle_subscribe(conn, req)
            elif op == "unsubscribe":
                _handle_unsubscribe(conn, req)
            elif op == "publish":
                _handle_publish(conn, req)
            elif op == "action_send_goal":
                await _handle_action_send_goal(conn, req)
            elif op == "action_cancel":
                await _handle_action_cancel(conn, req)
            else:
                _err(conn, req.get("id"), "bad_request", f"unknown op '{op}'")
    except WebSocketDisconnect:
        pass
    finally:
        sender_task.cancel()
        conn.cleanup()
