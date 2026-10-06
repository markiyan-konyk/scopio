# API

The client protocol of the SCOPIO gateway: every endpoint, the JSON rules, and the WebSocket. Read this if you write a client. The SDK in [scopio-apps](https://github.com/markiyan-konyk/scopio-apps) already implements all of it.

The base URL is `http://<pi-ip>:8000`. Every route is under `/api/v1`. Interactive docs for the routes are at `http://<pi-ip>:8000/docs`.

## Authentication

Send the key on every request, except `/api/v1/health`, in one of two ways:

| Way | Use it for |
|---|---|
| Header `X-API-Key: <key>` | everything you can set headers on (preferred) |
| Query parameter `?api_key=<key>` | a browser `<img>` tag or a WebSocket, which cannot set headers |

A missing or wrong key gets a 401 (HTTP) or an `unauthorized` error frame and close code 4401 (WebSocket). Keys are made on the Pi, see [SECURITY.md](SECURITY.md).

## Endpoints

| Method and path | Key | What it does |
|---|---|---|
| `GET /api/v1/health` | no | Liveness: `ok`, `ros_ok`, `camera_ok`, `auth_configured`, `uptime_s` |
| `GET /api/v1/status` | yes | A snapshot of every status topic (see [Status](#status)) |
| `GET /api/v1/interfaces` | yes | Every service, topic and action in the live graph, with field schemas |
| `POST /api/v1/service/{path}` | yes | Call any ROS service. `path` is relative to `/scopio`, e.g. `stage/jog` |
| `WS /api/v1/ws` | yes | Topic streams, publishing, and actions (see [WebSocket](#websocket)) |
| `GET /api/v1/stream.mjpg` | yes | Live camera video as MJPEG (`multipart/x-mixed-replace; boundary=FRAME`) |
| `GET /api/v1/camera/controls` | yes | Current camera settings, the sensor modes on offer, and the frame counter |
| `POST /api/v1/camera/controls` | yes | Set camera settings. Partial JSON: `framerate`, `exposure`, `analogue_gain`, `red_gain`, `blue_gain`, `contrast`, `saturation`, `brightness`, `sharpness` |
| `POST /api/v1/camera/mode` | yes | `{"mode": "detail"}` or `{"mode": "fast"}` |
| `POST /api/v1/camera/white_balance` | yes | One-shot auto white balance, then lock the gains |
| `GET /api/v1/camera/focus` | yes | A cheap focus metric: the size of the latest JPEG |

The camera routes pass the camera server's replies and error codes through. They are described in [camera_server/README.md](../camera_server/README.md#endpoints).

## Calling a service

```
POST /api/v1/service/stage/jog?timeout=10
{"dx": 100, "dy": 0, "dz": 0}
```

- The body maps to the request fields of that service. `GET /api/v1/interfaces` lists them.
- `timeout` is in seconds, more than 0 and at most 120. The default is 10.
- The reply is the service's response message as JSON, e.g. `{"success": true, "message": "ok", "x": 100, "y": 0, "z": 0}`.
- A path that starts with `/` is used as an absolute ROS name.

| Code | Meaning |
|---|---|
| 200 | The service answered. Check `success` in the body: the hardware can still refuse. |
| 401 | No key, or a wrong key. |
| 404 | No such service in the graph. |
| 422 | A field in the body is unknown or has the wrong type. Also an out-of-range `timeout`. |
| 503 | The gateway is not attached to the ROS graph (`ros_ok: false`). |
| 504 | The service did not answer within `timeout`. The node is down or the hardware is stuck. |

## The NaN rule

Several services take a partial "set only these fields" request. They use NaN to mean "leave this field unchanged". JSON has no NaN, so the gateway converts:

| You send, on a float field | The node receives |
|---|---|
| the field left out of a **service** request | NaN (leave unchanged) |
| `null` or the string `"nan"` | NaN |
| the field left out of an **action goal** or a published message | 0.0 |

On the way back, NaN and infinity become `null`. Byte arrays become lists of numbers.

This is why `POST /api/v1/service/camera/set_controls` with `{"contrast": 1.2}` changes only the contrast. Without the rule, every omitted gain would be set to zero. Integer fields have no NaN: an omitted integer is 0, and each service says what 0 means.

## Instrument calls

An instrument node exposes its whole driver class over one service, `<name>/call`. Today there are two: `awg/call` (the Rigol DG1022Z) and `temperature/call` (the TC10 LAB).

```
POST /api/v1/service/awg/call
{"method": "update", "args": "[1, 0.2]", "kwargs": ""}
```

| Field | Type | Meaning |
|---|---|---|
| `method` | string | A public method of the driver, or a meta-method |
| `args` | string | A JSON array of positional arguments. `""` means none. A bare scalar such as `"25.0"` counts as one argument. |
| `kwargs` | string | A JSON object of keyword arguments. `""` means none. |

`args` and `kwargs` are JSON **inside a string**. The reply is `{"success", "result", "error"}`, and `result` is the return value as a JSON string (`"null"` for none).

Every instrument node also answers three meta-methods:

| Method | Returns |
|---|---|
| `list_methods` | `[{name, signature, doc}]` for every callable method. Works while the instrument is disconnected. |
| `connected` | whether the node holds a live session |
| `reconnect` | re-opens the session; `true` on success |

Call `list_methods` instead of trusting examples: the methods are whatever the driver class has today. The `doc` is the first line of each method's docstring.

A bad method name, bad JSON or the wrong number of arguments gets `success: false` with the reason. It does not count against the instrument's health.

## Status

`GET /api/v1/status` returns:

```json
{
  "telemetry": {
    "stage/position": {"msg": {"connected": true, "x": 0, "...": "..."}, "stamp": 1759600000.0},
    "camera/state": null,
    "...": "..."
  },
  "camera_ok": true,
  "gateway_uptime_s": 123.4
}
```

- Six names are always present: `stage/position`, `camera/state`, `awg/status`, `temperature/status`, `relay/state`, `calibration`. A name is `null` until its node has published.
- Any other topic under `/scopio` whose last part is `status` or `state` appears as well, once it has a publisher. The gateway rescans the graph every 5 s.
- `stamp` is the Unix time the gateway received the message.

## WebSocket

Connect to `ws://<pi-ip>:8000/api/v1/ws?api_key=<key>`. Every frame in both directions is a JSON object with an `op`. You choose an `id` per request, and every server frame about that request carries the same `id`.

**Client to server**

| Frame | Does |
|---|---|
| `{"op": "subscribe", "id": "s1", "topic": "stage/position", "rate_hz": 10}` | Streams a topic. `rate_hz` is optional; without it every message is sent. |
| `{"op": "unsubscribe", "id": "s1"}` | Stops that stream. |
| `{"op": "publish", "id": "p1", "topic": "some/topic", "type": "pkg/msg/Type", "msg": {…}}` | Publishes one message. `type` may be left out if the topic already has a publisher. |
| `{"op": "action_send_goal", "id": "g1", "action": "camera/autofocus", "goal": {…}}` | Starts an action. |
| `{"op": "action_cancel", "id": "g1"}` | Cancels it. |

**Server to client**

| Frame | Means |
|---|---|
| `{"op": "ok", "id": …}` | The request worked. |
| `{"op": "error", "id": …, "code": …, "detail": …}` | It did not (codes below). |
| `{"op": "message", "id": …, "topic": …, "stamp": …, "msg": {…}}` | One topic message. |
| `{"op": "action_ack", "id": …, "accepted": true}` | The goal was accepted (or `false`, refused). |
| `{"op": "action_feedback", "id": …, "feedback": {…}}` | Progress. |
| `{"op": "action_result", "id": …, "status": "succeeded", "result": {…}}` | The end. `status` is `succeeded`, `aborted` or `canceled`. |

| Error code | Cause |
|---|---|
| `unauthorized` | Bad key. The socket closes with code 4401. |
| `bad_request` | A malformed frame, a missing `id`, a reused `id`, an unknown `op`, or anything else that failed |
| `unknown_topic` / `unknown_action` | No such name in the graph |
| `use_mjpeg` | You subscribed to a video topic. Use `GET /api/v1/stream.mjpg`. |
| `bad_fields` | The message or goal has a wrong field |
| `timeout` | The action server did not appear within about 2 s, or did not acknowledge the goal within 10 s |

Behaviour worth knowing:

- One bad request costs one error frame, never the connection.
- Slow clients lose messages instead of slowing the graph: `rate_hz` drops messages on the server, and the outgoing queue (256 frames) drops when full.
- Latched topics (`calibration`, `relay/state`) deliver their last value as soon as you subscribe.
- Closing the socket ends your subscriptions. It does **not** cancel running goals: an action runs to the end unless you cancel it.

## Next

- [scopio_interfaces/README.md](../ros2_ws/src/scopio_interfaces/README.md): every request and message field
- [scopio_gateway/README.md](../ros2_ws/src/scopio_gateway/README.md): how the gateway works inside
- [SECURITY.md](SECURITY.md): keys and exposure
