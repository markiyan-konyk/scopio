# scopio_gateway

How the API gateway works inside: the one process that turns authenticated HTTP and WebSocket requests into ROS 2 service calls, subscriptions and action goals. The client-facing protocol is in [docs/API.md](../../../docs/API.md).

It runs as `ros2 run scopio_gateway gateway` in the `gateway` container, from the same image as the nodes, on port 8000.

## Modules

| File | Job |
|---|---|
| `main.py` | Entry point. Starts the ROS side, then runs uvicorn. |
| `app.py` | The FastAPI routes. |
| `auth.py` | API-key checking and the hot-reloaded key file. |
| `ros_bridge.py` | The gateway's node in the graph: name resolution, type discovery, service calls, subscriptions, the telemetry cache. |
| `conversion.py` | JSON to ROS messages and back, including the NaN rule. |
| `introspection.py` | `GET /api/v1/interfaces`. |
| `ws.py` | The WebSocket protocol. |
| `camera_proxy.py` | The authenticated proxy to the loopback camera server. |

## Endpoints

All under `/api/v1`. Full request and reply formats are in [docs/API.md](../../../docs/API.md#endpoints).

| Route | Handled by |
|---|---|
| `GET health` (no key) | `app.py`: `ros_ok`, `camera_ok`, `auth_configured`, `uptime_s` |
| `GET status` | `ros_bridge.telemetry_snapshot()` |
| `GET interfaces` | `introspection.interfaces_payload()` |
| `POST service/{path}` | `ros_bridge.call_service()` |
| `WS ws` | `ws.websocket_endpoint()` |
| `GET stream.mjpg` | `camera_proxy.mjpeg_stream()` |
| `GET`/`POST camera/controls`, `POST camera/mode`, `POST camera/white_balance`, `GET camera/focus` | `camera_proxy.forward()` |

`GET /` returns links to `/docs` and `/api/v1/health`. FastAPI serves interactive docs at `/docs`, built from the route docstrings.

## Threads

uvicorn and FastAPI own the main thread's asyncio loop. One rclpy node, `/scopio/gateway`, spins on a `MultiThreadedExecutor` (4 threads) in a background thread. Request handlers never spin rclpy, and rclpy callbacks never touch asyncio directly: everything crosses with `loop.call_soon_threadsafe`. `await_ros_future()` turns an rclpy future into an awaitable with a timeout.

**It serves even when ROS is down.** If the rclpy side fails to start, `main.py` prints "Gateway starting WITHOUT ROS" and runs uvicorn anyway. `/health` then reports `ros_ok: false`, and graph routes answer 503. Crashing instead would crash-loop the container, and nothing would answer on 8000 to say why.

## Authentication

`auth.py` reads `SCOPIO_API_KEYS_FILE` (`/secrets/api_keys.json`, mounted read-only from `ros2_ws/secrets`).

- The key comes from the `X-API-Key` header or the `api_key` query parameter.
- The file is re-read whenever its modification time changes, so keys can be added or revoked without a restart.
- **Fail closed**: a missing, empty or unreadable file means no key matches, and every keyed route answers 401.
- Every stored key is compared in constant time (`hmac.compare_digest`), with no early exit. A non-ASCII key is simply wrong (401), never a crash.
- The WebSocket checks the key after accepting, sends an `unauthorized` error frame, and closes with code 4401.

## Name resolution and discovery

`ros_bridge.resolve()` makes `stage/jog` into `/scopio/stage/jog`. A path that starts with `/` is left as it is.

Types are looked up in the live graph and cached. A miss triggers one refresh before answering 404, so a node started after the gateway is reachable at once. Service clients are cached per name and type. Nothing about the nodes is hard-coded.

`introspection.py` lists every service, topic and action with its field schema, minus ROS plumbing (`/rosout`, `/parameter_events`, `rcl_interfaces`, `type_description_interfaces` and the `_action/` internals). Each topic says whether it can be subscribed over the WebSocket.

## The JSON and ROS conversion

`conversion.build_msg()` builds a message from a JSON dict with `rosidl_runtime_py.set_message_fields`. A wrong field or type raises `ValueError`, which becomes a 422 or a `bad_fields` frame.

**The NaN rule.** Several services use NaN as "leave this field unchanged". JSON has no NaN, and a float left out would otherwise be built as 0.0: an order to set that gain to zero. So, on the way in:

| Input on a float field | Becomes |
|---|---|
| left out of a **service** request | NaN |
| `null`, or the string `"nan"` | NaN |
| `null` inside a float array | NaN |
| left out of an action goal or a published message | 0.0 |

Every float field in the current interfaces belongs to a service that takes partial requests. Fields where 0 is meaningful (positions, ranges, step counts) are integers. Action goals are complete requests, where an absent `settle_s` really means "do not pause". That is why NaN-for-missing is a flag on the call (`nan_for_missing=True` for services only), not a property of the field. It applies even to an empty body: `POST camera/set_controls {}` must change nothing.

On the way out, `msg_to_jsonable()` turns NaN and infinity into `null`, and byte arrays into lists (capped at 65536 bytes).

Video types (`sensor_msgs/CompressedImage`, `sensor_msgs/Image`) are never serialised to JSON. The WebSocket refuses them with `use_mjpeg`, and the telemetry cache skips them.

## Telemetry discovery

`GET /api/v1/status` reads from a cache the gateway keeps by subscribing to small state topics:

- **Always listed**: `stage/position`, `camera/state`, `awg/status`, `temperature/status`, `relay/state`, `calibration`. Each is `null` until its node publishes. Clients key on these names, so they never go missing.
- **Discovered**: any topic under `/scopio` whose last part is `status` or `state`. The graph is rescanned every 5 s (`DISCOVERY_PERIOD_S`), so a node that starts later, or a new node, appears without a gateway change.

A topic is subscribed only once it has a publisher, because the subscription copies the publisher's QoS. That copy is what makes latched topics (`calibration`, `relay/state`) deliver their retained value to the gateway. A subscription made before the publisher existed would have to guess the durability, and a wrong guess on a publish-on-change topic receives nothing, ever.

## The WebSocket

`ws.py` keeps one `Connection` per socket: its subscriptions, publishers and running goals, and an outgoing queue of 256 frames.

- rclpy callbacks push frames with `call_soon_threadsafe`. A full queue drops frames, so telemetry never blocks the graph.
- `rate_hz` thins messages in the callback (drop, not queue).
- One bad request costs one `error` frame. Any exception in a handler is caught and reported, so a typo cannot end the socket and every other subscription on it.
- For an action, the gateway waits up to about 2 s for the action server, then up to 10 s for the goal to be accepted. The result, when it comes, is sent as `action_result` and the action client is destroyed.
- **Goals outlive the socket.** Closing the socket destroys its subscriptions, publishers and action clients, but sends no cancel. The goal runs to the end, as in ROS.

## The camera proxy

The camera server listens on `127.0.0.1:8081` with no authentication. `camera_proxy.py` is its authenticated front door.

- `GET stream.mjpg` passes the server's bytes through unchanged, with no decode and no re-encode. It checks the camera first, so a dead camera gives a clean 503 instead of an empty stream.
- The control routes forward small JSON requests and **pass the server's status codes through**: a 503 "sensor not open" or a 409 "no white balance on a monochrome sensor" reaches the client as that code, not as a 200. A non-JSON reply becomes 502; an unreachable server, 503.
- `camera_ok` is a cached (3 s) check that `GET /controls` returns 200.
- `CAMERA_URL` empty means "no camera": every camera route answers 503. The dev compose uses that.

## Environment

| Variable | Default | Meaning |
|---|---|---|
| `SCOPIO_GATEWAY_PORT` | `8000` | Listen port |
| `SCOPIO_GATEWAY_HOST` | `0.0.0.0` | Bind address |
| `SCOPIO_API_KEYS_FILE` | `/secrets/api_keys.json` | The key file |
| `CAMERA_URL` | `http://127.0.0.1:8081` | The camera server; empty for none |

## Next

- [docs/API.md](../../../docs/API.md): the client protocol
- [docs/SECURITY.md](../../../docs/SECURITY.md): keys and exposure
- [camera_server/README.md](../../../camera_server/README.md): the server behind the camera routes
