# ROS 2 from zero, taught through SCOPIO

> ⚠️ **Teaching document — predates both the v1.0 interface freeze AND the
> API-gateway rewrite.** Two kinds of staleness here:
> 1. Example interface names (`LaserState`, `ZeroTweezers`, `recording/*`,
>    `RunGalvoWaveform`, `tweezers/zero`) are from an early scaffold and were
>    renamed/removed in v1.0. For the real, current contract see
>    the `.msg`/`.srv`/`.action` files in `src/scopio_interfaces/`.
> 2. §9 (Docker) and §10 (architecture) describe an earlier design where a
>    `scopio_ui`/`ui_gateway` package lived *inside* this workspace and served
>    the browser directly from the Pi. That package never shipped this way —
>    today the UI is `../../ui` (a separate, external, plain-Flask program)
>    and there is also a `scopio_gateway` package in *this* workspace (a
>    FastAPI+rclpy HTTP/WebSocket API, not a browser-facing UI). See
>    [../../../docs/API.md](../../../docs/API.md) and
>    [../README.md](../README.md) for what's actually there now.
>
> The ROS **concepts** below (nodes, topics, services, actions, executors,
> colcon/ament) are all still accurate and worth learning from — only the
> specific package/file names in §9-10 are out of date.

You know how to program; you don't know ROS 2 yet. This document teaches ROS 2
from the ground up using **the exact code in this repo** as the running example.
By the end you'll understand every file under `ros2_ws/` and be able to write
your own node.

Read it top to bottom once. Then keep it open as a reference while you poke at
the running system.

**Contents**
1. [The mental model](#1-the-mental-model)
2. [The five concepts](#2-the-five-concepts-node-topic-service-action-parameter)
3. [Anatomy of a node, line by line](#3-anatomy-of-a-node-line-by-line)
4. [Our system as a graph](#4-our-system-as-a-graph)
5. [Interfaces: msg / srv / action](#5-interfaces-msg--srv--action)
6. [The build system: workspaces, colcon, ament](#6-the-build-system-workspaces-colcon-ament)
7. [Running things: run, launch, params](#7-running-things-run-launch-params)
8. [Executors, callbacks, threads](#8-executors-callbacks-and-threads)
9. [Docker, explained line by line](#9-docker-explained-line-by-line)
10. [The whole architecture](#10-the-whole-architecture)
11. [Write your own node from scratch](#11-write-your-own-node-from-scratch)
12. [Cheat sheet & glossary](#12-cheat-sheet--glossary)

---

## 1. The mental model

**ROS 2 is not a program. It's a way for many small programs to talk.**

You already know patterns for "programs talking":
- HTTP/REST (your Flask app): a client asks a server, server replies.
- WebSockets: a long-lived two-way pipe.
- Function calls: caller invokes callee directly.

ROS 2 is a **publish/subscribe + request/response middleware** designed for
robots, where you have *many* independent processes (camera, motors, planner,
UI, logger…) that must share data and commands with low latency, possibly
across multiple computers, and keep working if one of them dies.

The core picture is **the graph**: a set of **nodes** (processes) connected by
named **topics**, **services**, and **actions**. Nodes don't know each other's
addresses. They announce "I publish on `/scopio/stage/position`" or "I offer a
service `/scopio/awg/call`", and the middleware (**DDS**, the layer under ROS 2)
automatically connects whoever's interested. This is **discovery**: start a node
anywhere on the network and it just wires itself in.

> **Why this is the right tool for SCOPIO.** Your microscope is becoming a robot
> with a camera (sensor), a stage and a galvo laser (effectors), and an external
> "brain" computer that makes decisions. That brain is just another node on the
> graph. No bespoke protocol, no IP addresses hard-coded, no "if the network
> hiccups the control loop dies." That's what ROS buys you.

**ROS 2 vs your Flask app, concretely.** Today `microscope/` is one process that
*owns the hardware and serves the UI*. In ROS, those split into separate nodes
that talk over the graph. The UI stops owning hardware and becomes a node that
*subscribes and asks* — that's `scopio_ui`. More on that in §10.

---

## 2. The five concepts: node, topic, service, action, parameter

These five are 90% of ROS 2. Learn them and the rest is detail.

### Node
A **node** is one process that does one job and participates in the graph. In
this repo each node is a Python class subclassing `rclpy.node.Node`:

- `camera_node` — owns the camera.
- `stage_node` — owns the stage.
- `galvo_node` — owns the laser.
- `calibration_node` — owns the µm/px + steps/µm calibration (no hardware).
- `gateway` — the HTTP/WebSocket API (a pure client of the graph; owns no
  hardware).

`rclpy` is the **ROS Client Library for Python** — the API you call to make a
node, publish, subscribe, etc. (C++ has `rclcpp`; same concepts.)

### Topic (publish / subscribe) — for *streams of data*
A **topic** is a named, typed channel. Any node can **publish** messages to it;
any node can **subscribe**. Publishers and subscribers don't know about each
other. Use topics for continuous data where "the latest value" is what matters.

In SCOPIO: `camera_node` publishes frames on `image/compressed`; the `gateway`
subscribes (and so could any future node). `stage_node` publishes
`stage/position` at 5 Hz; anyone who cares subscribes. Fire-and-forget,
one-to-many.

```python
# publisher (stage_node.py)
self.pos_pub = self.create_publisher(StagePosition, "stage/position", 5)
self.pos_pub.publish(msg)

# subscriber (galvo_node.py) — wants stage pos to compute laser global position
self.create_subscription(StagePosition, "stage/position", self._on_stage, 5)
def _on_stage(self, msg):     # called by ROS whenever a message arrives
    self.stage_pos = {"x": msg.x, "y": msg.y, "z": msg.z}
```

The `5` is the **queue depth** (part of QoS — Quality of Service). It's how many
messages to buffer if the subscriber is briefly slow.

### Service (request / response) — for *commands with an immediate answer*
A **service** is a function call across the graph: a client sends a request, the
server runs and sends back a response. Synchronous-feeling, one-to-one. Use it
for quick commands: "zero the tweezers," "start recording," "turn tracking on."

In SCOPIO: `galvo_node` offers `tweezers/zero`; the UI calls it.

```python
# server (galvo_node.py)
self.create_service(ZeroTweezers, "tweezers/zero", self._on_zero)
def _on_zero(self, request, response):
    tweezer.set_home(self.galvo.vx, self.galvo.vy)
    response.success = True
    return response

# client (gateway_node.py)
self.cli_zero = self.create_client(ZeroTweezers, "tweezers/zero")
future = self.cli_zero.call_async(ZeroTweezers.Request())
```

### Action — for *long jobs with feedback you can cancel*
An **action** is a service's big sibling, built for tasks that take a while: you
send a **goal**, get streaming **feedback**, and eventually a **result** — and
you can **cancel** mid-way. Internally it's built from topics + services, but you
use it as one unit.

This is the perfect fit for your "send the whole path, run it, report back"
idea. In SCOPIO the actions are:
- `stage/move_path` — visit a list of stage targets.
- `galvo/run_waveform` — program a prolonged galvo waveform and run it.
- `scan_region` — jump-scan a grid of frames.

```python
# server (stage_node.py)
self._move_server = ActionServer(self, MoveStagePath, "stage/move_path",
                                 execute_callback=self._execute_move_path)
def _execute_move_path(self, goal_handle):
    for i, pt in enumerate(goal_handle.request.points):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled(); ...
        self._move_abs(pt.x, pt.y, pt.z)
        fb = MoveStagePath.Feedback(); fb.current_index = i
        goal_handle.publish_feedback(fb)     # stream progress
    goal_handle.succeed()
    return MoveStagePath.Result(success=True, points_reached=len(...))
```

**When to use which:**

| Need | Use |
|---|---|
| Continuous stream, latest value matters | **Topic** |
| Quick command, want an immediate yes/no | **Service** |
| Long task, want progress + cancel | **Action** |

### Parameter — for *configuration*
**Parameters** are named, typed settings a node declares and reads at startup
(and can be changed at runtime). They're how you configure a node without editing
code. We set them from `config/params.yaml`.

```python
# camera_node.py
self.declare_parameter("jpeg_quality", 70)     # declare with a default
self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)   # read it
```

```yaml
# config/params.yaml
/scopio/camera_node:
  ros__parameters:
    jpeg_quality: 70
    publish_fps: 15.0
```

### Namespaces (one more, small but important)
Every name (node, topic, service) lives in a **namespace**, like a folder path.
We launch all our nodes in the `scopio` namespace, so a node that publishes the
*relative* name `stage/position` actually publishes the *absolute* name
`/scopio/stage/position`. This keeps everything tidy and lets you run two
microscopes on one network under different namespaces. That's why the code uses
short relative names and the launch files set `namespace="scopio"`.

---

## 3. Anatomy of a node, line by line

Open [galvo_node.py](../src/scopio_microscope/scopio_microscope/galvo_node.py)
and follow along. Every ROS 2 Python node has this skeleton:

```python
import rclpy
from rclpy.node import Node

class GalvoNode(Node):                       # 1. subclass Node
    def __init__(self):
        super().__init__("galvo_node")       # 2. name the node

        self.declare_parameter("resource", "")          # 3. parameters
        # ... create publishers / subscriptions / services / actions / timers ...
        self.state_pub = self.create_publisher(LaserState, "laser/state", 5)
        self.create_subscription(StagePosition, "stage/position", self._on_stage, 5)
        self.create_service(ZeroTweezers, "tweezers/zero", self._on_zero)
        self.create_timer(1.0 / rate, self._publish_state)   # 4. periodic work

def main(args=None):                         # 5. the entry point
    rclpy.init(args=args)                    #    start ROS
    node = GalvoNode()                       #    build the node
    rclpy.spin(node)                         #    hand control to ROS forever
    node.destroy_node()
    rclpy.shutdown()
```

The crucial idea is in step 5: **`rclpy.spin(node)`**. A ROS node is
event-driven. After setup, you call `spin`, which blocks forever and runs your
**callbacks** when things happen: a subscription message arrives → your
`_on_stage` runs; a timer fires → your `_publish_state` runs; a service request
comes in → your `_on_zero` runs. You never write a `while True` loop yourself for
the main flow; ROS's event loop (the **executor**, §8) calls you.

**Timers** are how you do periodic work (publishing state at 10 Hz, etc.):
`create_timer(period_seconds, callback)`. Our nodes publish their state on a
timer and react to commands in callbacks.

**Graceful degradation pattern** (used in every driver node): hardware setup is
wrapped in try/except so a missing device doesn't crash the node — it just sets
the handle to `None` and reports `connected=false`. That's why the graph comes up
even on an incomplete rig. Look at `_connect_galvo()`:

```python
try:
    self.galvo = galvo_driver.Galvo(resource)
except Exception as e:
    self.get_logger().warning(f"Galvo connect failed ({e}); laser disabled.")
    self.galvo = None
```

`self.get_logger()` is the ROS logger — use it instead of `print()`; output is
tagged with the node name and severity and shows up in `docker compose logs`.

---

## 4. Our system as a graph

Here's every node and how they're wired. `pub`/`sub` are topics; `srv`
services; `act` actions. Everything is under `/scopio`.

```
   camera_server (picamera2, loopback :8081)
       │  MJPEG
       ▼
   camera_node ──── pub image/compressed ──────────────┐
       │  pub camera/state                             │
       │  srv camera/set_controls, set_framerate,      │
       │      white_balance                            │
       │  act camera/autofocus ──┐                     │
       │                         │ srv stage/jog       │
       ▼                         ▼                     ▼
   Sangaboard ◄──────────── stage_node            gateway (:8000)
                                │  pub stage/position   │  HTTP + WebSocket
                                │  srv stage/move_abs   │  + API key
                                │  act stage/move_path  │
                                │  act scan_region      │
   DG1022Z (galvo) ◄──── galvo_node                     │
                                │  pub awg/status       │
                                │  srv awg/call,        │
                                │      awg/write, query │
   calibration.json ◄─── calibration_node               │
                                │  pub calibration ─────┘  (latched)
                                │  srv calibration/set
                                ▼
                          stage_node (reads steps_per_um)

           ⇡  everything outside the Pi — the UI, galvo_draw, agents —
              talks ONLY to the gateway over HTTP/WS. No client joins
              the DDS graph.
```

Two things to notice:

1. **Nobody else grabs the camera.** `camera_node` is the single owner and
   publishes frames; anything that wants pixels subscribes (or, off-Pi, pulls
   the MJPEG stream through the gateway). One owner, many consumers — this is
   the ROS way and it's why there's no resource conflict.
2. **The backend senses and effectuates; it does not decide.** There is no
   analysis node in this graph on purpose. Detection, tracking and experiment
   logic live in client programs, which are all the same kind of thing: API
   clients of the gateway. The UI is not special.

---

## 5. Interfaces: msg / srv / action

Topics, services, and actions are **typed**. The types are defined in plain-text
files in the `scopio_interfaces` package, and a code generator (**rosidl**) turns
them into Python/C++ classes at build time.

### A message (`.msg`)
[StagePosition.msg](../src/scopio_interfaces/msg/StagePosition.msg):
```
std_msgs/Header header     # standard timestamp + frame_id
bool connected
int32 x
int32 y
int32 z
```
Each line is `type name`. Types are primitives (`bool`, `int32`, `float32`,
`string`), arrays (`StagePoint[] points`), or other messages (`std_msgs/Header`). After
building, this becomes a Python class you use as `msg.x`, `msg.connected`, etc.

### A service (`.srv`)
Two halves separated by `---`: request fields, then response fields.
[SetLaser.srv](../src/scopio_interfaces/srv/SetLaser.srv):
```
float32 vx
float32 vy
bool relative
---
bool success
float32 vx
float32 vy
string message
```

### An action (`.action`)
Three parts separated by `---`: goal, result, feedback.
[MoveStagePath.action](../src/scopio_interfaces/action/MoveStagePath.action):
```
scopio_interfaces/StagePoint[] points
float32 settle_s
---
bool success
int32 points_reached
---
int32 current_index
int32 x
int32 y
int32 z
```

### Why interfaces are their own package
`scopio_interfaces` is a separate package, and it's an **ament_cmake** package
(C/C++ build) rather than ament_python — because the message code generator
(rosidl) is wired through CMake. You list every interface file in
[CMakeLists.txt](../src/scopio_interfaces/CMakeLists.txt):

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StagePosition.msg"
  "srv/SetLaser.srv"
  "action/MoveStagePath.action"
  DEPENDENCIES std_msgs           # because StagePosition uses std_msgs/Header
)
```

Keeping interfaces in their own package is standard practice: many other
packages (drivers, UI, the external brain) depend on the *contract* without
depending on the *implementation*. Your Python node packages list it as a
dependency in their `package.xml` (`<exec_depend>scopio_interfaces</exec_depend>`)
and then just `from scopio_interfaces.msg import StagePosition`.

---

## 6. The build system: workspaces, colcon, ament

### The workspace
A **workspace** is a folder with a `src/` directory containing packages. Ours is
`ros2_ws/`:
```
ros2_ws/
  src/
    scopio_interfaces/     # the contract (ament_cmake)
    scopio_microscope/     # driver nodes (ament_python)
    scopio_ui/             # gateway (ament_python)
  build/    install/    log/      <-- created by colcon (git-ignored)
```

### colcon: the build tool
**colcon** builds every package in `src/`:
```bash
cd ros2_ws
colcon build --symlink-install
```
- It builds `scopio_interfaces` first (others depend on it), runs rosidl to
  generate the message classes, then builds the Python packages.
- `--symlink-install` symlinks your Python files into `install/` instead of
  copying, so editing a node doesn't require a rebuild (you still rebuild after
  changing interfaces or `setup.py`).
- Output goes to `install/`. `build/` and `log/` are scratch.

### Sourcing (the step everyone forgets)
After building, you must **source** the workspace so ROS can find your packages:
```bash
source /opt/ros/jazzy/setup.bash       # ROS itself
source install/setup.bash              # your workspace, layered on top
```
"Sourcing" just sets environment variables (`PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`)
so `ros2 run scopio_microscope camera_node` resolves. Our
[entrypoint.sh](../entrypoint.sh) does both lines automatically inside the
container — that's its whole job.

### package.xml and setup.py
Every package has a **`package.xml`** — its manifest: name, version,
dependencies, and build type. The build type matters:
```xml
<export><build_type>ament_python</build_type></export>   <!-- or ament_cmake -->
```

For **ament_python** packages, **`setup.py`** is a normal Python setuptools file
with two ROS-specific bits:
1. **`entry_points` → `console_scripts`**: maps a command name to a function.
   This is what makes `camera_node` runnable:
   ```python
   entry_points={"console_scripts": [
       "camera_node = scopio_microscope.camera_node:main",
   ]},
   ```
   Read it as: the executable `camera_node` runs `main()` in
   `scopio_microscope/camera_node.py`.
2. **`data_files`**: installs non-code files (launch files, `params.yaml`, the
   UI frontend) into `install/` so `ros2 launch` and `get_package_share_directory`
   can find them.

So the chain for "run a node" is:
`setup.py entry_point` → installed script → `main()` → `rclpy.init` + `Node` +
`spin`.

---

## 7. Running things: run, launch, params

### Run one node
```bash
ros2 run scopio_microscope camera_node
```
`ros2 run <package> <executable>`. Good for testing one node.

### Launch many nodes
Starting five nodes by hand is tedious. A **launch file** (Python) starts a whole
set with their parameters. [microscope.launch.py](../src/scopio_microscope/launch/microscope.launch.py):
```python
def generate_launch_description():
    params = os.path.join(get_package_share_directory("scopio_microscope"),
                          "config", "params.yaml")
    common = dict(package="scopio_microscope", namespace="scopio",
                  output="screen", parameters=[params])
    return LaunchDescription([
        Node(executable="camera_node", name="camera_node", **common),
        Node(executable="stage_node",  name="stage_node",  **common),
        ...
    ])
```
Each `Node(...)` says which executable to start, what to name it, which namespace
to put it in, and which parameter file to load. Run it with:
```bash
ros2 launch scopio_microscope microscope.launch.py
```

Launch files can **include** other launch files — that's how
[scopio.launch.py](../src/scopio_ui/launch/scopio.launch.py) starts the drivers
*and* the gateway in one command (it's what the container runs by default).

### Parameters from YAML
[params.yaml](../src/scopio_microscope/config/params.yaml) is keyed by the
node's full name:
```yaml
/scopio/galvo_node:
  ros__parameters:
    timeout_ms: 15000
```
The launch file passes this file to each node; each node picks out its own
section by name. Change behaviour without touching code.

### Introspection: the CLI is your microscope into the graph
These commands are how you *see* what's happening — learn them, you'll use them
constantly:
```bash
ros2 node list                       # who's running
ros2 topic list                      # what channels exist
ros2 topic echo /scopio/stage/position     # print messages as they arrive
ros2 topic hz   /scopio/image/compressed   # measure publish rate
ros2 topic info /scopio/stage/position -v  # types, publishers, QoS
ros2 service list
ros2 service call /scopio/awg/query scopio_interfaces/srv/AwgQuery "{command: '*IDN?'}"
ros2 action list
ros2 action send_goal -f /scopio/stage/move_path scopio_interfaces/action/MoveStagePath "{points: [...]}"
ros2 param list /scopio/galvo_node
```

---

## 8. Executors, callbacks, and threads

You don't write the main loop; the **executor** does. `rclpy.spin(node)` is
shorthand for "make a single-threaded executor and run it." The executor watches
all your callbacks (timers, subscriptions, services) and runs them **one at a
time** when work is ready.

**Single-threaded is fine** for simple nodes (calibration_node): one callback
runs to completion before the next starts. Simple and safe.

**But it breaks for long callbacks.** An action's `execute_callback` can run for
*seconds* (moving a stage path). With a single-threaded executor, while that runs
nothing else in the node can — no state publishing, no cancel handling. So nodes
with actions use a **MultiThreadedExecutor** plus a **ReentrantCallbackGroup**,
which lets callbacks run concurrently in a thread pool:

```python
# stage_node.py / galvo_node.py
cb = ReentrantCallbackGroup()
self._move_server = ActionServer(self, MoveStagePath, "stage/move_path",
                                 execute_callback=self._execute_move_path,
                                 callback_group=cb)
...
def main():
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
```

Now the long move can run on one thread while position keeps publishing and a
cancel request can be handled on another. **Rule of thumb:** if a node has an
action server (or any callback that blocks for a while), use a
MultiThreadedExecutor and put the slow callbacks in a reentrant group. When two
callbacks touch the same hardware, guard it with a `threading.Lock` (we use
`tweezer.galvo_move_lock` and the camera's `self._lock`).

**The gateway is a special case** ([gateway_node.py](../src/scopio_ui/scopio_ui/gateway_node.py)):
it runs ROS in a background thread (`MultiThreadedExecutor().spin()` in a
`threading.Thread`) and Flask in the main thread. When a browser request handler
needs to call a ROS service, it can't `spin` (ROS is already spinning elsewhere),
so it fires `call_async` and waits on the resulting future via a
`threading.Event` that the future's done-callback sets. That helper is `_call()`.
Read it — it's the canonical "call a ROS service from non-ROS code" pattern.

---

## 9. Docker, explained line by line

**Why Docker at all?** ROS 2 Jazzy targets Ubuntu 24.04, but your Pi runs
Raspberry Pi OS, and you want the camera stack untouched. A container gives ROS
its own clean Ubuntu userspace while sharing the Pi's kernel and devices. Bonus:
the *same* image runs later on a Jetson or an x86 box with no code changes —
that's the portability you asked about.

### The [Dockerfile](../Dockerfile)
```dockerfile
FROM ros:jazzy-ros-base
```
Start from the official ROS 2 Jazzy image (Ubuntu 24.04 + ROS preinstalled).

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv ffmpeg \
        ros-jazzy-sensor-msgs ros-jazzy-std-srvs \
        python3-picamera2 python3-libcamera python3-kms++ ...
```
Install system deps: OpenCV, ffmpeg (for H.264), the ROS message packages we use,
and the camera stack. `ros-jazzy-sensor-msgs` is the apt package that provides
`sensor_msgs` — ROS packages are just apt packages.

```dockerfile
RUN pip3 install --break-system-packages pyvisa pyvisa-py pyusb pyserial \
        sangaboard fastapi "uvicorn[standard]" httpx
```
Python deps for the hardware drivers + the gateway. `--break-system-packages`
sidesteps Ubuntu's "don't pip into the system Python" guard — fine inside a
throwaway container. Note what is *absent*: no trackpy/pandas/scipy, because
the backend never analyses images (see DECISIONS §8).

```dockerfile
WORKDIR /ros2_ws
COPY . /ros2_ws/
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```
Copy the workspace in and build it *into the image*, so the container starts
ready to run.

```dockerfile
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "scopio_microscope", "microscope.launch.py"]
```
The **entrypoint** sources ROS + the workspace, then runs the **command** — the
full bring-up launch. The nodes import nothing from outside the workspace; the
repo is still mounted at `/workspace` so relative paths (`calibration.json`)
land on the Pi's real disk.

### The [docker-compose.yml](../docker-compose.yml)
Compose records *how to run* the container so you don't type a giant
`docker run`. The important lines:
```yaml
network_mode: host     # share the Pi's network -> DDS discovery just works,
                       # and the external brain on the LAN sees the graph
privileged: true       # give the container access to /dev (camera, USB, serial)
volumes:
  - ..:/workspace                 # mount the repo: reused code + recordings out
  - /run/udev:/run/udev:ro        # libcamera needs udev to enumerate the camera
environment:
  - GALVO_RESOURCE=${GALVO_RESOURCE:-}   # pass the AWG address through
command: ros2 launch scopio_ui scopio.launch.py
```
`network_mode: host` is the key one for ROS: by default each container has its
own network namespace and DDS can't auto-discover across it; host networking
puts the container on the Pi's network so discovery (and the external computer)
work with zero config. `privileged: true` is the blunt way to expose all devices;
the file shows the least-privilege alternative (explicit `devices:`) for later.

### Running it
```bash
cd ros2_ws
GALVO_RESOURCE="USB0::..." docker compose up --build
```
`--build` builds the image first; `up` starts the container with all the compose
settings. `docker compose logs -f` tails the node output; `docker compose exec
scopio bash` gives you a shell *inside* the running container with ROS sourced,
where every `ros2 ...` CLI command works.

---

## 10. The whole architecture

Putting it together, and answering "what about the UI?"

**The split.** Hardware ownership lives in the driver nodes (`camera_node`,
`stage_node`, `galvo_node`). They vendor the instrument driver classes inside
the workspace (`scopio_microscope/drivers/`) rather than importing repo code, so
the image is self-contained. They publish state and accept commands. **That's
the "effectuator + sensor."**

**The brain is elsewhere.** All decision logic — *and all image analysis*: bead
detection, clump analysis, which bead goes where, viscosity decisions — lives
off the Pi, in programs that talk to the gateway over HTTP/WebSocket. The Pi
stays a clean, reusable robot that never looks at the picture.

**The UI is a client, not an owner.** This is the resolution of "Flask vs ROS":
- *Old:* Flask owned the camera/stage/galvo **and** served the browser — one
  fused program, so it couldn't coexist with the ROS drivers.
- *New:* `ui_gateway` owns nothing. It subscribes to the same topics and calls
  the same services/actions as any other client, and re-serves them to the
  browser over HTTP + MJPEG. So the UI runs happily **alongside** the drivers and
  the brain. One hardware owner (the drivers), many consumers (UI, brain,
  Foxglove, loggers).

That means you get **ROS sending data out *and* a live UI simultaneously** — they
were never really in conflict; the conflict was only the temporary one of two
programs grabbing the same camera. `ui_gateway` removes it.

**Two UI faces, both professional:**
- **Foxglove Studio** for engineering/ops — connect it to the graph and get live
  image, plots, and action panels for free. Industry standard.
- **`ui_gateway` + your SCOPIO frontend** for the operator-facing product.
  Right now the gateway serves a compact UI; porting the full SCOPIO frontend is
  a matter of pointing its buttons at the gateway's `/api/...` routes instead of
  the old Flask routes.

---

## 11. Write your own node from scratch

The best way to cement this. Let's add a trivial node that watches the stage and
warns when it wanders outside a safe box (a "travel-limit monitor").

**1. Create the file** `src/scopio_microscope/scopio_microscope/limit_node.py`:
```python
import rclpy
from rclpy.node import Node
from scopio_interfaces.msg import StagePosition

class LimitNode(Node):
    def __init__(self):
        super().__init__("limit_node")
        self.declare_parameter("max_steps", 20000)
        self.max = int(self.get_parameter("max_steps").value)
        self.create_subscription(StagePosition, "stage/position", self._on_pos, 5)
        self.get_logger().info(f"limit_node up, max_steps={self.max}")

    def _on_pos(self, msg):
        for axis, value in (("x", msg.x), ("y", msg.y), ("z", msg.z)):
            if abs(value) > self.max:
                self.get_logger().warning(f"{axis} out of range: {value} steps")

def main(args=None):
    rclpy.init(args=args)
    node = LimitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

**2. Register it** in `src/scopio_microscope/setup.py`:
```python
"limit_node = scopio_microscope.limit_node:main",
```

**3. (Optional) add it to the launch file** and a `params.yaml` section.

**4. Build, source, run:**
```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 run scopio_microscope limit_node --ros-args -r __ns:=/scopio
# in another shell, drive the stage past the limit and watch:
ros2 service call /scopio/stage/jog scopio_interfaces/srv/StageJog "{dx: 30000}"
```

You just wrote a node, declared a parameter, subscribed to a typed topic, and ran
it on the live graph. That's the whole loop. Everything else is more of the same
plus services/actions.

**Exercises to level up:**
1. Make `limit_node` *publish* a `std_msgs/Bool` "out_of_range" topic instead of
   just logging. (Add a publisher; publish in the callback.)
2. Give `galvo_node` a `SetBool` service that switches the laser output on/off
   through the driver's `output(on, channel)` method.
3. Add a `home` action to `stage_node` that returns the stage to (0,0,0).

> Note the shape of this example: it reasons about **state the backend already
> owns** (stage position), not about pixels. A node that decoded frames and
> measured the sample would belong on a client instead — see DECISIONS §8.

---

## 12. Cheat sheet & glossary

### CLI cheat sheet
```bash
# discovery / introspection
ros2 node list                         ros2 node info /scopio/galvo_node
ros2 topic list                        ros2 topic echo /scopio/stage/position
ros2 topic hz /scopio/image/compressed ros2 topic info -v /scopio/stage/position
ros2 service list                      ros2 service type /scopio/awg/call
ros2 action list                       ros2 interface show scopio_interfaces/action/MoveStagePath
ros2 param list /scopio/galvo_node     ros2 param get /scopio/galvo_node timeout_ms

# acting
ros2 service call /scopio/awg/call scopio_interfaces/srv/InstrumentCall "{method: 'list_methods'}"
ros2 action send_goal -f /scopio/camera/autofocus scopio_interfaces/action/Autofocus "{z_range: 2000, steps: 15, settle_s: 0.2}"

# build / run
colcon build --symlink-install         source install/setup.bash
ros2 run <pkg> <exe>                    ros2 launch <pkg> <file.launch.py>
```

### Glossary
- **node** — one process in the graph; a `Node` subclass.
- **rclpy** — the Python API for ROS 2 (`rclcpp` for C++).
- **topic** — named, typed pub/sub channel for streams.
- **message (msg)** — the data type of a topic.
- **service (srv)** — request/response call; a server, many clients.
- **action** — long goal with feedback + cancel + result.
- **parameter** — a node's typed config value.
- **namespace** — folder-like prefix for names (`/scopio`).
- **QoS** — Quality of Service: reliability, history/queue depth, etc.
- **DDS** — the middleware under ROS 2 that does discovery + transport.
- **executor** — the loop that runs your callbacks (`spin`).
- **callback group** — controls which callbacks may run concurrently.
- **rosidl** — the generator turning `.msg/.srv/.action` into code.
- **colcon** — the workspace build tool.
- **ament** — the build system (ament_python / ament_cmake) packages use.
- **workspace** — `src/` + `build/install/log`; built by colcon.
- **sourcing** — running `setup.bash` to put a workspace on your environment.
- **launch file** — Python that starts a set of nodes with params.

### Where to go next
- Official tutorials (do the beginner CLI + client-library ones):
  https://docs.ros.org/en/jazzy/Tutorials.html
- `ros2 <command> --help` is genuinely good.
- Foxglove Studio for visualization: https://foxglove.dev/

You now have the full mental model. Re-read §2 and §8 once the system is running
in front of you — they click much faster with live topics to `echo`.
