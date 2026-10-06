# ros2_ws

The ROS 2 workspace that runs on the Pi: three packages, the Docker image that builds them, and the compose files that run them.

## Folder map

| Path | What it is |
|---|---|
| `src/scopio_interfaces/` | The `.msg`, `.srv` and `.action` definitions. [README](src/scopio_interfaces/README.md) |
| `src/scopio_microscope/` | The hardware nodes, their drivers, the launch file and `params.yaml`. [README](src/scopio_microscope/README.md) |
| `src/scopio_gateway/` | The HTTP/WebSocket API. [README](src/scopio_gateway/README.md) |
| `scripts/` | Key management, smoke tests and debugging tools. [README](scripts/README.md) |
| `udev/` | udev rules for host-side instrument tools. See [INSTALL.md](../INSTALL.md#5-install-the-udev-rules-only-if-you-run-instrument-tools-on-the-host). |
| `Dockerfile` | Builds the `scopio-microscope:jazzy` image. |
| `entrypoint.sh` | Sources ROS 2 and the workspace, then runs the command. |
| `docker-compose.yml` | The production stack on the Pi: `scopio`, `camera`, `gateway`. |
| `docker-compose.dev.yml` | An override for a laptop with no hardware. |
| `.env.example` | Template for `.env`, this rig's wiring. See [CONFIGURATION.md](../docs/CONFIGURATION.md#ros2_wsenv). |
| `data/` | Created on the Pi. Runtime state (`calibration.json`), mounted at `/data`. Gitignored. |
| `secrets/` | Created on the Pi. `api_keys.json`, mounted read-only into the gateway. Gitignored. |

## The three packages

| Package | Build type | What it holds |
|---|---|---|
| `scopio_interfaces` | `ament_cmake` | The message contract every node and client shares. Slow to build. |
| `scopio_microscope` | `ament_python` | Six nodes: `camera_node`, `stage_node`, `galvo_node`, `temperature_node`, `relay_node`, `calibration_node`. |
| `scopio_gateway` | `ament_python` | One executable, `gateway`: FastAPI and uvicorn on one thread, an rclpy node on another. |

## The image

The base is `ros:jazzy-ros-base` (Ubuntu 24.04). On top come apt packages (`python3-opencv`, `python3-gpiozero`, `python3-lgpio`, `libusb-1.0-0`, …) and pip packages (`pyvisa`, `pyvisa-py`, `pyusb`, `pyserial`, `sangaboard`, `fastapi`, `uvicorn[standard]`, `httpx`). None of the pip packages is version-pinned yet.

Some choices that look odd and are not:

- `libusb-1.0-0` is installed explicitly. Without it, pyvisa-py cannot enumerate USB instruments, and the AWG and TC10 read as absent.
- `pyserial` is installed explicitly, though `sangaboard` pulls it in. pyvisa-py needs it for serial instruments, and relying on another package's dependencies hides that.
- There is no picamera2 in this image. It does not exist for Ubuntu. The camera has its own container (see [camera_server/README.md](../camera_server/README.md)).
- There is no image-analysis stack (no trackpy, pandas or scipy). The Pi never analyses frames.

### Build layers

The workspace is built in three layers, ordered by how rarely they change. A rebuild redoes only the layers from the first one that changed.

| Layer | What | Why separate | Rebuild time |
|---|---|---|---|
| 1 | `src/scopio_interfaces` alone, with `MAKEFLAGS=-j2` | Generating and compiling the message type support is by far the slowest step on a Pi, and it rarely changes. `-j2` caps parallel compiles: four at once can exhaust the Pi's RAM and swap to the SD card, which is slower than not parallelising. | long |
| 2 | the rest of `src`, built with `--packages-skip scopio_interfaces` | Pure Python. Skipping the interfaces reuses layer 1 instead of rebuilding it. | about a minute |
| 3 | `scripts/`, copied to `/ros2_ws/scripts` | Editing a script rebuilds nothing else. | seconds |

Changing the apt or pip lines, which come before layer 1, rebuilds everything. Docs, udev rules, `.env` and the compose files are not in the image, so editing them never triggers a build. `.dockerignore` keeps `data/`, `secrets/` and any local `build/`, `install/` and `log/` out of the build context.

Layer 2 also writes `/ros2_ws/BUILD_STAMP`. The launch file prints it as its first log line:

```
SCOPIO image built 2026-10-01T12:00:00Z -- older than your last edit? `docker compose up -d --build`
```

### The image runs the code, not the repo

The repo is **not** mounted into the containers. Only `./data` (and `./secrets` for the gateway) are. So after any edit to `src/` or `scripts/`, on the Pi:

```bash
docker compose up -d --build
```

Without `--build`, the old code keeps running, and the log looks exactly like a fix that did not work. The build stamp is how you tell.

Each rebuild leaves the old image behind. Reclaim the space now and then:

```bash
docker image prune -f && docker builder prune -f
```

## The compose stack

`docker-compose.yml` runs three services, all with `network_mode: host` and `restart: unless-stopped`:

| Service | Container | Image | Command |
|---|---|---|---|
| `scopio` | `scopio` | `scopio-microscope:jazzy` (built here) | `ros2 launch scopio_microscope microscope.launch.py` |
| `camera` | `scopio-camera` | `scopio-camera:bookworm` (built from `../camera_server`) | `python3 /app/pi_camera_server.py` |
| `gateway` | `scopio-gateway` | `scopio-microscope:jazzy` (reused) | `ros2 run scopio_gateway gateway` |

- **The gateway reuses the scopio image.** It has no `build:` block and `pull_policy: never`, because the tag exists only on this machine. Giving it its own `build:` made compose build the same image twice in parallel: two colcon builds fighting over the Pi's RAM.
- **`scopio` is privileged and bind-mounts `/dev`.** `privileged: true` alone fills the container's `/dev` once, when the container is created, so an instrument plugged in later would be invisible. The `/dev:/dev` mount makes new devices appear, and the retry timers pick them up without a restart. The camera service has the same mount for the same reason. The cost: the container shares the host's `/dev/pts` and `/dev/shm`. If `docker compose exec -it` ever misbehaves because of that, drop the mount and recreate the stack after plugging instruments in instead.
- **The working directory is `/data`**, so `calibration.json` lands in `ros2_ws/data` on the Pi and survives rebuilds.
- A least-privilege alternative to `privileged`, with an explicit `devices:` list, is sketched in comments in `docker-compose.yml`. It is not tested. A `devices:` entry that does not exist yet stops the container from starting.

## Development without hardware

`docker-compose.dev.yml` runs `scopio` and `gateway` on a laptop, including Docker Desktop on Windows or macOS, where host networking does not behave. Every node reports `connected=false` and the API is fully testable. It differs from production:

- Bridge networking. `scopio` publishes port 8000 and the gateway joins its network namespace, so their DDS traffic stays local as on the Pi.
- No camera service (its image is Pi-only). `CAMERA_URL` is empty, so `camera_ok` is false and the camera routes answer 503.
- Not privileged, and no automatic restart.

On the laptop, in `ros2_ws`, make a key once:

```bash
python3 scripts/generate_api_key.py dev
```

Start the stack:

```bash
docker compose -f docker-compose.yml -f docker-compose.dev.yml up --build
```

In another terminal, run the API smoke test. Use `127.0.0.1`: on Docker Desktop the WebSocket can hang on the IPv6 `localhost` route.

```bash
python3 scripts/smoke_test_api.py --url http://127.0.0.1:8000
```

The offline driver tests need no Docker at all:

```bash
python3 scripts/test_drivers.py
```

## Poking the graph

Backend work only: clients use the gateway. `docker compose exec` skips the image's entrypoint, so start the shell through it to have ROS sourced. On the Pi:

```bash
docker compose exec scopio /entrypoint.sh bash
```

Then, inside the container:

```bash
ros2 node list
```

```bash
ros2 topic echo /scopio/temperature/status
```

```bash
ros2 service call /scopio/awg/call scopio_interfaces/srv/InstrumentCall "{method: 'list_methods'}"
```

```bash
ros2 action list
```

The container's working directory is `/data`. Run the scripts by absolute path, e.g. `bash /ros2_ws/scripts/smoke_test.sh`.

## Next

- [scopio_microscope/README.md](src/scopio_microscope/README.md): the nodes
- [scopio_gateway/README.md](src/scopio_gateway/README.md): the API gateway
- [scripts/README.md](scripts/README.md): the tools
- [docs/ADDING_A_NODE.md](../docs/ADDING_A_NODE.md): extending the graph
