# Configuration

Every setting in SCOPIO: where it lives, its default, and what a change requires.

## What a change costs

The containers run code baked into the image. The repo is not mounted, so editing a file on the Pi changes nothing until you rebuild. The Dockerfile builds in layers, so a rebuild redoes only what changed.

| You changed | Run, in `ros2_ws` on the Pi | Cost |
|---|---|---|
| `.env` | `docker compose up -d` | seconds, no build |
| `docker-compose.yml` | `docker compose up -d` | seconds, no build |
| `scripts/` | `docker compose up -d --build` | seconds |
| a node, a driver, the gateway, `params.yaml` or the launch file (anything in `src/` except interfaces) | `docker compose up -d --build` | about a minute |
| `src/scopio_interfaces/` | `docker compose up -d --build` | **the long build**: every message recompiles |
| the base image, or the `apt-get` or `pip3` lines in `ros2_ws/Dockerfile` | `docker compose up -d --build` | everything rebuilds, including the long build |
| `camera_server/pi_camera_server.py` | `docker compose up -d --build` | seconds (camera image only) |
| docs, udev rules | nothing | none |
| `/boot/firmware/config.txt` | `sudo reboot` | a reboot |

To confirm a rebuild took, check the first log line: `docker compose logs scopio | grep "image built"`. The time changes whenever `src/` changes.

Each rebuild leaves the old image behind. Now and then, reclaim the SD card:

```bash
docker image prune -f && docker builder prune -f
```

## `ros2_ws/.env`

`ros2_ws/.env` describes this rig's wiring. It is gitignored. Create it with `cp .env.example .env`. Compose reads it automatically. A change needs only `docker compose up -d`.

| Variable | Default | What it does |
|---|---|---|
| `GALVO_RESOURCE` | empty | VISA address of the Rigol DG1022Z. Empty: the node opens the first USB device with Rigol's vendor id. Set it to pin one unit, or for an Ethernet unit. |
| `TCLAB_RESOURCE` | empty | Address of the TC10 LAB. **Leave it empty for USB**: the node picks the transport by who owns the device (see [drivers/README.md](../ros2_ws/src/scopio_microscope/scopio_microscope/drivers/README.md#tc10-transport-by-ownership)). Set it only for Ethernet: `TCPIP::<ip>::INSTR`, because pyvisa-py cannot scan a LAN. |
| `SANGABOARD_PORT` | empty | Serial port of the stage. Empty: the sangaboard library auto-detects USB boards, then the node tries the header UARTs. Set `/dev/serial0` for a v0.5 HAT, or a `/dev/ttyACM*` path for a board behind an unrecognised USB bridge. |
| `GPIOZERO_PIN_FACTORY` | `lgpio` | The GPIO backend for the relay. Name it: an unset factory can fall through to gpiozero's `native` backend, which loads anywhere and then does not drive a modern Pi's pins. |
| `CAM_W`, `CAM_H` | `640`, `480` | The frame size the camera server **reports before the sensor opens**. The running size comes from the sensor mode (see [camera_server/README.md](../camera_server/README.md#sensor-modes)), not from these. |

Rules for the file:

- Do not quote values.
- Put no comment on the same line as a value. Compose keeps it as part of the value, and a VISA address with junk on the end fails to parse.
- The nodes strip trailing whitespace, so a `\r` from a Windows editor is harmless. Other junk is not.
- Values in `.env` win over the same settings in `params.yaml`.

## `params.yaml`

`ros2_ws/src/scopio_microscope/config/params.yaml` holds one section per node. It is baked into the image, so a change needs `docker compose up -d --build` (about a minute). "Code default" is what the node uses if the line is missing.

**`camera_node`**

| Parameter | Value | Code default | What it does |
|---|---|---|---|
| `width`, `height` | 640, 480 | 640, 480 | Reported on `camera/state` until the camera server answers. Then the server's real size wins. |
| `framerate` | 30.0 | 30.0 | Capture fps pushed to the camera server when it first answers. Drives the exposure budget. |
| `publish_fps` | 15.0 | 15.0 | Maximum rate of `image/compressed`. |

**`stage_node`**

| Parameter | Value | Code default | What it does |
|---|---|---|---|
| `port` | `""` | `""` | Serial port. `SANGABOARD_PORT` in `.env` wins. |
| `reconnect_period` | 10.0 | 10.0 | Seconds between attempts to open the board (at least 2). |
| `publish_rate` | 5.0 | 5.0 | Hz of `stage/position`. |
| `step_x`, `step_y`, `step_z` | 40 | 40 | Declared but not used by the node. Step size is a client choice. |

**`galvo_node`**

| Parameter | Value | Code default | What it does |
|---|---|---|---|
| `resource` | `""` | `""` | VISA address. `GALVO_RESOURCE` in `.env` wins. |
| `init_on_connect` | true | true | On connect, run `dcinit`: both channels DC at their offsets, outputs ON. |
| `publish_rate` | 5.0 | 5.0 | Hz of `awg/status`. It publishes cached state; it never polls the AWG. |
| `reconnect_period` | 15.0 | 15.0 | Seconds between connect attempts. |
| `timeout_ms` | 15000 | 15000 | VISA timeout. Long, because a large waveform upload takes the AWG a few seconds. The connect deadline is 3 × this + 5 s. |

**`temperature_node`**

| Parameter | Value | Code default | What it does |
|---|---|---|---|
| `resource` | `""` | `""` | Address. `TCLAB_RESOURCE` in `.env` wins. |
| `units` | `"C"` | `"C"` | Forced on connect. `""` reads the instrument's units instead. A label only: a failure here never fails the connect. |
| `publish_rate` | 1.0 | 1.0 | Hz of `temperature/status`. Each poll is five queries. Lower it if the instrument reports timeouts under load. |
| `reconnect_period` | 5.0 | 15.0 | Seconds between connect attempts. Short, because opening this instrument is quick and a session is only dropped after three failures, so frequent retries cannot thrash. |
| `timeout_ms` | 5000 | 5000 | I/O timeout. The connect deadline is 3 × this + 5 s. |

**`relay_node`**

| Parameter | Value | Code default | What it does |
|---|---|---|---|
| `gpio_pin` | 17 | 17 | BCM pin of the laser relay (physical pin 11). Do not use 14, 15 or 23 to 25: the Sangaboard uses them. |
| `active_high` | **false** | true | Relay polarity. `false` on this rig. Read [the relay section of TROUBLESHOOTING.md](TROUBLESHOOTING.md#laser-relay-is-inverted) before changing it. |
| `reconnect_period` | 10.0 | 10.0 | Seconds between attempts to claim the pin. |

**`calibration_node`**

| Parameter | Value | Code default | What it does |
|---|---|---|---|
| `calibration_file` | `"calibration.json"` | same | Relative to the working directory, `/data`, which is `ros2_ws/data` on the Pi. |

## Compose environment

Set in `ros2_ws/docker-compose.yml`. A change needs `docker compose up -d`. The file is in git, so a local edit will conflict with `git pull`.

| Variable | Service | Value | What it does |
|---|---|---|---|
| `CAMERA_URL` | `scopio`, `gateway` | `http://127.0.0.1:8081` | Where the camera server is. Empty means "no camera": the camera endpoints answer 503 and `camera_ok` is false. |
| `SCOPIO_GATEWAY_PORT` | `gateway` | `8000` | The API port. |
| `SCOPIO_GATEWAY_HOST` | `gateway` | not set (code default `0.0.0.0`) | The address the API binds to. |
| `SCOPIO_API_KEYS_FILE` | `gateway` | `/secrets/api_keys.json` | The key file, inside the container. `./secrets` is mounted read-only at `/secrets`. |
| `RMW_IMPLEMENTATION` | `scopio`, `gateway` | `rmw_fastrtps_cpp` | The DDS implementation. Both sides must match. |
| `CAM_HOST`, `CAM_PORT` | `camera` | `127.0.0.1`, `8081` | Where the camera server listens. Keep it on loopback: it has no authentication. |

The camera server also reads two variables that compose does not set: `CAM_MODE` (start-up sensor mode, default `detail`) and `CAM_DETAIL_MAX_W` (the widest the detail stream may be before the ISP scales it, default 1640; raise it on wired gigabit).

## Driver and node constants

In the code. A change needs `docker compose up -d --build` (about a minute).

| Constant | Where | Value | What it does |
|---|---|---|---|
| `MIN_INTERVAL_S` | `DG1022Z`, `TC10LAB` | 1/50 s | Minimum gap between two I/Os on the wire. Both instruments lag above about 60 commands/s. |
| `MAX_FAILURES` | `galvo_node`, `temperature_node`, `stage_node` | 3 | Consecutive failures before the session is dropped and reopened. |
| `UsbtmcDevice.READ_SIZE` | `TC10LAB.py` | 256 | Bytes per read on the kernel usbtmc device. A longer reply raises instead of desynchronising the session. |
| `AF_BACKLASH` | `camera_node` | 256 steps | How far below each Z autofocus starts its approach. |
| `MIN_FPS`, `MAX_FPS` | `camera_node` | 1, 120 | Clamp on `camera/set_framerate`. The camera server clamps again to the running mode's maximum. |
| `SERVER_POLL_S` | `camera_node` | 2.0 s | How often `camera/state` re-reads the camera server. |
| `DISCOVERY_PERIOD_S` | gateway `ros_bridge.py` | 5.0 s | How often the gateway looks for new `*/status` and `*/state` topics. |
| `OUTBOUND_QUEUE_SIZE` | gateway `ws.py` | 256 | Frames queued per WebSocket before new ones are dropped. |

## Pi boot config

In `/boot/firmware/config.txt`. A change needs a reboot. Details in [INSTALL.md](../INSTALL.md#4-edit-the-boot-config-required).

| Line | When | Why |
|---|---|---|
| `gpio=17=op,dh` | always, on this rig | Holds the relay pin high from power-on, so the laser stays off until the relay node runs. |
| `enable_uart=1` | Sangaboard HAT | Enables the header UART. |
| `dtoverlay=disable-bt` | Sangaboard HAT on a Pi 4 | Puts the PL011 UART on pins 8/10 instead of the unreliable mini-UART. |

## Next

- [TROUBLESHOOTING.md](TROUBLESHOOTING.md): when a setting does not do what you expect
- [ros2_ws/README.md](../ros2_ws/README.md): the build layers in detail
- [ADDING_A_NODE.md](ADDING_A_NODE.md): adding a node's settings
