# SCOPIO backend — everything that runs on the Raspberry Pi

The microscope is the **sensor + effectuator** of the self-driving lab. This
workspace senses, streams and moves hardware; it makes no decisions and does no
image analysis. Every external program (`../ui`, `../galvo_draw`, agents) is an
**API client** over HTTP/WebSocket with an API key — nothing outside the Pi
speaks ROS.

## Run it

```bash
cd ros2_ws
cp .env.example .env                            # which instrument is which
python3 scripts/generate_api_key.py laptop      # once per client; note the key
docker compose up -d --build
curl http://127.0.0.1:8000/api/v1/health
```

Three services come up together:

| Service | What | Port |
|---|---|---|
| `scopio` | the ROS 2 driver graph under `/scopio` | — (DDS on localhost) |
| `camera` | `../camera_server`, picamera2 MJPEG — the **only** owner of the sensor | 8081 loopback |
| `gateway` | the public HTTP/WebSocket API, API-key auth | **8000, LAN** |

`restart: unless-stopped` on all three, so `sudo systemctl enable docker` is
enough to survive a reboot with no SSH session.

**`.env` is the one place this rig's wiring is written down** (gitignored,
because it describes *this* Pi). Compose reads it automatically. Both instrument
lines are optional — each node discovers its own instrument by USB **vendor id**
and will never open the other one — but naming them is faster and is *required*
for an Ethernet unit, since pyvisa-py cannot scan a LAN:

```bash
docker compose down                     # a running node holds its instrument
python3 scripts/list_instruments.py     # prints ready-to-paste GALVO_/TCLAB_ lines
```

Change `.env` → `docker compose up -d` again.

Once before trusting the USB instruments, on the Pi:

```bash
sudo cp udev/99-scopio-instruments.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# then UNPLUG AND REPLUG — a live libusb session keeps the old permissions
```

Without it a non-root `list_resources()` silently omits the instrument: libusb
has to open the device node just to read the descriptors, and those are
root-only by default.

## Verify

```bash
curl http://127.0.0.1:8000/api/v1/health            # no auth: ok, ros_ok, camera_ok
python3 scripts/smoke_test_api.py --url http://127.0.0.1:8000 [--hardware]
python3 scripts/test_drivers.py                     # driver logic, no hardware
docker compose exec scopio bash scripts/smoke_test.sh   # the raw ROS graph
```

Interactive API docs at `http://<pi>:8000/docs`; live schema for every service,
topic and action at `GET /api/v1/interfaces`. Writing a client? See
[`../docs/API.md`](../docs/API.md) and [`../scopio_client`](../scopio_client) —
not this workspace.

## Nodes

All under `/scopio`. **Every node degrades gracefully**: with its hardware
absent it still starts and reports `connected = false`, so the graph always
comes up and you bring hardware online piece by piece.

| Node | Publishes | Services | Actions |
|---|---|---|---|
| `camera_node` | `image/compressed`, `camera/state` | `camera/set_controls`, `camera/set_framerate`, `camera/white_balance` | `camera/autofocus` |
| `stage_node` | `stage/position` | `stage/jog`, `stage/move_abs` | `stage/move_path`, `scan_region` |
| `galvo_node` | `awg/status` | `awg/call`, `awg/write`, `awg/query` | — |
| `temperature_node` | `temperature/status` | `temperature/call` | — |
| `relay_node` | `relay/state` (latched) | `relay/set` (`std_srvs/SetBool`) | — |
| `calibration_node` | `calibration` (latched) | `calibration/set` | — |

`relay_node` drives the laser relay from one BCM GPIO pin on the Pi itself
(`gpio_pin`, default 17 — see `config/params.yaml` before changing `active_high`).
It is the one node whose "degrade gracefully" has a safety edge: when a GPIO call
throws, the relay's true position is unknown, and **unknown is published as ON**
until an `off()` actually succeeds. A green "laser off" next to a live laser is
the failure that must not happen.

The field-level contract is the `.msg`/`.srv`/`.action` files in
`src/scopio_interfaces/` — they carry their own comments and are the only
authority. Treat them as **frozen**: changing a field is an ABI break.

Three design rules explain most of what looks unusual here (the *why* is in
[`../docs/DECISIONS.md`](../docs/DECISIONS.md)):

- **The camera has exactly one owner: `../camera_server`.** picamera2/libcamera
  ship from Raspberry Pi OS and cannot run in the Ubuntu ROS container, so
  `camera_node` runs in *bridge mode*: it ingests the camera server's MJPEG over
  loopback, republishes `image/compressed`, forwards the camera services, and
  runs autofocus on the ingested frames. The frozen interface behaves
  identically either way.
- **The instrument nodes expose a whole driver CLASS**, not a curated subset.
  `awg/call` and `temperature/call` take `{method, args, kwargs}` as JSON, so
  every method of `scopio_microscope/drivers/{dg1022z,TC10LAB}.py` is reachable
  the instant it is written — no new `.srv`, no gateway change, no client
  update. Call `list_methods` for names, signatures and docstrings (it works
  while the hardware is disconnected — it introspects the class).
- **The backend never analyses a frame and never records.** Both are client
  jobs, off the Pi (`../ui`, `../viscosity`, `../viscosity_agent`).

**Calibration persists.** `calibration_node` writes `calibration.json` to the
bind-mounted repo root on the Pi's real disk, atomically, and logs the absolute
path at startup — it survives `docker compose down`, rebuilds and reboots.

## When something doesn't work

**Step zero: confirm you are running the code you think you are.** The nodes run
from the built image, *not* from the repo mounted at `/workspace` — so after any
edit or `git pull` on the Pi you need `--build`, and without it nothing changes
and the log looks exactly like a fix that didn't work. Every launch prints the
image's build time as its first line:

```
[INFO] [launch]: SCOPIO image built 2026-07-30T04:12:07Z -- older than your last edit? ...
```

If that predates your change: `docker compose up -d --build`.

Everything below assumes the software is right, which is the point: each node
reports its own state, so start with what the graph says.

```bash
curl -s http://127.0.0.1:8000/api/v1/health
curl -s -H "X-API-Key: $KEY" http://127.0.0.1:8000/api/v1/status | python3 -m json.tool
docker compose logs -f scopio camera gateway
```

`/api/v1/status` carries `connected` and `last_error` for every instrument.
That error string is the real diagnosis — read it before changing anything.

**First, rule out the container.** A device the host can see but the container
cannot is not a hardware fault, and it is the single most common way this stack
lies to you. `privileged: true` populates the container's `/dev` at *creation*
time, so anything plugged in afterwards is invisible without the `/dev:/dev`
bind mount (see the note in `docker-compose.yml`). Compare the two sides:

```bash
lsusb                                                  # host: is the box on the bus?
ls -l /dev/usbtmc* /dev/ttyACM* 2>&1                   # host
docker compose exec scopio ls -l /dev/usbtmc* /dev/ttyACM* 2>&1   # container
docker compose exec scopio python3 -c \
  "import pyvisa; print(pyvisa.ResourceManager('@py').list_resources())"
```

If the host lists it and the container doesn't → `docker compose up -d
--force-recreate`, and check the `/dev:/dev` mount is present. If **neither**
lists it → hardware, cable, or power.

**Camera.** Ask the camera server — it answers 503 with the cause, what
libcamera could see, and which of three problems you have:

```bash
curl -s http://127.0.0.1:8081/controls | python3 -m json.tool
```

A **200** with `frames` climbing means the camera is fine; anything still broken
is downstream (gateway, network, viewer). Otherwise read `cameras`: `[]` means
no sensor is visible — reseat the CSI ribbon at both ends; a non-empty list with
an open failure means **something else already holds it**; an `error` entry
means libcamera itself won't load, which is what the systemd fallback
(`../camera_server/install_systemd.sh`) is for.

> **Do not run `rpicam-hello` while the stack is up.** Exactly one process may
> hold the sensor, so it fails with *"Pipeline handler in use by another
> process"* whenever the camera server has it — i.e. whenever things are
> working. Stop the service first:
> `docker compose stop camera && rpicam-hello --list-cameras`, then
> `docker compose start camera`.

**Galvo (Rigol DG1022Z).** The node logs the resource string it tried — read
the VISA error, the two look alike and mean opposite things:

| Error | Meaning |
|---|---|
| `VI_ERROR_INV_RSRC_NAME` / "Parsing error" | The **string** is malformed. Not hardware. Almost always `GALVO_RESOURCE` in `.env`: a trailing `\r` from a Windows editor, quotes, or a `# comment` on the same line (compose keeps it as part of the value). The node strips whitespace; it cannot fix the rest. |
| `VI_ERROR_RSRC_NFOUND` | The name parsed, the instrument is not there. |
| `no Rigol AWG on USB` | Nothing with the Rigol vendor id enumerated at all — udev/libusb or the cable. |

Then `docker compose down && python3 scripts/list_instruments.py` for a
paste-ready address, and verify with
`POST /api/v1/service/awg/query {"command": "*IDN?"}`.

**Temperature (TC10 LAB).** If `list_instruments.py` finds it but every query
times out, the kernel's `usbtmc` driver has the interface and pyvisa-py's
detach-and-use-libusb dance is losing. Check the char device directly:

```bash
echo '*IDN?' > /dev/usbtmc0 && head -c 200 /dev/usbtmc0
```

If that answers, put **`TCLAB_RESOURCE=/dev/usbtmc*`** in `.env` — note the
glob. `/dev/usbtmc0` is not reliably this instrument: the Rigol AWG is USB-TMC
too and the kernel numbers the nodes in enumeration order, so a hard-coded path
can point the temperature node at the function generator. The driver probes
every match, asks `*IDN?`, and keeps the one that answers as a Wavelength.

Two other things that make this instrument look flaky when it isn't. A *single*
timeout no longer drops the session (it takes three in a row), so brief stalls
appear as `last_error` on `temperature/status` instead of a connect/disconnect
cycle. And the session is cleared and drained on connect — USB-TMC keeps no
framing between sessions, so a reply the previous owner never read stays queued
and puts every subsequent query one answer behind.

Do not run `tc10_read.py` against the instrument while the stack is up: both
would be reading one queue, and each would get the other's replies.

**Stage (Sangaboard).** A serial device, and *how* it is wired decides
everything:

- **On the 40-pin header** (Sangaboard v0.5, the RP2040 HAT) it is a **UART**
  device with no USB identity at all, so the library's USB auto-detection can
  never find it — the board is powered and working and still reports
  "unavailable". Set `SANGABOARD_PORT=/dev/serial0`. Two Pi-side settings must
  also be right, or nothing will work no matter what the software does:
  `enable_uart=1` in `/boot/firmware/config.txt`, and the serial **login
  console off** (`sudo raspi-config` → Interface Options → Serial Port → login
  shell **No**, hardware serial **Yes**) — otherwise a getty owns the port and
  talks over you. On a Pi 4, `dtoverlay=disable-bt` moves the reliable PL011
  UART onto pins 8/10; without it `/dev/serial0` is the mini-UART, whose baud
  rate follows the core clock and drops characters.
- **On USB** it is auto-detected, unless it sits behind an unrecognised bridge
  (CH340, FTDI) — then name it, e.g. `SANGABOARD_PORT=/dev/ttyACM0`.

The node prints every port it can see when it fails, tries the header UARTs
before giving up, and retries every 10 s — so a stage that appears after launch
comes up on its own.

**A node missing from `ros2 node list` entirely** means it crashed on import —
`docker compose logs scopio` has the traceback. That is the one failure mode
that is never hardware.

## Development

No-hardware stack, works on a laptop (including Docker Desktop):

```bash
docker compose -f docker-compose.yml -f docker-compose.dev.yml up --build
python3 scripts/smoke_test_api.py --url http://127.0.0.1:8000
```

Edit nodes without rebuilding the image:

```bash
docker compose run --rm scopio bash
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch scopio_microscope microscope.launch.py
```

Poking the graph directly (backend work only — clients use the gateway):

```bash
docker compose exec scopio bash
ros2 topic echo /scopio/temperature/status
ros2 service call /scopio/awg/call scopio_interfaces/srv/InstrumentCall "{method: 'list_methods'}"
```

New to ROS 2? [`docs/ROS2_TUTORIAL.md`](docs/ROS2_TUTORIAL.md) teaches it
through this exact codebase.

### Security model

The gateway is the only LAN-facing surface: API-key auth on every route except
`/api/v1/health`, keys in `secrets/api_keys.json` (hot-reloaded; generate and
revoke with `scripts/generate_api_key.py`). The camera server is loopback-only.
The DDS graph has no auth, which is fine *because* it never faces the network —
keep it that way. For remote access put the Pi on a mesh VPN (Tailscale) or
behind a TLS reverse proxy; never port-forward 8000 raw, the API key would
travel in clear.
