# Install

This takes a Raspberry Pi from a blank SD card to a running, verified SCOPIO. Do the steps in order. Each one is marked **required** or **only if …**.

## Assumptions

- A Raspberry Pi running **Raspberry Pi OS Bookworm, 64-bit**. <!-- TODO: name the Pi model on the rig (the code mentions both Pi 4 and Pi 5). -->
- The Pi camera on the CSI port. <!-- TODO: name the camera module (the code uses a Camera Module 2 / IMX219 as its example). -->
- The Sangaboard on USB, **or** a Sangaboard v0.5 HAT on the 40-pin header (UART).
- The Rigol DG1022Z on USB.
- The Wavelength TC10 LAB on USB, or on Ethernet.
- The laser relay module on **BCM GPIO17 (physical pin 11)**.

Commands run **on the Pi** unless a step says otherwise. Paths are relative to the clone.

## 1. Flash the SD card (required)

Flash Raspberry Pi OS Bookworm 64-bit, enable SSH, and boot the Pi on the lab network.

## 2. Install Docker (required)

Docker's convenience script installs the engine and the compose plugin:

```bash
curl -fsSL https://get.docker.com | sh
```

Let your user run Docker without `sudo`, then log out and back in:

```bash
sudo usermod -aG docker $USER
```

Start Docker at boot. Every service uses `restart: unless-stopped`, so the stack comes back by itself after a reboot, but only if Docker itself starts:

```bash
sudo systemctl enable docker
```

## 3. Clone the repo (required)

```bash
git clone https://github.com/markiyan-konyk/scopio.git
```

## 4. Edit the boot config (required)

Open `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

**Required on this rig:** add this line at the end.

```
gpio=17=op,dh
```

> **Laser safety.** The relay on this rig is configured active-low (`active_high: false`; why is not yet confirmed, see [the relay section of TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md#laser-relay-is-inverted)). Until the relay node claims GPIO17, the pin is an input with a pulldown, so it reads LOW, and on this rig LOW means laser ON. Without this line **the laser is ON** from power-on, through boot and Docker start, until the relay node is running, and for as long as that node is down. The line drives the pin high from power-on.

**Only if the stage is a Sangaboard v0.5 HAT on the header:** also add

```
enable_uart=1
```

and, **on a Pi 4 only**, add

```
dtoverlay=disable-bt
```

This puts the reliable PL011 UART on pins 8/10. Without it, `/dev/serial0` is the mini-UART, whose baud rate follows the core clock and drops characters.

Then turn the serial **login console** off, or a login shell holds the port and talks over the stage. Run `sudo raspi-config`, open Interface Options, then Serial Port. Answer **No** to the login shell and **Yes** to the serial hardware.

Reboot:

```bash
sudo reboot
```

## 5. Install the udev rules (only if you run instrument tools on the host)

The containers run as root and do not need these rules. The rules let host-side tools such as `scripts/list_instruments.py` open the AWG and the TC10 as a normal user. Without them, libusb cannot read the devices' descriptors and the instruments are silently missing from the list.

```bash
sudo cp scopio/ros2_ws/udev/99-scopio-instruments.rules /etc/udev/rules.d/
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug** each instrument. A device that is already open keeps its old permissions.

The rules use `MODE="0666"`, deliberately, on a single-purpose lab Pi. To tighten them, use `GROUP="plugdev", MODE="0660"` and add your user to `plugdev`.

## 6. Create `.env` (only if the defaults do not fit)

On USB, leave everything empty. The nodes find the instruments by themselves, and the galvo node only opens a Rigol, the temperature node only a Wavelength. You need `.env` only for these cases:

| Your hardware | Line in `.env` |
|---|---|
| Sangaboard v0.5 HAT on the header | `SANGABOARD_PORT=/dev/serial0` |
| TC10 LAB on Ethernet | `TCLAB_RESOURCE=TCPIP::<ip>::INSTR` |

```bash
cd scopio/ros2_ws
```

```bash
cp .env.example .env
```

Uncomment and edit the lines you need. Do not quote values, and put no comment on the same line as a value. Every knob is in [CONFIGURATION.md](docs/CONFIGURATION.md#ros2_wsenv).

## 7. Create the state folders (required)

In `scopio/ros2_ws`:

```bash
mkdir -p data secrets
```

`data/` holds the calibration and `secrets/` holds the API keys. Create them yourself: if Docker creates them, they belong to root, and the key script in step 9 cannot write to `secrets/`.

## 8. Build and start (required)

In `scopio/ros2_ws`:

```bash
docker compose up -d --build
```

The first build takes a long time, mostly compiling the message definitions. Later builds take about a minute. Three containers start: `scopio` (the ROS 2 nodes), `scopio-camera` (the camera server) and `scopio-gateway` (the API on port 8000).

## 9. Create an API key (required)

In `scopio/ros2_ws`, give each client its own name (`ui`, `agent`, …):

```bash
python3 scripts/generate_api_key.py ui
```

It prints the key once. The gateway picks it up without a restart. See [SECURITY.md](docs/SECURITY.md) for listing and revoking keys.

## 10. Check the camera (required)

On the Pi:

```bash
curl http://127.0.0.1:8081/controls
```

A JSON reply with a `frames` count that grows between calls means the camera works. A 503 reply says why it does not, in `diagnosis`.

To check the sensor directly, stop the camera container first. While it is running it holds the sensor, and `rpicam-hello` then fails with "Pipeline handler in use by another process".

```bash
docker compose stop camera
```

```bash
rpicam-hello --list-cameras
```

```bash
docker compose start camera
```

**Only if libcamera fails inside the container** (the diagnosis says "libcamera itself failed to load"): run the camera server as a systemd unit on the host instead. See [the fallback in camera_server/README.md](camera_server/README.md#systemd-fallback).

## 11. Verify (recommended)

Work through this list in `scopio/ros2_ws`.

- [ ] **The image is current.** The first lines of the log show when the image was built. The time should be from your build:

  ```bash
  docker compose logs scopio | grep "image built"
  ```

- [ ] **The ROS graph is up.** Every node should read `ok`. `docker compose exec` skips the image's entrypoint, so run the script through it to get ROS sourced:

  ```bash
  docker compose exec scopio /entrypoint.sh bash /ros2_ws/scripts/smoke_test.sh
  ```

- [ ] **The API works end to end.** Run this from a laptop with a clone of this repo and `pip install requests websocket-client`. With `--hardware` it also checks the instruments and the video. It runs a small autofocus, which moves Z by a few hundred steps.

  ```bash
  python3 scripts/smoke_test_api.py --url http://<pi-ip>:8000 --key <key> --hardware
  ```

- [ ] **The laser is OFF when SCOPIO is down.** Stop the nodes and look at the laser:

  ```bash
  docker compose stop scopio
  ```

  The laser must be OFF. If it is ON, stop here and read [the relay section of TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md#laser-relay-is-inverted). Then start the nodes again:

  ```bash
  docker compose start scopio
  ```

- [ ] **ON means ON.** Switch the laser from the UI, or with these calls from a laptop. The laser must match every time.

  ```bash
  curl -X POST -H "X-API-Key: <key>" -H "Content-Type: application/json" -d '{"data": true}' http://<pi-ip>:8000/api/v1/service/relay/set
  ```

  ```bash
  curl -X POST -H "X-API-Key: <key>" -H "Content-Type: application/json" -d '{"data": false}' http://<pi-ip>:8000/api/v1/service/relay/set
  ```

## 12. Back up the state (recommended)

Three things on the Pi are not in git and cannot be rebuilt from it:

| Path | Holds |
|---|---|
| `ros2_ws/data/` | `calibration.json`: the image scale and the stage steps per micrometre |
| `ros2_ws/secrets/` | `api_keys.json`: the API keys |
| `ros2_ws/.env` | which instrument is where on this rig |

Copy them somewhere safe. Keep the keys private.

## Next

- [README.md](README.md): the quick start and the docs map
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md): if any check failed
- [docs/CONFIGURATION.md](docs/CONFIGURATION.md): every setting
