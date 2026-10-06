# scripts

The tools for keys, health checks and debugging: what each is for, where to run it, and the exact command.

Inside the container these scripts live at `/ros2_ws/scripts`, and the working directory is `/data`, so run them there by absolute path. On the host or a laptop, run them from `ros2_ws`.

| Script | For | Where it runs | Needs |
|---|---|---|---|
| `generate_api_key.py` | create, list and revoke API keys | Pi host (or a laptop for the dev stack) | Python 3 only |
| `smoke_test.sh` | check the ROS graph | inside the `scopio` container | ROS sourced |
| `smoke_test_api.py` | check the API end to end | a laptop, or the Pi host | `requests`, `websocket-client` |
| `list_instruments.py` | find instruments, and how they are reached | Pi host, with the stack stopped | `pyvisa`, `pyvisa-py`, `pyusb`, `pyserial` for full output |
| `test_drivers.py` | offline tests of drivers, nodes and gateway logic | anywhere | Python 3 only |

## generate_api_key.py

Creates, rotates, lists and revokes the gateway's API keys in `ros2_ws/secrets/api_keys.json` (or `$SCOPIO_API_KEYS_FILE`). Keys are 48 hex characters. The file is made readable by its owner only. The gateway reloads it by itself.

On the Pi host, in `ros2_ws`. Create a key, or replace the key of an existing name:

```bash
python3 scripts/generate_api_key.py <name>
```

List the names:

```bash
python3 scripts/generate_api_key.py --list
```

Revoke one:

```bash
python3 scripts/generate_api_key.py --revoke <name>
```

Run it on the host, not in the container: the gateway mounts `secrets/` read-only. If Docker created `secrets/` before you did, it belongs to root and the script cannot write to it; `sudo chown $USER ros2_ws/secrets` fixes that.

## smoke_test.sh

Checks the ROS graph without needing hardware. It lists the nodes and marks each expected one `ok` or `MISSING`, waits up to 2 s for one message on each main topic, and lists the services and actions.

- `MISSING` node: it crashed. The traceback is in `docker compose logs scopio`.
- "(no message in 2s)": the node is up, but its hardware is absent (or, for `image/compressed`, nothing is ingesting video, which is normal).

On the Pi host, in `ros2_ws`. `docker compose exec` skips the image's entrypoint, so run it through the entrypoint to have ROS sourced:

```bash
docker compose exec scopio /entrypoint.sh bash /ros2_ws/scripts/smoke_test.sh
```

## smoke_test_api.py

Tests the whole path, JSON to gateway to ROS to reply, the way a client sees it: health, auth (401 without a key), `/interfaces`, generic service calls, `temperature/call list_methods`, 404 and 422 handling, the NaN rule, WebSocket telemetry, the latched calibration, the refusal of video over the WebSocket, an autofocus action, and the MJPEG stream.

- Without `--hardware`, it checks that everything **degrades** cleanly: instruments report not connected, the stream answers 503. Use it on the dev stack.
- With `--hardware`, it expects the instruments, the camera and live video. It runs a small autofocus (`z_range` 200, 3 steps) and a zero jog, so **the stage moves**.

The key comes from `--key`, then `$SCOPIO_API_KEY`, then the first key in `ros2_ws/secrets/api_keys.json`. It exits non-zero if any check fails.

On a laptop, once:

```bash
pip install requests websocket-client
```

Against the Pi, from `ros2_ws` in a clone of this repo:

```bash
python3 scripts/smoke_test_api.py --url http://<pi-ip>:8000 --key <key> --hardware
```

Against the dev stack on the same laptop (use `127.0.0.1`, not `localhost`: on Docker Desktop the WebSocket can hang on the IPv6 route):

```bash
python3 scripts/smoke_test_api.py --url http://127.0.0.1:8000
```

## list_instruments.py

Answers "why does the node not see my instrument?" in the order that matters:

1. Is the box on the USB bus at all? (from sysfs; no software setting fixes a "no")
2. Who owns it: the kernel usbtmc driver (`/dev/usbtmcN`), or nobody (VISA over libusb)?
3. Does it answer `*IDN?` on that transport?

It also lists serial ports (for the stage) and prints a paste-ready `.env` line for each instrument it finds. It follows the nodes' rules: a device the kernel owns is reached through its `/dev` node and is **never** opened over VISA, because that would detach the kernel driver. Every probe has a deadline, so a stuck libusb call cannot hang it.

On the Pi host, in `ros2_ws`. Stop the stack first: a running node holds its instrument open.

```bash
docker compose down
```

```bash
python3 scripts/list_instruments.py
```

To also probe an Ethernet unit, pass its IP:

```bash
python3 scripts/list_instruments.py 192.168.1.50
```

To learn a TC10's IP, ask it over USB:

```bash
echo 'TECH:IPADDR?' > /dev/usbtmc0 && head -c 100 /dev/usbtmc0
```

Afterwards:

```bash
docker compose up -d
```

As a normal user it needs the [udev rules](../../INSTALL.md#5-install-the-udev-rules-only-if-you-run-instrument-tools-on-the-host). Without `pyvisa`, `pyvisa-py`, `pyusb` and `pyserial` on the host, those sections print "unavailable" and the rest still runs.

## test_drivers.py

Offline checks of the logic that has no hardware in it. No ROS, no instruments: it stubs `pyvisa`, `rclpy` and `rosidl`, and drives the code against fakes. It covers the drivers (offsets, ramps, pacing, the lock, desync recovery, the TC10 transport choice), `dispatch.py`, the gateway's NaN rule and telemetry discovery, the camera server's mode choice and exposure rule, the camera node's bridge, the relay's safe-failure rule, and the contract between each instrument node and its driver.

On a laptop or the Pi host, from the repo root. It prints each check and the total, and stops at the first failure:

```bash
python3 ros2_ws/scripts/test_drivers.py
```

Every module-level function named `test_*` runs. To add a test, write one. See [drivers/README.md](../src/scopio_microscope/scopio_microscope/drivers/README.md#adding-a-method) and [ADDING_A_NODE.md](../../docs/ADDING_A_NODE.md#9-test-it-offline).

## Next

- [docs/TROUBLESHOOTING.md](../../docs/TROUBLESHOOTING.md): the debugging method that uses these tools
- [docs/SECURITY.md](../../docs/SECURITY.md): keys
- [ros2_ws/README.md](../README.md): the workspace
