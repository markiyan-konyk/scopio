# Troubleshooting

A method for finding what is wrong, then the known symptoms with their causes and fixes. Commands run on the Pi, in `scopio/ros2_ws`, unless noted.

## The method

Go in this order. Each step rules out a layer.

**1. Is the image current?** The nodes run from the image, not from the repo. After an edit or a `git pull` without `--build`, the old code runs and the log looks exactly like a fix that failed. The first log line says when the image was built:

```bash
docker compose logs scopio | grep "image built"
```

If that predates your change, rebuild:

```bash
docker compose up -d --build
```

**2. Are the containers up, and what do they say?**

```bash
docker compose ps
```

```bash
docker compose logs --tail 100 scopio camera gateway
```

Each node logs why its hardware is unavailable, once a minute while it retries. That message is the diagnosis: read it before changing anything.

**3. Is the ROS graph complete?** Every node should read `ok`. A `MISSING` node crashed, usually on import: its traceback is in `docker compose logs scopio`. A topic with "no message" means its hardware is absent, but the node is up.

```bash
docker compose exec scopio /entrypoint.sh bash /ros2_ws/scripts/smoke_test.sh
```

**4. Does the API work end to end?** From a laptop, with `pip install requests websocket-client`:

```bash
python3 scripts/smoke_test_api.py --url http://<pi-ip>:8000 --key <key> --hardware
```

**5. What does the gateway say?** `/health` needs no key:

```bash
curl http://localhost:8000/api/v1/health
```

| Field | `false` means |
|---|---|
| `ros_ok` | The gateway is not attached to the ROS graph. Check `docker compose logs gateway` for "Gateway starting WITHOUT ROS". |
| `camera_ok` | The camera server is down, or its sensor is not open (it answers 503 until it is). |
| `auth_configured` | `secrets/api_keys.json` is missing or empty. Every keyed route answers 401. |

`GET /api/v1/status` (with a key) shows `connected` and `last_error` for every instrument.

**6. Can the host see the instrument at all?** Stop the stack, because a running node holds its instrument open, then ask:

```bash
docker compose down
```

```bash
python3 scripts/list_instruments.py
```

It reports, per instrument: is it on the USB bus, who owns it (the kernel usbtmc driver or nobody), and does it answer `*IDN?`. It needs `pyvisa`, `pyvisa-py`, `pyusb` and `pyserial` on the host for the VISA and serial sections, and the [udev rules](../INSTALL.md#5-install-the-udev-rules-only-if-you-run-instrument-tools-on-the-host) to see instruments as a normal user. Pass an IP to probe an Ethernet unit. Start the stack again with `docker compose up -d`.

**The host sees it but the node does not?** Compare both sides:

```bash
ls -l /dev/usbtmc* /dev/ttyACM* /dev/bus/usb/*/*
```

```bash
docker compose exec scopio ls -l /dev/usbtmc* /dev/ttyACM* /dev/bus/usb/*/*
```

`privileged: true` alone copies the host's `/dev` only when the container is created. The `/dev:/dev` mount in the compose file is what makes later devices visible. If the lists differ, recreate the containers:

```bash
docker compose up -d --force-recreate
```

If neither side lists the device, it is the cable, the power or a hub.

## Symptoms

### Temperature controller (TC10 LAB)

| Log message | Cause | Fix |
|---|---|---|
| "no TC10 LAB on the USB bus: the kernel reports no 1a45 device" | The kernel does not see it. | Cable, the rear power switch, the hub. An Ethernet unit must be named: `TCLAB_RESOURCE=TCPIP::<ip>::INSTR`. |
| "the kernel usbtmc driver owns the TC10 but it did not answer as one" | `/dev/usbtmcN` exists and belongs to the TC10, but `*IDN?` got no proper reply. | Power-cycle the TC10. If it persists, test the kernel side on the host: `echo '*IDN?' > /dev/usbtmc0 && head -c 200 /dev/usbtmc0` (use the right N). |
| "a TC10 LAB IS on the USB bus … but no transport reached it" | It is on the bus, but has no `/dev/usbtmc` node and VISA cannot read it. | Replug it so the kernel driver binds again. Check nothing else holds it, such as a bench script on the host. |
| `ConnectHung`: "connect did not return within … s -- stuck inside the USB stack" | A connect attempt hung inside libusb and was abandoned. | Unplug and replug the TC10, or restart the container. |
| `ConnectHung`: "an earlier connect attempt is still stuck inside the USB stack" | The abandoned attempt is still stuck, so the node refuses to start another on the same device. | Unplug and replug the TC10, or restart the container. |
| "… answered *IDN? with … -- that is not a Wavelength TC10 LAB" | The configured address names another instrument, usually the AWG. | Empty `TCLAB_RESOURCE` for USB. |
| "TCLAB_RESOURCE … is a VISA address, but the kernel usbtmc driver owns the TC10" | `.env` names a VISA address the node will not use. | Harmless, the node uses the kernel device. Empty `TCLAB_RESOURCE` to silence it. |
| "reply to … exceeded 256 bytes; session would desync" | A query's reply was longer than one kernel read. | Use a shorter query, or extend `UsbtmcDevice.query` to read until a newline. |
| "TC10 LAB dropped after 3 failures" | Three I/O failures in a row. The node reconnects. | Read `last_error` on `temperature/status`. If it is timeouts under load, lower `publish_rate`. |

Do not run a bench script against the TC10 while the stack is up. Both would read one reply queue and get each other's answers.

### AWG and galvo (Rigol DG1022Z)

| Log message | Cause | Fix |
|---|---|---|
| "no Rigol AWG on the USB bus: the kernel reports no 1ab1 device" | The kernel does not see it. | Cable, power, hub. An Ethernet unit must be named in `GALVO_RESOURCE`. |
| "a Rigol IS on the USB bus … but VISA listed … none of them it" | It is on the bus, but libusb cannot read it. | Check nothing else holds it, such as a bench script. Replug. |
| `VI_ERROR_INV_RSRC_NAME` / "Parsing error" | The **string** in `GALVO_RESOURCE` is malformed. Not hardware. | Fix `.env`: no quotes, no comment on the same line. |
| `VI_ERROR_RSRC_NFOUND` | The string parsed, but nothing with that address is plugged in. | Check the serial number, or empty `GALVO_RESOURCE` to auto-discover. |
| "… answered *IDN? with … -- that is not a Rigol AWG" | `GALVO_RESOURCE` names another instrument. | Fix or empty `GALVO_RESOURCE`. |
| "AWG dropped after 3 failures" | Three I/O failures in a row. The node reconnects. | Read `last_error` on `awg/status`. |

Find a paste-ready address with `list_instruments.py` (method step 6). Verify the link with `POST /api/v1/service/awg/query` and `{"command": "*IDN?"}`.

After a reconnect, the galvo offsets are back to 0 V. Set them again with `offsets(x, y)`.

### Stage (Sangaboard)

| Log message | Cause | Fix |
|---|---|---|
| "Sangaboard unavailable; node runs, reports connected=false", followed by "serial ports here: …" | No port opened. The list shows what the container can see. | See below. |
| "Sangaboard dropped after 3 failed moves" | Three moves failed in a row. The node reopens the board. | If the board was power-cycled, the position is wrong from now on. Restart `scopio` to zero it at the current place. |

- **A v0.5 HAT on the header** has no USB identity, so auto-detection never finds it. Set `SANGABOARD_PORT=/dev/serial0`, enable the UART and turn the serial login console off ([INSTALL.md step 4](../INSTALL.md#4-edit-the-boot-config-required)). A login console on the port holds it and talks over the stage.
- **A USB board** behind an unrecognised bridge (CH340, FTDI) shows in the port list but is not detected. Name it: `SANGABOARD_PORT=/dev/ttyACM0` (the path from the list).
- **"serial ports here: (none visible to this process)"** means the board does not reach the container at all. Check the host with `ls /dev/serial* /dev/ttyACM*`.

### Camera

Ask the camera server. It answers 503 with the reason until the sensor opens:

```bash
curl http://127.0.0.1:8081/controls
```

| `diagnosis` in the reply (also in `docker compose logs camera`) | Fix |
|---|---|
| "libcamera itself failed to load" | The container's libcamera does not match the host kernel. Use the [systemd fallback](../camera_server/README.md#systemd-fallback). |
| "libcamera loaded but sees NO sensor" | Nothing holds the sensor; it is not there. Reseat the ribbon at both ends, then check the host with the camera stopped (below). |
| "libcamera SEES the sensor but could not open it" | Something else has it. Only one owner is allowed: the container, the `scopio-camera` systemd unit, or a stray `rpicam-hello`. Stop the others. |

`rpicam-hello` fails with "Pipeline handler in use by another process" whenever the camera server holds the sensor, which is whenever things work. Stop the server first:

```bash
docker compose stop camera
```

```bash
rpicam-hello --list-cameras
```

```bash
docker compose start camera
```

**Video frozen, or camera stopped?** Call `GET /controls` twice, a few seconds apart, and compare `frames`:

- `frames` keeps climbing: the camera is fine. The problem is downstream: the gateway, the network or the viewer. Reconnect the viewer.
- `frames` stops, and `frame_age_s` grows: the encoder stopped. The cause is the camera server or the sensor. Check `docker compose logs camera`, then `docker compose restart camera`.

A stream with no new frame for 10 s is closed by the server, so a viewer sees it end instead of hanging.

| Other camera log lines | Meaning |
|---|---|
| "Camera does not advertise 'AwbEnable'; ignoring it from now on (monochrome sensor?)" | A monochrome sensor. Colour gains and white balance do nothing. Not an error. |
| "exposure … us does not fit the frame; frame rate lowered to … fps" | The exposure is longer than one frame. The frame rate gave way. |
| "mode … failed (…); restoring …" | A sensor-mode switch failed. The previous mode is running again. |
| `camera_node`: "camera stream disconnected (…); retrying in 2 s" | The node lost the MJPEG stream. It reconnects by itself. |

### Laser relay is inverted

The relay is configured with `active_high: false` because, on the bench on 2026-10-01 with `active_high: true`, the UI said ON while the laser was off, and the reverse. Two different faults cause that, and they need different fixes. It is not yet confirmed which one this rig has.

Watch the **relay module itself** while you switch the laser: is the relay energised (its LED lit, after it clicks in) when the laser is on, or when it is off? The answer does not depend on how `active_high` is set.

| The relay is energised when the laser is… | Cause | Fix |
|---|---|---|
| **ON** | The wiring is right (NO contact). The board is active-low: it energises when the pin is LOW. | Keep `active_high: false` **and** `gpio=17=op,dh` in the boot config. |
| **OFF** | The laser is wired to the relay's **NC** (normally closed) contact. | Move the laser to the **NO** (normally open) contact, then set `active_high: true` and rebuild. |

**NC wiring is unsafe**: the laser is ON whenever the relay loses power or its drive, including whenever SCOPIO is down. Fix it, do not work around it in software.

After any change, check both: with `docker compose stop scopio` the laser is OFF, and the UI's ON/OFF matches the laser every time.

| Relay log message | Meaning |
|---|---|
| "Relay unavailable on BCM GPIO17; node runs, relay/set reports the failure." | The pin could not be claimed (held by another process, or the GPIO device is not visible in the container). The node retries every 10 s. |
| "Relay is stuck; TREAT THE LASER AS ON until it recovers." | A switch failed, and so did switching it off. `relay/state` reports ON. The node reopens the pin, which drives it off. |
| "Relay would NOT switch off" | At shutdown, the relay could not be switched off. Check the laser by hand. |

### Calibration

| Symptom | Cause | Fix |
|---|---|---|
| "Calibration file … is unreadable … using defaults" | `data/calibration.json` is corrupt. | Fix or delete it. The next `calibration/set` writes a new one. |
| `calibration/set` replies "applied in memory but NOT persisted" | The file could not be written. | Check that `ros2_ws/data/` exists and is writable. The node logs the absolute path it uses at start-up. |
| The calibration resets after a restart | The `./data` mount is missing, or the node wrote somewhere else. | The start-up log line "Calibration in …" must say `/data/calibration.json`. |

### Builds slow down or fail

Every rebuild leaves the old image on the SD card. Check the space:

```bash
df -h /
```

```bash
docker system df
```

Reclaim it:

```bash
docker image prune -f && docker builder prune -f
```

### Gateway status codes

| Code | Meaning |
|---|---|
| 401 | No API key, or a wrong one. |
| 404 | No such service in the ROS graph. Check the name with `GET /api/v1/interfaces`. |
| 422 | A request field is unknown or has the wrong type. |
| 502 | The camera server sent something that is not JSON. |
| 503 | The gateway is not attached to ROS, or the camera server is unreachable, or its sensor is not open. |
| 504 | The service did not answer in time. The node is down or the hardware is stuck. |

Camera routes pass the camera server's own codes through: 400 for a bad mode, 409 for white balance on a monochrome sensor.

## Next

- [CONFIGURATION.md](CONFIGURATION.md): every setting
- [scopio_microscope/README.md](../ros2_ws/src/scopio_microscope/README.md): how each node behaves
- [ros2_ws/scripts/README.md](../ros2_ws/scripts/README.md): the debugging tools
