# camera_server

The only owner of the Raspberry Pi camera. A small HTTP server that streams MJPEG and takes camera settings, on loopback only. Everything else uses the camera through it.

It exists because picamera2 and libcamera ship from Raspberry Pi OS (Debian), not Ubuntu, so the camera cannot run inside the Ubuntu-based ROS image. It runs as the `camera` compose service (Debian bookworm plus the Raspberry Pi apt archive, whose libcamera covers both the Pi 4 and the Pi 5 camera stacks), or as a systemd unit on the host. The HTTP surface is the same either way.

Its two clients, both on the Pi:

- the **gateway**, which proxies the stream and the controls to authenticated clients (`/api/v1/stream.mjpg`, `/api/v1/camera/*`);
- **`camera_node`**, which reads the stream when ROS needs frames and forwards the camera services here.

| File | What it is |
|---|---|
| `pi_camera_server.py` | The server |
| `Dockerfile` | The `scopio-camera:bookworm` image |
| `scopio-camera.service` | The systemd unit for the fallback |
| `install_systemd.sh` | Installs and starts that unit |

## Endpoints

On `http://127.0.0.1:8081`. There is **no authentication**: this server must never face the LAN.

| Method and path | Does | Codes |
|---|---|---|
| `GET /stream.mjpg` (also `GET /`) | Live MJPEG, `multipart/x-mixed-replace; boundary=FRAME` | 200 |
| `GET /controls` | Current settings merged with live sensor metadata, the modes on offer, and the frame counter | 200 |
| `POST /controls` | Set settings from a partial JSON body; returns the new settings | 200 |
| `POST /mode` | `{"mode": "detail"}` or `{"mode": "fast"}` | 200, or **400** if refused |
| `POST /white_balance` | One-shot auto white balance, then lock the measured gains; returns them | 200, or **409** if refused |
| `GET /focus` | `{"metric": …}`: the size in bytes of the newest JPEG. A sharper image compresses less. | 200 |

Every endpoint answers **503** until the sensor is open, with a body that says why (see [Before the sensor opens](#before-the-sensor-opens)). An unknown path is 404. A handler that raises answers 500 with the error, instead of dropping the connection.

**`GET /controls`** returns:

| Key | Meaning |
|---|---|
| `framerate`, `exposure`, `analogue_gain` | Frame rate (fps), exposure (µs) and gain. Exposure and gain are read live from the sensor. |
| `red_gain`, `blue_gain` | Colour gains, read live |
| `contrast`, `saturation`, `brightness`, `sharpness` | Image controls, as last set |
| `green_gain`, `colour_gain` | Kept for compatibility; unused here |
| `mode` | `detail` or `fast` |
| `width`, `height` | The size of the frames being streamed |
| `window` | Sensor pixels across the frame. The calibration needs it (see [calibration_node](../ros2_ws/src/scopio_microscope/README.md#calibration_node)). |
| `modes` | What this camera module offers: `{"detail": spec, "fast": spec}`, each `{sensor, size, fps, full_fov, window}` |
| `frames` | Frames encoded since start |
| `frame_age_s` | Seconds since the newest frame |

**`POST /controls`** takes any of `framerate`, `exposure` (µs), `analogue_gain`, `red_gain`, `blue_gain`, `contrast`, `saturation`, `brightness`, `sharpness`.

- Setting `exposure` or `analogue_gain` turns auto-exposure off. Setting a colour gain turns auto white balance off.
- A 0 for frame rate, exposure, gain or a colour gain is ignored: it is a half-filled request, not a request for a black frame.
- `framerate` is clamped to 1 and the running mode's maximum. Asking for more is silently clamped by libcamera otherwise, and the reported rate would be a lie.
- Controls this sensor does not advertise are dropped, with one log line. A monochrome sensor has no colour gains or white balance. Without this, one unknown key made picamera2 reject the whole request.

**`POST /white_balance`** turns auto white balance on, waits about 1.2 s, reads the gains libcamera chose and locks them as manual gains, so they do not drift. It answers 409 on a monochrome sensor.

## Sensor modes

You cannot have both resolution and frame rate: more pixels per frame means fewer frames per second. So the server offers two modes, chosen by `pick_modes()` from what the attached module advertises. Nothing is hard-coded per sensor.

| Mode | Picks | For |
|---|---|---|
| `detail` | Of the modes that see the sensor's **full field of view**, the fastest. Scaled down by the ISP to at most `CAM_DETAIL_MAX_W` pixels wide (default 1640), so it streams over a LAN. | Looking at the sample |
| `fast` | The mode with the highest frame rate, whatever field it reads (usually cropped). | Motion, such as tracking particles |

Each mode reads a sensor rectangle (picamera2's `crop_limits`). Modes that share the largest rectangle see the whole slide; a smaller one is a centre crop, which on a microscope silently throws field of view away. That is why "1080p" is not the detail mode on an IMX219 (Camera Module 2): its 1920×1080 mode is a crop, while 1640×1232 is the full sensor binned, and faster. On that module, detail is 1640×1232 at about 42 fps, and fast is 640×480 at about 207 fps. The full 3280×2464 mode is not chosen: it tops out near 21 fps and would be scaled down for the network anyway.

The server starts in `CAM_MODE` (default `detail`). A switch stops recording, reconfigures and starts again, so every open stream sees a short gap. **A switch that fails restores the previous mode** and answers 400. It never leaves the camera stopped.

## A frame cannot be shorter than its exposure

If the exposure is longer than one frame period, the frame rate gives way: the frame duration becomes the exposure plus 0.5 ms, and the server logs the new rate. Asking a sensor for a frame shorter than its exposure can stall it after one frame. `camera_node` already keeps the two consistent with its exposure budget, but a client posting here directly bypasses that, so the server enforces it too.

## Before the sensor opens

The server starts serving **first**, then opens the sensor on a background thread and retries until it works (2 s, doubling to 30 s). A camera fault is reported over HTTP instead of crash-looping the container, and a camera that appears late (a replug, or the systemd unit releasing it) is picked up without a restart.

Until then, every endpoint answers 503 with `error`, `cameras` (what libcamera can see) and a one-line `diagnosis`:

| Diagnosis | Meaning |
|---|---|
| "libcamera itself failed to load …" | The container's libcamera does not match the host kernel's camera stack. Use the [systemd fallback](#systemd-fallback). |
| "libcamera loaded but sees NO sensor …" | Nothing holds it; it is not there. Check the ribbon at both ends, then the host with the camera service stopped. |
| "libcamera SEES the sensor but could not open it …" | Something else has it. Exactly one owner is allowed: this container, the `scopio-camera` systemd unit, or a stray `rpicam-hello`. |

`rpicam-hello` on the host fails with "Pipeline handler in use by another process" whenever this server holds the camera, which is whenever things work. Stop the server before using it. On the Pi, in `scopio/ros2_ws`:

```bash
docker compose stop camera
```

```bash
rpicam-hello --list-cameras
```

```bash
docker compose start camera
```

Once open, the server logs the modes on offer and the controls the sensor advertises. A sensor without `AwbEnable` is monochrome.

**Is the video frozen, or the camera?** Compare `frames` in two `GET /controls` calls. If it climbs, the camera is fine and the problem is downstream. If it stops, the encoder stopped. A stream that gets no new frame for 10 s is ended by the server, so a viewer sees it end instead of hanging.

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `CAM_HOST` | `127.0.0.1` | Bind address. Keep it on loopback. |
| `CAM_PORT` | `8081` | Port |
| `CAM_MODE` | `detail` | Mode at start |
| `CAM_DETAIL_MAX_W` | `1640` | Widest the detail stream may be before the ISP scales it. The sensor mode, and so the field of view, is unaffected. Raise it on wired gigabit. |
| `CAM_W`, `CAM_H` | `640`, `480` | Size reported before the sensor opens. The running size comes from the mode. |

Compose sets `CAM_HOST`, `CAM_PORT`, `CAM_W` and `CAM_H` (the last two from `.env`).

## Checking the container's camera

To test picamera2 inside the image, stop the running service first, then, on the Pi in `scopio/ros2_ws`:

```bash
docker compose stop camera
```

```bash
docker compose run --rm camera python3 -c "from picamera2 import Picamera2; print(Picamera2.global_camera_info())"
```

```bash
docker compose start camera
```

An empty list `[]` means libcamera sees no sensor. An exception means libcamera does not load in the container.

## systemd fallback

If libcamera fails inside the container, run the same server on the host as a systemd unit. It uses the host's `python3-picamera2`, which Raspberry Pi OS Bookworm provides.

On the Pi, in `scopio/ros2_ws`, stop the container so it releases the sensor:

```bash
docker compose stop camera
```

From the repo root, install and start the unit (the script is not executable in git, so run it through `bash`):

```bash
sudo bash camera_server/install_systemd.sh
```

Check it:

```bash
systemctl status scopio-camera
```

```bash
curl http://127.0.0.1:8081/controls
```

From now on, a plain `docker compose up -d` would start the camera container again, and the two would fight over the sensor. Either comment out the `camera` service in `ros2_ws/docker-compose.yml` (a local change to a tracked file), or start only the other two:

```bash
docker compose up -d scopio gateway
```

To go back to the container:

```bash
sudo systemctl disable --now scopio-camera
```

## Next

- [scopio_gateway/README.md](../ros2_ws/src/scopio_gateway/README.md#the-camera-proxy): how the gateway exposes this server
- [scopio_microscope/README.md](../ros2_ws/src/scopio_microscope/README.md#camera_node): `camera_node`
- [docs/TROUBLESHOOTING.md](../docs/TROUBLESHOOTING.md#camera): camera problems
