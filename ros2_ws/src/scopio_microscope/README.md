# scopio_microscope

Reference for every hardware node: what it publishes and serves, its parameters, and the behaviour worth knowing before you use it or change it.

All names are under the `/scopio` namespace. All nodes start from `launch/microscope.launch.py` and read `config/params.yaml` (parameter tables with defaults are in [CONFIGURATION.md](../../../docs/CONFIGURATION.md#paramsyaml)). Message fields are in [scopio_interfaces/README.md](../scopio_interfaces/README.md).

Every node starts without its hardware, reports `connected=false`, and retries on a timer.

## Package layout

| File | What it is |
|---|---|
| `scopio_microscope/camera_node.py` | The camera's ROS surface, a client of the camera server |
| `scopio_microscope/stage_node.py` | The Sangaboard XYZ stage |
| `scopio_microscope/galvo_node.py` | The Rigol DG1022Z AWG that drives the galvo mirrors |
| `scopio_microscope/temperature_node.py` | The Wavelength TC10 LAB temperature controller |
| `scopio_microscope/relay_node.py` | The laser relay on GPIO17 |
| `scopio_microscope/calibration_node.py` | The persisted spatial calibration |
| `scopio_microscope/connect_guard.py` | A deadline for instrument connect attempts |
| `scopio_microscope/drivers/` | Instrument driver classes and the generic dispatcher. [README](scopio_microscope/drivers/README.md) |
| `launch/microscope.launch.py` | Starts all six nodes and prints the image build stamp |
| `config/params.yaml` | Node parameters |

## camera_node

| Kind | Name | Type |
|---|---|---|
| publishes | `image/compressed` | `sensor_msgs/CompressedImage` (JPEG) |
| publishes | `camera/state` | `CameraState`, at 2 Hz |
| service | `camera/set_controls` | `SetCameraControls` |
| service | `camera/set_framerate` | `SetFramerate` |
| service | `camera/white_balance` | `WhiteBalance` |
| action | `camera/autofocus` | `Autofocus` |
| calls | `stage/jog` | `StageJog` (for autofocus) |

It does not own the sensor. It is a client of the [camera server](../../../camera_server/README.md) at `CAMERA_URL` (`http://127.0.0.1:8081`).

- **It ingests video only when needed.** It opens the server's MJPEG stream only while `image/compressed` has a subscriber or autofocus runs. Nothing subscribes by default: clients get video from the gateway's MJPEG proxy, not from this topic. Scanning every frame of a 200 fps stream for nobody wasted the Pi's CPU.
- **It republishes each frame once.** Frames are passed through verbatim (no re-encode), and a frame already published is never sent again, so a slow camera never looks like a fast one.
- **`camera/state` follows the server.** Clients can change the camera two ways: these ROS services, or the gateway's `/camera/*` routes, which go straight to the server. So every 2 s the node reads the server's `/controls` back and adopts the real size, frame rate, exposure, gain, colour gains and image controls. `measured_fps` comes from the server's encoder frame counter.
- **It pushes its settings when the server first answers**, and again after the server restarts, so the two never disagree about what the camera is doing. Settings are adopted from the server only after that push, so a restarted server's defaults cannot wipe them.
- **`connected`** is true while the server answered `GET /controls` with 200 in the last 6 s, or a frame arrived in the last 5 s. The server answers 503 until its sensor is open.

**The exposure budget.** Brightness depends on exposure time × analogue gain. The node keeps that product, the budget, constant when the frame rate changes:

- `camera/set_framerate` clamps fps to 1–120, caps exposure at 92% of the frame period, and makes up the rest with gain (clamped 1–16). It returns the fps, exposure and gain it applied.
- `camera/set_controls` with an `analogue_gain` sets a new budget (exposure × that gain), then re-derives exposure and gain for the current frame rate. So the brightness change survives the next frame-rate change.
- The camera server enforces the same rule from its side: a frame cannot be shorter than its exposure, so a long exposure lowers the frame rate.

**`camera/set_controls`.** NaN in a field means "leave it unchanged". `red_gain`, `green_gain`, `blue_gain` and `colour_gain` must be greater than 0. `brightness` may be 0 (its neutral value). `colour_gain` multiplies red and blue before they reach the server. `green_gain` is kept for the interface but has no effect: the ISP has red and blue gains only.

**`camera/white_balance`.** The server turns on auto white balance, waits about 1.2 s, reads the gains it chose and locks them. A monochrome sensor refuses with a reason.

**`camera/autofocus`.**

1. Reads the current Z with a zero jog.
2. Plans `steps` points (at least 3) evenly from Z − `z_range` to Z + `z_range`.
3. At each point, moves to 256 steps (`AF_BACKLASH`) **below** the target, then up to it, so every Z is approached from the same side and the stage's backlash is taken up. Waits `settle_s`.
4. Scores a frame captured **after** the move (variance of the Laplacian; higher is sharper) and sends feedback.
5. Returns to the sharpest Z, the same way.

It can be cancelled between points. It aborts if the camera is not connected, a jog fails or times out (8 s), or no fresh frame arrives in 8 s. The sharpness score is the only pixel work on the Pi: it is a control loop, not scene analysis.

## stage_node

| Kind | Name | Type |
|---|---|---|
| publishes | `stage/position` | `StagePosition`, at `publish_rate` (5 Hz) |
| subscribes | `calibration` | `Calibration` (latched; for `steps_per_um`) |
| service | `stage/jog` | `StageJog`: relative move in steps |
| service | `stage/move_abs` | `MoveAbs`: absolute move in steps |
| action | `stage/move_path` | `MoveStagePath` |
| action | `scan_region` | `ScanRegion` |

**Positions are in steps and open-loop.** The Sangaboard has no encoder and the node does no homing. The node counts steps from wherever the stage was when it started. That origin is **lost whenever the node restarts**: position is relative to the current session. A board that is power-cycled while the node runs also loses its true position, but the node keeps counting.

- The position in micrometres is steps ÷ `steps_per_um` from the calibration (1.0 per axis until calibrated).
- Every move takes one lock. `move_abs` computes its delta **inside** that lock, so a jog from another client cannot land between reading the position and moving.
- **3 failed moves in a row** close the board, and the retry timer reopens it. The counted position is kept.
- The port is chosen in this order: `SANGABOARD_PORT` from `.env`, the `port` parameter, then auto-detection, then the header UARTs (`/dev/serial0`, `/dev/ttyAMA0`, `/dev/ttyS0`). On failure the node logs every serial port it can see.

**`stage/move_path`** visits a list of absolute points in order, waits `settle_s` at each, and sends feedback after each. The client sends the whole path at once, so execution does not depend on network latency. It stops at the first failed move.

**`scan_region`** visits a grid from (`x_min`, `y_min`) to (`x_max`, `y_max`) inclusive, `step` apart, at the current Z. It snakes (boustrophedon): even rows left to right, odd rows right to left, to cut travel. It waits `settle_s` at each stop. It only drives the pattern: what each frame shows is the client's business, from the video.

## galvo_node

| Kind | Name | Type |
|---|---|---|
| publishes | `awg/status` | `AwgStatus`, at `publish_rate` (5 Hz) |
| service | `awg/call` | `InstrumentCall`: any public method of `DG1022Z` |
| service | `awg/write` | `AwgWrite`: a raw SCPI command |
| service | `awg/query` | `AwgQuery`: a raw SCPI query and its reply |

The AWG drives the two galvo mirrors: **CH1 = X, CH2 = Y**. The node owns one `drivers/dg1022z.DG1022Z` object and exposes all of it (see [the driver contract](scopio_microscope/drivers/README.md)).

- **Galvo values are volts of deflection**, measured from each axis's offset (`offsets()`). The voltage on the connector is deflection + offset. `update(1, 0)` centres X at its offset, which is 0 V only if the offset is 0.
- **On connect**, with `init_on_connect: true`, it runs `dcinit`: HighZ load, both channels DC at their offsets, outputs ON.
- **Offsets and positions live in the driver object.** A reconnect builds a new one, so offsets go back to 0 V. `position()` is where the mirrors were last commanded, by any client. The AWG has no position readback.
- The galvo methods: `dcinit`, `offsets`, `update`, `position`, `move` (a ramp), `sininit`, `sinupdate`. Everything else in the class is the plain instrument. `list_methods` lists all of it.
- `awg/write` and `awg/query` go through the driver's locked, paced `command()` and `query()`, so raw SCPI cannot interleave with other calls.
- `awg/status` publishes cached state only: `connected`, the `*IDN?` reply, the last command and the last error. It never polls the AWG, because a status query would share the session with multi-second waveform uploads.
- **3 failed calls in a row** drop the session; the timer reconnects. A bad method name, bad JSON, wrong arity, or an argument the driver refuses (`ValueError`/`TypeError`, such as channel 3) does **not** count: the link is fine.
- On shutdown it turns both outputs off.

## temperature_node

| Kind | Name | Type |
|---|---|---|
| publishes | `temperature/status` | `TemperatureStatus`, polled at `publish_rate` (1 Hz) |
| service | `temperature/call` | `InstrumentCall`: any public method of `TC10LAB` |

The node owns one `drivers/TC10LAB.TC10LAB` object and exposes all of it. Raw SCPI goes through the same service: `{"method": "query", "args": "[\"TEC:ACT?\"]"}`.

- **It polls.** Each status publish calls `status()`: five queries (condition, actual temperature, setpoint, current, voltage). At 1 Hz that is five USB round trips per second.
- **The setpoint is separate from the output.** `set_setpoint(25)` changes the target, but nothing heats or cools until `output(True)`. The rear-panel Remote Enable input can override the output.
- **For a slow, near-linear change use `ramp(target, rate)`** (degrees per minute) instead of `set_setpoint`, which jumps and makes the loop ring. The ramp runs in the background; `ramp_status()` and `ramp_stop()` follow and end it. See [the setpoint ramp](scopio_microscope/drivers/README.md#tc10-setpoint-ramp).
- **The transport is chosen by who owns the device**, not by the form of `TCLAB_RESOURCE`. See [the driver contract](scopio_microscope/drivers/README.md#tc10-transport-by-ownership).
- Units are forced on connect (`units: "C"`). They are only a label: a failure there never fails the connect.
- **3 failed I/Os in a row** drop the session. One slow reply is normal on a polled USB-TMC instrument, and `last_error` on the status topic shows it at once. A refused argument or an unparseable reply in a call does not count.
- On shutdown it hands the front panel back (`LOCAL`) and **leaves the TEC output as it is**, so a sample held at temperature survives a backend restart.

## relay_node

| Kind | Name | Type |
|---|---|---|
| publishes | `relay/state` | `std_msgs/Bool`, latched (TRANSIENT_LOCAL, depth 1) |
| service | `relay/set` | `std_srvs/SetBool`: `data: true` = laser ON |

The laser relay on BCM GPIO17 (physical pin 11), through gpiozero with the `lgpio` backend.

- **`active_high: false` on this rig.** The pin is driven LOW for ON. The boot config must hold the pin high from power-on (`gpio=17=op,dh`): until this node claims it, the pin is pulled LOW, which here means laser ON. See [the relay section of TROUBLESHOOTING.md](../../../docs/TROUBLESHOOTING.md#laser-relay-is-inverted).
- **It never reports OFF for a state it did not reach.** If switching fails, it tries to switch off. If that fails too, it publishes ON ("unknown" is treated as energised), logs "TREAT THE LASER AS ON", and releases the pin. Reopening the pin drives it OFF, so the retry is also the safe action.
- It claims the pin OFF, publishes OFF, and retries every `reconnect_period` if the pin is busy. The topic exists even while the pin does not.
- **It switches the relay off on shutdown**, even if the ROS context is already gone.
- Do not use BCM 14, 15 or 23 to 25: the Sangaboard uses them.

## calibration_node

| Kind | Name | Type |
|---|---|---|
| publishes | `calibration` | `Calibration`, latched |
| service | `calibration/set` | `CalibrationSet` |

The single source of truth for how pixels and steps map to micrometres. It is stored in `calibration.json` in the working directory: `/data` in the container, `ros2_ws/data/` on the Pi. It survives rebuilds, `docker compose down` and reboots. The node logs the absolute path at start-up.

```json
{
  "um_per_px": 0.5,
  "um_per_px_width": 1640,
  "um_per_px_window": 3280,
  "steps_per_um": {"x": 1.0, "y": 1.0, "z": 1.0}
}
```

- `um_per_px`: micrometres per image pixel, measured on a frame `um_per_px_width` pixels wide that spanned `um_per_px_window` sensor pixels.
- `steps_per_um`: stage steps per micrometre, per axis.

**Why the scale needs both a width and a window.** The camera switches sensor modes, and a mode changes two independent things. Binning changes how much slide one image pixel covers. Cropping changes how much slide you see, and leaves the scale alone. The scale tracks sensor pixels per image pixel, window ÷ width, so a client converts with:

```
um_per_px_now = um_per_px × (window_now / width_now) / (um_per_px_window / um_per_px_width)
```

On a Camera Module 2, the detail mode (1640 of 3280 sensor pixels) and the fast mode (640 of 1280) are both 2× binned: the same scale, a different field of view. Scaling by width alone would be wrong by 2.56×. A width or window of 0 means "measured before these fields existed": assume the current mode and do not convert. The current width and window are in `GET /api/v1/camera/controls` (`width`, `window`).

**`calibration/set` rules.**

- NaN **or 0** in a float field leaves it unchanged. Negative or infinite values are refused.
- `um_per_px_width` and `um_per_px_window` are applied only together with a new `um_per_px`, and only when greater than 0.
- A request that sets nothing is refused with "nothing to set".
- The file is written atomically (a temporary file, fsync, then rename), so a power cut cannot leave half a file. If the write fails, the new values still apply and are published, and the reply says "applied in memory but NOT persisted".

## connect_guard.py

Opening a USB-TMC instrument through pyvisa-py can **hang** inside libusb: not time out, hang. On this Pi it happened when the kernel's usbtmc driver owned the TC10 and pyvisa-py tried to take it over. The VISA timeout does not cover the open. Without a guard, a node that connects on its timer thread freezes for good: the retry timer never fires again, the status topic stops, and nothing says why.

`ConnectGuard(deadline_s).run(attempt, discard)` runs the connect attempt on a worker thread:

- Past the deadline, the attempt is **abandoned** and `ConnectHung` is raised. A thread stuck in C code cannot be killed, so it is left behind.
- While the abandoned attempt is still alive, every new attempt is refused at once with `ConnectHung`. A second libusb session on a device the first is stuck on only makes it worse.
- If the abandoned attempt finishes late after all, `discard` closes whatever it opened.
- `attempt` must clean up after itself when it raises.

`galvo_node` and `temperature_node` use a deadline of 3 × `timeout_ms` + 5 s (50 s and 20 s). Standard library only.

## Next

- [drivers/README.md](scopio_microscope/drivers/README.md): the instrument driver contract
- [docs/ADDING_A_NODE.md](../../../docs/ADDING_A_NODE.md): adding a node
- [scopio_interfaces/README.md](../scopio_interfaces/README.md): every message field
