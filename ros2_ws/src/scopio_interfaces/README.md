# scopio_interfaces

Every message, service and action type SCOPIO defines, field by field. These types are the contract between the nodes, the gateway and every client.

**Changing anything here is expensive.** The interfaces are the first, slowest layer of the Docker build, so any edit in this folder recompiles every type on the Pi. It also changes the contract every client depends on: the SDK, the UI and the MCP all read these fields. Add fields or new types rather than change existing ones, and prefer reusing a type over adding one.

## Conventions

- **NaN means "leave unchanged"** in the float fields of a partial-update service (`SetCameraControls`, `CalibrationSet`). Integers have no NaN; each service says what 0 means. Over the API, an omitted float becomes NaN for you (see [the NaN rule](../../../docs/API.md#the-nan-rule)).
- **Replies carry `success`** and a `message` or `error` string that says why, when it failed.
- **Positions are in Sangaboard steps** (integers). Micrometres are derived through the calibration.
- **Status messages carry `connected`** and, for instruments, `last_error`.

## Messages

### StagePosition

Published on `stage/position`. The stage position, counted open-loop from wherever the stage was when `stage_node` started.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Time stamp |
| `connected` | `bool` | The board is open |
| `x`, `y`, `z` | `int32` | Position in steps |
| `x_um`, `y_um`, `z_um` | `float32` | Position in micrometres: steps ÷ `steps_per_um` (1.0 until calibrated, so equal to the steps) |

### StagePoint

One absolute target, used in `MoveStagePath`.

| Field | Type | Meaning |
|---|---|---|
| `x`, `y`, `z` | `int32` | Target in steps |

### CameraState

Published on `camera/state` at 2 Hz. The camera's live settings.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Time stamp |
| `connected` | `bool` | The camera server answers |
| `target_fps` | `float32` | The frame rate asked for |
| `measured_fps` | `float32` | The frame rate actually delivered, from the server's encoder frame counter. Can be well below `target_fps`. |
| `exposure_us` | `int32` | Exposure time in microseconds |
| `analogue_gain` | `float32` | Sensor gain |
| `red_gain`, `blue_gain` | `float32` | Colour gains |
| `green_gain` | `float32` | Kept for the contract; has no effect |
| `colour_gain` | `float32` | A multiplier on red and blue |
| `contrast`, `saturation`, `brightness`, `sharpness` | `float32` | Image controls |
| `width`, `height` | `int32` | Frame size in pixels, from the running sensor mode |

### AwgStatus

Published on `awg/status`. The AWG link's health. The node does not interpret commands.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Time stamp |
| `connected` | `bool` | A session is open |
| `idn` | `string` | The instrument's `*IDN?` reply |
| `last_command` | `string` | The last command or method relayed |
| `last_error` | `string` | The last error, empty if none |

### TemperatureStatus

Published on `temperature/status`, polled. What a control loop needs: the reading, the setpoint it is chasing, the TEC drive and the faults. That is why it is not `sensor_msgs/Temperature`.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Time stamp |
| `connected` | `bool` | A session is open |
| `idn` | `string` | The controller's `*IDN?` reply |
| `temperature` | `float32` | Actual temperature at the selected sensor, in `units`. NaN when unknown. |
| `setpoint` | `float32` | Target temperature, in `units` |
| `current` | `float32` | TEC current in amps; positive heats, negative cools |
| `voltage` | `float32` | TEC voltage in volts |
| `output` | `bool` | TEC output enabled. Nothing heats or cools until it is. |
| `in_tolerance` | `bool` | Inside the tolerance window (`set_tolerance`) |
| `units` | `string` | `C`, `K`, `F` or `raw` |
| `condition` | `uint16` | Raw `TEC:COND?` bit field |
| `faults` | `string[]` | Decoded fault bits, e.g. `["sensor_open"]`. Empty means healthy. |
| `last_error` | `string` | The last error, empty if none |

### Calibration

Published on `calibration`, latched. The spatial calibration. How to use `um_per_px_width` and `um_per_px_window` is explained in [scopio_microscope/README.md](../scopio_microscope/README.md#calibration_node).

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Time stamp |
| `has_um_per_px` | `bool` | An image scale has been set |
| `um_per_px` | `float64` | Micrometres per image pixel (0 when unset) |
| `um_per_px_width` | `int32` | Frame width, in pixels, the scale was measured at. 0: unknown. |
| `um_per_px_window` | `int32` | Sensor window width, in sensor pixels, that frame spanned. 0: unknown. |
| `steps_per_um_x`, `_y`, `_z` | `float64` | Stage steps per micrometre, per axis |

## Services

### StageJog

`stage/jog`. Move the stage by a relative amount. Step size is the client's choice.

| Request | Type | | Response | Type |
|---|---|---|---|---|
| `dx`, `dy`, `dz` | `int32` | | `success` | `bool` |
| | | | `message` | `string` |
| | | | `x`, `y`, `z` | `int32`: the position afterwards, in steps |

### MoveAbs

`stage/move_abs`. Move to one absolute target. For several, use the `stage/move_path` action.

| Request | Type | | Response | Type |
|---|---|---|---|---|
| `x`, `y`, `z` | `int32`: target in steps | | `success`, `message` | `bool`, `string` |
| | | | `x`, `y`, `z` | `int32`: the position afterwards |

### SetCameraControls

`camera/set_controls`. Set any subset of the manual controls. NaN leaves a field unchanged.

| Request | Type | Meaning |
|---|---|---|
| `red_gain`, `green_gain`, `blue_gain`, `colour_gain` | `float64` | Must be > 0. `green_gain` has no effect. |
| `analogue_gain` | `float64` | Treated as a brightness change: it sets the exposure budget, so it survives the next frame-rate change. |
| `contrast`, `saturation`, `brightness`, `sharpness` | `float64` | Image controls. `brightness` may be 0. |

Response: `success` (`bool`), `message` (`string`).

### SetFramerate

`camera/set_framerate`. Set the capture frame rate. The node derives exposure and gain to keep the brightness.

| Request | Type | | Response | Type |
|---|---|---|---|---|
| `fps` | `float64` (required; NaN is refused) | | `success` | `bool` |
| | | | `framerate` | `float64`: the fps applied, after clamping to 1–120 |
| | | | `exposure_us` | `int32` |
| | | | `analogue_gain` | `float64` |

### WhiteBalance

`camera/white_balance`. One-shot auto white balance: measure the gains, then lock them. Empty request.

Response: `success` (`bool`), `red_gain`, `blue_gain` (`float64`), `message` (`string`).

### AwgWrite

`awg/write`. Send a raw SCPI command to the AWG. The caller must know the instrument's command language.

| Request | Type | | Response | Type |
|---|---|---|---|---|
| `command` | `string`, e.g. `:OUTPut1 OFF` | | `success` | `bool` |
| | | | `error` | `string`, empty on success |

### AwgQuery

`awg/query`. Send a raw SCPI query to the AWG and return its reply.

| Request | Type | | Response | Type |
|---|---|---|---|---|
| `command` | `string`, e.g. `*IDN?` | | `success` | `bool` |
| | | | `response` | `string`: the reply |
| | | | `error` | `string`, empty on success |

### CalibrationSet

`calibration/set`. Update the calibration. It is saved to disk and republished.

| Request | Type | Meaning |
|---|---|---|
| `um_per_px` | `float64` | New image scale. NaN or 0: leave unchanged. |
| `um_per_px_width` | `int32` | Frame width the scale was measured at. Send it with every `um_per_px`. 0: leave unchanged. |
| `um_per_px_window` | `int32` | Sensor window width that frame spanned. Send it with every `um_per_px`. 0: leave unchanged. |
| `steps_per_um_x`, `_y`, `_z` | `float64` | Stage steps per micrometre. NaN or 0: leave unchanged. |

Response: `success` (`bool`), `message` (`string`).

### InstrumentCall

`awg/call`, `temperature/call`. Call any public method of the driver class inside an instrument node. One service covers every present and future capability, so the contract never grows a service per method. How to use it is in [docs/API.md](../../../docs/API.md#instrument-calls).

| Request | Type | Meaning |
|---|---|---|
| `method` | `string` | A driver method, or `list_methods`, `connected`, `reconnect` |
| `args` | `string` | A JSON array of positional arguments; `""` for none |
| `kwargs` | `string` | A JSON object of keyword arguments; `""` for none |

| Response | Type | Meaning |
|---|---|---|
| `success` | `bool` | |
| `result` | `string` | The return value as JSON; `"null"` for none |
| `error` | `string` | Empty on success |

Method names are not part of this contract. They are whatever the driver class has today: call `list_methods`.

## Actions

### MoveStagePath

`stage/move_path`. Visit a list of absolute targets in order. The client sends the whole path, so execution does not depend on network latency.

| Part | Field | Type | Meaning |
|---|---|---|---|
| Goal | `points` | `StagePoint[]` | The targets, in steps |
| Goal | `settle_s` | `float32` | Pause at each point |
| Result | `success` | `bool` | |
| Result | `points_reached` | `int32` | |
| Feedback | `current_index` | `int32` | The point just reached |
| Feedback | `x`, `y`, `z` | `int32` | The position there |

### ScanRegion

`scan_region`. Visit a grid of stops over a rectangle, snaking row by row, at the current Z. The Pi only drives the pattern; what each frame shows is the client's job.

| Part | Field | Type | Meaning |
|---|---|---|---|
| Goal | `x_min`, `x_max`, `y_min`, `y_max` | `int32` | The rectangle in steps, inclusive |
| Goal | `step` | `int32` | Steps between stops (at least 1) |
| Goal | `settle_s` | `float32` | Pause at each stop |
| Result | `success` | `bool` | |
| Result | `frames_visited` | `int32` | |
| Feedback | `frames_visited` | `int32` | Stops so far |
| Feedback | `x`, `y` | `int32` | The current stop |

### Autofocus

`camera/autofocus`. Sweep Z around the current position, score sharpness at each point, and park at the sharpest. Every Z is approached from below to take up backlash.

| Part | Field | Type | Meaning |
|---|---|---|---|
| Goal | `z_range` | `int32` | Half-range of the sweep in steps, e.g. 2000 |
| Goal | `steps` | `int32` | Number of points, at least 3, e.g. 15 |
| Goal | `settle_s` | `float32` | Pause after each move before scoring, e.g. 0.2 |
| Result | `success` | `bool` | |
| Result | `best_z` | `int32` | The Z the stage ended at |
| Result | `best_score` | `float32` | Its sharpness score |
| Result | `message` | `string` | |
| Feedback | `index` | `int32` | The point just measured |
| Feedback | `z` | `int32` | Its Z |
| Feedback | `score` | `float32` | Its sharpness score |

## Next

- [docs/API.md](../../../docs/API.md): calling these over HTTP
- [scopio_microscope/README.md](../scopio_microscope/README.md): the nodes behind them
- [docs/ADDING_A_NODE.md](../../../docs/ADDING_A_NODE.md#6-the-status-message-only-if-no-existing-type-fits): adding a type
