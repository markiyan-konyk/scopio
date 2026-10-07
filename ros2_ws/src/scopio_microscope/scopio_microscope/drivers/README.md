# drivers

The instrument driver classes, and the contract every driver must keep. An instrument node owns one driver object and exposes all of its public methods over one `InstrumentCall` service.

| File | Class | Node | Service |
|---|---|---|---|
| `dg1022z.py` | `DG1022Z` (Rigol AWG, the galvo mirrors) | `galvo_node` | `awg/call` |
| `TC10LAB.py` | `TC10LAB` (Wavelength temperature controller) | `temperature_node` | `temperature/call` |
| `dispatch.py` | turns a driver into a service: call a method by name with JSON arguments | both | |

These are the drivers the backend runs. Edit them in place.

## The contract

**1. Never open on construction.** `DG1022Z(resource, timeout_ms)` only stores its settings. The node calls `_open()`, under a [connect guard](../../README.md#connect_guardpy) with a deadline, and handles failure. This lets the node build the object, retry, and stay up without hardware.

**2. Only `command()` and `query()` touch the wire.** Every other method calls these two. They:

- hold `_lock` (an `RLock`). The node serves calls on a reentrant callback group while a timer may poll, so two threads are often inside the driver at once. Two threads interleaved on one USB-TMC session give garbled replies and timeouts, not a traceable error.
- **pace the wire at `MIN_INTERVAL_S` = 1/50 s.** Both instruments have tiny input buffers and lag seconds behind above about 60 commands per second. The lock alone serialises callers but lets them queue back to back, so a UI, an agent and a script together could still flood the instrument. The pacing covers every caller at once. It is a class attribute, so a test or bench script can change it.
- raise `ConnectionError` when the session is closed.

`test_every_scpi_call_goes_through_the_lock` in `scripts/test_drivers.py` fails if a line in `dg1022z.py` uses `self.device` outside the allowed places.

**3. Clear before you trust a reply.** USB-TMC does not match replies to requests. A query that times out was still sent, and its reply arrives later and sits in the instrument's output queue. Every later query then returns the **previous** answer: numbers that parse cleanly and are wrong (`frequency()` answering with the amplitude). So:

- a failed query sets `_desynced`, and the next query first runs `_resync()`;
- `_resync()` sends the USB-TMC CLEAR request, which flushes both of the instrument's buffers, then `*CLS`. Where the transport has no CLEAR, it **reads** until nothing is left. Never drain with queries: each one writes a request for every reply it reads, so the backlog survives.
- on connect, the session is cleared **before** `*IDN?`. A reply left by a session that died mid-query would otherwise be read back as the identity, and the right instrument rejected.

**4. Every connection identifies itself.** After clearing, `_open()` checks `*IDN?`: `DG1022Z` wants "RIGOL", `TC10LAB` wants "TC10" or "WAVELENGTH". A wrong address must never hand a node another node's instrument: two nodes on one instrument look like both of them flapping.

**5. Public methods are the API.** `dispatch.py` exposes every method whose name does not start with `_`, except `close` (blocked, because it would tear down the session the node owns). Private methods (`_open`, `_close`, `_drop`, `_resync`, …) are unreachable. Clients reconnect through the node's `reconnect` meta-method instead.

**6. The first line of each public docstring is functional.** `list_methods` returns it as the method's `doc`, and clients and AI agents read it to decide what to call. Write it for them: what the method does, the units, and what it returns. Some one-line methods in `TC10LAB.py` carry a trailing comment instead of a docstring, so their `doc` is empty. Give new methods a docstring.

**7. Import only the standard library and `pyvisa`.** A driver must work in the container, on the host for bench scripts, and under `test_drivers.py`, which stubs only `pyvisa`. A bad import kills the node on start-up: it is simply missing from the graph, with the traceback in `docker compose logs scopio`. Shared helpers such as `usb_vid` and `usb_present` are duplicated in each driver on purpose, so each file stands alone.

**8. Arguments refused before the wire raise `ValueError` or `TypeError`.** The nodes do not count those as instrument faults, so a client's typo never drops the session for everyone. I/O failures are `VisaIOError` or `OSError` and do count.

## How `dispatch.py` calls a method

A client sends `method`, `args` (a JSON array, as a string) and `kwargs` (a JSON object, as a string). `dispatch.call()`:

1. rejects private, blocked and unknown names with `DispatchError`;
2. parses the JSON (a bare scalar becomes one argument);
3. binds the arguments to the signature, so a wrong count is a `DispatchError`, not an instrument fault;
4. calls the method and returns its result as JSON. NaN and infinity become `null`, bytes become lists, anything else unserialisable becomes a string.

`dispatch.describe(cls)` introspects the **class**, so `list_methods` works while the instrument is disconnected. It walks the base classes too.

## DG1022Z

`DG1022Z` drives the galvo mirrors and also exposes the whole instrument.

**The galvo block** (CH1 = X, CH2 = Y):

| Method | Does |
|---|---|
| `dcinit()` | HighZ load on both channels, DC at the offsets, outputs on. Resets the remembered position to 0. |
| `offsets(x=None, y=None)` | Reads or sets the per-axis offset in volts: the zero of the position scale. Returns both. |
| `update(ch, val)` | Jumps one mirror to `val` volts of deflection: the connector gets `val` + that axis's offset. Returns `{x, y}`. |
| `position()` | Where both mirrors were last commanded, by any client. The AWG has no readback. |
| `move(ch, endval, t=1.0, steps=60)` | Ramps one mirror over `t` seconds. `steps` is capped so the ramp never outruns the pacing. |
| `sininit(freq, amp, phase)` | Starts a sine on both channels about their current positions. Omitted arguments keep the last values. Refuses an amplitude of 0. |
| `sinupdate(ch, freq, amp, phase)` | Changes one channel's sine. |

Offsets and positions live in the object only. A reconnect builds a new object, so they reset to 0.

**The rest of the instrument** follows one convention: a scalar setting is one method that sets and reads. Pass the value to set it; leave it out to read it. `frequency(1, 1000)` sets CH1 to 1 kHz; `frequency(1)` returns `1000.0`. Grouped readers (`waveform`, `am_config`, `sweep_config`, …) return a dict, so a UI panel fills itself in one round trip. `snapshot()` returns everything a UI panel needs for both channels in one call. `list_methods` shows the full list.

**AWG physics.** These rules come from the maintainer's bench work. The driver does not enforce them; the client-side `galvo.move_xy` in [scopio-apps](https://github.com/markiyan-konyk/scopio-apps) does.

- Switching which channel is being commanded makes a mechanical change inside the AWG. Move one axis, wait, then move the other.
- A connector voltage within ±2 V stays in one output range. Crossing ±2 V flips a relay inside the AWG, which is slow and wears the relay.

<!-- TODO: the settle times for these rules in scopio-apps are untuned guesses; record the measured values here once they exist. -->

## TC10LAB

`TC10LAB` exposes the whole controller: the temperature loop (`temperature`, `set_setpoint`, `output`, …), sensors, PID and autotune, safety limits, status registers, stored profiles, scripts and network settings. The command reference is the vendor's "COMMAND SET, LAB Series Instruments" (COMMAND-00400 rev H).

- **`status()` costs five queries** (`TEC:COND?`, `TEC:ACT?`, `TEC:SET?`, `TEC:I?`, `TEC:V?`). It runs on the node's poll timer, so every query added there is paid every second. `units` in it is the value cached by `set_units()` or `get_units()`.
- **`query_float()` tolerates decorated replies.** This firmware does not always answer in the documented form (`TEC:UNITS?` answers `CELSIUS`, not `0`), so it takes the leading number when a plain `float()` fails.
- **Set the safety limits before enabling the output.** `set_current_limits`, `set_temperature_limits` and `set_sensor_limits` exist; the node sets none of them.
- Transient faults (sensor open or shorted) trip the output and then clear. `faults()` may miss them; `event()` latches them (and reading it clears it).

### TC10 setpoint ramp

`set_setpoint(25)` jumps the target. The PID loop then overshoots and rings around it. `ramp(25, rate=0.5)` instead moves the setpoint in small steps, at 0.5 degrees per minute, so the loop only ever chases a small error and the temperature follows a near-straight line.

```
POST /api/v1/service/temperature/call
{"method": "ramp", "args": "[25]", "kwargs": "{\"rate\": 0.5}"}
```

| Method | What it does |
|---|---|
| `ramp(target, rate, interval=1.0, start=None)` | Starts the ramp and returns at once. `rate` is in active units per minute. `interval` is the seconds between setpoint writes (0.1 to 60). |
| `ramp_status()` | `state` (`idle`, `ramping`, `done`, `stopped`, `failed`), `start`, `target`, `rate`, `setpoint` (last written), `elapsed_s`, `remaining_s`, `error`. Costs no I/O. |
| `ramp_stop()` | Stops the ramp and holds the setpoint where it got to. |

- **It runs on the host**, in a thread inside the driver. Each tick writes `start + rate × elapsed`, so a slow or failed write never slows the ramp; the next tick catches up. Three failed writes in a row end it as `failed`.
- **It starts from the measured temperature** (`TEC:ACT?`), so the setpoint does not jump at the start. Pass `start` to override.
- **Change the slope by calling `ramp()` again**, with the same or a new target. It continues from the setpoint already reached, so changing the rate mid-way does not jump either.
- `set_setpoint`, `step_up`, `step_down`, `reset` and `recall_profile` stop a running ramp, and so does dropping the session. A ramp does not survive a reconnect or a backend restart: the setpoint stays where it got to.
- `target` and `start` must lie inside the instrument's temperature limits (`get_temperature_limits`). A ramp moves only the setpoint: nothing heats or cools until `output(True)`.
- `temperature/status` shows the moving setpoint, because `status()` already reads `TEC:SET?`.

**Choosing the rate.** The temperature lags the setpoint by about rate × the loop's response time. It overshoots the target by roughly that lag at the end, and then settles. Slower ramps give a smaller lag and a straighter line. `interval` only sets how fine the staircase is: at 1 degree per minute and 1 s, each step is 0.017 degrees, well below what the loop can resolve.
<!-- TODO: measure the lag and end overshoot on the real stage at a few rates and record a recommended range here. -->

The instrument also has its own ramps (`step_up`/`step_down` with a pause, and `profile_scan`). The host ramp exists because its rate is given directly in degrees per minute, can be changed or stopped mid-way, and reports its progress.

### TC10 transport by ownership

A USB TC10 is either owned by the kernel's `usbtmc` driver, which binds at plug-in and creates `/dev/usbtmcN`, or owned by nobody. That, not the form of `TCLAB_RESOURCE`, decides how `_open()` reaches it:

| Who owns it | Transport | How it is found |
|---|---|---|
| The kernel usbtmc driver | the `/dev/usbtmcN` character device, **never VISA** | each `/dev/usbtmc*` node's vendor id, read from sysfs; nodes sysfs cannot identify are probed with `*IDN?` |
| Nobody | VISA over libusb (pyvisa-py) | the Wavelength vendor id (`0x1A45`) in the resource string |
| Ethernet | VISA | must be named: `TCLAB_RESOURCE=TCPIP::<ip>::INSTR` (pyvisa-py cannot scan a LAN) |

**Why never VISA on a kernel-owned TC10.** pyvisa-py would have to detach the kernel driver. On this Pi that open hangs. The detach also deletes `/dev/usbtmcN` until a replug, so one failed attempt changed what the next attempt saw: "sometimes it works, sometimes it doesn't".

- **Leave `TCLAB_RESOURCE` empty for USB.** Every case above is then found.
- A configured address is tried first where it can work and overridden where it cannot. `probe_note` says so in the node's log.
- Reading the vendor id from sysfs tells the TC10's node from the AWG's without writing to either. Probing the AWG with `*IDN?` from this node would put a second client on the galvo's instrument.
- The kernel device reads in one 256-byte chunk (`READ_SIZE`). The usbtmc driver reads until it has that many bytes or the device flags end-of-message, so an over-large count makes every read wait on a device that has finished talking; 256 is the value that worked against this instrument. A longer reply raises instead of silently leaving its tail queued. If a long query (`TEC:SENSORLIST?`) is ever needed, loop until the reply ends in a newline.
- The node's `timeout_ms` is applied to the kernel device with an ioctl. Without it, the kernel's own 5 s default applies.
- When nothing is found, the error says first whether the TC10 is on the USB bus at all (from sysfs), which separates a cable problem from a software one.

## Adding a method

1. Add a public method to the driver class. Use `self.command(...)` or `self.query(...)`, never `self.device` directly.
2. Give it a docstring whose first line tells a client what it does, with units.
3. Raise `ValueError` for an argument you refuse before sending anything.
4. Add an offline test to `ros2_ws/scripts/test_drivers.py`. `awg()` builds a `DG1022Z` on a `FakeDevice` that records writes and answers queries from a table:

   ```python
   def test_frequency_sets_and_reads():
       gen = awg(**{":SOURce1:FREQuency?": "1000"})
       gen.frequency(1, 1000)
       assert ":SOURce1:FREQuency 1000" in gen.device.writes
       assert gen.frequency(1) == 1000.0
   ```

5. Run the tests on your laptop or the Pi:

   ```bash
   python3 ros2_ws/scripts/test_drivers.py
   ```

6. Rebuild on the Pi (about a minute). The method is callable over `<name>/call` and listed by `list_methods` at once. No interface, gateway, SDK or MCP change.

## Next

- [scopio_microscope/README.md](../../README.md): the nodes that own these drivers
- [docs/ADDING_A_NODE.md](../../../../../docs/ADDING_A_NODE.md): a new instrument end to end
- [docs/API.md](../../../../../docs/API.md#instrument-calls): how clients call driver methods
