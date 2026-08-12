"""Wavelength Electronics TC10 LAB temperature controller.

Same shape as dg1022z.DG1022Z: does not open on construction, the node calls
_open(). Every public method is reachable over the temperature/call service.

command() and query() are the only two methods the rest of the class uses, and
they hold _lock -- the node serves temperature/call on a reentrant callback
group while a timer polls status(), so two threads are regularly inside here.

TWO TRANSPORTS, chosen by what TCLAB_RESOURCE looks like:

  VISA         a resource string ("USB0::0x1A45::...", "TCPIP::10.0.0.5::INSTR"),
               or empty to discover the first Wavelength box on USB.
  kernel tmc   a path ("/dev/usbtmc0"). Use this when the kernel's usbtmc driver
               has claimed the instrument: pyvisa-py then has to detach that
               driver to reach it over libusb, and on some kernels the result is
               a device that enumerates fine and answers nothing -- every query
               times out. Reading and writing the char device the kernel already
               owns sidesteps the fight. The udev rule ships 0666 on it.

Command reference: COMMAND SET, LAB Series Instruments (COMMAND-00400 rev H).
Temperatures follow set_units() -- Celsius by default.
"""

import glob
import os
import re
import threading

import pyvisa

WAVELENGTH_VID = 0x1A45
USBTMC_GLOB = "/dev/usbtmc*"

CONDITION_BITS = {
    0: "current_limit",
    2: "sensor_limit",
    3: "temp_high_limit",
    4: "temp_low_limit",
    5: "sensor_shorted",
    6: "sensor_open",
    7: "tec_open_circuit",
    9: "in_tolerance",
    10: "output_on",
    11: "laser_shutdown_triggered",
    15: "front_panel_power_on",
}
FAULT_BITS = (0, 2, 3, 4, 5, 6, 7, 11)

UNITS = {0: "C", 1: "K", 2: "F", 3: "raw"}
# TEC:UNITS? answers a code on some firmware and a word ('CELSIUS') on others;
# both reduce to a first letter. TEC:UNITS takes the code either way.
UNIT_NAMES = {"C": "C", "K": "K", "F": "F", "R": "raw"}
UNIT_CODES = {"C": 0, "K": 1, "F": 2, "R": 3}


def usb_vid(resource):
    """Vendor id from a VISA resource string; pyvisa-py writes it in decimal,
    NI-VISA in hex. None for non-USB resources."""
    parts = resource.split("::")
    if len(parts) < 2 or not parts[0].upper().startswith("USB"):
        return None
    try:
        return int(parts[1], 0)
    except ValueError:
        return None


def is_tc10(idn):
    return "TC10" in (idn or "").upper() or "WAVELENGTH" in (idn or "").upper()


class UsbtmcDevice:
    """The kernel's usbtmc character device, with the slice of the pyvisa
    resource API this driver uses. The kernel driver applies its own read
    timeout, so a silent instrument surfaces as OSError(ETIMEDOUT)."""

    # The usbtmc driver reads until it has READ_SIZE bytes OR the device flags
    # end-of-message, so an over-large count makes every read wait on a device
    # that has already finished talking. 256 is the value tc10_read.py settled
    # on against this instrument.
    READ_SIZE = 256
    # USBTMC_IOCTL_CLEAR = _IO('[', 2). The kernel driver's name for the same
    # USB-TMC CLEAR request pyvisa exposes as .clear(): flush both buffers.
    IOCTL_CLEAR = 0x5B02

    def __init__(self, path):
        self.path = path
        self._fd = os.open(path, os.O_RDWR)

    def clear(self):
        import fcntl      # Linux-only; imported here so this module still loads
        fcntl.ioctl(self._fd, self.IOCTL_CLEAR)

    def write(self, cmd):
        os.write(self._fd, (cmd + "\n").encode())

    def query(self, cmd):
        self.write(cmd)
        raw = os.read(self._fd, self.READ_SIZE)
        if len(raw) == self.READ_SIZE:
            # ponytail: single read. A reply longer than READ_SIZE leaves the
            # tail queued, and every later query then returns the PREVIOUS
            # answer -- a silent, permanent desync. Fail loudly instead; if a
            # long query (TEC:SENSORLIST?) is ever needed, loop until the reply
            # ends in a newline rather than raising the constant.
            raise IOError(f"reply to {cmd!r} exceeded {self.READ_SIZE} bytes; "
                          "session would desync")
        return raw.decode(errors="replace")

    def close(self):
        os.close(self._fd)


class TC10LAB:
    def __init__(self, resource="", timeout_ms=5000):
        self.resource = resource
        self.timeout_ms = timeout_ms
        self._lock = threading.RLock()
        self.rm = None
        self.device = None
        self.units = ""       # cached by set_units()/get_units(); see status()
        self.probe_note = ""  # set when _open_usbtmc had to work around the config
        self._desynced = False  # a query failed; drain before trusting the next

    def _open(self):
        if self.resource.startswith("/dev/"):
            self._open_usbtmc()
            self._resync()
            return
        self._open_visa()
        # BOTH transports resync. This used to run on the usbtmc path only, and
        # the VISA path is if anything more exposed: a connect attempt that
        # times out (VI_ERROR_TMO) has already sent a query whose reply nobody
        # read, so the very next session starts one answer behind.
        self._resync()

    def _open_visa(self):
        self.rm = pyvisa.ResourceManager("@py")
        if not self.resource:
            # Match the vendor id in the resource string so we never open
            # another instrument just to ask what it is.
            usb = [r for r in self.rm.list_resources("USB?*INSTR")
                   if usb_vid(r) == WAVELENGTH_VID]
            if not usb:
                raise RuntimeError(
                    "no TC10 LAB on USB. Set TCLAB_RESOURCE: an Ethernet unit "
                    "must be named (pyvisa-py cannot scan the LAN), and "
                    "/dev/usbtmc0 works when the kernel driver holds the box.")
            self.resource = usb[0]
        self.device = self.rm.open_resource(self.resource)
        self.device.read_termination = "\n"
        self.device.write_termination = "\n"
        self.device.timeout = self.timeout_ms

    def _open_usbtmc(self):
        """Open a kernel usbtmc char device, VERIFYING it is this instrument.

        A /dev/... resource selects the TRANSPORT, not the node. The exact path
        was never load-bearing: /dev/usbtmc0 is not reliably the TC10 (the Rigol
        AWG is USB-TMC too, and the kernel numbers them in enumeration order, so
        a hard-coded node can point this node at the function generator -- two
        nodes on one instrument, which reads as a flapping link). So every
        /dev/usbtmc* is a candidate, each is asked *IDN?, and only a Wavelength
        box is accepted. Whatever was configured is tried FIRST, to honour an
        explicit choice; a pattern that matches nothing is a typo, not a reason
        to ignore an instrument that is plainly present.
        """
        named = sorted(glob.glob(self.resource))
        candidates = named + [p for p in sorted(glob.glob(USBTMC_GLOB))
                              if p not in named]
        if not candidates:
            raise FileNotFoundError(
                f"no usbtmc device matches {self.resource!r}, and no "
                f"{USBTMC_GLOB} exists in THIS process's /dev. In a container, "
                "compare with the host: if the host has the node and the "
                "container does not, recreate the container (its /dev is "
                "populated at creation) and check the /dev:/dev mount. If the "
                "HOST has none either, the kernel usbtmc driver is not bound: "
                "set TCLAB_RESOURCE to a VISA address instead.")
        if not named:
            self.probe_note = (f"{self.resource!r} matched nothing -- fix it in "
                               f"ros2_ws/.env; probing {candidates} instead")
        rejected = []
        for path in candidates:
            try:
                dev = UsbtmcDevice(path)
            except OSError as exc:
                rejected.append(f"{path}: {exc}")
                continue
            try:
                dev.write("*CLS")           # clear status + error queue
                idn = dev.query("*IDN?").strip()
                if is_tc10(idn):
                    self.device = dev
                    self.resource = path
                    return
                rejected.append(f"{path}: not a TC10 ({idn!r})")
            except Exception as exc:
                rejected.append(f"{path}: {type(exc).__name__}: {exc}")
            dev.close()
        raise RuntimeError("no Wavelength TC10 answered on " + ", ".join(rejected))

    def _resync(self):
        """Throw away any reply still queued in the instrument.

        Neither transport frames replies to requests: a reply nobody read stays
        queued, so the next query returns IT and everything after is one answer
        behind -- numbers that parse cleanly and are wrong.

        This uses the USB-TMC CLEAR control request, which flushes the
        instrument's input AND output buffers in-protocol. It does NOT try to
        read the backlog away with *STB?: every query WRITES one request and
        READS one reply, so a drain made of queries removes exactly as many
        replies as it adds and the backlog survives untouched. That is not
        theoretical -- it is what made this node report its instrument's
        identity as "0".

        Called on connect and after any failed query. Best effort: if the link
        is genuinely dead the caller's own query reports it, and raising from
        here would only hide that.
        """
        self._desynced = False
        try:
            self.device.clear()      # USB-TMC CLEAR: flush both buffers
        except Exception:
            pass
        try:
            self.command("*CLS")     # then the status + error queues
        except Exception:
            pass

    def _close(self):
        """Drop the session. Safe to call twice, and on an already-dead link."""
        with self._lock:
            for handle in (self.device, self.rm):
                try:
                    if handle is not None:
                        handle.close()
                except Exception:
                    pass
            self.device = self.rm = None

    def command(self, cmd):
        """Write a raw SCPI command."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("TC10 LAB session is closed")
            self.device.write(cmd)

    def query(self, cmd):
        """Write a raw SCPI query and return the reply, stripped."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("TC10 LAB session is closed")
            if self._desynced:
                self._resync()
            try:
                return self.device.query(cmd).strip()
            except Exception:
                # THE reason to care: a query that times out has still been SENT,
                # so its reply arrives later and sits in the instrument's output
                # queue. Every query after it returns the PREVIOUS answer --
                # numbers that parse cleanly, publish happily, and are wrong.
                # Nothing raises, so the failure counter never trips and the
                # node reports a healthy instrument reading its setpoint as its
                # temperature. Drain before the next query instead.
                self._desynced = True
                raise

    def query_float(self, cmd):
        """float() of a reply, tolerating a decoration the firmware may add.

        This unit does not always answer in the form the command reference
        implies -- TEC:UNITS? returns 'CELSIUS', not '0'. So a bare float() on a
        reply is a landmine: one decorated number used to take the whole node
        down at connect. Take the leading number if there is one, and only then
        give up.
        """
        reply = self.query(cmd)
        try:
            return float(reply)
        except ValueError:
            match = re.match(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",
                             reply.strip())
            if match:
                return float(match.group())
            raise ValueError(f"{cmd} answered {reply!r}, which is not a number")

    def query_int(self, cmd):
        return int(self.query_float(cmd))

    # ======================================================================
    # Identity & housekeeping
    # ======================================================================
    def idn(self):              return self.query("*IDN?")
    def model(self):            return self.query("EQUIPment?")
    def serial(self):           return self.query("SN?")
    def firmware(self):         return self.query("VER?")
    def calibration_date(self): return self.query("CALdate?")
    def uptime(self):           return self.query("TIME?")          # D:HH:MM:SS.ss
    def stopwatch(self):        return self.query("TIMER?")         # since last call
    def reset(self):            return self.command("*RST")         # factory defaults, output OFF
    def clear_status(self):     return self.command("*CLS")
    def opc(self):              return self.query("*OPC?")
    def local(self):            return self.command("LOCAL")        # give the front panel back
    def beep(self, mode=2):     return self.command(f"BEEP {int(mode)}")   # 0 off, 1 on, 2 one beep
    def brightness(self, pct):  return self.command(f"BRIGHT {int(pct)}")
    def get_brightness(self):   return self.query_int("BRIGHT?")
    def message(self, text=""): return self.command(f"MESsage {text}".rstrip())  # 32 chars on screen
    def get_message(self):      return self.query("MESsage?")
    def display(self, on):      return self.command(f"TEC:DISplay {1 if on else 0}")
    def get_display(self):      return self.query("TEC:DISplay?") == "1"
    def power_button(self, on): return self.command(f"PWR {1 if on else 0}")
    def show_remote_errors(self, on): return self.command(f"REMERR {1 if on else 0}")
    def delay(self, ms):        return self.command(f"DELAY {int(ms)}")   # 1..30000, blocks the parser

    def errors(self):
        """Drain the error queue -> '0' when clean, else e.g. '201,"Out of range"'."""
        return self.query("ERRSTR?")

    def error_codes(self):
        """Same queue, numeric codes only: '0' or '201,124'."""
        return self.query("ERRors?")

    # ======================================================================
    # The temperature loop -- the everyday methods
    # ======================================================================
    def temperature(self):      return self.query_float("TEC:ACT?")     # actual, active units
    def get_setpoint(self):     return self.query_float("TEC:SET?")
    def set_setpoint(self, degrees): return self.command(f"TEC:SET {degrees}")   # active units
    def current(self):          return self.query_float("TEC:I?")       # TEC current, A (signed)
    def voltage(self):          return self.query_float("TEC:V?")       # TEC voltage, V
    def aux_temperature(self):  return self.query_float("TEC:AUX?")     # 2nd sensor (heatsink)

    def output(self, on):
        """Enable/disable TEC current. NOTHING heats or cools until this is on.
        Overridden by the rear-panel Remote Enable input (DB-9 pin 1)."""
        return self.command(f"TEC:OUTput {1 if on else 0}")

    def output_enabled(self):   return self.query("TEC:OUTput?") == "1"

    def set_units(self, units):
        """Active temperature units. Accepts a code (0/1/2/3) or a letter
        (C/K/F/raw) and always sends the CODE, which is what the command
        reference documents. Reads them back, so `units` stays correct without
        status() spending a round trip on it."""
        code = UNIT_CODES.get(str(units).strip().upper()[:1], units)
        self.command(f"TEC:UNITS {code}")
        return self.get_units()

    def get_units(self):
        """Active units as 'C'/'K'/'F'/'raw'.

        The reply is a code on some firmware and a word ('CELSIUS') on others,
        so both are accepted. This is a LABEL for the status topic -- never let
        it decide whether the instrument is usable.
        """
        reply = self.query("TEC:UNITS?").strip().upper()
        self.units = (UNITS.get(int(reply), "?") if reply.isdigit()
                      else UNIT_NAMES.get(reply[:1], "?"))
        return self.units

    def set_tolerance(self, deg=0.05, seconds=1.0):
        """In-tolerance window: within +/-deg for `seconds` sets condition bit 9."""
        return self.command(f"TEC:TOLerance {deg},{seconds}")

    def get_tolerance(self):    return self.query("TEC:TOLerance?")   # "deg,seconds"
    def in_tolerance(self):     return bool(self.query_int("TEC:COND?") & (1 << 9))

    def set_cable_resistance(self, ohms): return self.command(f"TEC:CABLER {ohms}")  # 0..10
    def get_cable_resistance(self):       return self.query_float("TEC:CABLER?")

    # Step / increment -- legacy but handy for slow manual ramps.
    def set_step(self, hundredths):  return self.command(f"TEC:STEP {int(hundredths)}")  # 1 == 0.01 C
    def get_step(self):              return self.query_int("TEC:STEP?")
    def step_up(self, steps=1, pause_ms=0):
        return self.command(f"TEC:INC {int(steps)},{int(pause_ms)}")
    def step_down(self, steps=1, pause_ms=0):
        return self.command(f"TEC:DEC {int(steps)},{int(pause_ms)}")

    # ======================================================================
    # Sensor selection & bias
    # ======================================================================
    def set_sensor(self, name):
        """Pick the feedback sensor by NAME. Factory names (manual p.89):
        TCS605-10/-100 (5k), TCS610-10/-100 (10k, the default), TCS620-*,
        TCS650-*, TCS651-* (100k), 'RTD 100 DIN', 'RTD 1k DIN', '*IR SENSOR',
        LM335, AD590 -- or any profile you made with add_thermistor() etc.
        The -10/-100 suffix is the bias current the calibration was taken at."""
        return self.command(f"TEC:SENSOR {name}")

    def get_sensor(self):       return self.query("TEC:SENSOR?")       # calibration coefficients
    def list_sensors(self):     return self.query("TEC:SENSORLIST?").split(",")
    def delete_sensor(self, name): return self.command(f"TEC:SENSORDEL {name}")

    def set_bias(self, code):
        """Sensor bias current: 0 auto (default), 1 10uA, 2 100uA, 3 1mA, 4 10mA.
        Any non-zero value DISABLES auto-ranging."""
        return self.command(f"TEC:BIAS {int(code)}")

    def get_bias(self):         return self.query("TEC:BIAS?")         # "AUTO,1" / "MAN,1"
    def set_aux_bias(self, code): return self.command(f"TEC:AUX:BIAS {int(code)}")
    def get_aux_bias(self):     return self.query("TEC:AUX:BIAS?")

    # ======================================================================
    # Custom sensor calibration (CONST:*) -- name is max 15 chars, no commas.
    # A sensor cannot be edited once made: delete_sensor() then re-create.
    # ======================================================================
    def add_thermistor(self, name, a, b, c):
        """Steinhart-Hart: 1/T = A + B*ln(R) + C*ln(R)^3 (from the datasheet).
        Append '.F1'/'.F2'/'.F3'/'.F4' to `name` to pin the bias current to
        10uA/100uA/1mA/10mA instead of auto-ranging.
        e.g. add_thermistor('Therm10k', 1.1279e-03, 2.3429e-04, 8.7298e-08)"""
        return self.command(f"CONST:THERM {name},{a},{b},{c}")

    def add_thermistor_points(self, name, t1, r1, t2, r2, t3, r3):
        """Same, from three (temperature C, resistance ohm) pairs -- the
        instrument fits Steinhart-Hart for you."""
        return self.command(f"CONST:THERM {name},{t1},{r1},{t2},{r2},{t3},{r3}")

    def add_rtd(self, name, standard="D", r0=100, wires=4):
        """Callendar-Van Dusen RTD. standard: 'D' DIN 43760, 'A' American,
        'I' ITS-90. r0 = resistance at 0 C. wires: 3 or 4."""
        return self.command(f"CONST:RTD{int(wires)} {name},{standard},{r0}")

    def add_rtd_linear(self, name, t1, r1, t2, r2, wires=4):
        """Linear RTD fit from two (temperature C, resistance ohm) pairs."""
        return self.command(f"CONST:RTD{int(wires)} {name},L,{t1},{r1},{t2},{r2}")

    def add_voltage_sensor(self, name, t1, v1, t2, v2):
        """LM335 / other constant-voltage sensor from two (C, V) pairs."""
        return self.command(f"CONST:ICV {name},{t1},{v1},{t2},{v2}")

    def add_optical_sensor(self, name, slope, offset):
        """Infrared optical sensor: slope V/K, offset."""
        return self.command(f"CONST:OPT {name},{slope},{offset}")

    def delete_custom_sensor(self, name): return self.command(f"CONST:DEL {name}")
    def list_custom_sensors(self):        return self.query("CONST:LIST?").split(",")

    # ======================================================================
    # PID & IntelliTune
    # ======================================================================
    def set_pid(self, p, i=None, d=None):
        """P 0.1..1000 (default 12), I 0..200 (0.1), D OFF or 1..100 (0).
        Pass p only, p+i, or all three -- the instrument reads them in order."""
        parts = [str(p)] + ([str(i)] if i is not None else []) + \
                ([str(d)] if d is not None else [])
        return self.command("TEC:PID " + ",".join(parts))

    def get_pid(self):          return self.query("TEC:PID?")          # "p,i,d"

    def set_autotune(self, mode):
        """IntelliTune method: 0 manual, 1 disturbance rejection, 2 setpoint response."""
        return self.command(f"TEC:AUTOTUNE {int(mode)}")

    def get_autotune(self):     return self.query_int("TEC:AUTOTUNE?")

    def tune_start(self):
        """Run IntelliTune in the configured mode. Preconditions (manual p.92):
        output OFF, temperature units (not RAW), setpoint at least 5 C off
        ambient. Current limits drop to 10% for the duration. Takes minutes."""
        return self.command("TEC:TUNESTART")

    def tune_abort(self):       return self.command("TEC:TUNEABORT")   # reverts to old PID
    def tune_valid(self):       return self.query("TEC:VALID?") == "1"

    # ======================================================================
    # Safety limits -- set these BEFORE enabling output
    # ======================================================================
    def set_current_limits(self, positive, negative):
        """Amps, both given POSITIVE. One of them 0 == resistive-heater mode."""
        self.command(f"TEC:LIMit:IPOS {positive}")
        return self.command(f"TEC:LIMit:INEG {negative}")

    def get_current_limits(self):
        return (self.query_float("TEC:LIMit:IPOS?"), self.query_float("TEC:LIMit:INEG?"))

    def set_temperature_limits(self, low, high):
        """Active units, -99..250 C. Exceeding one can trip the LD Shutdown BNC."""
        self.command(f"TEC:LIMit:TLO {low}")
        return self.command(f"TEC:LIMit:THI {high}")

    def get_temperature_limits(self):
        return (self.query_float("TEC:LIMit:TLO?"), self.query_float("TEC:LIMit:THI?"))

    def set_sensor_limits(self, low, high):
        """In the sensor's PHYSICAL units (ohms for a thermistor/RTD, volts for
        LM335/AD590). Note a thermistor's resistance falls as temperature rises,
        so `low` resistance == high temperature."""
        self.command(f"TEC:LIMit:RLO {low}")
        return self.command(f"TEC:LIMit:RHI {high}")

    def get_sensor_limits(self):
        return (self.query_float("TEC:LIMit:RLO?"), self.query_float("TEC:LIMit:RHI?"))

    def set_voltage_limit(self, volts):
        """Internal supply compliance. TC10 LAB rev A-C: 9..18 V, rev D: 10..27 V.
        IntelliTune sets this itself -- keep its value for best settling."""
        return self.command(f"TEC:VLIM {volts}")

    def get_voltage_limit(self): return self.query_float("TEC:VLIM?")

    def set_remote_enable_polarity(self, level):
        """Rear DB-9 pin 1 gating. 1 (default) = +5 V enables, 0 = 0 V enables."""
        return self.command(f"TEC:INTPOL {int(level)}")

    def get_remote_enable(self): return self.query("TEC:INTSTAT?")

    def set_shutdown_polarity(self, polarity):
        """LD Shutdown BNC TTL level: 0 = 5 V on fault, 1 = 0 V on fault."""
        return self.command(f"TEC:LDSHUTdown:POL {int(polarity)}")

    def get_shutdown_polarity(self): return self.query_int("TEC:LDSHUTdown:POL?")

    # ======================================================================
    # Status registers
    # ======================================================================
    def condition(self):        return self.query_int("TEC:COND?")     # live state, see CONDITION_BITS
    def event(self):            return self.query_int("TEC:EVEnt?")    # latched changes; READING CLEARS IT
    def status_byte(self):      return self.query_int("*STB?")
    def enable_condition(self, mask): return self.command(f"TEC:ENABle:COND {int(mask)}")
    def enable_event(self, mask):     return self.command(f"TEC:ENABle:EVEnt {int(mask)}")

    def faults(self):
        """Decoded TEC:COND? -> ['sensor_open', 'temp_high_limit', ...]. Empty == healthy.
        Open/short circuits are TRANSIENT: the output trips off and the bit clears,
        so a fault seen on the front panel may already be gone here -- event()
        latches those."""
        cond = self.query_int("TEC:COND?")
        return [CONDITION_BITS[b] for b in FAULT_BITS if cond & (1 << b)]

    def status(self):
        """Everything the status topic needs, in FIVE round trips. This runs on
        the node's poll timer, so every query added here is one more chance per
        second for the instrument to be mid-reply when the next one arrives.
        `units` is the cached value from set_units()/get_units()."""
        cond = self.query_int("TEC:COND?")
        return {
            "temperature": self.query_float("TEC:ACT?"),
            "setpoint": self.query_float("TEC:SET?"),
            "current": self.query_float("TEC:I?"),
            "voltage": self.query_float("TEC:V?"),
            "units": self.units,
            "output": bool(cond & (1 << 10)),
            "in_tolerance": bool(cond & (1 << 9)),
            "condition": cond,
            "faults": [CONDITION_BITS[b] for b in FAULT_BITS if cond & (1 << b)],
        }

    # ======================================================================
    # Stored profiles (1..10; 0 is the read-only factory profile)
    # ======================================================================
    def save_profile(self, n):    return self.command(f"*SAV {int(n)}")
    def recall_profile(self, n):  return self.command(f"*RCL {int(n)}")   # output shuts off
    def name_profile(self, n, line1="", line2=""):
        return self.command(f"PROFile:DESC {int(n)},{line1},{line2}")
    def get_profile_name(self, n):     return self.query(f"PROFile:DESC? {int(n)}")
    def profile_setpoint(self, n, c):  return self.command(f"PROFile:SET {int(n)},{c}")
    def get_profile_setpoint(self, n): return self.query_float(f"PROFile:SET? {int(n)}")
    def profile_pid(self, n, p, i, d): return self.command(f"PROFile:PID {int(n)},{p},{i},{d}")
    def get_profile_pid(self, n):      return self.query(f"PROFile:PID? {int(n)}")
    def profile_sensor(self, n, name): return self.command(f"PROFile:SENsor {int(n)},{name}")
    def profile_units(self, n, u):     return self.command(f"PROFile:UNITS {int(n)},{u}")
    def profile_tolerance(self, n, deg, seconds):
        return self.command(f"PROFile:TOLerance {int(n)},{deg},{seconds}")
    def profile_current_limits(self, n, positive, negative):
        self.command(f"PROFile:IPOS {int(n)},{positive}")
        return self.command(f"PROFile:INEG {int(n)},{negative}")
    def profile_temperature_limits(self, n, low, high):
        self.command(f"PROFile:TLO {int(n)},{low}")
        return self.command(f"PROFile:THI {int(n)},{high}")

    # ======================================================================
    # Instrument-side scans and scripts
    # ======================================================================
    def profile_scan(self, n, start, stop, step, wait_s):
        """Configure the front-panel temperature scan stored in profile n.
        wait_s = 0 means 'wait until in tolerance' instead of a fixed dwell.
        Scans cannot run in RAW units."""
        self.command(f"PROFile:SCANSTART {int(n)},{start}")
        self.command(f"PROFile:SCANSTOP {int(n)},{stop}")
        self.command(f"PROFile:SCANSTEP {int(n)},{step}")
        return self.command(f"PROFile:SCANWAIT {int(n)},{wait_s}")

    def put_script(self, index, script):
        """Store a script (index 1..4, max 200 chars). Commands are separated by
        CARATS, not semicolons, and the command path is not repeated:
            put_script(1, 'TEC:SET 25^LIMit:IPOS 3.2^OUT 1')"""
        return self.command(f"SCRIPT:PUT {int(index)},{script}")

    def run_script(self, index):  return self.command(f"SCRIPT:GO {int(index)}")
    def get_script(self, index):  return self.query(f"SCRIPT:GET? {int(index)}")

    # ======================================================================
    # Network settings (take effect after a rear-panel power cycle)
    # ======================================================================
    def get_ip(self):           return self.query("TECH:IPADDR?")
    def set_ip(self, addr):     return self.command(f"TECH:IPADDR {addr}")
    def get_netmask(self):      return self.query("TECH:IPMASK?")
    def set_netmask(self, m):   return self.command(f"TECH:IPMASK {m}")
    def get_gateway(self):      return self.query("TECH:IPGW?")
    def set_gateway(self, g):   return self.command(f"TECH:IPGW {g}")
    def get_mac(self):          return self.query("TECH:HWADDR?")


# ================================================================================
# Bench smoke test:  python3 TC10LAB.py  [resource]
# ================================================================================
if __name__ == "__main__":
    import sys

    tc = TC10LAB(sys.argv[1] if len(sys.argv) > 1 else "")
    tc._open()
    print("resource :", tc.resource)
    print("idn      :", tc.idn())
    print("sensor   :", tc.get_sensor())
    print("units    :", tc.get_units())
    print("status   :", tc.status())
    print("errors   :", tc.errors())
    tc.local()
    tc._close()
