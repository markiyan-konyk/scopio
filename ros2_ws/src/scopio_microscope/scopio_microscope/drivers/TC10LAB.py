"""Wavelength Electronics TC10 LAB temperature controller (command set COMMAND-00400 rev H; see README.md)."""

import glob
import os
import re
import threading
import time

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
# TEC:UNITS? answers a code or a word ('CELSIUS') depending on firmware; TEC:UNITS takes the code.
UNIT_NAMES = {"C": "C", "K": "K", "F": "F", "R": "raw"}
UNIT_CODES = {"C": 0, "K": 1, "F": 2, "R": 3}


def usb_vid(resource):
    """Vendor id from a VISA resource string (decimal or hex); None for non-USB resources."""
    parts = resource.split("::")
    if len(parts) < 2 or not parts[0].upper().startswith("USB"):
        return None
    try:
        return int(parts[1], 0)
    except ValueError:
        return None


def is_tc10(idn):
    return "TC10" in (idn or "").upper() or "WAVELENGTH" in (idn or "").upper()


# --------------------------------------------------------------------------
# Asking the KERNEL what is plugged in (sysfs; standard library only)
# --------------------------------------------------------------------------
SYSFS_USBMISC = "/sys/class/usbmisc"     # where the usbtmcN char devices live
SYSFS_USB = "/sys/bus/usb/devices"


def usbtmc_vid(path):
    """Vendor id behind a /dev/usbtmcN node from sysfs, without writing to it; None if sysfs cannot say."""
    try:
        iface = os.path.realpath(os.path.join(SYSFS_USBMISC,
                                              os.path.basename(path), "device"))
        with open(os.path.join(os.path.dirname(iface), "idVendor"),
                  encoding="ascii") as f:
            return int(f.read().strip(), 16)
    except (OSError, ValueError):
        return None


def usb_present(vid):
    """sysfs names ('1-1.3') of every USB device with this vendor id: is the box on the bus at all?"""
    found = []
    for id_file in glob.glob(os.path.join(SYSFS_USB, "*", "idVendor")):
        try:
            with open(id_file, encoding="ascii") as f:
                if int(f.read().strip(), 16) == vid:
                    found.append(os.path.basename(os.path.dirname(id_file)))
        except (OSError, ValueError):
            continue
    return sorted(found)


class UsbtmcDevice:
    """The kernel's usbtmc char device, with the slice of the pyvisa resource API used here."""

    # The driver reads until this many bytes or end-of-message; too large and every read waits.
    READ_SIZE = 256
    # USBTMC_IOCTL_CLEAR = _IO('[', 2): the USB-TMC CLEAR, flushing both buffers.
    IOCTL_CLEAR = 0x5B02
    # USBTMC_IOCTL_SET_TIMEOUT = _IOW('[', 10, __u32), in milliseconds.
    IOCTL_SET_TIMEOUT = 0x40045B0A

    def __init__(self, path, timeout_ms=None):
        self.path = path
        self._fd = os.open(path, os.O_RDWR)
        if timeout_ms:
            self._set_timeout(int(timeout_ms))

    def _set_timeout(self, ms):
        """Apply the node's timeout_ms (otherwise the kernel's 5 s default applies)."""
        try:
            import fcntl
            import struct
            fcntl.ioctl(self._fd, self.IOCTL_SET_TIMEOUT,
                        struct.pack("I", max(100, ms)))   # kernel minimum: 100
        except (ImportError, OSError):
            pass          # an old kernel without the ioctl keeps its default

    def clear(self):
        import fcntl      # Linux-only; imported here so this module still loads
        fcntl.ioctl(self._fd, self.IOCTL_CLEAR)

    def write(self, cmd):
        os.write(self._fd, (cmd + "\n").encode())

    def query(self, cmd):
        self.write(cmd)
        raw = os.read(self._fd, self.READ_SIZE)
        if len(raw) == self.READ_SIZE:
            # A longer reply would leave its tail queued and desync every later query: fail loudly.
            raise IOError(f"reply to {cmd!r} exceeded {self.READ_SIZE} bytes; "
                          "session would desync")
        return raw.decode(errors="replace")

    def close(self):
        os.close(self._fd)


class TC10LAB:
    # Pacing: at most 50 I/Os per second; the TC10 lags badly past ~60/s.
    MIN_INTERVAL_S = 1.0 / 50

    def __init__(self, resource="", timeout_ms=5000):
        self.resource = resource
        self.timeout_ms = timeout_ms
        self._lock = threading.RLock()
        self._last_io = 0.0     # monotonic end of the previous I/O
        self.rm = None
        self.device = None
        self.units = ""       # cached by set_units()/get_units(); see status()
        self.identity = ""    # *IDN? reply, verified on connect
        self.probe_note = ""  # set when _open had to work around the config
        self._rejected = []   # usbtmc nodes tried and why they were refused
        self._desynced = False  # a query failed; drain before trusting the next

    # ======================================================================
    # Connecting
    # ======================================================================
    def _open(self):
        """Open a session, choosing the transport by kernel ownership: usbtmc if the kernel owns it, else VISA."""
        res = self.resource.strip()
        if res and not res.startswith("/dev/") and not res.upper().startswith("USB"):
            self._open_visa(res)                  # TCPIP::... -- VISA or nothing
        else:
            owned, unknown = self._usbtmc_candidates(res)
            if owned:
                self._open_usbtmc(owned, required=True)
            elif not (unknown and self._open_usbtmc(unknown, required=False)):
                if res.startswith("/dev/"):
                    self.probe_note = (
                        f"{res!r}: no usbtmc node belongs to a TC10 (the kernel "
                        "driver is not bound -- a VISA session detaches it until "
                        "the box is replugged); reaching it over VISA instead")
                self._open_visa(res if res.upper().startswith("USB") else "")
        # CLEAR before *IDN?: a timed-out attempt may have left a reply queued.
        self._resync()
        self._verify()

    def _usbtmc_candidates(self, res):
        """(usbtmc nodes sysfs says are Wavelength, nodes it cannot identify); others (the AWG) are left alone."""
        named = sorted(glob.glob(res)) if res.startswith("/dev/") else []
        nodes = named + [p for p in sorted(glob.glob(USBTMC_GLOB)) if p not in named]
        if res.startswith("/dev/") and not named and nodes:
            self.probe_note = (f"{res!r} matched nothing -- fix it in "
                               f"ros2_ws/.env; probing {nodes} instead")
        owned, unknown = [], []
        for path in nodes:
            vid = usbtmc_vid(path)
            if vid == WAVELENGTH_VID:
                owned.append(path)
            elif vid is None:
                unknown.append(path)
        if owned and res.upper().startswith("USB"):
            self.probe_note = (
                f"{res!r} is a VISA address, but the kernel usbtmc driver owns "
                f"the TC10 ({', '.join(owned)}); using that -- VISA would have to "
                "detach the driver, which hangs on this Pi")
        return owned, unknown

    def _open_usbtmc(self, paths, required):
        """Open the first of `paths` that answers as a TC10; required=True never falls back to VISA."""
        for path in paths:
            try:
                dev = UsbtmcDevice(path, self.timeout_ms)
            except OSError as exc:
                self._rejected.append(f"{path}: {exc}")
                continue
            try:
                idn = self._probe_idn(dev)
                if is_tc10(idn):
                    self.device, self.resource = dev, path
                    return True
                self._rejected.append(f"{path}: not a TC10 ({idn!r})")
            except Exception as exc:
                self._rejected.append(f"{path}: {type(exc).__name__}: {exc}")
            try:
                dev.close()
            except OSError:
                pass
        if required:
            raise RuntimeError(
                "the kernel usbtmc driver owns the TC10 but it did not answer as "
                "one: " + "; ".join(self._rejected) + ". Power-cycle the TC10; if "
                "it persists, `echo '*IDN?' > /dev/usbtmcN && head -c 200 "
                "/dev/usbtmcN` on the host tells the kernel side from this node.")
        return False

    @staticmethod
    def _probe_idn(dev):
        """*IDN? on an uncleared device: CLEAR first, so a stale reply is not read as the identity."""
        idn = ""
        for _ in range(2):
            try:
                dev.clear()
            except Exception:
                pass
            dev.write("*CLS")
            idn = dev.query("*IDN?").strip()
            if is_tc10(idn) or "," in idn:        # an identity, ours or not
                break
        return idn

    def _open_visa(self, res):
        self.rm = pyvisa.ResourceManager("@py")
        if not res:
            # Match the vendor id in the string: never open another instrument to ask what it is.
            listed = list(self.rm.list_resources("USB?*INSTR"))
            usb = [r for r in listed if usb_vid(r) == WAVELENGTH_VID]
            if not usb:
                raise RuntimeError(self._not_found(listed))
            res = usb[0]
        self.device = self.rm.open_resource(res)
        self.resource = res
        self.device.read_termination = "\n"
        self.device.write_termination = "\n"
        self.device.timeout = self.timeout_ms

    def _not_found(self, listed):
        """Why no transport found a TC10, starting from whether it is on the bus at all."""
        bus = usb_present(WAVELENGTH_VID)
        if not bus:
            return ("no TC10 LAB on the USB bus: the kernel reports no 1a45 "
                    "device. Check the cable, the rear power switch and any hub. "
                    "An Ethernet unit must be named: "
                    "TCLAB_RESOURCE=TCPIP::<ip>::INSTR.")
        tried = f" usbtmc nodes tried: {'; '.join(self._rejected)}." if self._rejected else ""
        return (f"a TC10 LAB IS on the USB bus ({', '.join(bus)}), but no "
                "transport reached it. It has no /dev/usbtmc node (the kernel "
                "driver is not bound -- replug to rebind it), and VISA listed "
                f"{len(listed)} USB instrument(s), none of them it: libusb could "
                "not read its descriptors -- device permissions (ros2_ws/udev) "
                "or another process holding it." + tried)

    def _verify(self):
        """The session must answer *IDN? as a TC10, never as another node's instrument."""
        idn = self.query("*IDN?")
        if not is_tc10(idn):
            raise RuntimeError(f"{self.resource} answered *IDN? with {idn!r} -- "
                               "that is not a Wavelength TC10 LAB")
        self.identity = idn

    def _resync(self):
        """Flush queued replies with the USB-TMC CLEAR (never a query drain), then *CLS. Best effort."""
        self._desynced = False
        try:
            self.device.clear()      # USB-TMC CLEAR: flush both buffers
        except Exception:
            self._drain()            # transport without CLEAR: read it away
        try:
            self.command("*CLS")     # then the status + error queues
        except Exception:
            pass

    def _drain(self):
        """Fallback without CLEAR: read, never write, until nothing is queued."""
        read = getattr(self.device, "read", None)
        if read is None:
            return
        old = getattr(self.device, "timeout", None)
        try:
            self.device.timeout = 200
            for _ in range(16):
                read()
        except Exception:
            pass                     # the timeout IS "nothing left"
        finally:
            if old is not None:
                try:
                    self.device.timeout = old
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

    def _pace(self):
        """Wait out MIN_INTERVAL_S since the previous I/O; call with _lock held."""
        wait = self._last_io + self.MIN_INTERVAL_S - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    def command(self, cmd):
        """Write a raw SCPI command."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("TC10 LAB session is closed")
            self._pace()
            try:
                self.device.write(cmd)
            finally:
                self._last_io = time.monotonic()

    def query(self, cmd):
        """Write a raw SCPI query and return the reply, stripped."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("TC10 LAB session is closed")
            if self._desynced:
                self._resync()
            self._pace()
            try:
                return self.device.query(cmd).strip()
            except Exception:
                # Its reply may still arrive: resync before the next query, or every answer is one behind.
                self._desynced = True
                raise
            finally:
                self._last_io = time.monotonic()

    def query_float(self, cmd):
        """Query and return a float, taking the leading number if the firmware decorates the reply."""
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
        """Enable/disable TEC current; nothing heats or cools until on (rear Remote Enable can override)."""
        return self.command(f"TEC:OUTput {1 if on else 0}")

    def output_enabled(self):   return self.query("TEC:OUTput?") == "1"

    def set_units(self, units):
        """Set the active units by code (0-3) or letter (C/K/F/raw); reads them back and returns them."""
        code = UNIT_CODES.get(str(units).strip().upper()[:1], units)
        self.command(f"TEC:UNITS {code}")
        return self.get_units()

    def get_units(self):
        """Active units as 'C'/'K'/'F'/'raw' (a label for the status topic)."""
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
        """Pick the feedback sensor by name, e.g. 'TCS610-10' (default), 'RTD 100 DIN', LM335 (manual p.89)."""
        return self.command(f"TEC:SENSOR {name}")

    def get_sensor(self):       return self.query("TEC:SENSOR?")       # calibration coefficients
    def list_sensors(self):     return self.query("TEC:SENSORLIST?").split(",")
    def delete_sensor(self, name): return self.command(f"TEC:SENSORDEL {name}")

    def set_bias(self, code):
        """Sensor bias: 0 auto (default), 1 10uA, 2 100uA, 3 1mA, 4 10mA; non-zero disables auto-ranging."""
        return self.command(f"TEC:BIAS {int(code)}")

    def get_bias(self):         return self.query("TEC:BIAS?")         # "AUTO,1" / "MAN,1"
    def set_aux_bias(self, code): return self.command(f"TEC:AUX:BIAS {int(code)}")
    def get_aux_bias(self):     return self.query("TEC:AUX:BIAS?")

    # ======================================================================
    # Custom sensors (CONST:*): names max 15 chars, no commas; delete to edit
    # ======================================================================
    def add_thermistor(self, name, a, b, c):
        """Add a Steinhart-Hart thermistor (1/T = A + B ln R + C ln^3 R); a '.F1'-'.F4' name suffix pins the bias."""
        return self.command(f"CONST:THERM {name},{a},{b},{c}")

    def add_thermistor_points(self, name, t1, r1, t2, r2, t3, r3):
        """Add a thermistor from three (temperature C, resistance ohm) pairs; the instrument fits it."""
        return self.command(f"CONST:THERM {name},{t1},{r1},{t2},{r2},{t3},{r3}")

    def add_rtd(self, name, standard="D", r0=100, wires=4):
        """Add a Callendar-Van Dusen RTD: standard 'D' DIN 43760, 'A' American or 'I' ITS-90; r0 at 0 C; 3 or 4 wires."""
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
        """Set PID: P 0.1-1000 (default 12), I 0-200 (0.1), D OFF or 1-100 (0); pass p, p+i, or all three."""
        parts = [str(p)] + ([str(i)] if i is not None else []) + \
                ([str(d)] if d is not None else [])
        return self.command("TEC:PID " + ",".join(parts))

    def get_pid(self):          return self.query("TEC:PID?")          # "p,i,d"

    def set_autotune(self, mode):
        """IntelliTune method: 0 manual, 1 disturbance rejection, 2 setpoint response."""
        return self.command(f"TEC:AUTOTUNE {int(mode)}")

    def get_autotune(self):     return self.query_int("TEC:AUTOTUNE?")

    def tune_start(self):
        """Run IntelliTune (minutes): needs output OFF, real units, setpoint 5 C or more from ambient (manual p.92)."""
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
        """Sensor limits in physical units (ohms or volts); for a thermistor, low resistance = high temperature."""
        self.command(f"TEC:LIMit:RLO {low}")
        return self.command(f"TEC:LIMit:RHI {high}")

    def get_sensor_limits(self):
        return (self.query_float("TEC:LIMit:RLO?"), self.query_float("TEC:LIMit:RHI?"))

    def set_voltage_limit(self, volts):
        """Supply compliance in volts (rev A-C 9-18 V, rev D 10-27 V); IntelliTune sets it itself."""
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
        """Decoded fault names from TEC:COND?, empty when healthy (transient faults clear; event() latches them)."""
        cond = self.query_int("TEC:COND?")
        return [CONDITION_BITS[b] for b in FAULT_BITS if cond & (1 << b)]

    def status(self):
        """Everything the status topic needs, in five queries; polled every second, so keep it lean."""
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
        """Configure the front-panel scan stored in profile n; wait_s = 0 waits for tolerance (not in RAW units)."""
        self.command(f"PROFile:SCANSTART {int(n)},{start}")
        self.command(f"PROFile:SCANSTOP {int(n)},{stop}")
        self.command(f"PROFile:SCANSTEP {int(n)},{step}")
        return self.command(f"PROFile:SCANWAIT {int(n)},{wait_s}")

    def put_script(self, index, script):
        """Store a script (index 1-4, max 200 chars), commands separated by ^, e.g. 'TEC:SET 25^OUT 1'."""
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
