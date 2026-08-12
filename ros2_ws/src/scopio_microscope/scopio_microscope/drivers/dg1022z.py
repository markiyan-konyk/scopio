"""Rigol DG1022Z arbitrary-waveform generator. CH1 = X mirror, CH2 = Y mirror.

Does not open on construction -- galvo_node calls _open(). Every public method
is reachable over the awg/call service; private (underscore) ones are not.

ALL instrument I/O goes through command()/query(), which hold _lock. galvo_node
serves awg/* on a reentrant callback group, so two clients can be inside this
driver at the same time, and two threads interleaved on one USB-TMC session
produce garbled replies and timeouts, not an error you can trace.

The first block of methods drives the galvo mirrors and keeps its own
xpos/ypos/offset bookkeeping. Everything after it is the plain instrument, one
method per thing the front panel can do.
"""

import threading
import time

import pyvisa

RIGOL_VID = 0x1AB1

# Every waveform name :SOURce<n>:FUNCtion accepts. Kept here so a UI can fill a
# picker from shapes() instead of hard-coding the list on the client side.
SHAPES = (
    "SIN", "SQU", "RAMP", "PULS", "NOIS", "USER", "HARM", "DC",
    "KAISER", "ROUNDPM", "SINC", "NEGRAMP", "ATTALT", "AMPALT", "STAIRDN",
    "STAIRUP", "STAIRUD", "CPULSE", "PPULSE", "NPULSE", "TRAPEZIA",
    "ROUNDHALF", "ABSSINE", "ABSSINEHALF", "SINETRA", "SINEVER", "EXPRISE",
    "EXPFALL", "TAN", "COT", "SQRT", "X2DATA", "GAUSS", "HAVERSINE",
    "LORENTZ", "DIRICHLET", "GAUSSPULSE", "AIRY", "CARDIAC", "QUAKE", "GAMMA",
    "VOICE", "TV", "COMBIN", "BANDLIMITED", "STEPRESP", "BUTTERWORTH",
    "CHEBYSHEV1", "CHEBYSHEV2", "BOXCAR", "BARLETT", "TRIANG", "BLACKMAN",
    "HAMMING", "HANNING", "DUALTONE", "ACOS", "ACOSH", "ACOTCON", "ACOTPRO",
    "ACOTHCON", "ACOTHPRO", "ACSCCON", "ACSCPRO", "ACSCHCON", "ACSCHPRO",
    "ASECCON", "ASECPRO", "ASECH", "ASIN", "ASINH", "ATAN", "ATANH",
    "BESSELJ", "BESSELY", "CAUCHY", "COSH", "COSINT", "COTHCON", "COTHPRO",
    "CSCCON", "CSCPRO", "CSCHCON", "CSCHPRO", "CUBIC", "ERF", "ERFC",
    "ERFCINV", "ERFINV", "LAGUERRE", "LAPLACE", "LEGEND", "LOG", "LOGNORMAL",
    "MAXWELL", "RAYLEIGH", "RECIPCON", "RECIPPRO", "SECCON", "SECPRO",
    "SECH", "SINH", "SININT", "TANH", "VERSIERA", "WEIBULL", "BARTHANN",
    "BLACKMANH", "BOHMANWIN", "CHEBWIN", "FLATTOPWIN", "NUTTALLWIN",
    "PARZENWIN", "TAYLORWIN", "TUKEYWIN", "CWPUSLE", "LFPULSE", "LFMPULSE",
    "EOG", "EEG", "EMG", "PULSILOGRAM", "TENS1", "TENS2", "TENS3", "SURGE",
    "DAMPEDOSC", "SWINGOSC", "RADAR", "THREEAM", "THREEFM", "THREEPM",
    "THREEPWM", "THREEPFM", "RESSPEED", "MCNOSIE", "PAHCUR", "RIPPLE",
    "ISO76372TP1", "ISO76372TP2A", "ISO76372TP2B", "ISO76372TP3A",
    "ISO76372TP3B", "ISO76372TP4", "ISO76372TP5A", "ISO76372TP5B",
    "ISO167502SP", "ISO167502VR", "SCR", "IGNITION", "NIMHDISCHARGE",
    "GATEVIBR",
)

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


class DG1022Z:
    def __init__(self, resource="", timeout_ms=5000):
        self.resource = resource
        self.timeout_ms = timeout_ms
        self._lock = threading.RLock()   # VISA sessions are NOT thread-safe
        self.rm = None
        self.device = None

        # Galvo bookkeeping, in volts. pos is the commanded deflection, offset
        # the per-axis trim that centres the mirror -- a real rig needs it.
        self.xpos = 0.0
        self.ypos = 0.0
        self.xoffset = 0.0
        self.yoffset = 0.0
        self.freq = 10.0
        self.amp = 0.0
        self.phase = 0.0

    def _open(self):
        """Open the VISA session. With no resource, take the first RIGOL on USB
        -- matched by vendor id in the resource string, so another vendor's
        instrument (the temperature controller) is never opened just to ask
        what it is."""
        self.rm = pyvisa.ResourceManager("@py")
        if not self.resource:
            usb = [r for r in self.rm.list_resources("USB?*INSTR")
                   if usb_vid(r) == RIGOL_VID]
            if not usb:
                raise RuntimeError("no Rigol AWG on USB; set GALVO_RESOURCE for an "
                                   "Ethernet unit (pyvisa-py cannot scan the LAN)")
            self.resource = usb[0]
        self.device = self.rm.open_resource(self.resource)
        self.device.read_termination = "\n"
        self.device.write_termination = "\n"
        self.device.timeout = self.timeout_ms

    def _close(self):
        """Outputs off, then drop the session. Safe to call twice; a dead link
        must not stop the session being released."""
        with self._lock:
            if self.device is not None:
                try:
                    self.command(":OUTP1 OFF;:OUTP2 OFF")
                except Exception:
                    pass
                self.device.close()
            if self.rm is not None:
                self.rm.close()
            self.device = self.rm = None

    def _drop(self):
        """Release the session WITHOUT sending SCPI -- for when the link is
        already gone and a write would only burn a full timeout."""
        with self._lock:
            for handle in (self.device, self.rm):
                try:
                    if handle is not None:
                        handle.close()
                except Exception:
                    pass
            self.device = self.rm = None

    # ------------------------------------------------------------------ #
    #  The only two methods that touch the wire. Everything else uses them.
    # ------------------------------------------------------------------ #
    def command(self, cmd):
        """Write a raw SCPI command."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("AWG session is closed")
            self.device.write(cmd)

    def query(self, cmd):
        """Write a raw SCPI query and return the reply, stripped."""
        with self._lock:
            if self.device is None:
                raise ConnectionError("AWG session is closed")
            return self.device.query(cmd).strip()

    # ------------------------------------------------------------------ #
    #  The galvo mirrors
    # ------------------------------------------------------------------ #
    def dcinit(self):
        """Park both mirrors: HighZ load, DC at the calibrated offsets, outputs on."""
        self.command(":OUTPut1:LOAD INFinity;:OUTPut2:LOAD INFinity")
        self.command(f":SOURce1:APPLy:DC 1,1,{self.xoffset:.3f}")
        self.command(f":SOURce2:APPLy:DC 1,1,{self.yoffset:.3f}")
        self.command(":OUTP1 ON;:OUTP2 ON")
        self.xpos = self.ypos = 0.0      # what we just wrote IS the zero position

    def offsets(self, x=None, y=None):
        """The per-axis trim that centres each mirror, in volts. Read it with no
        arguments. Every update()/move()/sininit() position is relative to it."""
        if x is None and y is None:
            return {"x": self.xoffset, "y": self.yoffset}
        if x is not None:
            self.xoffset = float(x)
        if y is not None:
            self.yoffset = float(y)

    def update(self, ch: int, val: float):
        """Jump a mirror to a position in volts, on top of its calibrated offset."""
        val = float(val)
        if ch == 1:
            self.xpos, out = val, val + self.xoffset
        elif ch == 2:
            self.ypos, out = val, val + self.yoffset
        else:
            raise ValueError(f"channel must be 1 (X) or 2 (Y), got {ch}")
        self.command(f":SOURce{ch}:VOLTage:OFFSet {out:.3f}")

    def position(self):
        """Where the mirrors were last commanded to, in volts."""
        return {"x": self.xpos, "y": self.ypos}

    def move(self, ch: int, endval: float, t: float = 1.0, steps: int = 60):
        """Ramp a mirror to a position over t seconds instead of jumping there."""
        start = self.xpos if ch == 1 else self.ypos
        steps = max(1, int(steps))
        dwell = max(0.0, float(t)) / steps
        for i in range(1, steps + 1):
            self.update(ch, start + (float(endval) - start) * i / steps)
            time.sleep(dwell)

    def sininit(self, freq=None, amp=None, phase=None):
        """Start both mirrors scanning: a sine on each channel about its current
        position. Omitted arguments keep the value from the last sin* call."""
        self._remember(freq, amp, phase)
        if self.amp <= 0:
            raise ValueError("sine amplitude is 0 Vpp -- call with amp=<Vpp>")
        self.command(f":SOURce1:APPLy:SINusoid {self.freq},{self.amp},"
                     f"{self.xoffset + self.xpos:.3f},{self.phase}")
        self.command(f":SOURce2:APPLy:SINusoid {self.freq},{self.amp},"
                     f"{self.yoffset + self.ypos:.3f},{self.phase}")

    def sinupdate(self, ch: int, freq=None, amp=None, phase=None):
        """Change one scanning mirror's sine frequency, amplitude or phase.
        Omitted arguments keep the value from the last sin* call."""
        if ch not in (1, 2):
            raise ValueError(f"channel must be 1 (X) or 2 (Y), got {ch}")
        self._remember(freq, amp, phase)
        self.command(f":SOURce{ch}:FREQuency {self.freq}")
        self.command(f":SOURce{ch}:PHASe {self.phase}")
        self.command(f":SOURce{ch}:VOLTage {self.amp}")

    def _remember(self, freq, amp, phase):
        if freq is not None:
            self.freq = float(freq)
        if amp is not None:
            self.amp = float(amp)
        if phase is not None:
            self.phase = float(phase)

    # ================================================================== #
    #  The rest of the instrument -- one method per thing the front panel
    #  can do, so a UI never has to compose SCPI.
    #
    #  Convention: a scalar setting is ONE method that both sets and
    #  reads. Pass the value to set it, leave it out to read it back:
    #      frequency(1, 1000)  -> sets CH1 to 1 kHz
    #      frequency(1)        -> returns 1000.0
    #  Grouped readers (waveform, am_config, snapshot, ...) return a dict
    #  so a UI panel fills itself in one round trip instead of twenty.
    # ================================================================== #

    # ------------------------------------------------------------------ #
    #  Identity, reset, errors
    # ------------------------------------------------------------------ #
    def idn(self):
        """Manufacturer, model, serial number and firmware version."""
        return self.query("*IDN?").strip()

    def reset(self):
        """Restore the factory state (*RST). Turns both outputs off."""
        self.command("*RST")

    def clear_status(self):
        """Clear the event registers and the error queue (*CLS)."""
        self.command("*CLS")

    def wait(self):
        """Block until every queued command has finished; True when done."""
        return self.query("*OPC?").strip() == "1"

    def error(self):
        """Oldest entry in the error queue, e.g. '0,"No error"'."""
        return self.query(":SYSTem:ERRor?").strip()

    def errors(self, limit:int=20):
        """Drain the error queue into a list; empty when nothing is wrong."""
        out = []
        for _ in range(limit):
            entry = self.query(":SYSTem:ERRor?").strip()
            if entry.startswith("0,"):
                break
            out.append(entry)
        return out

    def scpi_version(self):
        """SCPI version the instrument implements, e.g. '1999.0'."""
        return self.query(":SYSTem:VERSion?").strip()

    def channel_count(self):
        """Number of output channels the instrument has."""
        return int(float(self.query(":SYSTem:CHANnel:NUMber?")))

    def shapes(self):
        """Every waveform name shape() accepts - fills a UI picker."""
        return list(SHAPES)

    # ------------------------------------------------------------------ #
    #  Output connector
    # ------------------------------------------------------------------ #
    def output(self, ch:int=1, on:bool=None):
        """Turn a channel's output connector on/off, or read whether it is on."""
        if on is None:
            return self.query(f":OUTPut{ch}:STATe?").strip().upper() == "ON"
        self.command(f":OUTPut{ch}:STATe {'ON' if on else 'OFF'}")

    def load(self, ch:int=1, ohms=None):
        """Output load, 1 to 10000 ohms or 'INF' for HighZ; reads back None for HighZ."""
        if ohms is None:
            value = float(self.query(f":OUTPut{ch}:LOAD?"))
            return None if value > 1e30 else value      # HighZ answers 9.9E+37
        self.command(f":OUTPut{ch}:LOAD {ohms}")

    def polarity(self, ch:int=1, inverted:bool=None):
        """Invert a channel's output about its offset, or read the setting."""
        if inverted is None:
            return self.query(f":OUTPut{ch}:POLarity?").strip().upper() == "INVERTED"
        self.command(f":OUTPut{ch}:POLarity {'INVerted' if inverted else 'NORMal'}")

    def output_mode(self, ch:int=1, gated:bool=None):
        """Gate the output from the rear Mod/Trig connector, or read the mode."""
        if gated is None:
            return self.query(f":OUTPut{ch}:MODE?").strip().upper() == "GATED"
        self.command(f":OUTPut{ch}:MODE {'GATed' if gated else 'NORMal'}")

    def gate_polarity(self, ch:int=1, negative:bool=None):
        """Whether the gate signal is active low rather than active high."""
        if negative is None:
            return self.query(f":OUTPut{ch}:GATe:POLarity?").strip().upper() == "NEGATIVE"
        self.command(f":OUTPut{ch}:GATe:POLarity {'NEGative' if negative else 'POSitive'}")

    def sync(self, ch:int=1, on:bool=None):
        """Turn the rear-panel sync output on/off, or read whether it is on."""
        if on is None:
            return self.query(f":OUTPut{ch}:SYNC:STATe?").strip().upper() == "ON"
        self.command(f":OUTPut{ch}:SYNC:STATe {'ON' if on else 'OFF'}")

    def sync_polarity(self, ch:int=1, negative:bool=None):
        """Whether the sync signal is inverted."""
        if negative is None:
            return self.query(f":OUTPut{ch}:SYNC:POLarity?").strip().upper() == "NEG"
        self.command(f":OUTPut{ch}:SYNC:POLarity {'NEGative' if negative else 'POSitive'}")

    def sync_delay(self, ch:int=1, seconds:float=None):
        """Delay of the sync pulse relative to the output, 0 s to one carrier period."""
        if seconds is None:
            return float(self.query(f":OUTPut{ch}:SYNC:DELay?"))
        self.command(f":OUTPut{ch}:SYNC:DELay {seconds}")

    # ------------------------------------------------------------------ #
    #  Pick a waveform. One call sets shape and all four parameters.
    # ------------------------------------------------------------------ #
    def sine(self, ch:int=1, freq:float=1000, amp:float=5, offset:float=0, phase:float=0):
        """Output a sine wave (Hz, Vpp, VDC, degrees)."""
        self.command(f":SOURce{ch}:APPLy:SINusoid {freq},{amp},{offset},{phase}")

    def square(self, ch:int=1, freq:float=1000, amp:float=5, offset:float=0, phase:float=0):
        """Output a square wave; set the mark-space ratio with duty()."""
        self.command(f":SOURce{ch}:APPLy:SQUare {freq},{amp},{offset},{phase}")

    def ramp(self, ch:int=1, freq:float=1000, amp:float=5, offset:float=0, phase:float=0):
        """Output a ramp; set the rise/fall balance with symmetry()."""
        self.command(f":SOURce{ch}:APPLy:RAMP {freq},{amp},{offset},{phase}")

    def triangle(self, ch:int=1, freq:float=1000, amp:float=5, offset:float=0, phase:float=0):
        """Output a triangle wave (a ramp at 100% symmetry)."""
        self.command(f":SOURce{ch}:APPLy:TRIangle {freq},{amp},{offset},{phase}")

    def pulse(self, ch:int=1, freq:float=1000, amp:float=5, offset:float=0, phase:float=0):
        """Output a pulse train; shape it with pulse_width()/pulse_duty()/pulse_edge()."""
        self.command(f":SOURce{ch}:APPLy:PULSe {freq},{amp},{offset},{phase}")

    def noise(self, ch:int=1, amp:float=5, offset:float=0):
        """Output broadband noise (no frequency or phase to set)."""
        self.command(f":SOURce{ch}:APPLy:NOISe {amp},{offset}")

    def dc(self, ch:int=1, offset:float=0):
        """Output a steady DC level in volts."""
        self.command(f":SOURce{ch}:APPLy:DC 1,1,{offset}")

    def user(self, ch:int=1, freq:float=1000, amp:float=5, offset:float=0, phase:float=0):
        """Output the selected arbitrary waveform, clocked by frequency."""
        self.command(f":SOURce{ch}:APPLy:USER {freq},{amp},{offset},{phase}")

    def arbitrary(self, ch:int=1, rate:float=20e6, amp:float=5, offset:float=0):
        """Output the selected arbitrary waveform, clocked point-by-point by sample rate."""
        self.command(f":SOURce{ch}:APPLy:ARBitrary {rate},{amp},{offset}")

    def shape(self, ch:int=1, name:str=None):
        """Waveform type by name - any entry from shapes(), e.g. 'SINC'."""
        if name is None:
            return self.query(f":SOURce{ch}:FUNCtion?").strip()
        self.command(f":SOURce{ch}:FUNCtion {name}")

    def waveform(self, ch:int=1):
        """Shape plus frequency, amplitude, offset and phase in one query."""
        parts = self.query(f":SOURce{ch}:APPLy?").strip().strip('"').split(",")
        # Parameters a shape does not have (noise has no frequency) answer 'DEF'.
        values = [None if p.strip().upper() == "DEF" else float(p) for p in parts[1:]]
        values += [None] * (4 - len(values))
        return {"shape": parts[0], "frequency": values[0], "amplitude": values[1],
                "offset": values[2], "phase": values[3]}

    # ------------------------------------------------------------------ #
    #  Individual parameters - what a UI's sliders and number boxes bind to
    # ------------------------------------------------------------------ #
    def frequency(self, ch:int=1, hz:float=None):
        """Waveform frequency in Hz."""
        if hz is None:
            return float(self.query(f":SOURce{ch}:FREQuency?"))
        self.command(f":SOURce{ch}:FREQuency {hz}")

    def period(self, ch:int=1, seconds:float=None):
        """Waveform period in seconds (the reciprocal of frequency())."""
        if seconds is None:
            return float(self.query(f":SOURce{ch}:PERiod?"))
        self.command(f":SOURce{ch}:PERiod {seconds}")

    def amplitude(self, ch:int=1, vpp:float=None):
        """Waveform amplitude, in whatever amplitude_unit() is set to."""
        if vpp is None:
            return float(self.query(f":SOURce{ch}:VOLTage?"))
        self.command(f":SOURce{ch}:VOLTage {vpp}")

    def offset(self, ch:int=1, volts:float=None):
        """DC offset in volts. Raw - unlike update(), it ignores the galvo offsets."""
        if volts is None:
            return float(self.query(f":SOURce{ch}:VOLTage:OFFSet?"))
        self.command(f":SOURce{ch}:VOLTage:OFFSet {volts}")

    def high_level(self, ch:int=1, volts:float=None):
        """Top of the waveform in volts (amplitude and offset follow)."""
        if volts is None:
            return float(self.query(f":SOURce{ch}:VOLTage:HIGH?"))
        self.command(f":SOURce{ch}:VOLTage:HIGH {volts}")

    def low_level(self, ch:int=1, volts:float=None):
        """Bottom of the waveform in volts (amplitude and offset follow)."""
        if volts is None:
            return float(self.query(f":SOURce{ch}:VOLTage:LOW?"))
        self.command(f":SOURce{ch}:VOLTage:LOW {volts}")

    def amplitude_unit(self, ch:int=1, unit:str=None):
        """Unit amplitudes are given in: 'VPP', 'VRMS' or 'DBM'."""
        if unit is None:
            return self.query(f":SOURce{ch}:VOLTage:UNIT?").strip()
        self.command(f":SOURce{ch}:VOLTage:UNIT {unit}")

    def autorange(self, ch:int=1, on:bool=None):
        """Automatic attenuator selection. Off avoids amplitude glitches on range changes."""
        if on is None:
            return self.query(f":SOURce{ch}:VOLTage:RANGe:AUTO?").strip().upper() == "ON"
        self.command(f":SOURce{ch}:VOLTage:RANGe:AUTO {'ON' if on else 'OFF'}")

    def start_phase(self, ch:int=1, degrees:float=None):
        """Start phase of the waveform, 0 to 360 degrees."""
        if degrees is None:
            return float(self.query(f":SOURce{ch}:PHASe?"))
        self.command(f":SOURce{ch}:PHASe {degrees}")

    def align_phase(self, ch:int=1):
        """Re-sync both channels so their phase difference is the one you set."""
        self.command(f":SOURce{ch}:PHASe:INITiate")

    def duty(self, ch:int=1, percent:float=None):
        """Square-wave duty cycle in percent."""
        if percent is None:
            return float(self.query(f":SOURce{ch}:FUNCtion:SQUare:DCYCle?"))
        self.command(f":SOURce{ch}:FUNCtion:SQUare:DCYCle {percent}")

    def symmetry(self, ch:int=1, percent:float=None):
        """Ramp symmetry in percent - 100 is a rising sawtooth, 0 a falling one."""
        if percent is None:
            return float(self.query(f":SOURce{ch}:FUNCtion:RAMP:SYMMetry?"))
        self.command(f":SOURce{ch}:FUNCtion:RAMP:SYMMetry {percent}")

    def pulse_width(self, ch:int=1, seconds:float=None):
        """Pulse width in seconds, measured between the 50% points."""
        if seconds is None:
            return float(self.query(f":SOURce{ch}:FUNCtion:PULSe:WIDTh?"))
        self.command(f":SOURce{ch}:FUNCtion:PULSe:WIDTh {seconds}")

    def pulse_duty(self, ch:int=1, percent:float=None):
        """Pulse duty cycle in percent (an alternative to pulse_width())."""
        if percent is None:
            return float(self.query(f":SOURce{ch}:FUNCtion:PULSe:DCYCle?"))
        self.command(f":SOURce{ch}:FUNCtion:PULSe:DCYCle {percent}")

    def pulse_edge(self, ch:int=1, seconds:float=None, edge:str="both"):
        """Pulse rise/fall time in seconds; edge is 'both', 'lead' or 'trail'."""
        key = {"both": "TRANsition", "lead": "TRANsition:LEADing",
               "trail": "TRANsition:TRAiling"}[edge]
        if seconds is None:
            if edge == "both":
                key = "TRANsition:LEADing"      # :BOTH is write-only
            return float(self.query(f":SOURce{ch}:FUNCtion:PULSe:{key}?"))
        self.command(f":SOURce{ch}:FUNCtion:PULSe:{key} {seconds}")

    def pulse_hold(self, ch:int=1, keep:str=None):
        """Which of 'WIDTh' or 'DCYCle' stays fixed when the pulse period changes."""
        if keep is None:
            return self.query(f":SOURce{ch}:FUNCtion:PULSe:HOLD?").strip()
        self.command(f":SOURce{ch}:FUNCtion:PULSe:HOLD {keep}")

    # ------------------------------------------------------------------ #
    #  Arbitrary waveforms
    # ------------------------------------------------------------------ #
    def upload(self, ch:int=1, points=(), rate:float=None):
        """Send 8-16384 normalised points (-1..1) to volatile memory and play them.

        This is how a UI ships a waveform it drew. The instrument switches to
        arbitrary output on its own. A full 16k upload is a long single write -
        open the driver with a generous timeout_ms if you send them often.
        """
        if not 8 <= len(points) <= 16384:
            raise ValueError(f"need between 8 and 16384 points, got {len(points)}")
        data = ",".join(f"{float(p):.5f}" for p in points)
        self.command(f":SOURce{ch}:DATA VOLATILE,{data}")
        if rate is not None:
            self.command(f":SOURce{ch}:FUNCtion:ARBitrary:SRATe {rate}")

    def upload_dac(self, ch:int=1, values=()):
        """Send 8-16384 raw DAC codes (0..16383) to volatile memory and play them."""
        if not 8 <= len(values) <= 16384:
            raise ValueError(f"need between 8 and 16384 points, got {len(values)}")
        data = ",".join(str(int(v)) for v in values)
        self.command(f":SOURce{ch}:DATA:DAC VOLATILE,{data}")

    def arb_mode(self, ch:int=1, mode:str=None):
        """How the arbitrary waveform is clocked: 'FREQ' or 'SRATe'."""
        if mode is None:
            return self.query(f":SOURce{ch}:FUNCtion:ARBitrary:MODE?").strip()
        self.command(f":SOURce{ch}:FUNCtion:ARBitrary:MODE {mode}")

    def sample_rate(self, ch:int=1, samples_per_s:float=None):
        """Arbitrary-waveform sample rate, 1 uSa/s to 60 MSa/s."""
        if samples_per_s is None:
            return float(self.query(f":SOURce{ch}:FUNCtion:ARBitrary:SRATe?"))
        self.command(f":SOURce{ch}:FUNCtion:ARBitrary:SRATe {samples_per_s}")

    def arb_points(self, ch:int=1, count:int=None):
        """Length of the volatile waveform; setting it zeroes every point."""
        if count is None:
            return int(float(self.query(f":SOURce{ch}:DATA:POINts? VOLATILE")))
        self.command(f":SOURce{ch}:DATA:POINts VOLATILE,{count}")

    def arb_point(self, ch:int=1, index:int=1, value:int=None):
        """One point of the volatile waveform as a DAC code (0..16383)."""
        if value is None:
            return int(float(self.query(f":SOURce{ch}:DATA:VALue? VOLATILE,{index}")))
        self.command(f":SOURce{ch}:DATA:VALue VOLATILE,{index},{value}")

    def arb_catalog(self, ch:int=1):
        """Names of the arbitrary waveform files stored in the instrument."""
        reply = self.query(f":SOURce{ch}:DATA:CATalog?").strip()
        return [name.strip().strip('"') for name in reply.split(",") if name.strip('" ')]

    def arb_load(self, ch:int=1, name:str=""):
        """Copy a stored arbitrary waveform file into volatile memory and play it."""
        self.command(f":SOURce{ch}:DATA:COPY {name},VOLATILE")

    def arb_delete(self, ch:int=1, name:str=""):
        """Delete a stored arbitrary waveform file (fails if it is locked)."""
        self.command(f":SOURce{ch}:DATA:DELete {name}")

    def arb_lock(self, ch:int=1, name:str="", on:bool=None):
        """Lock a stored arbitrary waveform against deletion, or read its lock."""
        if on is None:
            return self.query(f":SOURce{ch}:DATA:LOCK? {name}").strip().upper() == "ON"
        self.command(f":SOURce{ch}:DATA:LOCK {name},{'ON' if on else 'OFF'}")

    # ------------------------------------------------------------------ #
    #  Modulation. Each type is one method: pass only the fields you want
    #  to change, and on=True to switch that modulation on.
    # ------------------------------------------------------------------ #
    def modulation(self, ch:int=1, on:bool=None):
        """Master modulation switch for a channel."""
        if on is None:
            return self.query(f":SOURce{ch}:MOD:STATe?").strip().upper() == "ON"
        self.command(f":SOURce{ch}:MOD:STATe {'ON' if on else 'OFF'}")

    def modulation_type(self, ch:int=1, kind:str=None):
        """Which modulation is active: AM, FM, PM, ASK, FSK, PSK or PWM."""
        if kind is None:
            return self.query(f":SOURce{ch}:MOD:TYPe?").strip()
        self.command(f":SOURce{ch}:MOD:TYPe {kind}")

    def am(self, ch:int=1, depth:float=None, freq:float=None, shape:str=None,
           source:str=None, dssc:bool=None, on:bool=None):
        """Amplitude modulation: depth %, modulating frequency Hz and shape."""
        if depth is not None:
            self.command(f":SOURce{ch}:AM:DEPTh {depth}")
        if freq is not None:
            self.command(f":SOURce{ch}:AM:INTernal:FREQuency {freq}")
        if shape is not None:
            self.command(f":SOURce{ch}:AM:INTernal:FUNCtion {shape}")
        if source is not None:
            self.command(f":SOURce{ch}:AM:SOURce {source}")
        if dssc is not None:
            self.command(f":SOURce{ch}:AM:DSSC {'ON' if dssc else 'OFF'}")
        if on is not None:
            self.command(f":SOURce{ch}:AM:STATe {'ON' if on else 'OFF'}")

    def am_config(self, ch:int=1):
        """Read back every AM setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:AM:STATe?").strip().upper() == "ON",
                "depth": float(self.query(f":SOURce{ch}:AM:DEPTh?")),
                "frequency": float(self.query(f":SOURce{ch}:AM:INTernal:FREQuency?")),
                "shape": self.query(f":SOURce{ch}:AM:INTernal:FUNCtion?").strip(),
                "source": self.query(f":SOURce{ch}:AM:SOURce?").strip(),
                "dssc": self.query(f":SOURce{ch}:AM:DSSC?").strip().upper() == "ON"}

    def fm(self, ch:int=1, deviation:float=None, freq:float=None, shape:str=None,
           source:str=None, on:bool=None):
        """Frequency modulation: deviation Hz, modulating frequency Hz and shape."""
        if deviation is not None:
            self.command(f":SOURce{ch}:FM:DEViation {deviation}")
        if freq is not None:
            self.command(f":SOURce{ch}:FM:INTernal:FREQuency {freq}")
        if shape is not None:
            self.command(f":SOURce{ch}:FM:INTernal:FUNCtion {shape}")
        if source is not None:
            self.command(f":SOURce{ch}:FM:SOURce {source}")
        if on is not None:
            self.command(f":SOURce{ch}:FM:STATe {'ON' if on else 'OFF'}")

    def fm_config(self, ch:int=1):
        """Read back every FM setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:FM:STATe?").strip().upper() == "ON",
                "deviation": float(self.query(f":SOURce{ch}:FM:DEViation?")),
                "frequency": float(self.query(f":SOURce{ch}:FM:INTernal:FREQuency?")),
                "shape": self.query(f":SOURce{ch}:FM:INTernal:FUNCtion?").strip(),
                "source": self.query(f":SOURce{ch}:FM:SOURce?").strip()}

    def pm(self, ch:int=1, deviation:float=None, freq:float=None, shape:str=None,
           source:str=None, on:bool=None):
        """Phase modulation: deviation in degrees, modulating frequency Hz and shape."""
        if deviation is not None:
            self.command(f":SOURce{ch}:PM:DEViation {deviation}")
        if freq is not None:
            self.command(f":SOURce{ch}:PM:INTernal:FREQuency {freq}")
        if shape is not None:
            self.command(f":SOURce{ch}:PM:INTernal:FUNCtion {shape}")
        if source is not None:
            self.command(f":SOURce{ch}:PM:SOURce {source}")
        if on is not None:
            self.command(f":SOURce{ch}:PM:STATe {'ON' if on else 'OFF'}")

    def pm_config(self, ch:int=1):
        """Read back every PM setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:PM:STATe?").strip().upper() == "ON",
                "deviation": float(self.query(f":SOURce{ch}:PM:DEViation?")),
                "frequency": float(self.query(f":SOURce{ch}:PM:INTernal:FREQuency?")),
                "shape": self.query(f":SOURce{ch}:PM:INTernal:FUNCtion?").strip(),
                "source": self.query(f":SOURce{ch}:PM:SOURce?").strip()}

    def ask(self, ch:int=1, amp:float=None, rate:float=None, polarity:str=None,
            source:str=None, on:bool=None):
        """Amplitude-shift keying: the second amplitude Vpp and the hop rate Hz."""
        if amp is not None:
            self.command(f":SOURce{ch}:ASKey:AMPLitude {amp}")
        if rate is not None:
            self.command(f":SOURce{ch}:ASKey:INTernal:RATE {rate}")
        if polarity is not None:
            self.command(f":SOURce{ch}:ASKey:POLarity {polarity}")
        if source is not None:
            self.command(f":SOURce{ch}:ASKey:SOURce {source}")
        if on is not None:
            self.command(f":SOURce{ch}:ASKey:STATe {'ON' if on else 'OFF'}")

    def ask_config(self, ch:int=1):
        """Read back every ASK setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:ASKey:STATe?").strip().upper() == "ON",
                "amplitude": float(self.query(f":SOURce{ch}:ASKey:AMPLitude?")),
                "rate": float(self.query(f":SOURce{ch}:ASKey:INTernal:RATE?")),
                "polarity": self.query(f":SOURce{ch}:ASKey:POLarity?").strip(),
                "source": self.query(f":SOURce{ch}:ASKey:SOURce?").strip()}

    def fsk(self, ch:int=1, hop:float=None, rate:float=None, polarity:str=None,
            source:str=None, on:bool=None):
        """Frequency-shift keying: the hop frequency Hz and the hop rate Hz."""
        if hop is not None:
            self.command(f":SOURce{ch}:FSKey:FREQuency {hop}")
        if rate is not None:
            self.command(f":SOURce{ch}:FSKey:INTernal:RATE {rate}")
        if polarity is not None:
            self.command(f":SOURce{ch}:FSKey:POLarity {polarity}")
        if source is not None:
            self.command(f":SOURce{ch}:FSKey:SOURce {source}")
        if on is not None:
            self.command(f":SOURce{ch}:FSKey:STATe {'ON' if on else 'OFF'}")

    def fsk_config(self, ch:int=1):
        """Read back every FSK setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:FSKey:STATe?").strip().upper() == "ON",
                "hop": float(self.query(f":SOURce{ch}:FSKey:FREQuency?")),
                "rate": float(self.query(f":SOURce{ch}:FSKey:INTernal:RATE?")),
                "polarity": self.query(f":SOURce{ch}:FSKey:POLarity?").strip(),
                "source": self.query(f":SOURce{ch}:FSKey:SOURce?").strip()}

    def psk(self, ch:int=1, phase:float=None, rate:float=None, polarity:str=None,
            source:str=None, on:bool=None):
        """Phase-shift keying: the second phase in degrees and the hop rate Hz."""
        if phase is not None:
            self.command(f":SOURce{ch}:PSKey:PHASe {phase}")
        if rate is not None:
            self.command(f":SOURce{ch}:PSKey:INTernal:RATE {rate}")
        if polarity is not None:
            self.command(f":SOURce{ch}:PSKey:POLarity {polarity}")
        if source is not None:
            self.command(f":SOURce{ch}:PSKey:SOURce {source}")
        if on is not None:
            self.command(f":SOURce{ch}:PSKey:STATe {'ON' if on else 'OFF'}")

    def psk_config(self, ch:int=1):
        """Read back every PSK setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:PSKey:STATe?").strip().upper() == "ON",
                "phase": float(self.query(f":SOURce{ch}:PSKey:PHASe?")),
                "rate": float(self.query(f":SOURce{ch}:PSKey:INTernal:RATE?")),
                "polarity": self.query(f":SOURce{ch}:PSKey:POLarity?").strip(),
                "source": self.query(f":SOURce{ch}:PSKey:SOURce?").strip()}

    def pwm(self, ch:int=1, width:float=None, duty:float=None, freq:float=None,
            shape:str=None, source:str=None, on:bool=None):
        """Pulse-width modulation of a pulse carrier: width s or duty %, plus rate Hz."""
        if width is not None:
            self.command(f":SOURce{ch}:PWM:DEViation:WIDTh {width}")
        if duty is not None:
            self.command(f":SOURce{ch}:PWM:DEViation:DCYCle {duty}")
        if freq is not None:
            self.command(f":SOURce{ch}:PWM:INTernal:FREQuency {freq}")
        if shape is not None:
            self.command(f":SOURce{ch}:PWM:INTernal:FUNCtion {shape}")
        if source is not None:
            self.command(f":SOURce{ch}:PWM:SOURce {source}")
        if on is not None:
            self.command(f":SOURce{ch}:PWM:STATe {'ON' if on else 'OFF'}")

    def pwm_config(self, ch:int=1):
        """Read back every PWM setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:PWM:STATe?").strip().upper() == "ON",
                "width": float(self.query(f":SOURce{ch}:PWM:DEViation:WIDTh?")),
                "duty": float(self.query(f":SOURce{ch}:PWM:DEViation:DCYCle?")),
                "frequency": float(self.query(f":SOURce{ch}:PWM:INTernal:FREQuency?")),
                "shape": self.query(f":SOURce{ch}:PWM:INTernal:FUNCtion?").strip(),
                "source": self.query(f":SOURce{ch}:PWM:SOURce?").strip()}

    # ------------------------------------------------------------------ #
    #  Sweep
    # ------------------------------------------------------------------ #
    def sweep(self, ch:int=1, on:bool=None):
        """Sweep mode on/off for a channel."""
        if on is None:
            return self.query(f":SOURce{ch}:SWEep:STATe?").strip().upper() == "ON"
        self.command(f":SOURce{ch}:SWEep:STATe {'ON' if on else 'OFF'}")

    def sweep_setup(self, ch:int=1, start:float=None, stop:float=None, time:float=None,
                    spacing:str=None, steps:int=None, return_time:float=None,
                    start_hold:float=None, stop_hold:float=None, on:bool=None):
        """Configure a frequency sweep: start/stop Hz, duration s, LIN/LOG/STE spacing."""
        if start is not None:
            self.command(f":SOURce{ch}:FREQuency:STARt {start}")
        if stop is not None:
            self.command(f":SOURce{ch}:FREQuency:STOP {stop}")
        if time is not None:
            self.command(f":SOURce{ch}:SWEep:TIME {time}")
        if spacing is not None:
            self.command(f":SOURce{ch}:SWEep:SPACing {spacing}")
        if steps is not None:
            self.command(f":SOURce{ch}:SWEep:STEP {steps}")
        if return_time is not None:
            self.command(f":SOURce{ch}:SWEep:RTIMe {return_time}")
        if start_hold is not None:
            self.command(f":SOURce{ch}:SWEep:HTIMe:STARt {start_hold}")
        if stop_hold is not None:
            self.command(f":SOURce{ch}:SWEep:HTIMe:STOP {stop_hold}")
        if on is not None:
            self.command(f":SOURce{ch}:SWEep:STATe {'ON' if on else 'OFF'}")

    def sweep_config(self, ch:int=1):
        """Read back every sweep setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:SWEep:STATe?").strip().upper() == "ON",
                "start": float(self.query(f":SOURce{ch}:FREQuency:STARt?")),
                "stop": float(self.query(f":SOURce{ch}:FREQuency:STOP?")),
                "center": float(self.query(f":SOURce{ch}:FREQuency:CENTer?")),
                "span": float(self.query(f":SOURce{ch}:FREQuency:SPAN?")),
                "time": float(self.query(f":SOURce{ch}:SWEep:TIME?")),
                "spacing": self.query(f":SOURce{ch}:SWEep:SPACing?").strip(),
                "steps": int(float(self.query(f":SOURce{ch}:SWEep:STEP?"))),
                "return_time": float(self.query(f":SOURce{ch}:SWEep:RTIMe?")),
                "start_hold": float(self.query(f":SOURce{ch}:SWEep:HTIMe:STARt?")),
                "stop_hold": float(self.query(f":SOURce{ch}:SWEep:HTIMe:STOP?")),
                "trigger": self.query(f":SOURce{ch}:SWEep:TRIGger:SOURce?").strip()}

    def sweep_span(self, ch:int=1, center:float=None, span:float=None):
        """Set the sweep by centre and span in Hz instead of start and stop."""
        if center is None and span is None:
            return {"center": float(self.query(f":SOURce{ch}:FREQuency:CENTer?")),
                    "span": float(self.query(f":SOURce{ch}:FREQuency:SPAN?"))}
        if center is not None:
            self.command(f":SOURce{ch}:FREQuency:CENTer {center}")
        if span is not None:
            self.command(f":SOURce{ch}:FREQuency:SPAN {span}")

    def sweep_trigger(self, ch:int=1, source:str=None, slope:str=None, trigout:str=None):
        """Sweep trigger: source INT/EXT/MAN, input slope POS/NEG, output POS/NEG/OFF."""
        if source is None and slope is None and trigout is None:
            return {"source": self.query(f":SOURce{ch}:SWEep:TRIGger:SOURce?").strip(),
                    "slope": self.query(f":SOURce{ch}:SWEep:TRIGger:SLOPe?").strip(),
                    "trigout": self.query(f":SOURce{ch}:SWEep:TRIGger:TRIGOut?").strip()}
        if source is not None:
            self.command(f":SOURce{ch}:SWEep:TRIGger:SOURce {source}")
        if slope is not None:
            self.command(f":SOURce{ch}:SWEep:TRIGger:SLOPe {slope}")
        if trigout is not None:
            self.command(f":SOURce{ch}:SWEep:TRIGger:TRIGOut {trigout}")

    def sweep_now(self, ch:int=1):
        """Fire one sweep immediately (manual trigger source only)."""
        self.command(f":SOURce{ch}:SWEep:TRIGger:IMMediate")

    def mark(self, ch:int=1, freq:float=None, on:bool=None):
        """Frequency at which the sync output drops during a sweep."""
        if freq is None and on is None:
            return {"on": self.query(f":SOURce{ch}:MARKer:STATe?").strip().upper() == "ON",
                    "frequency": float(self.query(f":SOURce{ch}:MARKer:FREQuency?"))}
        if freq is not None:
            self.command(f":SOURce{ch}:MARKer:FREQuency {freq}")
        if on is not None:
            self.command(f":SOURce{ch}:MARKer:STATe {'ON' if on else 'OFF'}")

    # ------------------------------------------------------------------ #
    #  Burst
    # ------------------------------------------------------------------ #
    def burst(self, ch:int=1, on:bool=None):
        """Burst mode on/off for a channel."""
        if on is None:
            return self.query(f":SOURce{ch}:BURSt:STATe?").strip().upper() == "ON"
        self.command(f":SOURce{ch}:BURSt:STATe {'ON' if on else 'OFF'}")

    def burst_setup(self, ch:int=1, cycles:int=None, period:float=None, mode:str=None,
                    phase:float=None, delay:float=None, idle:str=None, on:bool=None):
        """Configure a burst: cycles per burst, burst period s, TRIG/INF/GAT mode.

        Set the burst parameters BEFORE switching it on, so the output does not
        run through a string of intermediate configurations.
        """
        if mode is not None:
            self.command(f":SOURce{ch}:BURSt:MODE {mode}")
        if cycles is not None:
            self.command(f":SOURce{ch}:BURSt:NCYCles {cycles}")
        if period is not None:
            self.command(f":SOURce{ch}:BURSt:INTernal:PERiod {period}")
        if phase is not None:
            self.command(f":SOURce{ch}:BURSt:PHASe {phase}")
        if delay is not None:
            self.command(f":SOURce{ch}:BURSt:TDELay {delay}")
        if idle is not None:
            self.command(f":SOURce{ch}:BURSt:IDLE {idle}")
        if on is not None:
            self.command(f":SOURce{ch}:BURSt:STATe {'ON' if on else 'OFF'}")

    def burst_config(self, ch:int=1):
        """Read back every burst setting of a channel."""
        return {"on": self.query(f":SOURce{ch}:BURSt:STATe?").strip().upper() == "ON",
                "mode": self.query(f":SOURce{ch}:BURSt:MODE?").strip(),
                "cycles": int(float(self.query(f":SOURce{ch}:BURSt:NCYCles?"))),
                "period": float(self.query(f":SOURce{ch}:BURSt:INTernal:PERiod?")),
                "phase": float(self.query(f":SOURce{ch}:BURSt:PHASe?")),
                "delay": float(self.query(f":SOURce{ch}:BURSt:TDELay?")),
                "idle": self.query(f":SOURce{ch}:BURSt:IDLE?").strip(),
                "trigger": self.query(f":SOURce{ch}:BURSt:TRIGger:SOURce?").strip()}

    def burst_trigger(self, ch:int=1, source:str=None, slope:str=None, trigout:str=None,
                      gate_polarity:str=None):
        """Burst trigger: source INT/EXT/MAN, input slope, output edge, gate polarity."""
        if source is None and slope is None and trigout is None and gate_polarity is None:
            return {"source": self.query(f":SOURce{ch}:BURSt:TRIGger:SOURce?").strip(),
                    "slope": self.query(f":SOURce{ch}:BURSt:TRIGger:SLOPe?").strip(),
                    "trigout": self.query(f":SOURce{ch}:BURSt:TRIGger:TRIGOut?").strip(),
                    "gate_polarity": self.query(f":SOURce{ch}:BURSt:GATE:POLarity?").strip()}
        if source is not None:
            self.command(f":SOURce{ch}:BURSt:TRIGger:SOURce {source}")
        if slope is not None:
            self.command(f":SOURce{ch}:BURSt:TRIGger:SLOPe {slope}")
        if trigout is not None:
            self.command(f":SOURce{ch}:BURSt:TRIGger:TRIGOut {trigout}")
        if gate_polarity is not None:
            self.command(f":SOURce{ch}:BURSt:GATE:POLarity {gate_polarity}")

    def burst_now(self, ch:int=1):
        """Fire one burst immediately (manual trigger source only)."""
        self.command(f":SOURce{ch}:BURSt:TRIGger:IMMediate")

    def trigger(self, ch:int=1):
        """Trigger whichever of sweep or burst is armed on this channel."""
        self.command(f":TRIGger{ch}:IMMediate")

    # ------------------------------------------------------------------ #
    #  Harmonics and waveform summing
    # ------------------------------------------------------------------ #
    def harmonic(self, ch:int=1, order:int=None, kind:str=None, user:str=None, on:bool=None):
        """Harmonic generator: highest order 2-8 and type EVEN/ODD/ALL/USER."""
        if order is not None:
            self.command(f":SOURce{ch}:HARMonic:ORDEr {order}")
        if kind is not None:
            self.command(f":SOURce{ch}:HARMonic:TYPe {kind}")
        if user is not None:
            self.command(f":SOURce{ch}:HARMonic:USER {user}")
        if on is not None:
            self.command(f":SOURce{ch}:HARMonic:STATe {'ON' if on else 'OFF'}")

    def harmonic_amplitude(self, ch:int=1, order:int=2, vpp:float=None):
        """Amplitude of one harmonic (order 2 to 8) in Vpp."""
        if vpp is None:
            return float(self.query(f":SOURce{ch}:HARMonic:AMPL? {order}"))
        self.command(f":SOURce{ch}:HARMonic:AMPL {order},{vpp}")

    def harmonic_phase(self, ch:int=1, order:int=2, degrees:float=None):
        """Phase of one harmonic (order 2 to 8) in degrees."""
        if degrees is None:
            return float(self.query(f":SOURce{ch}:HARMonic:PHASe? {order}"))
        self.command(f":SOURce{ch}:HARMonic:PHASe {order},{degrees}")

    def harmonic_config(self, ch:int=1):
        """Read back the harmonic settings, including each order's amplitude and phase."""
        order = int(float(self.query(f":SOURce{ch}:HARMonic:ORDEr?")))
        return {"on": self.query(f":SOURce{ch}:HARMonic:STATe?").strip().upper() == "ON",
                "order": order,
                "type": self.query(f":SOURce{ch}:HARMonic:TYPe?").strip(),
                "user": self.query(f":SOURce{ch}:HARMonic:USER?").strip(),
                "amplitudes": {n: float(self.query(f":SOURce{ch}:HARMonic:AMPL? {n}"))
                               for n in range(2, order + 1)},
                "phases": {n: float(self.query(f":SOURce{ch}:HARMonic:PHASe? {n}"))
                           for n in range(2, order + 1)}}

    def wave_sum(self, ch:int=1, ratio:float=None, freq:float=None, shape:str=None,
                 on:bool=None):
        """Add a second waveform on top of the carrier: ratio %, frequency Hz, shape."""
        if ratio is not None:
            self.command(f":SOURce{ch}:SUM:AMPLitude {ratio}")
        if freq is not None:
            self.command(f":SOURce{ch}:SUM:INTernal:FREQuency {freq}")
        if shape is not None:
            self.command(f":SOURce{ch}:SUM:INTernal:FUNCtion {shape}")
        if on is not None:
            self.command(f":SOURce{ch}:SUM:STATe {'ON' if on else 'OFF'}")

    def wave_sum_config(self, ch:int=1):
        """Read back the waveform-summing settings of a channel."""
        return {"on": self.query(f":SOURce{ch}:SUM:STATe?").strip().upper() == "ON",
                "ratio": float(self.query(f":SOURce{ch}:SUM:AMPLitude?")),
                "frequency": float(self.query(f":SOURce{ch}:SUM:INTernal:FREQuency?")),
                "shape": self.query(f":SOURce{ch}:SUM:INTernal:FUNCtion?").strip()}

    # ------------------------------------------------------------------ #
    #  Frequency counter (measures a signal fed into the CH2 connector -
    #  switching it on disables the CH2 sync output)
    # ------------------------------------------------------------------ #
    def counter(self, on:bool=None):
        """Turn the frequency counter on/off, or read its running state."""
        if on is None:
            return self.query(":COUNter:STATe?").strip().upper() != "OFF"
        self.command(f":COUNter:STATe {'ON' if on else 'OFF'}")

    def counter_measure(self):
        """Latest counter reading: frequency, period, duty cycle and both pulse widths."""
        values = [float(v) for v in self.query(":COUNter:MEASure?").strip().split(",")]
        return {"frequency": values[0], "period": values[1], "duty": values[2],
                "positive_width": values[3], "negative_width": values[4]}

    def counter_setup(self, gate:str=None, coupling:str=None, sensitivity:float=None,
                      level:float=None, hf_reject:bool=None):
        """Counter input: gate time USER1-USER6, AC/DC coupling, sensitivity %, level V."""
        if gate is not None:
            self.command(f":COUNter:GATEtime {gate}")
        if coupling is not None:
            self.command(f":COUNter:COUPling {coupling}")
        if sensitivity is not None:
            self.command(f":COUNter:SENSitive {sensitivity}")
        if level is not None:
            self.command(f":COUNter:LEVEl {level}")
        if hf_reject is not None:
            self.command(f":COUNter:HF {'ON' if hf_reject else 'OFF'}")

    def counter_auto(self):
        """Let the counter choose its own gate time from the signal it sees."""
        self.command(":COUNter:AUTO")

    def counter_config(self):
        """Read back every frequency-counter setting."""
        return {"state": self.query(":COUNter:STATe?").strip(),
                "gate": self.query(":COUNter:GATEtime?").strip(),
                "coupling": self.query(":COUNter:COUPling?").strip(),
                "sensitivity": float(self.query(":COUNter:SENSitive?")),
                "level": float(self.query(":COUNter:LEVEl?")),
                "hf_reject": self.query(":COUNter:HF?").strip().upper() == "ON",
                "statistics": self.query(":COUNter:STATIstics:STATe?").strip().upper() == "ON"}

    def counter_statistics(self, on:bool=None, display:str=None):
        """Counter statistics on/off, shown as 'DIGITAL' or 'CURVE'."""
        if on is None and display is None:
            return self.query(":COUNter:STATIstics:STATe?").strip().upper() == "ON"
        if on is not None:
            self.command(f":COUNter:STATIstics:STATe {'ON' if on else 'OFF'}")
        if display is not None:
            self.command(f":COUNter:STATIstics:DISPlay {display}")

    def counter_clear(self):
        """Throw away the accumulated counter statistics."""
        self.command(":COUNter:STATIstics:CLEAr")

    # ------------------------------------------------------------------ #
    #  Two-channel coupling, tracking and copying
    # ------------------------------------------------------------------ #
    def couple(self, on:bool=None):
        """Frequency, phase and amplitude coupling together; reads back all three."""
        if on is None:
            reply = self.query(":COUPling:STATe?").strip()
            return {p.split(":")[0].lower(): p.split(":")[1].upper() == "ON"
                    for p in reply.split(",") if ":" in p}
        self.command(f":COUPling:STATe {'ON' if on else 'OFF'}")

    def couple_frequency(self, on:bool=None, mode:str=None, deviation:float=None,
                         ratio:float=None):
        """Frequency coupling: OFFSet or RATio mode, plus the deviation Hz or ratio.

        Set mode and deviation/ratio BEFORE switching it on - the instrument
        refuses those commands while the coupling is already enabled.
        """
        if on is None and mode is None and deviation is None and ratio is None:
            return {"on": self.query(":COUPling:FREQuency:STATe?").strip().upper() == "ON",
                    "mode": self.query(":COUPling:FREQuency:MODE?").strip(),
                    "deviation": float(self.query(":COUPling:FREQuency:DEViation?")),
                    "ratio": float(self.query(":COUPling:FREQuency:RATio?"))}
        if mode is not None:
            self.command(f":COUPling:FREQuency:MODE {mode}")
        if deviation is not None:
            self.command(f":COUPling:FREQuency:DEViation {deviation}")
        if ratio is not None:
            self.command(f":COUPling:FREQuency:RATio {ratio}")
        if on is not None:
            self.command(f":COUPling:FREQuency:STATe {'ON' if on else 'OFF'}")

    def couple_amplitude(self, on:bool=None, mode:str=None, deviation:float=None,
                         ratio:float=None):
        """Amplitude coupling: OFFSet or RATio mode, plus the deviation Vpp or ratio."""
        if on is None and mode is None and deviation is None and ratio is None:
            return {"on": self.query(":COUPling:AMPL:STATe?").strip().upper() == "ON",
                    "mode": self.query(":COUPling:AMPL:MODE?").strip(),
                    "deviation": float(self.query(":COUPling:AMPL:DEViation?")),
                    "ratio": float(self.query(":COUPling:AMPL:RATio?"))}
        if mode is not None:
            self.command(f":COUPling:AMPL:MODE {mode}")
        if deviation is not None:
            self.command(f":COUPling:AMPL:DEViation {deviation}")
        if ratio is not None:
            self.command(f":COUPling:AMPL:RATio {ratio}")
        if on is not None:
            self.command(f":COUPling:AMPL:STATe {'ON' if on else 'OFF'}")

    def couple_phase(self, on:bool=None, mode:str=None, deviation:float=None,
                     ratio:float=None):
        """Phase coupling: OFFSet or RATio mode, plus the deviation in degrees or ratio."""
        if on is None and mode is None and deviation is None and ratio is None:
            return {"on": self.query(":COUPling:PHASe:STATe?").strip().upper() == "ON",
                    "mode": self.query(":COUPling:PHASe:MODE?").strip(),
                    "deviation": float(self.query(":COUPling:PHASe:DEViation?")),
                    "ratio": float(self.query(":COUPling:PHASe:RATio?"))}
        if mode is not None:
            self.command(f":COUPling:PHASe:MODE {mode}")
        if deviation is not None:
            self.command(f":COUPling:PHASe:DEViation {deviation}")
        if ratio is not None:
            self.command(f":COUPling:PHASe:RATio {ratio}")
        if on is not None:
            self.command(f":COUPling:PHASe:STATe {'ON' if on else 'OFF'}")

    def track(self, mode:str=None):
        """Make CH2 mirror CH1: 'ON', 'OFF' or 'INVerted'."""
        if mode is None:
            return self.query(":SOURce1:TRACK?").strip()
        self.command(f":SOURce1:TRACK {mode}")

    def copy_channel(self, source:int=1, target:int=2):
        """Copy every setting (not the output on/off state) from one channel to the other."""
        self.command(f":SYSTem:CSCopy CH{source},CH{target}")

    # ------------------------------------------------------------------ #
    #  Saved states
    # ------------------------------------------------------------------ #
    def save_state(self, slot:int=1):
        """Save the whole instrument state to internal slot 1-10."""
        self.command(f"*SAV USER{slot}")

    def recall_state(self, slot:int=1):
        """Recall the instrument state stored in internal slot 1-10."""
        self.command(f"*RCL USER{slot}")

    def saved_states(self):
        """Filenames in the ten internal state slots; '' where a slot is empty."""
        reply = self.query(":MEMory:STATe:CATalog?").strip()
        return [name.strip().strip('"') for name in reply.split(",")]

    def delete_state(self, slot:int=1):
        """Delete the state stored in internal slot 1-10 (fails if it is locked)."""
        self.command(f":MEMory:STATe:DELete USER{slot}")

    def lock_state(self, slot:int=1, on:bool=None):
        """Lock a saved state against deletion, or read whether it is locked."""
        if on is None:
            return self.query(f":MEMory:STATe:LOCK? USER{slot}").strip().upper() == "ON"
        self.command(f":MEMory:STATe:LOCK USER{slot},{'ON' if on else 'OFF'}")

    def preset(self, name:str="DEFault"):
        """Restore 'DEFault', or recall a saved state by name ('USER1'...'USER10')."""
        self.command(f":SYSTem:PRESet {name}")

    def power_on_state(self, mode:str=None):
        """What the instrument loads at power-up: 'DEFault' or 'LAST'."""
        if mode is None:
            return self.query(":SYSTem:POWeron?").strip()
        self.command(f":SYSTem:POWeron {mode}")

    # ------------------------------------------------------------------ #
    #  Front panel and system
    # ------------------------------------------------------------------ #
    def display(self, on:bool=None):
        """Turn the screen on/off (it comes back when the instrument leaves remote)."""
        if on is None:
            return self.query(":DISPlay:STATe?").strip().upper() == "ON"
        self.command(f":DISPlay:STATe {'ON' if on else 'OFF'}")

    def brightness(self, percent:int=None):
        """Screen brightness, 1 to 100 percent."""
        if percent is None:
            return float(self.query(":DISPlay:BRIGhtness?"))
        self.command(f":DISPlay:BRIGhtness {percent}")

    def screen_text(self, text:str=None, x:int=2, y:int=2):
        """Write up to 45 characters on the instrument's screen, or read what is there."""
        if text is None:
            return self.query(":DISPlay:TEXT?").strip().strip('"')
        self.command(f':DISPlay:TEXT "{text}",{x},{y}')

    def clear_text(self):
        """Clear text written by screen_text()."""
        self.command(":DISPlay:TEXT:CLEar")

    def beeper(self, on:bool=None):
        """Whether the instrument beeps on errors."""
        if on is None:
            return self.query(":SYSTem:BEEPer:STATe?").strip().upper() == "ON"
        self.command(f":SYSTem:BEEPer:STATe {'ON' if on else 'OFF'}")

    def beep(self):
        """Beep once, whether or not the beeper is enabled - handy to find the box."""
        self.command(":SYSTem:BEEPer:IMMediate")

    def key_lock(self, key:str="ALL", on:bool=None):
        """Lock a front-panel key ('ALL' for the whole panel) so nobody can fight the UI."""
        if on is None:
            return self.query(f":SYSTem:KLOCk? {key}").strip() == "1"
        self.command(f":SYSTem:KLOCk {key},{'ON' if on else 'OFF'}")

    def clock_source(self, source:str=None):
        """Reference clock: 'INTernal', or 'EXTernal' for the rear 10 MHz input."""
        if source is None:
            return self.query(":SYSTem:ROSCillator:SOURce?").strip()
        self.command(f":SYSTem:ROSCillator:SOURce {source}")

    def current_channel(self, ch:int=None):
        """Which channel the front panel shows; purely cosmetic over remote."""
        if ch is None:
            return self.query(":SYSTem:CHANnel:CURrent?").strip()
        self.command(f":SYSTem:CHANnel:CURrent CH{ch}")

    # ------------------------------------------------------------------ #
    #  One call that fills a whole UI
    # ------------------------------------------------------------------ #
    def snapshot(self, ch:int=None):
        """Everything a UI panel needs. Omit `ch` for both channels at once.

        One dispatch round trip instead of twenty, which matters because every
        call crosses the API gateway and the ROS service before it reaches USB.
        """
        if ch is None:
            return {"idn": self.idn(),
                    "channels": [self.snapshot(1), self.snapshot(2)]}
        info = self.waveform(ch)
        info["channel"] = ch
        info["output"] = self.output(ch)
        info["load"] = self.load(ch)
        info["polarity"] = self.polarity(ch)
        info["unit"] = self.amplitude_unit(ch)
        info["sync"] = self.sync(ch)
        info["modulation"] = self.modulation(ch)
        info["modulation_type"] = self.modulation_type(ch)
        info["sweep"] = self.sweep(ch)
        info["burst"] = self.burst(ch)
        return info
