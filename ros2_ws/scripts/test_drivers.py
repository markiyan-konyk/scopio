#!/usr/bin/env python3
"""Offline check of the logic that has no hardware in it.

    python3 scripts/test_drivers.py        # exits non-zero on the first failure

No ROS, no pyvisa session, no instruments -- it drives the drivers against a
fake device that records the SCPI it is given. It exists to catch the class of
bug that used to hide in here: a ramp that silently did nothing, a sentinel that
resolved to an invalid amplitude, a partial request that clobbered every field
it did not mention.
"""

import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "scopio_microscope"))

# The drivers import pyvisa at module scope; stub it so this runs anywhere.
sys.modules.setdefault("pyvisa", types.ModuleType("pyvisa"))

from scopio_microscope.drivers import dispatch                      # noqa: E402
from scopio_microscope.drivers.TC10LAB import (CONDITION_BITS, FAULT_BITS,  # noqa: E402
                                               TC10LAB, usb_vid)
from scopio_microscope.drivers.dg1022z import DG1022Z               # noqa: E402


class FakeDevice:
    """Records writes; answers queries from a canned table."""

    def __init__(self, replies=None):
        self.writes = []
        self.replies = replies or {}

    def write(self, cmd):
        self.writes.append(cmd)

    def query(self, cmd):
        self.writes.append(cmd)
        return self.replies.get(cmd, "0")

    def close(self):
        pass


def awg(**replies):
    gen = DG1022Z("USB0::0x1AB1::0x0642::FAKE::INSTR")
    gen.device = FakeDevice(replies)
    return gen


def offsets(writes, prefix):
    """The offset values written for one channel, in order."""
    return [float(w.rsplit(" ", 1)[1]) for w in writes if w.startswith(prefix)]


# --------------------------------------------------------------- resources
def test_usb_vid():
    assert usb_vid("USB0::0x1AB1::0x0642::DG1ZA::INSTR") == 0x1AB1
    assert usb_vid("USB0::6833::1602::DG1ZA::INSTR") == 6833   # pyvisa-py decimal
    assert usb_vid("TCPIP::192.168.1.50::INSTR") is None
    assert usb_vid("ASRL/dev/ttyACM0::INSTR") is None
    assert usb_vid("USB0") is None                             # too few fields


# ------------------------------------------------------------- galvo moves
def test_update_applies_offset_and_validates_channel():
    gen = awg()
    gen.offsets(x=0.25, y=-0.10)
    gen.update(1, 1.0)
    gen.update(2, 1.0)
    assert gen.position() == {"x": 1.0, "y": 1.0}
    assert gen.device.writes == [":SOURce1:VOLTage:OFFSet 1.250",
                                 ":SOURce2:VOLTage:OFFSet 0.900"]
    for bad in (0, 3, -1):
        try:
            gen.update(bad, 1.0)
        except ValueError:
            continue
        raise AssertionError(f"channel {bad} should be refused")


def test_move_ramps_in_both_directions():
    """The old if/if/else meant an UPWARD move returned without doing anything."""
    for target in (2.0, -2.0):
        gen = awg()
        gen.move(1, target, t=0.0, steps=8)
        written = offsets(gen.device.writes, ":SOURce1")
        assert len(written) == 8, f"{target}: {len(written)} steps, expected 8"
        assert written == sorted(written, reverse=target < 0), \
            f"{target}: not monotonic: {written}"
        assert abs(written[-1] - target) < 1e-9, f"{target}: ended at {written[-1]}"
        assert gen.xpos == target


def test_move_honours_the_offset_and_starting_position():
    gen = awg()
    gen.offsets(x=0.5)
    gen.update(1, 1.0)
    gen.device.writes.clear()
    gen.move(1, 2.0, t=0.0, steps=4)
    # From xpos=1.0 to 2.0 in 4 steps, each written on top of the 0.5 offset.
    assert offsets(gen.device.writes, ":SOURce1") == [1.75, 2.0, 2.25, 2.5]


def test_dcinit_zeroes_the_remembered_position():
    """dcinit writes the offsets, so the position it leaves behind IS zero --
    otherwise the next update() jumps by a stale amount."""
    gen = awg()
    gen.offsets(x=0.3, y=0.4)
    gen.update(1, 1.5)
    gen.dcinit()
    assert gen.position() == {"x": 0.0, "y": 0.0}
    assert ":SOURce1:APPLy:DC 1,1,0.300" in gen.device.writes
    assert ":OUTP1 ON;:OUTP2 ON" in gen.device.writes


def test_sin_sentinels_never_command_zero_amplitude():
    gen = awg()
    try:
        gen.sininit()                    # nothing remembered yet
    except ValueError:
        pass
    else:
        raise AssertionError("sininit with amp 0 must refuse, not emit 0 Vpp SCPI")
    gen.sininit(freq=50, amp=2.0)
    assert gen.device.writes == [":SOURce1:APPLy:SINusoid 50.0,2.0,0.000,0.0",
                                 ":SOURce2:APPLy:SINusoid 50.0,2.0,0.000,0.0"]
    gen.device.writes.clear()
    gen.sinupdate(1, freq=120)           # amp/phase must be REMEMBERED, not zeroed
    assert gen.device.writes == [":SOURce1:FREQuency 120.0",
                                 ":SOURce1:PHASe 0.0",
                                 ":SOURce1:VOLTage 2.0"]
    assert (gen.freq, gen.amp) == (120.0, 2.0)


def test_every_scpi_call_goes_through_the_lock():
    """A method that touches self.device directly would bypass _lock and let two
    service calls interleave on one USB-TMC session."""
    import inspect

    from scopio_microscope.drivers import dg1022z
    source = inspect.getsource(dg1022z)
    allowed = ("self.device.read_termination", "self.device.write_termination",
               "self.device.timeout", "self.device.close()",
               "self.device.write(cmd)", "self.device.query(cmd)")
    for lineno, line in enumerate(source.splitlines(), 1):
        if "self.device." in line and not any(a in line for a in allowed):
            raise AssertionError(f"dg1022z.py:{lineno} bypasses command()/query(): "
                                 f"{line.strip()}")


def test_the_nodes_and_their_drivers_still_agree():
    """THE bug this closes: dg1022z.py was overwritten with a bench copy that
    had no command()/query()/_drop(), so galvo_node raised AttributeError on
    every connect and the AWG read as absent hardware forever. Every test still
    passed, because they all exercised the DRIVER and nothing checked the NODE's
    half of the contract. Import alone cannot catch it -- the calls are inside
    methods that only run against real hardware."""
    import re

    nodes = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "src", "scopio_microscope", "scopio_microscope")
    for module, driver, handle in (("galvo_node", DG1022Z, "gen"),
                                   ("temperature_node", TC10LAB, "tc")):
        with open(os.path.join(nodes, module + ".py"), encoding="utf-8") as f:
            source = f.read()
        called = set(re.findall(rf"\b{handle}\.([A-Za-z_]\w*)\s*\(", source))
        missing = sorted(m for m in called if not hasattr(driver, m))
        assert not missing, (f"{module}.py calls {handle}.{missing} -- "
                             f"{driver.__name__} has no such method")
        assert called, f"found no {handle}.* calls in {module}.py; regex stale?"


def test_closed_session_is_a_clear_error():
    gen = awg()
    gen._drop()
    for call in (lambda: gen.command(":OUTP1 ON"), lambda: gen.query("*IDN?")):
        try:
            call()
        except ConnectionError:
            continue
        raise AssertionError("a closed session must raise, not AttributeError")
    gen._drop()          # idempotent


# --------------------------------------------------------------- TC10 LAB
def test_condition_bits_decode():
    tc = TC10LAB.__new__(TC10LAB)          # no session needed for pure decoding
    for bit in FAULT_BITS:
        assert bit in CONDITION_BITS, f"FAULT_BITS names bit {bit} with no label"
    cond = (1 << 6) | (1 << 10) | (1 << 9)      # sensor_open + output_on + in_tol
    tc.query_int = lambda cmd: cond
    tc.query_float = lambda cmd: 25.0
    tc.units = "C"
    s = tc.status()
    assert s["output"] is True and s["in_tolerance"] is True
    assert s["faults"] == ["sensor_open"], s["faults"]
    assert s["units"] == "C"


def test_units_accept_a_word_or_a_code():
    """This firmware answers TEC:UNITS? with 'CELSIUS', not '0'. A bare
    int(float(reply)) took the whole node down at connect."""
    tc = TC10LAB.__new__(TC10LAB)
    sent = []
    for reply, expected in [("CELSIUS", "C"), ("0", "C"), ("KELVIN", "K"),
                            ("2", "F"), ("FAHRENHEIT", "F"), ("RAW", "raw"),
                            ("3", "raw"), ("wat", "?")]:
        tc.query = lambda cmd, r=reply: r
        assert tc.get_units() == expected, f"{reply!r} -> {tc.units!r}"

    # set_units sends the numeric CODE, whichever form the caller used.
    tc.command = lambda cmd: sent.append(cmd)
    tc.query = lambda cmd: "CELSIUS"
    tc.set_units("C")
    tc.set_units(0)
    assert sent == ["TEC:UNITS 0", "TEC:UNITS 0"], sent


def test_query_float_tolerates_a_decorated_reply():
    """Same firmware quirk, on a number: take the leading value rather than
    letting one decorated reply kill the poll."""
    tc = TC10LAB.__new__(TC10LAB)
    for reply, expected in [("25.0", 25.0), ("25.0 C", 25.0), ("-1.25", -1.25),
                            ("+3.5 A", 3.5), ("1.2e-3", 0.0012), (".5", 0.5)]:
        tc.query = lambda cmd, r=reply: r
        assert tc.query_float("TEC:ACT?") == expected, f"{reply!r}"
    tc.query = lambda cmd: "CELSIUS"
    try:
        tc.query_float("TEC:ACT?")
    except ValueError as exc:
        assert "CELSIUS" in str(exc), "the error must quote what came back"
    else:
        raise AssertionError("a reply with no number at all must still raise")


def test_a_timed_out_query_does_not_leave_the_session_one_answer_behind():
    """The silent-corruption case: a query that times out has still been SENT,
    so its reply queues up and every later query returns the PREVIOUS answer.
    Those parse fine and publish happily -- the setpoint shows up as the
    temperature and nothing ever raises."""
    tc = TC10LAB("USB0::0x1A45::0x3101::X::INSTR")

    class FlakyVisa:
        def __init__(self):
            self.sent, self.fail_next = [], False

        def write(self, cmd):
            self.sent.append(cmd)

        def query(self, cmd):
            self.sent.append(cmd)
            if self.fail_next:
                self.fail_next = False
                raise TimeoutError("VI_ERROR_TMO")
            return "0"

        def clear(self):
            self.sent.append("cleared")

        def close(self):
            pass

    tc.device = FlakyVisa()
    tc.device.fail_next = True
    try:
        tc.query("TEC:ACT?")
    except TimeoutError:
        pass
    else:
        raise AssertionError("the timeout must still reach the caller")
    assert tc._desynced, "a failed query must mark the session suspect"

    tc.device.sent.clear()
    tc.query("TEC:SET?")
    # The resync must use the protocol CLEAR, never a *STB? read-back loop:
    # every query writes one request and reads one reply, so a drain made of
    # queries removes exactly as many replies as it adds. That is what once
    # made the node report its instrument's identity as "0".
    assert tc.device.sent == ["cleared", "*CLS", "TEC:SET?"], tc.device.sent
    assert "*STB?" not in tc.device.sent, "a query cannot drain a query backlog"
    assert not tc._desynced

    # And a healthy query must not pay for the drain every time.
    tc.device.sent.clear()
    tc.query("TEC:ACT?")
    assert tc.device.sent == ["TEC:ACT?"], tc.device.sent


def test_status_costs_five_round_trips():
    """Every extra query is another chance per second for the instrument to be
    mid-reply when the next one arrives."""
    tc = TC10LAB.__new__(TC10LAB)
    asked = []
    tc.query_int = lambda cmd: asked.append(cmd) or 0
    tc.query_float = lambda cmd: asked.append(cmd) or 0.0
    tc.units = "C"
    tc.status()
    assert len(asked) == 5, asked
    assert "TEC:UNITS?" not in asked      # cached by set_units()/get_units()


def test_dev_path_uses_the_kernel_usbtmc_transport():
    """TCLAB_RESOURCE=/dev/usbtmc0 must not be handed to pyvisa (which cannot
    parse it) -- that silently looked like 'the instrument is not there'."""
    tc = TC10LAB("/dev/usbtmc-does-not-exist")
    try:
        tc._open()
    except OSError:
        pass                              # reached the char device, as intended
    else:
        raise AssertionError("expected an OSError from the missing device node")
    assert tc.rm is None, "pyvisa must not be involved for a /dev path"


def test_usbtmc_picks_the_tc10_not_the_other_usbtmc_box():
    """/dev/usbtmc0 is not reliably the TC10 -- the Rigol AWG is USB-TMC too and
    the kernel numbers them in enumeration order. Opening the wrong one puts two
    nodes on one instrument, which reads as a flapping link."""
    from scopio_microscope.drivers import TC10LAB as mod

    class FakeTmc:
        opened, closed = [], []

        def __init__(self, path):
            self.path, self.sent = path, []
            FakeTmc.opened.append(path)

        def write(self, cmd):
            self.sent.append(cmd)

        def query(self, cmd):
            self.sent.append(cmd)
            if cmd == "*IDN?":
                return {"/dev/usbtmc0": "Rigol Technologies,DG1022Z,DG1ZA,00.02",
                        "/dev/usbtmc1": "Wavelength Electronics,TC10 LAB,123,1.0"}[self.path]
            return "0"          # *STB?: no Message Available

        def clear(self):
            self.sent.append("cleared")

        def close(self):
            FakeTmc.closed.append(self.path)

    real_glob, real_dev = mod.glob.glob, mod.UsbtmcDevice
    both = ["/dev/usbtmc0", "/dev/usbtmc1"]
    mod.glob.glob = lambda p: both if p == mod.USBTMC_GLOB else []
    mod.UsbtmcDevice = FakeTmc
    try:
        tc = TC10LAB(mod.USBTMC_GLOB)
        tc._open()
        assert tc.resource == "/dev/usbtmc1", f"picked {tc.resource}"
        assert FakeTmc.opened == both
        assert FakeTmc.closed == ["/dev/usbtmc0"], "the wrong device must be released"
        assert "*CLS" in tc.device.sent, "the session must be cleared on connect"
        assert "cleared" in tc.device.sent, "the session must be CLEARed on connect"

        # A typo'd pattern must not hide an instrument that is plainly present:
        # find it anyway, and say the config needs fixing.
        FakeTmc.opened, FakeTmc.closed = [], []
        typo = TC10LAB("/dev/usbtmc*.")          # trailing '.' -- matches nothing
        typo._open()
        assert typo.resource == "/dev/usbtmc1", f"picked {typo.resource}"
        assert "matched nothing" in typo.probe_note, typo.probe_note
    finally:
        mod.glob.glob, mod.UsbtmcDevice = real_glob, real_dev


def test_usbtmc_read_refuses_to_desync():
    """A reply longer than READ_SIZE leaves the tail queued, and every later
    query then returns the previous answer. Fail loudly rather than silently."""
    from scopio_microscope.drivers.TC10LAB import UsbtmcDevice

    dev = UsbtmcDevice.__new__(UsbtmcDevice)
    dev._fd = -1
    reads = [b"x" * UsbtmcDevice.READ_SIZE, b"short\n"]
    dev.write = lambda cmd: None
    import os as _os
    real_read = _os.read
    _os.read = lambda fd, n: reads.pop(0)
    try:
        try:
            dev.query("TEC:SENSORLIST?")
        except IOError:
            pass
        else:
            raise AssertionError("a full-buffer read must raise, not desync")
        assert dev.query("TEC:ACT?") == "short\n"
    finally:
        _os.read = real_read


# ---------------------------------------------------------------- dispatch
def test_dispatch_arguments():
    gen = awg()
    assert dispatch.call(gen, "position") == '{"x": 0.0, "y": 0.0}'
    dispatch.call(gen, "update", "[1, 0.5]")
    assert gen.xpos == 0.5
    dispatch.call(gen, "update", "", '{"ch": 2, "val": 0.25}')
    assert gen.ypos == 0.25
    dispatch.call(gen, "offsets", "0.75")          # bare scalar -> one positional
    assert gen.xoffset == 0.75

    for method, args, kwargs in [("_open", "", ""),          # private
                                 ("nope", "", ""),           # unknown
                                 ("update", "[1]", ""),      # wrong arity
                                 ("update", "[1,", ""),      # bad JSON
                                 ("update", "", "[1]"),      # kwargs not an object
                                 ("xpos", "", "")]:          # attribute, not method
        try:
            dispatch.call(gen, method, args, kwargs)
        except dispatch.DispatchError:
            continue
        raise AssertionError(f"{method}({args}, {kwargs}) should be a DispatchError")


def test_dispatch_json_is_strict_parser_safe():
    assert dispatch.to_json(float("nan")) == "null"
    assert dispatch.to_json({"a": [float("inf"), 1.5]}) == '{"a": [null, 1.5]}'
    assert dispatch.to_json(b"\x00\x01") == "[0, 1]"


def test_list_methods_works_while_disconnected():
    described = dispatch.describe(DG1022Z, ({"name": "reconnect",
                                             "signature": "()", "doc": ""},))
    names = {d["name"] for d in described}
    for expected in ("dcinit", "update", "move", "sininit", "sinupdate",
                     "offsets", "position", "command", "query", "snapshot"):
        assert expected in names, f"{expected} is not discoverable"
    assert not any(n.startswith("_") for n in names)
    assert "reconnect" in names
    sig = next(d["signature"] for d in described if d["name"] == "update")
    assert not sig.startswith("(self"), sig


# ------------------------------------------------- gateway JSON -> message
def _conversion():
    """Import the gateway's conversion module with rosidl stubbed out."""
    if "scopio_gateway.conversion" in sys.modules:
        return sys.modules["scopio_gateway.conversion"]
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "src", "scopio_gateway"))
    rosidl = types.ModuleType("rosidl_runtime_py")
    rosidl.message_to_ordereddict = dict
    rosidl.set_message_fields = lambda msg, data: [setattr(msg, k, v)
                                                   for k, v in data.items()]
    utils = types.ModuleType("rosidl_runtime_py.utilities")
    utils.get_message = utils.get_service = lambda t: None
    sys.modules["rosidl_runtime_py"] = rosidl
    sys.modules["rosidl_runtime_py.utilities"] = utils
    from scopio_gateway import conversion
    return conversion


class FakeControls:
    """Stands in for SetCameraControls.Request: floats defaulting to 0.0."""

    _FIELDS = {"red_gain": "double", "blue_gain": "double", "contrast": "double"}

    def __init__(self):
        for name in self._FIELDS:
            setattr(self, name, 0.0)

    @classmethod
    def get_fields_and_field_types(cls):
        return dict(cls._FIELDS)


def test_a_partial_service_body_leaves_other_floats_alone():
    """The bug this closes: POST camera/set_controls {"contrast": 1.2} built a
    request with red_gain=0.0 -- a command to zero that gain, not "unchanged".
    An EMPTY body did it to every field at once."""
    conversion = _conversion()
    import math as _math

    msg = conversion.build_msg(FakeControls, {"contrast": 1.2}, nan_for_missing=True)
    assert msg.contrast == 1.2
    assert _math.isnan(msg.red_gain) and _math.isnan(msg.blue_gain)

    empty = conversion.build_msg(FakeControls, {}, nan_for_missing=True)
    assert all(_math.isnan(getattr(empty, f)) for f in FakeControls._FIELDS)

    # Explicit null still means "unchanged"; action goals keep the 0.0 default.
    assert _math.isnan(conversion.build_msg(FakeControls, {"contrast": None}).contrast)
    assert conversion.build_msg(FakeControls, {"contrast": 1.2}).red_gain == 0.0


# ------------------------------------------------------- camera server
def _camera_server():
    """Import the camera server with picamera2 stubbed out."""
    if "pi_camera_server" in sys.modules:
        return sys.modules["pi_camera_server"]
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    sys.path.insert(0, os.path.join(root, "camera_server"))
    for name, attrs in [("picamera2", {"Picamera2": object}),
                        ("picamera2.encoders", {"MJPEGEncoder": object}),
                        ("picamera2.outputs", {"FileOutput": object})]:
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
    import pi_camera_server
    return pi_camera_server


class MonoCamera:
    """A monochrome sensor: no AwbEnable, no ColourGains, no Saturation."""

    camera_controls = {"AeEnable": None, "ExposureTime": None, "AnalogueGain": None,
                       "FrameDurationLimits": None, "Brightness": None,
                       "Contrast": None, "Sharpness": None}

    def __init__(self):
        self.applied = None

    def set_controls(self, c):
        for name in c:
            if name not in self.camera_controls:
                raise RuntimeError(f"Control {name} is not advertised by libcamera")
        self.applied = c

    def capture_metadata(self):
        return {}


def test_mono_sensor_does_not_reject_the_whole_request():
    """The bug: one unsupported control (AwbEnable on a mono sensor) raised out
    of the handler, so NOTHING was applied, the socket hung up, and the caller
    retried forever."""
    cs = _camera_server()
    cam = MonoCamera()
    cs.picam2 = cam
    try:
        cs.apply_controls({"exposure": 12000, "analogue_gain": 2.0, "contrast": 1.5,
                           "red_gain": 2.4, "blue_gain": 2.5, "framerate": 30})
        assert cam.applied is not None, "nothing was applied"
        assert "AwbEnable" not in cam.applied and "ColourGains" not in cam.applied
        # ...and everything the sensor DOES have still went through.
        assert cam.applied["ExposureTime"] == 12000
        assert cam.applied["AnalogueGain"] == 2.0
        assert cam.applied["Contrast"] == 1.5
        assert "FrameDurationLimits" in cam.applied
        # White balance answers instead of raising.
        assert "error" in cs.do_white_balance()
    finally:
        cs.picam2 = None


def test_exposure_longer_than_the_frame_lowers_the_frame_rate():
    """A frame cannot be shorter than its own exposure. Pinning
    FrameDurationLimits below ExposureTime asks the sensor for the impossible;
    some stall on it rather than clamp, which looks like frozen video."""
    cs = _camera_server()
    cam = MonoCamera()
    cs.picam2 = cam
    try:
        cs.state.update(framerate=30.0, exposure=20000)
        # 200 ms exposure at 30 fps (33.3 ms frames) is not satisfiable.
        cs.apply_controls({"exposure": 200000})
        lo, hi = cam.applied["FrameDurationLimits"]
        assert lo == hi >= 200000, cam.applied["FrameDurationLimits"]
        assert cs.state["framerate"] < 5.0, cs.state["framerate"]

        # Going back to a short exposure must restore the requested rate.
        cs.apply_controls({"framerate": 30.0, "exposure": 5000})
        lo, hi = cam.applied["FrameDurationLimits"]
        assert lo == hi == 33333, cam.applied["FrameDurationLimits"]
        assert cs.state["framerate"] == 30.0
    finally:
        cs.picam2 = None
        cs.state.update(framerate=30.0, exposure=20000)


# ------------------------------------------------------- relay node
class FakePin:
    """A gpiozero OutputDevice that can be made to throw, per direction.

    The flags are CLASS attributes on purpose: _open() constructs a fresh
    device, and a re-opened pin has to inherit the fault being simulated."""

    fail_on = fail_off = False

    def __init__(self, pin, active_high=True, initial_value=False):
        self.pin, self.value, self.closed = pin, initial_value, False

    def on(self):
        if FakePin.fail_on:
            raise OSError("GPIO busy")
        self.value = True

    def off(self):
        if FakePin.fail_off:
            raise OSError("GPIO busy")
        self.value = False

    def close(self):
        self.closed = True


class FakeNode:
    """Just enough rclpy.node.Node for the relay's state machine."""

    def __init__(self, name):
        self._params = {}
        self.sent = []                       # every Bool published, in order

    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, _type, _topic, _qos):
        return types.SimpleNamespace(publish=lambda m: self.sent.append(m.data))

    def create_service(self, *a, **kw):
        return None

    def create_timer(self, *a, **kw):
        return None

    def get_logger(self):
        noop = lambda *a, **kw: None         # noqa: E731
        return types.SimpleNamespace(info=noop, warning=noop, error=noop)

    def destroy_node(self):
        return None


def _relay_node():
    """Import relay_node with gpiozero and rclpy stubbed out."""
    if "scopio_microscope.relay_node" in sys.modules:
        return sys.modules["scopio_microscope.relay_node"]
    stubs = {"gpiozero": {"OutputDevice": FakePin},
             "rclpy": {},
             "rclpy.node": {"Node": FakeNode},
             "rclpy.qos": {"QoSProfile": lambda **kw: types.SimpleNamespace(**kw),
                           "DurabilityPolicy": types.SimpleNamespace(
                               TRANSIENT_LOCAL="transient_local")},
             "std_msgs.msg": {"Bool": lambda data=False: types.SimpleNamespace(data=data)},
             "std_srvs.srv": {"SetBool": object}}
    for name, attrs in stubs.items():
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        if "." in name:                      # `from rclpy.node import Node`
            parent, _, child = name.rpartition(".")
            sys.modules.setdefault(parent, types.ModuleType(parent))
            setattr(sys.modules[parent], child, mod)
    from scopio_microscope import relay_node
    return relay_node


def test_a_relay_that_will_not_switch_off_is_never_reported_off():
    """THE laser rule. When a GPIO call throws, the relay's real position is
    unknown -- and unknown published as False is a green 'Laser OFF' button next
    to a live laser. Only an off() that actually SUCCEEDED may report OFF."""
    relay_node = _relay_node()
    req = types.SimpleNamespace(data=True)
    resp = lambda: types.SimpleNamespace(success=None, message="")   # noqa: E731
    try:
        node = relay_node.RelayNode()
        assert node.sent == [False], node.sent      # claimed the pin, OFF

        # Healthy pin: on() takes, and the state reported is the state reached.
        assert node._set_relay(req, resp()).success is True
        assert node._state is True and node.sent[-1] is True

        # on() throws but the recovery off() works -> OFF is TRUE, so report it.
        FakePin.fail_on, FakePin.fail_off = True, False
        assert node._set_relay(req, resp()).success is False
        assert node._state is False and node.sent[-1] is False

        # Both directions throw: the position is unknown, so it is reported ON,
        # and the device is dropped so the retry timer re-opens it.
        FakePin.fail_on = FakePin.fail_off = True
        assert node._set_relay(req, resp()).success is False
        assert node._state is True and node.sent[-1] is True
        assert node._relay is None

        # Re-opening drives the pin off, which is both the recovery AND the safe
        # action -- and only now may False be published again.
        FakePin.fail_on = FakePin.fail_off = False
        node._retry_open()
        assert node._relay is not None
        assert node._state is False and node.sent[-1] is False

        # Shutdown leaves it off even when the (post-context) publish throws.
        node.sent = _RaisesOnAppend()
        pin = node._relay
        node.destroy_node()
        assert pin.value is False and pin.closed is True
    finally:
        FakePin.fail_on = FakePin.fail_off = False


class _RaisesOnAppend(list):
    """Publishing after rclpy has shut down raises; the relay must still be
    switched off and released when it does."""

    def append(self, item):
        raise RuntimeError("InvalidHandle: context already shut down")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"  ok  {test.__name__}")
    print(f"\n{len(tests)} checks passed.")


if __name__ == "__main__":
    main()
