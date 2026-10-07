#!/usr/bin/env python3
"""Offline checks of the hardware-free logic (see README.md); stops at the first failure."""

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
    assert gen.offsets(x=0.25, y=-0.10) == {"x": 0.25, "y": -0.10}
    # Every galvo write returns the position it produced, saving a round trip.
    assert gen.update(1, 1.0) == {"x": 1.0, "y": 0.0}
    assert gen.update(2, 1.0) == {"x": 1.0, "y": 1.0}
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


class TimedDevice(FakeDevice):
    """FakeDevice that also records WHEN each I/O reached the wire."""

    def __init__(self, replies=None):
        super().__init__(replies)
        self.at = []

    def write(self, cmd):
        import time
        self.at.append(time.monotonic())
        super().write(cmd)

    def query(self, cmd):
        import time
        self.at.append(time.monotonic())
        return super().query(cmd)


def test_both_instruments_are_paced_below_60_per_second():
    """Both instruments lag for seconds past ~60 commands/s, so I/O is paced to 50/s."""
    import threading
    for inst in (awg(), TC10LAB("TCPIP::fake::INSTR")):
        inst.device = TimedDevice()
        threads = [threading.Thread(target=lambda: [inst.command("*CLS"),
                                                    inst.query("*IDN?")])
                   for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        gaps = [b - a for a, b in zip(inst.device.at, inst.device.at[1:])]
        assert len(inst.device.at) == 8
        # A hair of slack for the sleep granularity of the OS timer.
        assert min(gaps) >= inst.MIN_INTERVAL_S * 0.9, \
            f"{type(inst).__name__}: {1 / min(gaps):.0f} I/Os per second"
        assert inst.MIN_INTERVAL_S >= 1 / 60


def test_a_short_ramp_is_not_a_flood():
    """A short ramp caps its steps instead of flooding the AWG (60 steps in 0.1 s would be 600/s)."""
    gen = awg()
    gen.move(1, 1.0, t=0.1, steps=60)
    written = offsets(gen.device.writes, ":SOURce1")
    assert len(written) == int(0.1 / gen.MIN_INTERVAL_S), len(written)
    assert abs(written[-1] - 1.0) < 1e-9


def test_dcinit_zeroes_the_remembered_position():
    """dcinit writes the offsets, so the remembered position must be zero afterwards."""
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
    """Touching self.device outside command()/query() would bypass the lock and the pacing."""
    import inspect

    from scopio_microscope.drivers import dg1022z
    source = inspect.getsource(dg1022z)
    allowed = ("self.device.read_termination", "self.device.write_termination",
               "self.device.timeout", "self.device.close()",
               "self.device.write(cmd)", "self.device.query(cmd)",
               # _resync: the protocol CLEAR and its read-drain fallback, both
               # inside `with self._lock` -- flushing, not SCPI.
               "self.device.clear()", "self.device.read()")
    for lineno, line in enumerate(source.splitlines(), 1):
        if "self.device." in line and not any(a in line for a in allowed):
            raise AssertionError(f"dg1022z.py:{lineno} bypasses command()/query(): "
                                 f"{line.strip()}")


def test_the_nodes_and_their_drivers_still_agree():
    """Every driver method a node calls must exist (a bench-copy driver once broke every connect silently)."""
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
    """TEC:UNITS? may answer 'CELSIUS', not '0'; both forms must work."""
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
    """A decorated numeric reply yields its leading number instead of crashing the node."""
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
    """A timed-out query's late reply must be flushed, or every later answer is one behind."""
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
    # Resync must use CLEAR, never a query loop: each query adds a reply for every one it removes.
    assert tc.device.sent == ["cleared", "*CLS", "TEC:SET?"], tc.device.sent
    assert "*STB?" not in tc.device.sent, "a query cannot drain a query backlog"
    assert not tc._desynced

    # And a healthy query must not pay for the drain every time.
    tc.device.sent.clear()
    tc.query("TEC:ACT?")
    assert tc.device.sent == ["TEC:ACT?"], tc.device.sent


def test_status_costs_five_round_trips():
    """status() is exactly five queries: each extra one is paid every second."""
    tc = TC10LAB.__new__(TC10LAB)
    asked = []
    tc.query_int = lambda cmd: asked.append(cmd) or 0
    tc.query_float = lambda cmd: asked.append(cmd) or 0.0
    tc.units = "C"
    tc.status()
    assert len(asked) == 5, asked
    assert "TEC:UNITS?" not in asked      # cached by set_units()/get_units()


# ------------------------------------------------- TC10 LAB: setpoint ramp
def tc10(**replies):
    tc = TC10LAB("TCPIP::fake::INSTR")
    tc.device = FakeDevice({"TEC:LIMit:TLO?": "-10", "TEC:LIMit:THI?": "80",
                            "TEC:ACT?": "20.0", **replies})
    return tc


def setpoints(tc):
    return [float(w.split()[1]) for w in tc.device.writes if w.startswith("TEC:SET ")]


def wait_for(tc, state, limit=3.0):
    import time
    end = time.monotonic() + limit
    while tc.ramp_status()["state"] != state:
        assert time.monotonic() < end, f"ramp never reached {state!r}: {tc.ramp_status()}"
        time.sleep(0.01)


def test_ramp_walks_the_setpoint_from_the_measured_temperature():
    """A ramp starts at the measured temperature (no jump), climbs monotonically and ends exactly on target."""
    tc = tc10()
    status = tc.ramp(21, rate=300, interval=0.1)       # 1 degree at 5 degrees/s: 0.2 s
    assert status["state"] == "ramping" and status["start"] == 20.0, status
    wait_for(tc, "done")
    sent = setpoints(tc)
    assert len(sent) >= 2, sent
    assert sent == sorted(sent) and 20.0 <= sent[0] < 21.0, sent
    assert sent[-1] == 21.0 and tc.device.writes[-1] == "TEC:SET 21.000"
    assert tc.ramp_status()["setpoint"] == 21.0


def test_ramp_rate_can_be_changed_mid_way_without_a_jump():
    """Calling ramp() again re-rates it from the setpoint already reached, not from the start."""
    import time
    tc = tc10()
    tc.ramp(15, rate=60, interval=0.1)                 # 1 degree/s: 5 s down to 15
    time.sleep(0.35)
    reached = tc.ramp_status()["setpoint"]
    assert reached < 20.0, reached
    status = tc.ramp(15, rate=1200, interval=0.1)      # now 20 degrees/s
    assert status["start"] <= reached, (status, reached)
    wait_for(tc, "done")
    sent = setpoints(tc)
    assert sent == sorted(sent, reverse=True), sent
    assert sent[-1] == 15.0


def test_set_setpoint_and_close_stop_a_ramp():
    import time
    tc = tc10()
    tc.ramp(60, rate=60, interval=0.1)
    time.sleep(0.15)
    tc.set_setpoint(30)
    assert tc.ramp_status()["state"] == "stopped"
    time.sleep(0.25)
    assert tc.device.writes[-1] == "TEC:SET 30", tc.device.writes[-3:]

    tc.ramp(60, rate=60, interval=0.1)
    tc._close()                                        # must not deadlock on the I/O lock
    assert tc.ramp_status()["state"] == "stopped"


def test_ramp_refuses_bad_arguments_before_writing():
    tc = tc10()
    for kwargs in ({"rate": 0}, {"rate": -1}, {"rate": float("nan")},
                   {"rate": float("inf")}, {"rate": 1, "interval": 0.01},
                   {"rate": 1, "target": 95}, {"rate": 1, "start": -20}):
        kwargs.setdefault("target", 25)
        try:
            tc.ramp(**kwargs)
        except ValueError:
            continue
        raise AssertionError(f"ramp({kwargs}) must refuse")
    assert not setpoints(tc) and tc.ramp_status()["state"] == "idle"


def test_ramp_gives_up_after_repeated_write_failures():
    tc = tc10()

    def broken(cmd):
        if cmd.startswith("TEC:SET"):
            raise TimeoutError("VI_ERROR_TMO")
    tc.device.write = broken
    tc.ramp(25, rate=60, interval=0.1)
    wait_for(tc, "failed")
    assert "TimeoutError" in tc.ramp_status()["error"]


# ------------------------------------------------ TC10 LAB: finding the box
class FakeRM:
    """pyvisa.ResourceManager('@py') stand-in that lists and opens fake devices."""

    listed, idn, opened = [], "", []

    def __init__(self, backend):
        pass

    def list_resources(self, query="?*::INSTR"):
        return list(FakeRM.listed)

    def open_resource(self, res):
        FakeRM.opened.append(res)
        dev = FakeDevice({"*IDN?": FakeRM.idn})
        dev.clear = lambda: None
        return dev

    def close(self):
        pass


class FakeTmc:
    """A fake /dev/usbtmcN; `queued` is a reply left by a session that died mid-query."""

    replies, queued, opened, closed = {}, {}, [], []

    def __init__(self, path, timeout_ms=None):
        self.path, self.sent = path, []
        FakeTmc.opened.append(path)

    def write(self, cmd):
        self.sent.append(cmd)

    def query(self, cmd):
        self.sent.append(cmd)
        stale = FakeTmc.queued.pop(self.path, None)
        if stale is not None:
            return stale
        return FakeTmc.replies.get(self.path, {}).get(cmd, "0")

    def clear(self):
        self.sent.append("cleared")
        FakeTmc.queued.pop(self.path, None)

    def close(self):
        FakeTmc.closed.append(self.path)


RIGOL_IDN = "Rigol Technologies,DG1022Z,DG1ZA,00.02"
TC10_IDN = "Wavelength Electronics,TC10 LAB,123,1.0"


def _usb_world(nodes, sysfs_vids=None, bus_vids=(), listed=(), visa_idn=TC10_IDN):
    """Stand the TC10 driver up in a fake machine and return the module."""
    import tempfile
    from scopio_microscope.drivers import TC10LAB as mod

    root = tempfile.mkdtemp()
    usbmisc, bus = os.path.join(root, "usbmisc"), os.path.join(root, "bus")
    for path, vid in (sysfs_vids or {}).items():
        # dirname(realpath(<usbmisc>/usbtmcN/device)) must hold idVendor. A
        # plain directory stands in for the kernel's symlink.
        node = os.path.join(usbmisc, os.path.basename(path))
        os.makedirs(os.path.join(node, "device"))
        with open(os.path.join(node, "idVendor"), "w") as f:
            f.write(f"{vid:04x}\n")
    for i, vid in enumerate(bus_vids):
        dev = os.path.join(bus, f"1-1.{i + 1}")
        os.makedirs(dev)
        with open(os.path.join(dev, "idVendor"), "w") as f:
            f.write(f"{vid:04x}\n")

    real_glob = _usb_world.real_glob

    def fake_glob(pattern):
        if pattern.startswith("/dev/"):
            import fnmatch
            return [p for p in nodes if fnmatch.fnmatch(p, pattern)]
        return real_glob(pattern)

    mod.glob.glob = fake_glob
    mod.SYSFS_USBMISC, mod.SYSFS_USB = usbmisc, bus
    mod.UsbtmcDevice = FakeTmc
    FakeTmc.replies = {p: {"*IDN?": idn} for p, idn in nodes.items()}
    FakeTmc.queued, FakeTmc.opened, FakeTmc.closed = {}, [], []
    mod.pyvisa.ResourceManager = FakeRM
    FakeRM.listed, FakeRM.idn, FakeRM.opened = list(listed), visa_idn, []
    return mod


def _restore_usb_world():
    from scopio_microscope.drivers import TC10LAB as mod
    mod.glob.glob = _usb_world.real_glob
    mod.SYSFS_USBMISC, mod.SYSFS_USB = "/sys/class/usbmisc", "/sys/bus/usb/devices"
    mod.UsbtmcDevice = _usb_world.real_dev
    if hasattr(mod.pyvisa, "ResourceManager"):
        del mod.pyvisa.ResourceManager


def _init_usb_world():
    import glob as _glob
    from scopio_microscope.drivers import TC10LAB as mod
    _usb_world.real_glob = _glob.glob
    _usb_world.real_dev = mod.UsbtmcDevice


_init_usb_world()
TC10_USB = "USB0::6725::12545::SN123::0::INSTR"   # pyvisa-py writes ids in decimal


def test_when_the_kernel_owns_the_tc10_visa_is_never_touched():
    """When the kernel owns the TC10, VISA is never touched (a VISA open hangs on this Pi)."""
    try:
        for configured in ("", "/dev/usbtmc0", TC10_USB):
            _usb_world({"/dev/usbtmc0": RIGOL_IDN, "/dev/usbtmc1": TC10_IDN},
                       sysfs_vids={"/dev/usbtmc0": 0x1AB1, "/dev/usbtmc1": 0x1A45},
                       listed=[TC10_USB])
            tc = TC10LAB(configured)
            tc._open()
            assert tc.resource == "/dev/usbtmc1", (configured, tc.resource)
            assert FakeRM.opened == [], f"{configured!r}: VISA opened {FakeRM.opened}"
            # The Rigol's node belongs to the galvo node: never written to.
            assert "/dev/usbtmc0" not in FakeTmc.opened, (configured, FakeTmc.opened)
            assert tc.identity == TC10_IDN
            if configured == TC10_USB:
                assert "owns the TC10" in tc.probe_note, tc.probe_note
    finally:
        _restore_usb_world()


def test_a_stale_reply_cannot_make_the_tc10_reject_itself():
    """A stale queued reply must not be read as the identity and get the TC10 rejected."""
    try:
        _usb_world({"/dev/usbtmc0": TC10_IDN}, sysfs_vids={"/dev/usbtmc0": 0x1A45})
        FakeTmc.queued = {"/dev/usbtmc0": "25.0"}
        tc = TC10LAB("")
        tc._open()
        assert tc.resource == "/dev/usbtmc0"
        sent = tc.device.sent
        assert sent.index("cleared") < sent.index("*IDN?"), sent
    finally:
        _restore_usb_world()


def test_a_configured_dev_path_with_no_node_falls_back_to_visa():
    """A configured /dev path with no node behind it falls back to VISA, with a note."""
    try:
        _usb_world({}, listed=["USB0::6833::1602::DG1ZA::0::INSTR", TC10_USB])
        tc = TC10LAB("/dev/usbtmc*")
        tc._open()
        assert tc.resource == TC10_USB, tc.resource
        assert FakeRM.opened == [TC10_USB], "only the Wavelength box, by vendor id"
        assert "not bound" in tc.probe_note, tc.probe_note
        assert not any(r.startswith("/dev/") for r in FakeRM.opened), \
            "a /dev path must never be handed to pyvisa"
    finally:
        _restore_usb_world()


def test_sysfs_unreadable_still_finds_it_by_asking():
    """Where sysfs cannot identify a node, every node is asked and the TC10 is still found."""
    try:
        _usb_world({"/dev/usbtmc0": RIGOL_IDN, "/dev/usbtmc1": TC10_IDN})
        tc = TC10LAB("")
        tc._open()
        assert tc.resource == "/dev/usbtmc1"
        assert FakeTmc.closed == ["/dev/usbtmc0"]
        assert FakeRM.opened == []
    finally:
        _restore_usb_world()


def test_not_found_says_whether_the_box_is_on_the_bus():
    """The not-found error says first whether the TC10 is on the USB bus at all."""
    try:
        _usb_world({}, bus_vids=())
        try:
            TC10LAB("")._open()
        except RuntimeError as exc:
            assert "no TC10 LAB on the USB bus" in str(exc), exc
        else:
            raise AssertionError("nothing plugged in must raise")

        _usb_world({}, bus_vids=(0x1A45,), listed=[])
        try:
            TC10LAB("")._open()
        except RuntimeError as exc:
            assert "IS on the USB bus" in str(exc) and "permissions" in str(exc), exc
        else:
            raise AssertionError("an unlisted box must raise")
    finally:
        _restore_usb_world()


def test_the_wrong_instrument_is_refused_by_identity():
    """An address naming the AWG is refused by *IDN?, so two nodes never share one instrument."""
    try:
        _usb_world({}, visa_idn=RIGOL_IDN)
        try:
            TC10LAB("TCPIP::10.0.0.9::INSTR")._open()
        except RuntimeError as exc:
            assert "not a Wavelength TC10" in str(exc), exc
        else:
            raise AssertionError("a Rigol answering *IDN? must be refused")
    finally:
        _restore_usb_world()


def test_the_kernel_owns_it_but_it_is_silent_does_not_fight_for_it():
    """A silent kernel-owned TC10 raises instead of falling back to VISA (the fight that hangs)."""
    try:
        _usb_world({"/dev/usbtmc0": ""}, sysfs_vids={"/dev/usbtmc0": 0x1A45},
                   listed=[TC10_USB])
        try:
            TC10LAB("")._open()
        except RuntimeError as exc:
            assert "owns the TC10" in str(exc), exc
        else:
            raise AssertionError("a silent kernel-owned TC10 must raise")
        assert FakeRM.opened == []
    finally:
        _restore_usb_world()


def test_a_hung_connect_is_abandoned_not_waited_on_forever():
    """A connect that hangs inside the open is abandoned at the deadline, and no second one starts."""
    import threading
    import time as _time
    from scopio_microscope.connect_guard import ConnectGuard, ConnectHung

    guard = ConnectGuard(0.3)
    release, discarded = threading.Event(), []

    t0 = _time.monotonic()
    try:
        guard.run(lambda: release.wait() and "session", discarded.append)
    except ConnectHung as exc:
        assert "USB stack" in str(exc)
    else:
        raise AssertionError("a hung attempt must raise ConnectHung")
    assert _time.monotonic() - t0 < 1.0, "the deadline must actually bound it"

    # While the first is still stuck, no second session on the same device.
    try:
        guard.run(lambda: "second", discarded.append)
    except ConnectHung as exc:
        assert "still stuck" in str(exc)
    else:
        raise AssertionError("a second attempt must not start while one is stuck")

    # It comes back late after all: what it opened is closed, not leaked.
    release.set()
    for _ in range(50):
        if discarded:
            break
        _time.sleep(0.02)
    assert discarded == ["session"], discarded
    assert guard.run(lambda: "fresh", discarded.append) == "fresh"

    # And an attempt's own failure reaches the caller unchanged.
    def fails():
        raise OSError("VI_ERROR_RSRC_NFOUND")
    try:
        guard.run(fails, discarded.append)
    except OSError as exc:
        assert "NFOUND" in str(exc)
    else:
        raise AssertionError("the attempt's exception must propagate")


def test_the_awg_session_is_cleared_and_identified_on_connect():
    """The AWG session is cleared before *IDN? and refused if it is not a Rigol."""
    from scopio_microscope.drivers import dg1022z as mod

    class Session(FakeDevice):
        def __init__(self, idn):
            super().__init__({"*IDN?": idn})
            self.stale = "1000.0"            # left by a session that died

        def clear(self):
            self.writes.append("cleared")
            self.stale = None

        def query(self, cmd):
            self.writes.append(cmd)
            if self.stale is not None:
                reply, self.stale = self.stale, None
                return reply
            return self.replies.get(cmd, "0")

    for idn, ok in ((RIGOL_IDN, True), (TC10_IDN, False)):
        session = Session(idn)
        mod.pyvisa.ResourceManager = lambda backend: types.SimpleNamespace(
            open_resource=lambda res: session, close=lambda: None)
        try:
            gen = DG1022Z("USB0::6833::1602::DG1ZA::0::INSTR")
            gen._open()
            assert ok, "a Wavelength box must not pass as the AWG"
            assert gen.identity == RIGOL_IDN, gen.identity
            assert session.writes[0] == "cleared", session.writes
        except RuntimeError as exc:
            assert not ok and "not a Rigol" in str(exc), exc
        finally:
            del mod.pyvisa.ResourceManager


def test_a_timed_out_awg_query_does_not_leave_it_one_answer_behind():
    gen = awg()
    calls = {"n": 0}

    def flaky(cmd):
        gen.device.writes.append(cmd)
        calls["n"] += 1
        if calls["n"] == 1:
            raise TimeoutError("VI_ERROR_TMO")
        return "1000"

    gen.device.query = flaky
    gen.device.clear = lambda: gen.device.writes.append("cleared")
    try:
        gen.query(":SOURce1:FREQuency?")
    except TimeoutError:
        pass
    assert gen._desynced
    gen.device.writes.clear()
    assert gen.query(":SOURce1:FREQuency?") == "1000"
    assert gen.device.writes == ["cleared", "*CLS", ":SOURce1:FREQuency?"], \
        gen.device.writes


def _temperature_node():
    """Import temperature_node with rclpy and the message packages stubbed."""
    _camera_node()                       # installs the rclpy stubs
    anything = type("Anything", (), {})
    for name, attrs in {"scopio_interfaces.msg": {"TemperatureStatus": anything},
                        "scopio_interfaces.srv": {"InstrumentCall": anything}}.items():
        for k, v in attrs.items():
            if not hasattr(sys.modules[name], k):
                setattr(sys.modules[name], k, v)
    from scopio_microscope import temperature_node
    return temperature_node


def test_the_temperature_node_survives_a_connect_that_hangs():
    """temperature_node connects through the guard and keeps running when a connect hangs."""
    import threading
    import time as _time
    from scopio_microscope.connect_guard import ConnectGuard

    mod = _temperature_node()
    release, closed = threading.Event(), []

    class HangingTC10:
        def __init__(self, resource, timeout_ms=5000):
            self.resource, self.identity, self.probe_note = resource, "", ""

        def _open(self):
            release.wait()

        def get_units(self):
            return "C"

        def set_units(self, units):
            return "C"

        def _close(self):
            closed.append(self)

    real = mod.TC10LAB
    mod.TC10LAB = HangingTC10
    try:
        node = object.__new__(mod.TemperatureNode)
        FakeNode.__init__(node, "temperature_node")
        for name, value in (("resource", ""), ("timeout_ms", 5000), ("units", "C")):
            node.declare_parameter(name, value)
        node._lock, node.tc, node.idn, node.last_error = threading.RLock(), None, "", ""
        node._failures, node.state = 0, {}
        node._guard = ConnectGuard(0.3)

        t0 = _time.monotonic()
        assert node._connect() is False
        assert _time.monotonic() - t0 < 1.5, "the node thread must get back"
        assert "ConnectHung" in node.last_error, node.last_error
        assert node._connect() is False and "still stuck" in node.last_error

        release.set()
        for _ in range(50):
            if closed:
                break
            _time.sleep(0.02)
        assert len(closed) == 1, "the late session must be closed, not leaked"
        assert node.tc is None
    finally:
        mod.TC10LAB = real


def test_usbtmc_picks_the_tc10_not_the_other_usbtmc_box():
    """Of several usbtmc nodes, the TC10's is chosen and the AWG's is left alone."""
    from scopio_microscope.drivers import TC10LAB as mod

    class FakeTmc:
        opened, closed = [], []

        def __init__(self, path, timeout_ms=None):
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
    """A reply longer than READ_SIZE raises instead of silently desyncing the session."""
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


def _ros_bridge():
    """Import the gateway's ros_bridge with rclpy stubbed out."""
    _conversion()
    enum = lambda *names: types.SimpleNamespace(**{n: n.lower() for n in names})  # noqa: E731
    stubs = {"rclpy": {"ok": lambda: True},
             "rclpy.action": {},
             "rclpy.action.graph": {"get_action_names_and_types": lambda n: []},
             "rclpy.executors": {"MultiThreadedExecutor": object},
             "rclpy.node": {"Node": FakeNode},
             "rclpy.qos": {"QoSProfile": lambda **kw: types.SimpleNamespace(**kw),
                           "DurabilityPolicy": enum("VOLATILE", "TRANSIENT_LOCAL"),
                           "HistoryPolicy": enum("KEEP_LAST"),
                           "ReliabilityPolicy": enum("RELIABLE", "BEST_EFFORT")}}
    for name, attrs in stubs.items():
        mod = sys.modules.get(name) or types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)     # overwrite: other stubs may lack these
        sys.modules[name] = mod
        if "." in name:
            parent, _, child = name.rpartition(".")
            setattr(sys.modules.setdefault(parent, types.ModuleType(parent)),
                    child, mod)
    from scopio_gateway import ros_bridge
    return ros_bridge


def test_a_new_nodes_status_reaches_status_with_no_gateway_change():
    """A new node's */status topic reaches /api/v1/status with no gateway change."""
    mod = _ros_bridge()
    durability = sys.modules["rclpy.qos"].DurabilityPolicy
    reliability = sys.modules["rclpy.qos"].ReliabilityPolicy

    def pub(latched=False):
        return types.SimpleNamespace(qos_profile=types.SimpleNamespace(
            durability=durability.TRANSIENT_LOCAL if latched else durability.VOLATILE,
            reliability=reliability.RELIABLE))

    graph = {"/scopio/stage/position": ("scopio_interfaces/msg/StagePosition", [pub()]),
             "/scopio/relay/state": ("std_msgs/msg/Bool", [pub(latched=True)]),
             "/scopio/pressure/status": ("new_pkg/msg/Pressure", [pub()]),   # NEW node
             "/scopio/pressure/raw": ("std_msgs/msg/Float64", [pub()]),       # not status
             "/scopio/image/compressed": ("sensor_msgs/msg/CompressedImage", [pub()]),
             "/scopio/vacuum/status": ("new_pkg/msg/Vacuum", []),             # no publisher yet
             "/elsewhere/status": ("std_msgs/msg/String", [pub()])}
    subs = {}

    class Graph:
        def get_topic_names_and_types(self):
            return [(n, [t]) for n, (t, _) in graph.items()]

        def get_publishers_info_by_topic(self, name):
            return graph[name][1]

        def create_subscription(self, cls, name, cb, qos):
            subs[name] = (cb, qos)
            return name

    bridge = mod.RosBridge()
    bridge.node = Graph()
    bridge._discover_telemetry()
    assert set(subs) == {"/scopio/stage/position", "/scopio/relay/state",
                         "/scopio/pressure/status"}, sorted(subs)
    # Latched publishers are joined latched -- or the relay's retained value
    # never reaches a gateway that started after it.
    assert subs["/scopio/relay/state"][1].durability == durability.TRANSIENT_LOCAL

    subs["/scopio/pressure/status"][0]({"mbar": 1.2e-6})
    snap = bridge.telemetry_snapshot()
    assert snap["pressure/status"]["msg"] == {"mbar": 1.2e-6}
    assert snap["awg/status"] is None, "fixed names stay present, null until published"
    assert list(snap)[:6] == mod.TELEMETRY_TOPICS

    # A node that starts later is found on the next scan -- once, not twice.
    graph["/scopio/vacuum/status"] = ("new_pkg/msg/Vacuum", [pub()])
    bridge._discover_telemetry()
    bridge._discover_telemetry()
    assert "/scopio/vacuum/status" in subs
    assert len(bridge._telemetry_subs) == 4


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
    """A partial set_controls body leaves every omitted float as NaN, not 0."""
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


# Real sensor_modes tables, abbreviated; crop_limits (x, y, w, h) is the sensor rectangle read.
IMX219 = [   # Camera Module 2
    {"size": (640, 480), "fps": 206.65, "crop_limits": (1000, 752, 1280, 960)},
    {"size": (1640, 1232), "fps": 41.85, "crop_limits": (0, 0, 3280, 2464)},
    {"size": (1920, 1080), "fps": 47.57, "crop_limits": (680, 692, 1920, 1080)},
    {"size": (3280, 2464), "fps": 21.19, "crop_limits": (0, 0, 3280, 2464)},
]
IMX708 = [   # Camera Module 3
    {"size": (1536, 864), "fps": 120.13, "crop_limits": (768, 432, 3072, 1728)},
    {"size": (2304, 1296), "fps": 56.03, "crop_limits": (0, 0, 4608, 2592)},
    {"size": (4608, 2592), "fps": 14.35, "crop_limits": (0, 0, 4608, 2592)},
]


def test_detail_mode_keeps_the_whole_field_of_view():
    """The detail mode is the fastest full-field mode, never a crop such as 1920x1080."""
    cs = _camera_server()

    modes = cs.pick_modes(IMX219, max_detail_w=1640)
    assert modes["detail"]["sensor"] == [1640, 1232], modes["detail"]
    assert modes["detail"]["full_fov"] is True
    assert modes["detail"]["fps"] == 41.9
    # 3280x2464 is bigger but half the rate, and gets scaled for the network
    # anyway -- it buys nothing over the binned full-frame mode.
    assert modes["fast"]["sensor"] == [640, 480]
    assert modes["fast"]["fps"] == 206.7
    assert modes["fast"]["full_fov"] is False, "the fast mode IS a crop; say so"

    # Both modes are 2x binned: same um/px, widths 2.56x apart, so the window must travel with each.
    assert modes["detail"]["window"] == [3280, 2464]
    assert modes["fast"]["window"] == [1280, 960]
    binning = lambda m: m["window"][0] / m["size"][0]
    assert binning(modes["detail"]) == binning(modes["fast"]) == 2.0

    # A different module must get its own best two, with no code change.
    m3 = cs.pick_modes(IMX708, max_detail_w=1640)
    assert m3["detail"]["sensor"] == [2304, 1296] and m3["detail"]["full_fov"]
    assert m3["fast"]["sensor"] == [1536, 864]

    # The detail STREAM is capped for the network while the SENSOR mode is not,
    # so the field of view survives the cap; aspect ratio has to survive it too.
    capped = cs.pick_modes(IMX708, max_detail_w=1152)
    assert capped["detail"]["sensor"] == [2304, 1296], "cap must not change the mode"
    assert capped["detail"]["size"] == [1152, 648]
    assert cs.pick_modes([]) == {}


def test_mono_sensor_does_not_reject_the_whole_request():
    """A control a mono sensor lacks is dropped, not allowed to reject the whole request."""
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
    """An exposure longer than the frame lowers the frame rate instead of stalling the sensor."""
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


# ------------------------------------------------------- camera node (bridge)
def _camera_node():
    """Import camera_node with rclpy and the message packages stubbed out."""
    if "scopio_microscope.camera_node" in sys.modules:
        return sys.modules["scopio_microscope.camera_node"]
    anything = type("Anything", (), {})
    stubs = {"rclpy": {},
             "rclpy.action": {"ActionServer": anything, "CancelResponse": anything,
                              "GoalResponse": anything},
             "rclpy.callback_groups": {"ReentrantCallbackGroup": anything},
             "rclpy.executors": {"MultiThreadedExecutor": anything},
             "rclpy.node": {"Node": FakeNode},
             "sensor_msgs.msg": {"CompressedImage": anything},
             "scopio_interfaces.msg": {"CameraState": anything},
             "scopio_interfaces.srv": {"SetCameraControls": anything,
                                       "SetFramerate": anything,
                                       "WhiteBalance": anything,
                                       "StageJog": anything},
             "scopio_interfaces.action": {"Autofocus": anything}}
    for name, attrs in stubs.items():
        mod = sys.modules.get(name) or types.ModuleType(name)
        for k, v in attrs.items():
            if not hasattr(mod, k):
                setattr(mod, k, v)
        sys.modules[name] = mod
        if "." in name:
            parent, _, child = name.rpartition(".")
            setattr(sys.modules.setdefault(parent, types.ModuleType(parent)),
                    child, mod)
    from scopio_microscope import camera_node
    return camera_node


def _bridge(server):
    """A CameraNode in bridge mode whose camera server is the dict `server`."""
    mod = _camera_node()
    node = object.__new__(mod.CameraNode)
    FakeNode.__init__(node, "camera_node")
    node.bridge_url, node.picam2, node.width, node.height = "http://cam", None, 640, 480
    node.cam = {"red_gain": 2.0, "green_gain": 1.0, "blue_gain": 2.0,
                "framerate": 30.0, "exposure": 20000, "analogue_gain": 1.0,
                "colour_gain": 2.0, "contrast": 1.0, "saturation": 1.0,
                "brightness": 0.0, "sharpness": 1.0}
    node.exposure_budget = 20000.0
    node._latest_jpeg_at = 0.0
    node._server_ok_at, node._server_frames, node._server_fps = 0.0, None, 0.0
    node._refresh_at, node._pushed_controls, node._af_active = 0.0, False, False
    node._bridge_http = lambda method, path, payload=None, timeout=12.0: dict(server)
    return node


def test_camera_state_follows_what_the_camera_server_is_doing():
    """camera/state adopts settings changed through the gateway, after the node's own push."""
    server = {"width": 1640, "height": 1232, "mode": "detail", "framerate": 20.0,
              "exposure": 40000, "analogue_gain": 2.0, "red_gain": 3.0,
              "blue_gain": 5.0, "contrast": 1.5, "frames": 100}
    node = _bridge(server)
    assert node.connected is False

    # Before this connection's push: geometry is adopted, settings are NOT --
    # a restarted server is on defaults, and the push is there to undo that.
    node._refresh_from_server()
    assert node.connected is True, "a 200 from /controls is a live camera"
    assert (node.width, node.height) == (1640, 1232)
    assert node.cam["exposure"] == 20000

    node._pushed_controls = True
    node._refresh_at = 0.0
    server["frames"] = 100 + 40
    node._server_frames = (100, node._server_frames[1] - 2.0)   # 2 s ago
    node._refresh_from_server()
    assert node.cam["exposure"] == 40000 and node.cam["analogue_gain"] == 2.0
    assert node.cam["contrast"] == 1.5 and node.cam["framerate"] == 20.0
    # The server applies red*colour_gain; the node keeps them apart.
    assert node.cam["red_gain"] == 1.5 and node.cam["blue_gain"] == 2.5
    assert node.exposure_budget == 80000
    assert 19.0 <= node._server_fps <= 20.5, node._server_fps


def test_the_bridge_ingests_only_when_someone_wants_frames():
    node = _bridge({})
    subscribers = [0]
    node.image_pub = types.SimpleNamespace(
        get_subscription_count=lambda: subscribers[0])
    assert node._want_frames() is False, "nobody subscribed: no Pi CPU spent"
    node._af_active = True
    assert node._want_frames() is True, "autofocus scores the frames itself"
    node._af_active, subscribers[0] = False, 1
    assert node._want_frames() is True


# ------------------------------------------------------- relay node
class FakePin:
    """A gpiozero OutputDevice that can be made to throw, per direction."""

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
    """A relay that will not switch off is reported ON (the laser rule)."""
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
    """Publishing after rclpy shutdown raises; the relay must still switch off and release."""

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
