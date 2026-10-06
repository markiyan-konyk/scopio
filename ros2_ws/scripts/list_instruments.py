#!/usr/bin/env python3
"""Which instruments can this machine reach, and how? Run with the stack stopped (see README.md)."""

import glob
import os
import sys
import threading

KNOWN = {0x1AB1: ("Rigol DG1022Z AWG (galvo)", "GALVO_RESOURCE"),
         0x1A45: ("Wavelength TC10 LAB (temperature)", "TCLAB_RESOURCE")}
PROBE_S = 4.0
USBTMC_CLEAR = 0x5B02          # USBTMC_IOCTL_CLEAR = _IO('[', 2)


def usb_vid(resource):
    parts = resource.split("::")
    if len(parts) < 2 or not parts[0].upper().startswith("USB"):
        return None
    try:
        return int(parts[1], 0)
    except ValueError:
        return None


def read_hex(path):
    try:
        with open(path, encoding="ascii") as f:
            return int(f.read().strip(), 16)
    except (OSError, ValueError):
        return None


def usbtmc_vid(node):
    """Vendor id behind /dev/usbtmcN, from sysfs (None if sysfs cannot say)."""
    iface = os.path.realpath(f"/sys/class/usbmisc/{os.path.basename(node)}/device")
    return read_hex(os.path.join(os.path.dirname(iface), "idVendor"))


def with_deadline(fn, seconds=PROBE_S + 2):
    """fn() on a daemon thread; ('ok', value) | ('error', exc) | ('hung', None)."""
    box = {}

    def run():
        try:
            box["value"] = fn()
        except Exception as exc:
            box["error"] = exc
    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(seconds)
    if t.is_alive():
        return "hung", None
    return ("error", box["error"]) if "error" in box else ("ok", box["value"])


def ask_usbtmc(node):
    """CLEAR, then *IDN?: otherwise a dead session's queued reply reads as the identity."""
    fd = os.open(node, os.O_RDWR)
    try:
        try:
            import fcntl
            fcntl.ioctl(fd, USBTMC_CLEAR)
        except (ImportError, OSError):
            pass
        os.write(fd, b"*IDN?\n")
        return os.read(fd, 256).decode(errors="replace").strip()
    finally:
        os.close(fd)


def suggest(idn, resource):
    up = (idn or "").upper()
    if "RIGOL" in up:
        return f"    GALVO_RESOURCE={resource}"
    if "WAVELENGTH" in up or "TC10" in up:
        return f"    TCLAB_RESOURCE={resource}"
    return ""


# --- 1. the USB bus, as the kernel sees it ---------------------------------
print("== on the USB bus (kernel) ==")
on_bus = {}
for id_file in sorted(glob.glob("/sys/bus/usb/devices/*/idVendor")):
    vid = read_hex(id_file)
    if vid in KNOWN:
        on_bus.setdefault(vid, []).append(os.path.basename(os.path.dirname(id_file)))
if not os.path.isdir("/sys/bus/usb/devices"):
    print("  (no Linux sysfs here -- run this on the Pi for a real answer)")
for vid, (name, _) in KNOWN.items():
    where = on_bus.get(vid)
    print(f"  {name:<36} {'PRESENT at ' + ', '.join(where) if where else 'NOT on the bus -- cable / power / hub'}")

# --- 2. kernel-owned instruments: /dev/usbtmcN ------------------------------
print("\n== owned by the kernel usbtmc driver (/dev/usbtmc*) ==")
kernel_owned = set()
nodes = sorted(glob.glob("/dev/usbtmc*"))
if not nodes:
    print("  (none -- no instrument is bound to the kernel usbtmc driver)")
for node in nodes:
    vid = usbtmc_vid(node)
    if vid is not None:
        kernel_owned.add(vid)
    label = KNOWN.get(vid, (f"vendor {vid:04x}" if vid else "vendor unknown",))[0]
    status, value = with_deadline(lambda n=node: ask_usbtmc(n))
    idn = (value if status == "ok" else
           "<no reply within the deadline>" if status == "hung" else
           f"<{type(value).__name__}: {value}>")
    print(f"  {node}  [{label}]  {idn}")
    if suggest(idn, ""):
        print("    (leave the .env line EMPTY: the node finds it here by itself)")

# --- 3. serial ports --------------------------------------------------------
# The stage: a board behind an unrecognised USB bridge is listed here but not auto-detected.
print("\n== serial ports ==")
try:
    from serial.tools import list_ports
    ports = list(list_ports.comports())
except Exception as exc:
    print(f"  ! pyserial unavailable: {type(exc).__name__}: {exc}")
    ports = []
if not ports:
    print("  (none)")
for p in ports:
    ids = f"{p.vid:04x}:{p.pid:04x}" if p.vid else "no USB id"
    print(f"  {p.device}  [{ids}]  {p.description}")
    print(f"    SANGABOARD_PORT={p.device}        # if this is the stage")

# --- 4. VISA (libusb) -------------------------------------------------------
print("\n== VISA over libusb ==")
rm = None
found = []
status, value = with_deadline(lambda: __import__("pyvisa").ResourceManager("@py"))
if status != "ok":
    print(f"  ! VISA unavailable: {value!r}" if status == "error" else
          "  ! creating the VISA resource manager hung")
else:
    rm = value
    status, found = with_deadline(lambda: list(rm.list_resources("?*INSTR")))
    if status != "ok":
        print(f"  ! VISA could not enumerate ({'hung' if status == 'hung' else repr(found)})")
        found = []
    if not found:
        print("  (none)")
    for res in found:
        vid = usb_vid(res)
        if vid not in KNOWN:
            print(f"  {res}  <unknown vendor, not probed>")
            continue
        if vid in kernel_owned:
            print(f"  {res}  <NOT probed: the kernel owns it (see /dev/usbtmc "
                  "above); a VISA open would detach its driver>")
            continue

        def probe(r=res):
            dev = rm.open_resource(r, open_timeout=int(PROBE_S * 1000))
            try:
                dev.timeout = int(PROBE_S * 1000)
                dev.read_termination = dev.write_termination = "\n"
                try:
                    dev.clear()
                except Exception:
                    pass
                return dev.query("*IDN?").strip()
            finally:
                dev.close()
        status, idn = with_deadline(probe)
        if status == "hung":
            idn = "<HUNG inside libusb -- replug the instrument before retrying>"
        elif status == "error":
            idn = f"<{type(idn).__name__}: {idn}>"
        print(f"  {res}  {idn}")
        print(f"    (leave {KNOWN[vid][1]} EMPTY: the node finds it by vendor id)")

for vid, where in on_bus.items():
    if vid not in kernel_owned and not any(usb_vid(r) == vid for r in found):
        print(f"\n  ! {KNOWN[vid][0]} is on the bus but neither the kernel nor "
              "VISA lists it: libusb cannot read its descriptors -- install "
              "ros2_ws/udev and replug, or stop whatever else holds it")

# --- 5. Ethernet units named on the command line ----------------------------
hosts = [a for a in sys.argv[1:] if not a.startswith("-")]
if hosts and rm is not None:
    print("\n== Ethernet ==")
for host in hosts:
    if rm is None:
        break
    # VXI-11 first, then a raw SCPI socket: instruments implement one or the
    # other and there is no way to tell from the outside which.
    for res in (f"TCPIP::{host}::INSTR", f"TCPIP::{host}::5025::SOCKET"):
        def probe(r=res):
            dev = rm.open_resource(r, open_timeout=int(PROBE_S * 1000))
            try:
                dev.timeout = int(PROBE_S * 1000)
                dev.read_termination = dev.write_termination = "\n"
                return dev.query("*IDN?").strip()
            finally:
                dev.close()
        status, idn = with_deadline(probe)
        if status != "ok":
            print(f"  {res}  <{'hung' if status == 'hung' else repr(idn)}>")
            continue
        print(f"  {res}  {idn}")
        print(suggest(idn, res) or f"    TCLAB_RESOURCE={res}")
        break

if not hosts:
    print("\n(pass an IP to probe an Ethernet instrument: "
          "list_instruments.py 192.168.1.50)")
