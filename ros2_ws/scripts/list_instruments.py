#!/usr/bin/env python3
"""Debug tool: list every instrument this machine can reach, in all three forms
TCLAB_RESOURCE / GALVO_RESOURCE accept. Prints ready-to-paste .env lines.

    docker compose down            # a running node holds its instrument open
    python3 scripts/list_instruments.py
    python3 scripts/list_instruments.py 192.168.1.50    # also probe an Ethernet unit

The nodes auto-discover and do not need this. Use it when an instrument is not
showing up, or to get an address for ros2_ws/.env.

An Ethernet unit cannot be discovered -- pyvisa-py cannot scan a LAN -- so pass
its IP. To learn that IP, ask the instrument over USB (it knows):

    echo 'TECH:IPADDR?' > /dev/usbtmc0 && head -c 100 /dev/usbtmc0

or look its MAC (TECH:HWADDR?) up in the router's DHCP leases.
"""

import glob
import os
import sys

KNOWN = {0x1AB1: "GALVO_RESOURCE", 0x1A45: "TCLAB_RESOURCE"}
PROBE_MS = 4000


def usb_vid(resource):
    parts = resource.split("::")
    if len(parts) < 2 or not parts[0].upper().startswith("USB"):
        return None
    try:
        return int(parts[1], 0)
    except ValueError:
        return None


def suggest(idn, resource):
    """The .env line for whichever instrument this is."""
    up = (idn or "").upper()
    if "RIGOL" in up or "DG10" in up:
        return f"    GALVO_RESOURCE={resource}"
    if "WAVELENGTH" in up or "TC10" in up:
        return f"    TCLAB_RESOURCE={resource}"
    return ""


# --- 1. kernel usbtmc char devices -----------------------------------------
# The transport the TC10 needs when the kernel driver has claimed it, which is
# what makes a plain VISA scan hang on that instrument.
print("== kernel usbtmc char devices ==")
nodes = sorted(glob.glob("/dev/usbtmc*"))
if not nodes:
    print("  (none -- no USB-TMC instrument is bound to the kernel driver)")
for path in nodes:
    try:
        fd = os.open(path, os.O_RDWR)
        try:
            os.write(fd, b"*IDN?\n")
            idn = os.read(fd, 256).decode(errors="replace").strip()
        finally:
            os.close(fd)
    except OSError as exc:
        idn = f"<{type(exc).__name__}: {exc}>"
    print(f"  {path}  {idn}")
    hint = suggest(idn, "/dev/usbtmc*")
    if hint:
        print(hint + "        # a GLOB: the node probes each and keeps this one")

# --- 2. serial ports --------------------------------------------------------
# The Sangaboard stage lives here, and its library auto-detects by USB
# vendor/product id -- a board behind an unrecognised bridge (CH340, FTDI) is
# present but not found, which is why SANGABOARD_PORT exists.
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

# --- 3. VISA resources ------------------------------------------------------
print("\n== VISA resources ==")
try:
    import pyvisa
    rm = pyvisa.ResourceManager("@py")
    found = list(rm.list_resources())
except Exception as exc:
    print(f"  ! VISA could not enumerate: {type(exc).__name__}: {exc}")
    found, rm = [], None

if rm is not None and not found:
    print("  (none)")
for res in found:
    vid = usb_vid(res)
    if vid not in KNOWN:
        print(f"  {res}  <unknown vendor, not probed>")
        continue
    try:
        dev = rm.open_resource(res, open_timeout=PROBE_MS)
        dev.timeout = PROBE_MS
        idn = dev.query("*IDN?").strip()
        dev.close()
    except Exception as exc:
        idn = f"<{type(exc).__name__}: {exc}>"
    print(f"  {res}  {idn}")
    print(f"    {KNOWN[vid]}={res}")

# --- 4. Ethernet units named on the command line ----------------------------
hosts = [a for a in sys.argv[1:] if not a.startswith("-")]
if hosts and rm is not None:
    print("\n== Ethernet ==")
for host in hosts:
    if rm is None:
        break
    # VXI-11 first, then a raw SCPI socket: instruments implement one or the
    # other and there is no way to tell from the outside which.
    for res in (f"TCPIP::{host}::INSTR", f"TCPIP::{host}::5025::SOCKET"):
        try:
            dev = rm.open_resource(res, open_timeout=PROBE_MS)
            dev.timeout = PROBE_MS
            dev.read_termination = dev.write_termination = "\n"
            idn = dev.query("*IDN?").strip()
            dev.close()
        except Exception as exc:
            print(f"  {res}  <{type(exc).__name__}: {exc}>")
            continue
        print(f"  {res}  {idn}")
        print(suggest(idn, res) or f"    TCLAB_RESOURCE={res}")
        break

if not hosts:
    print("\n(pass an IP to probe an Ethernet instrument: "
          "list_instruments.py 192.168.1.50)")
