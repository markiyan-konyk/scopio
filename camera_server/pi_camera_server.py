#!/usr/bin/env python3
"""MJPEG stream and control server, the single owner of the Pi camera (see README.md)."""

import io
import os
import json
import time
import socketserver
import threading
from http import server
from threading import Condition

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

# Loopback by default: this server has no auth, so it must never face the LAN.
HOST = os.environ.get("CAM_HOST", "127.0.0.1")
PORT = int(os.environ.get("CAM_PORT", 8081))
SIZE = (int(os.environ.get("CAM_W", 640)), int(os.environ.get("CAM_H", 480)))

picam2 = None
# Why the sensor is not open, and what libcamera last saw; both go in the 503 body.
camera_error = "camera not opened yet"
camera_list = []
_mode_lock = threading.Lock()      # one reconfigure at a time

# The ISP scales the full-FOV detail mode down to this width; MJPEG of more will not keep up on a LAN.
DETAIL_MAX_W = int(os.environ.get("CAM_DETAIL_MAX_W", 1640))

# Last-commanded settings, echoed back by GET /controls (merged with live metadata).
state = {
    "framerate": 30.0, "exposure": 20000, "analogue_gain": 1.0,
    "red_gain": 2.4, "blue_gain": 2.5, "green_gain": 1.0, "colour_gain": 1.0,
    "contrast": 1.0, "saturation": 1.0, "brightness": 0.0, "sharpness": 1.0,
    # Which of the two sensor modes is running, and what it is delivering.
    "mode": os.environ.get("CAM_MODE", "detail"), "width": SIZE[0], "height": SIZE[1],
    "window": SIZE[0],   # sensor pixels across the frame; see pick_modes
}
# Filled in when the sensor opens: {"detail": {...}, "fast": {...}}.
available_modes = {}


def pick_modes(sensor_modes, max_detail_w=None):
    """Pick 'detail' (fastest full-FOV mode) and 'fast' (highest fps) from the advertised modes."""
    modes = [m for m in (sensor_modes or []) if m.get("size")]
    if not modes:
        return {}

    def area(size):
        return size[0] * size[1]

    def window(m):
        crop = m.get("crop_limits")
        return area(m["size"]) if not crop else crop[2] * crop[3]

    def fps(m):
        return float(m.get("fps") or 0.0)

    widest = max(window(m) for m in modes)
    full_fov = [m for m in modes if window(m) == widest]

    def spec(m, cap=None):
        w, h = m["size"]
        if cap and w > cap:                     # scale down, keep the aspect
            w, h = cap, max(1, round(h * cap / m["size"][0]))
        crop = m.get("crop_limits")
        return {"sensor": list(m["size"]), "size": [w, h], "fps": round(fps(m), 1),
                "full_fov": window(m) == widest,
                # um/px depends on window/size, not size alone: a crop keeps the scale.
                "window": [crop[2], crop[3]] if crop else [m["size"][0], m["size"][1]]}

    # The fastest full-FOV mode, not the biggest: 3280x2464 runs near 21 fps and gets scaled down anyway.
    detail = max(full_fov, key=lambda m: (fps(m), area(m["size"])))
    fastest = max(modes, key=lambda m: (fps(m), -area(m["size"])))
    return {"detail": spec(detail, max_detail_w), "fast": spec(fastest)}


_warned_unsupported = set()


def _num(v):
    """Return float(v) if v is a real number, else None (skips NaN / None / bad)."""
    try:
        f = float(v)
        return f if f == f else None      # f != f is True only for NaN
    except (TypeError, ValueError):
        return None


def supported(controls):
    """Keep only the controls this sensor advertises; picamera2 rejects a whole request on one unknown key."""
    known = set(picam2.camera_controls) if picam2 is not None else set()
    dropped = set(controls) - known
    for name in sorted(dropped - _warned_unsupported):
        _warned_unsupported.add(name)
        print(f"Camera does not advertise {name!r}; ignoring it from now on "
              f"(monochrome sensor?)", flush=True)
    return {k: v for k, v in controls.items() if k in known}


def apply_controls(d):
    """Apply a partial dict of settings (framerate, exposure, gains, image controls)."""
    c = {}
    fps = _num(d.get("framerate"))
    if fps:
        # The running mode's own ceiling: libcamera clamps silently, and state would then lie.
        ceiling = (available_modes.get(state["mode"], {}).get("fps") or 120.0)
        fps = max(1.0, min(float(ceiling), fps))
        dur = int(1_000_000 / fps)
        c["FrameDurationLimits"] = (dur, dur)
        state["framerate"] = fps
    exp = _num(d.get("exposure"))
    if exp:
        c["AeEnable"] = False
        c["ExposureTime"] = int(exp)
        state["exposure"] = int(exp)
    ag = _num(d.get("analogue_gain"))
    if ag:
        c["AeEnable"] = False
        c["AnalogueGain"] = ag
        state["analogue_gain"] = ag
    # Gains only if positive: a 0 is a half-filled request, not a black frame (same for `if x:` above).
    red, blue = _num(d.get("red_gain")), _num(d.get("blue_gain"))
    if red or blue:
        r = red if red else state["red_gain"]
        b = blue if blue else state["blue_gain"]
        c["AwbEnable"] = False
        c["ColourGains"] = (r, b)
        state["red_gain"], state["blue_gain"] = r, b
    for key, ctrl in (("contrast", "Contrast"), ("saturation", "Saturation"),
                      ("brightness", "Brightness"), ("sharpness", "Sharpness")):
        val = _num(d.get(key))
        if val is not None:
            c[ctrl] = val
            state[key] = val
    # A frame cannot be shorter than its exposure (a sensor may stall on it): exposure wins, fps gives way.
    if "ExposureTime" in c or "FrameDurationLimits" in c:
        dur = int(1_000_000 / state["framerate"])
        if state["exposure"] > dur:
            dur = state["exposure"] + 500
            state["framerate"] = round(1_000_000 / dur, 2)
            print(f"exposure {state['exposure']} us does not fit the frame; "
                  f"frame rate lowered to {state['framerate']} fps", flush=True)
        c["FrameDurationLimits"] = (dur, dur)

    if c and picam2 is not None:
        picam2.set_controls(supported(c))
    return get_controls()


def do_white_balance():
    """One-shot AWB: enable auto, let it settle, then lock the measured gains; returns them."""
    if "AwbEnable" not in picam2.camera_controls:
        return {"error": "this sensor has no auto white balance "
                         "(monochrome — there are no colour gains to measure)"}
    picam2.set_controls({"AwbEnable": True})
    time.sleep(1.2)
    md = picam2.capture_metadata()
    gains = md.get("ColourGains")
    if not gains:
        return {"error": "no gains reported"}
    r, b = float(gains[0]), float(gains[1])
    picam2.set_controls({"AwbEnable": False, "ColourGains": (r, b)})
    state["red_gain"], state["blue_gain"] = r, b
    return {"red_gain": r, "blue_gain": b}


def _configure(cam, name):
    """Configure `cam` for one of pick_modes()'s modes and start MJPEG recording."""
    spec = available_modes.get(name) or available_modes["detail"]
    fps = spec["fps"] or state["framerate"]
    cam.configure(cam.create_video_configuration(
        sensor={"output_size": tuple(spec["sensor"])},
        main={"size": tuple(spec["size"]), "format": "RGB888"},
        controls={"FrameRate": fps},
    ))
    cam.start_recording(MJPEGEncoder(), FileOutput(output))
    state.update(mode=name, width=spec["size"][0], height=spec["size"][1],
                 window=spec["window"][0], framerate=fps)
    print(f"Camera mode {name}: {spec['size'][0]}x{spec['size'][1]} @ {fps:g} fps "
          f"from sensor {spec['sensor'][0]}x{spec['sensor'][1]}"
          f"{'' if spec['full_fov'] else '  (CROPPED -- narrower field of view)'}",
          flush=True)


def set_mode(name):
    """Switch sensor mode. Returns the new controls, or {'error': ...}."""
    if name not in available_modes:
        return {"error": f"mode must be one of {sorted(available_modes)}"}
    with _mode_lock:
        if picam2 is None:
            return {"error": camera_error or "camera not open"}
        if name == state["mode"]:
            return get_controls()
        previous = state["mode"]
        picam2.stop_recording()
        try:
            _configure(picam2, name)
        except Exception as exc:
            # The sensor is stopped here: restore the old mode, never leave it dark.
            print(f"mode {name} failed ({exc}); restoring {previous}", flush=True)
            _configure(picam2, previous)
            apply_controls({})
            return {"error": f"could not switch to {name}: {exc}; "
                             f"still in {previous}"}
        apply_controls({})      # re-assert exposure/gains onto the new config
    return get_controls()


def get_controls():
    """Commanded settings merged with live metadata; `frames` tells a frozen viewer from a stopped encoder."""
    out = dict(state)
    out["modes"] = available_modes
    out["frames"] = output.frames
    out["frame_age_s"] = (round(time.monotonic() - output.at, 2)
                          if output.at else None)
    try:
        md = picam2.capture_metadata() if picam2 is not None else {}
        if "ExposureTime" in md:
            out["exposure"] = int(md["ExposureTime"])
        if "AnalogueGain" in md:
            out["analogue_gain"] = round(float(md["AnalogueGain"]), 3)
        if md.get("ColourGains"):
            out["red_gain"] = round(float(md["ColourGains"][0]), 3)
            out["blue_gain"] = round(float(md["ColourGains"][1]), 3)
    except Exception:
        pass
    return out


def unavailable():
    """503 body while the sensor is not open; a non-200 keeps the gateway's camera_ok false."""
    return {"error": camera_error, "cameras": camera_list,
            "diagnosis": diagnose(camera_list)}


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.frames = 0          # total encoded; reported by GET /controls
        self.at = 0.0            # monotonic time of the newest frame
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.frames += 1
            self.at = time.monotonic()
            self.condition.notify_all()


output = StreamingOutput()


class Handler(server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _guard(self, fn):
        """Answer 500 with the error instead of dropping the connection."""
        try:
            fn()
        except Exception as exc:
            try:
                self._json({"error": f"{type(exc).__name__}: {exc}"}, 500)
            except Exception:
                pass

    def _reply(self, obj, error_code):
        """200 with obj, or error_code when the handler refused ({'error': ...})."""
        self._json(obj, error_code if isinstance(obj, dict) and "error" in obj
                   else 200)

    def do_GET(self):
        # send_error() already ends the headers; ending them again appends stray bytes.
        if self.path not in ("/", "/stream.mjpg", "/controls", "/focus"):
            self.send_error(404)
        elif picam2 is None:
            self._json(unavailable(), 503)
        elif self.path == "/controls":
            self._guard(lambda: self._json(get_controls()))
        elif self.path == "/focus":
            self._json({"metric": len(output.frame or b"")})
        else:
            self._stream()

    def do_POST(self):
        if self.path not in ("/controls", "/white_balance", "/mode"):
            self.send_error(404)
            return
        if picam2 is None:
            self._json(unavailable(), 503)
            return
        n = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(n) if n else b"{}"
        try:
            d = json.loads(raw or b"{}")
        except ValueError:
            d = {}
        if self.path == "/controls":
            self._guard(lambda: self._json(apply_controls(d)))
        elif self.path == "/mode":
            self._guard(lambda: self._reply(set_mode(str(d.get("mode", ""))), 400))
        else:
            self._guard(lambda: self._reply(do_white_balance(), 409))

    def _stream(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        try:
            while True:
                with output.condition:
                    # Bounded wait: end the response if the encoder stops, rather than hang the client.
                    if not output.condition.wait(timeout=10.0):
                        return
                    frame = output.frame
                if frame:
                    self.wfile.write(b"--FRAME\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def enumerate_cameras():
    """What libcamera can see now; [] means the sensor is not visible to this process."""
    try:
        return Picamera2.global_camera_info()
    except Exception as exc:                       # libcamera itself failed to load
        return [{"error": f"{type(exc).__name__}: {exc}"}]


def diagnose(camera_list):
    """One line naming the problem: libcamera broken, no sensor, or sensor held elsewhere."""
    if camera_list and isinstance(camera_list[0], dict) and "error" in camera_list[0]:
        return ("libcamera itself failed to load -- the container's libcamera "
                "does not match the host kernel's camera stack; use the systemd "
                "fallback (camera_server/install_systemd.sh).")
    if not camera_list:
        return ("libcamera loaded but sees NO sensor -- nothing is holding it, "
                "it is not there. Check the ribbon at BOTH ends, then the host "
                "with the camera service stopped: "
                "`docker compose stop camera && rpicam-hello --list-cameras`.")
    return ("libcamera SEES the sensor but could not open it -- something else "
            "already has it. Exactly one owner is allowed: this service, the "
            "scopio-camera systemd unit (`systemctl status scopio-camera`), or "
            "a stray rpicam-hello -- never two.")


def open_camera_forever():
    """Open the sensor and start recording, retrying in the background until it works."""
    global picam2, camera_error, camera_list, available_modes
    delay = 2.0
    while True:
        cam = None
        try:
            cam = Picamera2()
            available_modes = pick_modes(cam.sensor_modes, DETAIL_MAX_W)
            if not available_modes:
                raise RuntimeError("sensor advertises no usable modes")
            _configure(cam, state["mode"])
            picam2 = cam
            camera_error, camera_list = None, []
            print(f"Modes offered: {available_modes}", flush=True)
            # What this sensor offers: answers "why did that control not take", and mono vs colour.
            print(f"Controls advertised: {sorted(cam.camera_controls)}", flush=True)
            if "AwbEnable" not in cam.camera_controls:
                print("  NOTE: no AwbEnable/ColourGains -- monochrome sensor. "
                      "Colour gains and /white_balance do nothing on this camera.",
                      flush=True)
            return
        except Exception as exc:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
            seen = enumerate_cameras()
            camera_error = f"{type(exc).__name__}: {exc}"
            camera_list = seen
            print(f"Camera unavailable ({camera_error})\n"
                  f"  libcamera sees: {seen}\n"
                  f"  {diagnose(seen)}\n"
                  f"  retrying in {delay:.0f}s", flush=True)
            time.sleep(delay)
            delay = min(delay * 2, 30.0)


def main():
    # Serve first, open the sensor second: a camera fault must be reportable over HTTP.
    threading.Thread(target=open_camera_forever, daemon=True,
                     name="camera-open").start()
    print(f"SCOPIO Pi camera server on http://{HOST}:{PORT} "
          f"(stream /stream.mjpg, controls /controls) {SIZE[0]}x{SIZE[1]}",
          flush=True)
    try:
        StreamingServer((HOST, PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if picam2 is not None:
            picam2.stop_recording()


if __name__ == "__main__":
    main()
