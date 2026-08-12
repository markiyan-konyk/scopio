#!/usr/bin/env python3
"""Camera stream + control server for the Raspberry Pi (SCOPIO).

WHY THIS EXISTS
---------------
picamera2/libcamera ship from Raspberry Pi OS, NOT Ubuntu, so the camera cannot
live inside the Ubuntu ROS container. It lives here instead -- the SINGLE OWNER
of the sensor -- and everything else consumes it over loopback HTTP:

  GET  /stream.mjpg   -> live MJPEG video
  GET  /controls      -> current camera settings (incl. live exposure/gain)
  POST /controls      -> set framerate/exposure/gain/colour/contrast/... (JSON)
  POST /white_balance -> one-shot auto white balance; locks the measured gains
  GET  /focus         -> a focus metric (JPEG size)

Consumers (both on 127.0.0.1 -- this server is deliberately loopback-only):
  * the API gateway (scopio_gateway) proxies /stream.mjpg and the controls to
    authenticated external clients;
  * camera_node (ROS) ingests the stream in BRIDGE mode and republishes it on
    image/compressed + forwards the camera services here.

HOW IT RUNS (pick one; identical HTTP surface either way):
  * as the `camera` service in ros2_ws/docker-compose.yml (default -- comes up
    with the rest of the backend automatically), or
  * as a systemd unit on the Pi host if libcamera misbehaves in-container:
        camera_server/install_systemd.sh

Env: CAM_HOST (127.0.0.1), CAM_PORT (8081), CAM_W (640), CAM_H (480).
Requires: python3-picamera2 (Raspberry Pi OS Bookworm / the RPi apt archive).
"""

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

# Loopback by default: this server has NO auth, so it must never face the LAN.
# The API gateway is the authenticated front door.
HOST = os.environ.get("CAM_HOST", "127.0.0.1")
PORT = int(os.environ.get("CAM_PORT", 8081))
SIZE = (int(os.environ.get("CAM_W", 640)), int(os.environ.get("CAM_H", 480)))

picam2 = None
# Why the sensor is not open, and what libcamera could actually see when it was
# last tried. Both are reported in the 503 body -- an empty camera list is THE
# diagnostic (picamera2 imported fine, libcamera loaded, the sensor just is not
# visible to this process).
camera_error = "camera not opened yet"
camera_list = []

# Last-commanded settings, echoed back by GET /controls (merged with live metadata).
state = {
    "framerate": 30.0, "exposure": 20000, "analogue_gain": 1.0,
    "red_gain": 2.4, "blue_gain": 2.5, "green_gain": 1.0, "colour_gain": 1.0,
    "contrast": 1.0, "saturation": 1.0, "brightness": 0.0, "sharpness": 1.0,
}


_warned_unsupported = set()


def _num(v):
    """Return float(v) if v is a real number, else None (skips NaN / None / bad)."""
    try:
        f = float(v)
        return f if f == f else None      # f != f is True only for NaN
    except (TypeError, ValueError):
        return None


def supported(controls):
    """Keep only the controls THIS sensor advertises.

    Not every camera has every control: a monochrome sensor has no AwbEnable or
    ColourGains (no Bayer filter, so there is nothing to white-balance), and some
    sensors lack Saturation or Sharpness. picamera2 raises on the FIRST unknown
    key, so one unsupported control used to reject the whole request -- and the
    caller then retried it forever. Dropping is right: the request is still
    meaningful, that one knob simply does not exist on this hardware.
    """
    known = set(picam2.camera_controls) if picam2 is not None else set()
    dropped = set(controls) - known
    for name in sorted(dropped - _warned_unsupported):
        _warned_unsupported.add(name)
        print(f"Camera does not advertise {name!r}; ignoring it from now on "
              f"(monochrome sensor?)", flush=True)
    return {k: v for k, v in controls.items() if k in known}


def apply_controls(d):
    """Apply a partial dict of settings to the camera. Keys match the UI:
    framerate, exposure, analogue_gain, red_gain, blue_gain, contrast,
    saturation, brightness, sharpness."""
    c = {}
    fps = _num(d.get("framerate"))
    if fps:
        fps = max(1.0, min(120.0, fps))
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
    # Gains only if POSITIVE: a 0 here is a half-filled request, not a request
    # for a black frame. (Same reason fps/exposure/gain above use `if x:`.)
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
    # Exposure and frame duration must stay coherent: a frame cannot be shorter
    # than its own exposure. Pinning FrameDurationLimits below ExposureTime asks
    # the sensor for something impossible, and a sensor may stall on that rather
    # than clamp -- one frame, then nothing. camera_node's fps budget keeps them
    # consistent, but a client POSTing here goes straight past that, so enforce
    # it at the only place that owns the camera. Exposure wins; fps gives way.
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
    """One-shot AWB: enable auto, let it settle, read the measured colour gains,
    then lock them in as manual gains (so they don't drift). Returns the gains."""
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


def get_controls():
    """Current settings, merging commanded state with live camera metadata.

    `frames` and `frame_age_s` are the diagnosis when video freezes: if frames
    keeps climbing while a viewer is stuck, the sensor is fine and the problem
    is downstream (proxy, browser). If it stops climbing, the ENCODER stopped
    and the cause is here or in the camera.
    """
    out = dict(state)
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
    """503 body for every endpoint while the sensor is not open. Keeping this a
    non-200 is what preserves the gateway's `camera_ok: false` (camera_proxy
    only checks the status of GET /controls) now that the server itself stays
    up instead of crash-looping."""
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
        """Answer with a 500 instead of dying. A handler that raises leaves the
        socket hung up, so the client sees "connection reset" and never learns
        which control the camera rejected -- and the stack trace lands in the
        container log instead of in the reply."""
        try:
            fn()
        except Exception as exc:
            try:
                self._json({"error": f"{type(exc).__name__}: {exc}"}, 500)
            except Exception:
                pass

    def do_GET(self):
        if self.path not in ("/", "/stream.mjpg", "/controls", "/focus"):
            self.send_error(404)
            self.end_headers()
        elif picam2 is None:
            self._json(unavailable(), 503)
        elif self.path == "/controls":
            self._guard(lambda: self._json(get_controls()))
        elif self.path == "/focus":
            self._json({"metric": len(output.frame or b"")})
        else:
            self._stream()

    def do_POST(self):
        if self.path not in ("/controls", "/white_balance"):
            self.send_error(404)
            self.end_headers()
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
        else:
            self._guard(lambda: self._json(do_white_balance()))

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
                    # Bounded wait: if the encoder stops delivering (sensor
                    # yanked mid-stream), end the response instead of holding
                    # the client open forever with no way to tell it apart from
                    # a slow camera.
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
    """What libcamera can see right now. [] means the sensor is not visible to
    this process -- the cable/CSI port, the host's config.txt, or (in the
    container) a libcamera that does not match the host kernel's camera stack."""
    try:
        return Picamera2.global_camera_info()
    except Exception as exc:                       # libcamera itself failed to load
        return [{"error": f"{type(exc).__name__}: {exc}"}]


def diagnose(camera_list):
    """The one line that says which problem you actually have.

    Note what the host check is NOT: `rpicam-hello` fails with "Pipeline
    handler in use by another process" whenever THIS server holds the camera,
    which is whenever things are working. Stop this service before running it,
    or you are just watching your own stack own the sensor.
    """
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
    """Open the sensor and start MJPEG recording, retrying until it works.

    The sensor is NOT a precondition for serving. Raising out of here used to
    kill the process, which under `restart: unless-stopped` crash-loops the
    container: the HTTP surface never comes up, so /controls cannot say what
    went wrong and the gateway reports a bare `camera_ok: false`. Retrying in
    the background instead matches how every ROS node in this backend degrades,
    and a camera that appears late (replug, or the systemd unit releasing it)
    heals with no restart.
    """
    global picam2, camera_error, camera_list
    delay = 2.0
    while True:
        try:

            cam = Picamera2()
            for i, mode in enumerate(cam.sensor_modes):
              print(f"SENSOR MODE {i}: {mode}", flush=True)
            
            full_fov_mode = cam.sensor_modes[5]
            
            config = cam.create_video_configuration(
                sensor={
                    "output_size": full_fov_mode["size"],
                    "bit_depth": full_fov_mode["bit_depth"],
                },
                main={
                    "size": SIZE,
                    "format": "RGB888",
                },
                controls={"FrameRate": 30},
            )
            
            cam.configure(config)
            
            # Explicitly request the complete sensor area.
            crop = full_fov_mode.get("crop_limits")
            if crop:
                cam.set_controls({"ScalerCrop": crop})
            
            cam.start_recording(MJPEGEncoder(), FileOutput(output))
            picam2 = cam
            camera_error, camera_list = None, []
            # Print what this sensor actually offers: it is the fastest answer to
            # "why did that control not take" and it says mono vs colour outright.
            print(f"Camera open {SIZE[0]}x{SIZE[1]}; controls advertised: "
                  f"{sorted(cam.camera_controls)}", flush=True)
            if "AwbEnable" not in cam.camera_controls:
                print("  NOTE: no AwbEnable/ColourGains -- monochrome sensor. "
                      "Colour gains and /white_balance do nothing on this camera.",
                      flush=True)
            return
        except Exception as exc:
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
    # Serve FIRST, open the sensor second: a camera fault must be reportable
    # over HTTP, not a reason nothing answers at all.
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
