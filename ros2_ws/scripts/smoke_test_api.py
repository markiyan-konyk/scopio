#!/usr/bin/env python3
"""End-to-end smoke test of the SCOPIO API gateway.

Needs only `pip install requests websocket-client` (or the scopio_client SDK's
deps). Run against:

  * the no-hardware dev stack (docker-compose.dev.yml) on a laptop:
        python3 scripts/smoke_test_api.py --url http://127.0.0.1:8000
    (use 127.0.0.1, not localhost -- on Windows/Docker Desktop the WebSocket
    connect can hang on the IPv6 localhost route)
  * the real Pi (add --hardware once the rig is attached):
        python3 scripts/smoke_test_api.py --url http://<pi>:8000 --hardware

The key is read from --key, $SCOPIO_API_KEY, or ros2_ws/secrets/api_keys.json
(first key found), in that order.

Without --hardware it asserts the DEGRADED behaviors (this is deliberate: it
proves the full JSON->gateway->ROS->response path against nodes that have no
hardware). With --hardware it additionally does a +-0-step stage jog
round-trip, an AWG *IDN? query, and checks live MJPEG frames.
"""

import argparse
import json
import os
import sys
import time

import requests

PASS, FAIL = "\033[92mPASS\033[0m", "\033[91mFAIL\033[0m"
_failures = []


def check(name, ok, detail=""):
    print(f"  [{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        _failures.append(name)


def find_key(args):
    if args.key:
        return args.key
    if os.environ.get("SCOPIO_API_KEY"):
        return os.environ["SCOPIO_API_KEY"]
    keys_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "secrets", "api_keys.json")
    try:
        with open(keys_file, encoding="utf-8") as f:
            keys = json.load(f)
        if keys:
            name = sorted(keys)[0]
            print(f"(using key '{name}' from {keys_file})")
            return keys[name]
    except (OSError, ValueError):
        pass
    sys.exit("No API key: pass --key, set SCOPIO_API_KEY, or run "
             "scripts/generate_api_key.py first.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--key", default=None)
    ap.add_argument("--hardware", action="store_true",
                    help="expect real hardware (stage/AWG/camera attached)")
    args = ap.parse_args()
    base = args.url.rstrip("/")
    key = find_key(args)
    H = {"X-API-Key": key}

    print(f"\n== SCOPIO API smoke test against {base} ==\n")

    # 1. health (no auth)
    r = requests.get(f"{base}/api/v1/health", timeout=10)
    h = r.json()
    check("health reachable", r.status_code == 200, str(h))
    check("ros_ok", h.get("ros_ok") is True)
    check("auth_configured", h.get("auth_configured") is True)
    if args.hardware:
        check("camera_ok", h.get("camera_ok") is True)

    # 2. auth enforcement
    r = requests.get(f"{base}/api/v1/status", timeout=10)
    check("no key -> 401", r.status_code == 401, f"got {r.status_code}")
    r = requests.get(f"{base}/api/v1/status", headers=H, timeout=10)
    check("with key -> 200", r.status_code == 200, f"got {r.status_code}")
    status = r.json() if r.status_code == 200 else {}

    # 3. interfaces discovery
    r = requests.get(f"{base}/api/v1/interfaces", headers=H, timeout=15)
    ifs = r.json() if r.status_code == 200 else {}
    services = ifs.get("services", {})
    actions = ifs.get("actions", {})
    for s in ("/scopio/stage/jog", "/scopio/awg/write", "/scopio/awg/query",
              "/scopio/awg/call", "/scopio/temperature/call",
              "/scopio/camera/set_controls", "/scopio/calibration/set"):
        check(f"interfaces lists {s}", s in services)
    for a in ("/scopio/camera/autofocus", "/scopio/stage/move_path",
              "/scopio/scan_region"):
        check(f"interfaces lists action {a}", a in actions)

    # 4. generic service path end-to-end (works with or without hardware)
    r = requests.post(f"{base}/api/v1/service/awg/query", headers=H,
                      json={"command": "*IDN?"}, timeout=20)
    resp = r.json() if r.status_code == 200 else {}
    check("awg/query returns 200", r.status_code == 200, f"got {r.status_code}: {r.text[:120]}")
    if args.hardware:
        check("AWG answers *IDN?", resp.get("success") is True, str(resp.get("response", ""))[:60])
    else:
        check("AWG degraded gracefully", resp.get("success") is False,
              str(resp.get("error", ""))[:60])

    # 4b. the instrument-call surface. list_methods is answered by the NODE
    #     (introspecting the driver class), so it proves the whole path even
    #     with no controller attached -- and tells you the node is the version
    #     you think it is.
    r = requests.post(f"{base}/api/v1/service/temperature/call", headers=H,
                      json={"method": "list_methods"}, timeout=20)
    resp = r.json() if r.status_code == 200 else {}
    check("temperature/call reachable", r.status_code == 200,
          f"got {r.status_code}: {r.text[:120]}")
    methods = json.loads(resp.get("result") or "[]") if resp.get("success") else []
    check("temperature exposes the driver class", len(methods) > 50,
          f"{len(methods)} methods")
    check("temperature exposes set_setpoint",
          any(m.get("name") == "set_setpoint" for m in methods))

    r = requests.post(f"{base}/api/v1/service/temperature/call", headers=H,
                      json={"method": "connected"}, timeout=20)
    resp = r.json() if r.status_code == 200 else {}
    connected = resp.get("result") == "true"
    if args.hardware:
        check("TC LAB connected", connected,
              "set TCLAB_RESOURCE in ros2_ws/.env, then docker compose up -d")
        r = requests.post(f"{base}/api/v1/service/temperature/call", headers=H,
                          json={"method": "temperature"}, timeout=20)
        resp = r.json() if r.status_code == 200 else {}
        check("TC LAB reads a temperature", resp.get("success") is True,
              str(resp.get("result") or resp.get("error"))[:60])
    else:
        check("temperature degraded gracefully", not connected,
              "reports connected=false without hardware, as it should")

    # unknown service -> 404
    r = requests.post(f"{base}/api/v1/service/no/such/service", headers=H,
                      json={}, timeout=10)
    check("unknown service -> 404", r.status_code == 404, f"got {r.status_code}")

    # bad fields -> 422
    r = requests.post(f"{base}/api/v1/service/stage/jog", headers=H,
                      json={"bogus_field": 1}, timeout=10)
    check("bad fields -> 422", r.status_code == 422, f"got {r.status_code}")

    # 5. the NaN rule: null on float fields must not error and must mean
    #    "leave unchanged" (response success=False without hardware is fine --
    #    the point is it parses and reaches the node).
    body = {"contrast": 1.2, "red_gain": None, "green_gain": None,
            "blue_gain": None, "colour_gain": None, "analogue_gain": None,
            "saturation": None, "brightness": None, "sharpness": None}
    r = requests.post(f"{base}/api/v1/service/camera/set_controls", headers=H,
                      json=body, timeout=15)
    check("null->NaN accepted on set_controls", r.status_code == 200,
          f"got {r.status_code}: {r.text[:120]}")

    # 6. WebSocket: telemetry, latched topic, video refusal, action lifecycle
    try:
        import websocket
    except ImportError:
        check("websocket-client installed", False, "pip install websocket-client")
        _summary()
        return

    ws_url = base.replace("http", "ws", 1) + f"/api/v1/ws?api_key={key}"
    ws = websocket.create_connection(ws_url, timeout=15)

    def send(d):
        ws.send(json.dumps(d))

    def recv_until(pred, timeout=15):
        deadline = time.time() + timeout
        while time.time() < deadline:
            env = json.loads(ws.recv())
            if pred(env):
                return env
        return None

    # stage/position streams
    send({"op": "subscribe", "id": "s1", "topic": "stage/position", "rate_hz": 10})
    env = recv_until(lambda e: e.get("id") == "s1" and e.get("op") in ("ok", "error"))
    check("subscribe stage/position ok", env and env["op"] == "ok", str(env))
    got = 0
    deadline = time.time() + 6
    while got < 3 and time.time() < deadline:
        env = json.loads(ws.recv())
        if env.get("op") == "message" and env.get("id") == "s1":
            got += 1
    check("received >=3 stage/position messages", got >= 3, f"got {got}")
    send({"op": "unsubscribe", "id": "s1"})
    recv_until(lambda e: e.get("id") == "s1" and e.get("op") in ("ok", "error"))

    # latched calibration delivers immediately
    send({"op": "subscribe", "id": "s2", "topic": "calibration"})
    env = recv_until(lambda e: e.get("id") == "s2" and e.get("op") == "message",
                     timeout=8)
    check("latched calibration delivered on subscribe", env is not None,
          str(env.get("msg") if env else None)[:80])
    send({"op": "unsubscribe", "id": "s2"})
    recv_until(lambda e: e.get("id") == "s2" and e.get("op") in ("ok", "error"))

    # video topics are refused
    send({"op": "subscribe", "id": "s3", "topic": "image/compressed"})
    env = recv_until(lambda e: e.get("id") == "s3")
    check("image/compressed -> use_mjpeg error",
          env and env.get("op") == "error" and env.get("code") == "use_mjpeg",
          str(env))

    # autofocus action lifecycle
    send({"op": "action_send_goal", "id": "g1", "action": "camera/autofocus",
          "goal": {"z_range": 200, "steps": 3, "settle_s": 0.0}})
    env = recv_until(lambda e: e.get("id") == "g1"
                     and e.get("op") in ("action_ack", "error"), timeout=20)
    check("autofocus goal acknowledged",
          env and env.get("op") == "action_ack" and env.get("accepted"), str(env))
    env = recv_until(lambda e: e.get("id") == "g1" and e.get("op") == "action_result",
                     timeout=120)
    if args.hardware:
        check("autofocus succeeded", env and env.get("status") == "succeeded", str(env))
    else:
        check("autofocus aborted cleanly (no camera)",
              env and env.get("status") == "aborted",
              str((env or {}).get("result", {}).get("message"))[:60])
    ws.close()

    # 7. stage jog round-trip + MJPEG (hardware only)
    if args.hardware:
        r = requests.post(f"{base}/api/v1/service/stage/jog", headers=H,
                          json={"dx": 0, "dy": 0, "dz": 0}, timeout=15)
        resp = r.json()
        check("stage jog(0,0,0) round-trip", resp.get("success") is True, str(resp))

        r = requests.get(f"{base}/api/v1/stream.mjpg", headers=H, stream=True,
                         timeout=(10, 10))
        chunk = next(r.iter_content(chunk_size=65536), b"")
        check("MJPEG stream delivers bytes", len(chunk) > 1000, f"{len(chunk)} bytes")
        r.close()
    else:
        r = requests.get(f"{base}/api/v1/stream.mjpg", headers=H, timeout=10)
        check("stream 503s cleanly without camera", r.status_code == 503,
              f"got {r.status_code}")

    # status telemetry sanity
    tel = status.get("telemetry", {})
    check("status caches stage/position", tel.get("stage/position") is not None)
    check("status caches temperature/status", tel.get("temperature/status") is not None,
          "the temperature node isn't publishing -- is it in the launch file / image?")
    if args.hardware:
        t = (tel.get("temperature/status") or {}).get("msg", {})
        check("temperature telemetry shows connected", t.get("connected") is True,
              f"last_error: {str(t.get('last_error'))[:60]}")

    _summary()


def _summary():
    print()
    if _failures:
        print(f"{len(_failures)} FAILURE(S): {_failures}")
        sys.exit(1)
    print("All checks passed.")


if __name__ == "__main__":
    main()
