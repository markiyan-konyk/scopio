"""Authenticated front door for the loopback-only camera server.

The picamera2 camera server (camera_server/pi_camera_server.py) binds to
127.0.0.1:8081 -- it has no auth of its own, so it must never face the LAN.
The gateway proxies it:

    GET  /api/v1/stream.mjpg          -> chunk passthrough of /stream.mjpg
                                         (no decode, no re-encode -- cheap)
    GET  /api/v1/camera/controls      -> /controls
    POST /api/v1/camera/controls      -> /controls
    POST /api/v1/camera/white_balance -> /white_balance
    GET  /api/v1/camera/focus         -> /focus

CAMERA_URL env selects the upstream (default http://127.0.0.1:8081). Set it
empty to declare "no camera" -- endpoints then 503 cleanly and /health says
camera_ok=false (used by the no-hardware dev compose).
"""

import os
import time

import httpx
from fastapi import HTTPException
from fastapi.responses import StreamingResponse

CAMERA_URL = os.environ.get("CAMERA_URL", "http://127.0.0.1:8081").rstrip("/")

_client = None
_health_cache = {"ok": False, "at": 0.0}


def _http():
    global _client
    if _client is None:
        _client = httpx.AsyncClient(timeout=httpx.Timeout(5.0, read=30.0))
    return _client


async def camera_ok():
    """Cheap cached camera-server reachability check (3 s cache)."""
    if not CAMERA_URL:
        return False
    now = time.time()
    if now - _health_cache["at"] < 3.0:
        return _health_cache["ok"]
    try:
        r = await _http().get(f"{CAMERA_URL}/controls", timeout=1.5)
        ok = r.status_code == 200
    except httpx.HTTPError:
        ok = False
    _health_cache.update(ok=ok, at=now)
    return ok


def _require_camera():
    if not CAMERA_URL:
        raise HTTPException(503, "No camera server configured (CAMERA_URL is empty).")


async def forward(method, path, json_body=None):
    """Forward a small JSON request to the camera server."""
    _require_camera()
    try:
        r = await _http().request(method, f"{CAMERA_URL}{path}", json=json_body)
    except httpx.HTTPError as exc:
        raise HTTPException(503, f"Camera server unreachable: {exc}") from exc
    try:
        return r.json()
    except ValueError:
        raise HTTPException(502, "Camera server returned non-JSON.")


async def mjpeg_stream():
    """StreamingResponse that passes the multipart MJPEG bytes through."""
    _require_camera()
    url = f"{CAMERA_URL}/stream.mjpg"

    # Probe first so a dead camera yields a clean 503 instead of an empty stream.
    if not await camera_ok():
        raise HTTPException(503, "Camera server unreachable.")

    async def _gen():
        async with _http().stream("GET", url, timeout=None) as resp:
            async for chunk in resp.aiter_raw():
                yield chunk

    # The camera server always uses this boundary (see pi_camera_server.py).
    return StreamingResponse(_gen(),
                             media_type="multipart/x-mixed-replace; boundary=FRAME")
