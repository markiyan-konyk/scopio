"""FastAPI application: the SCOPIO microscope's public API surface.

All routes live under /api/v1. Interactive docs at /docs (they double as a
live, always-correct reference for the curated routes; the generic
service/topic/action surface is documented in docs/API.md and discoverable at
GET /api/v1/interfaces).

Route map (auth = X-API-Key header or ?api_key= unless noted):

  GET  /api/v1/health                 no auth -- liveness for scripts/monitors
  GET  /api/v1/interfaces             discover services/topics/actions + schemas
  GET  /api/v1/status                 one-call snapshot of the whole microscope
  POST /api/v1/service/{path}         GENERIC: call any ROS service as JSON
  WS   /api/v1/ws                     topic streams + actions (see ws.py)
  GET  /api/v1/stream.mjpg            live camera video (MJPEG)
  GET  /api/v1/camera/controls        current camera settings
  POST /api/v1/camera/controls        set camera settings (partial JSON)
  POST /api/v1/camera/white_balance   one-shot AWB
  GET  /api/v1/camera/focus           cheap focus metric
"""

import asyncio

from fastapi import Body, Depends, FastAPI, HTTPException, Query, Request

from . import camera_proxy
from .auth import keystore, require_api_key
from .introspection import interfaces_payload
from .ros_bridge import UnknownInterface, bridge
from .ws import websocket_endpoint

app = FastAPI(
    title="SCOPIO Microscope API",
    version="1.0",
    description=(
        "HTTP/WebSocket gateway to the SCOPIO self-driving-lab microscope. "
        "Generic endpoints mirror the frozen ROS 2 interface contract "
        "(scopio_interfaces); see docs/API.md in the repo for the full manual "
        "and GET /api/v1/interfaces for live discovery."
    ),
)


@app.on_event("startup")
async def _capture_loop():
    # The rclpy executor thread needs a handle on this loop to hand results
    # and telemetry back to request handlers / websocket queues.
    bridge.loop = asyncio.get_running_loop()


@app.get("/api/v1/health")
async def health():
    return {
        "ok": bridge.ok,
        "ros_ok": bridge.ok,
        "camera_ok": await camera_proxy.camera_ok(),
        "auth_configured": keystore.configured,
        "uptime_s": round(bridge.uptime_s, 1),
    }


@app.get("/api/v1/interfaces", dependencies=[Depends(require_api_key)])
async def interfaces():
    try:
        return interfaces_payload(bridge)
    except RuntimeError as exc:
        raise HTTPException(503, str(exc))


@app.get("/api/v1/status", dependencies=[Depends(require_api_key)])
async def status():
    return {
        "telemetry": bridge.telemetry_snapshot(),
        "camera_ok": await camera_proxy.camera_ok(),
        "gateway_uptime_s": round(bridge.uptime_s, 1),
    }


@app.post("/api/v1/service/{service_path:path}",
          dependencies=[Depends(require_api_key)])
async def call_service(
    service_path: str,
    body: dict = Body(default={}),
    timeout: float = Query(default=10.0, gt=0, le=120),
):
    """Call any ROS 2 service on the microscope.

    `service_path` is relative to /scopio (e.g. `stage/jog`); the JSON body
    maps to the service's request fields (see /api/v1/interfaces). A float
    field you leave out is sent as NaN, which every SCOPIO service reads as
    "leave this one alone" -- so a partial body is safe (see conversion.py).
    """
    try:
        return await bridge.call_service(service_path, body, timeout=timeout)
    except UnknownInterface as exc:
        raise HTTPException(404, f"No such service in the graph: {exc}")
    except ValueError as exc:
        raise HTTPException(422, f"Bad request fields: {exc}")
    except asyncio.TimeoutError:
        raise HTTPException(504, f"Service call timed out after {timeout}s "
                                 "(is the node running and the hardware alive?)")
    except RuntimeError as exc:
        raise HTTPException(503, str(exc))


# ------------------------------------------------------------------ camera
@app.get("/api/v1/stream.mjpg", dependencies=[Depends(require_api_key)])
async def stream_mjpg():
    """Live MJPEG video. Usable directly as an <img src=...> (append
    ?api_key=...) or ingested programmatically (scopio_client.stream_frames)."""
    return await camera_proxy.mjpeg_stream()


@app.get("/api/v1/camera/controls", dependencies=[Depends(require_api_key)])
async def get_camera_controls():
    return await camera_proxy.forward("GET", "/controls")


@app.post("/api/v1/camera/controls", dependencies=[Depends(require_api_key)])
async def set_camera_controls(body: dict = Body(default={})):
    return await camera_proxy.forward("POST", "/controls", json_body=body)


@app.post("/api/v1/camera/white_balance", dependencies=[Depends(require_api_key)])
async def white_balance():
    return await camera_proxy.forward("POST", "/white_balance", json_body={})


@app.get("/api/v1/camera/focus", dependencies=[Depends(require_api_key)])
async def camera_focus():
    return await camera_proxy.forward("GET", "/focus")


app.websocket("/api/v1/ws")(websocket_endpoint)


@app.get("/")
async def root(request: Request):
    return {
        "name": "SCOPIO Microscope API",
        "docs": str(request.base_url) + "docs",
        "manual": "docs/API.md in the repository",
        "health": str(request.base_url) + "api/v1/health",
    }
