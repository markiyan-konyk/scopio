"""FastAPI routes of the SCOPIO API, all under /api/v1 (see README.md and docs/API.md)."""

import asyncio
from contextlib import asynccontextmanager

from fastapi import Body, Depends, FastAPI, HTTPException, Query, Request

from . import camera_proxy
from .auth import keystore, require_api_key
from .introspection import interfaces_payload
from .ros_bridge import UnknownInterface, bridge
from .ws import websocket_endpoint


@asynccontextmanager
async def _lifespan(_app):
    # The rclpy thread needs this loop; a lifespan, since on_event is deprecated and FastAPI is unpinned.
    bridge.loop = asyncio.get_running_loop()
    yield


app = FastAPI(
    lifespan=_lifespan,
    title="SCOPIO Microscope API",
    version="1.0",
    description=(
        "HTTP/WebSocket gateway to the SCOPIO self-driving-lab microscope. "
        "Generic endpoints mirror the frozen ROS 2 interface contract "
        "(scopio_interfaces); see docs/API.md in the repo for the full manual "
        "and GET /api/v1/interfaces for live discovery."
    ),
)


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
    """Call any ROS 2 service under /scopio (e.g. stage/jog); an omitted float field means "leave unchanged"."""
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
    """Live MJPEG video; usable as an <img src> with ?api_key=."""
    return await camera_proxy.mjpeg_stream()


@app.get("/api/v1/camera/controls", dependencies=[Depends(require_api_key)])
async def get_camera_controls():
    return await camera_proxy.forward("GET", "/controls")


@app.post("/api/v1/camera/controls", dependencies=[Depends(require_api_key)])
async def set_camera_controls(body: dict = Body(default={})):
    return await camera_proxy.forward("POST", "/controls", json_body=body)


@app.post("/api/v1/camera/mode", dependencies=[Depends(require_api_key)])
async def set_camera_mode(body: dict = Body(default={})):
    """Switch the sensor between 'detail' (full field of view) and 'fast' (highest fps); see `modes` in controls."""
    return await camera_proxy.forward("POST", "/mode", json_body=body)


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
