"""HTTP, MJPEG, JSON and WebSocket endpoints for browser sensor data."""

from __future__ import annotations

import asyncio

from aiohttp import web

from .state import SensorStreamState


MJPEG_BOUNDARY = "xczs-camera-frame"


async def _add_cors_headers(
    request: web.Request,
    response: web.StreamResponse,
) -> None:
    del request
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"


class SensorWebHandlers:
    """Serve the latest sensor values from shared state."""

    def __init__(
        self,
        state: SensorStreamState,
        stream_fps: float,
        lidar_fps: float,
    ) -> None:
        self._state = state
        self._stream_period = 1.0 / max(1.0, stream_fps)
        self._lidar_period = 1.0 / max(1.0, lidar_fps)

    async def index(self, request: web.Request) -> web.Response:
        del request
        return web.json_response(
            {
                "service": "xczs_web_sensor_stream",
                "endpoints": {
                    "health": "/health",
                    "camera_mjpeg": "/camera.mjpg",
                    "camera_jpeg": "/camera.jpg",
                    "lidar_json": "/lidar.json",
                    "lidar_websocket": "/lidar/ws",
                },
            }
        )

    async def health(self, request: web.Request) -> web.Response:
        del request
        return web.json_response(self._state.health())

    async def options(self, request: web.Request) -> web.Response:
        del request
        return web.Response(status=204)

    async def camera_jpeg(self, request: web.Request) -> web.Response:
        del request
        _, jpeg, metadata = self._state.fresh_camera_snapshot()
        if jpeg is None:
            return web.json_response(
                {"error": "camera frame is not available"},
                status=503,
            )
        return web.Response(
            body=jpeg,
            headers={
                "Cache-Control": "no-store",
                "X-Frame-Id": str((metadata or {}).get("frame_id", "")),
            },
            content_type="image/jpeg",
        )

    async def camera_mjpeg(
        self,
        request: web.Request,
    ) -> web.StreamResponse:
        _, jpeg, _ = self._state.fresh_camera_snapshot()
        if jpeg is None:
            return web.json_response(
                {"error": "camera frame is not available"},
                status=503,
            )
        response = web.StreamResponse(
            status=200,
            headers={
                "Cache-Control": "no-store",
                "Pragma": "no-cache",
                "Content-Type": (
                    "multipart/x-mixed-replace;"
                    f"boundary={MJPEG_BOUNDARY}"
                ),
            },
        )
        await response.prepare(request)

        last_sequence = -1
        try:
            while True:
                sequence, jpeg, _ = self._state.fresh_camera_snapshot()
                if jpeg is None:
                    break
                if sequence != last_sequence:
                    part = (
                        f"--{MJPEG_BOUNDARY}\r\n"
                        "Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n"
                        "\r\n"
                    ).encode("ascii")
                    await response.write(part + jpeg + b"\r\n")
                    last_sequence = sequence
                await asyncio.sleep(self._stream_period)
        except (
            asyncio.CancelledError,
            ConnectionResetError,
            RuntimeError,
        ):
            pass
        return response

    async def lidar_json(self, request: web.Request) -> web.Response:
        del request
        _, _, json_payload = self._state.fresh_lidar_snapshot()
        if json_payload is None:
            return web.json_response(
                {"error": "lidar scan is not available"},
                status=503,
            )
        return web.Response(
            text=json_payload,
            headers={"Cache-Control": "no-store"},
            content_type="application/json",
        )

    async def lidar_websocket(
        self,
        request: web.Request,
    ) -> web.WebSocketResponse:
        websocket = web.WebSocketResponse(
            heartbeat=15.0,
            compress=False,
        )
        await websocket.prepare(request)
        last_sequence = -1
        try:
            while not websocket.closed:
                sequence, _, json_payload = self._state.fresh_lidar_snapshot()
                health = self._state.health()
                if (
                    json_payload is None
                    and health.get("lidar", {}).get("stale", False)
                ):
                    await websocket.close(
                        code=1013,
                        message=b"LiDAR scan is stale.",
                    )
                    break
                if (
                    json_payload is not None
                    and sequence != last_sequence
                ):
                    await websocket.send_str(json_payload)
                    last_sequence = sequence
                try:
                    message = await websocket.receive(
                        timeout=self._lidar_period,
                    )
                    if message.type in (
                        web.WSMsgType.CLOSE,
                        web.WSMsgType.CLOSED,
                        web.WSMsgType.ERROR,
                    ):
                        break
                except asyncio.TimeoutError:
                    pass
        except (
            asyncio.CancelledError,
            ConnectionResetError,
            RuntimeError,
        ):
            pass
        finally:
            if not websocket.closed:
                try:
                    await websocket.close()
                except (ConnectionResetError, RuntimeError):
                    # The peer may already have vanished without completing
                    # the WebSocket close handshake.
                    pass
        return websocket


def create_sensor_app(
    state: SensorStreamState,
    stream_fps: float = 10.0,
    lidar_fps: float = 10.0,
) -> web.Application:
    """Create the sensor streaming web application."""
    handlers = SensorWebHandlers(
        state=state,
        stream_fps=stream_fps,
        lidar_fps=lidar_fps,
    )
    app = web.Application()
    app.on_response_prepare.append(_add_cors_headers)
    app.add_routes(
        [
            web.get("/", handlers.index),
            web.get("/health", handlers.health),
            web.get("/camera.jpg", handlers.camera_jpeg),
            web.get("/camera.mjpg", handlers.camera_mjpeg),
            web.get("/lidar.json", handlers.lidar_json),
            web.get("/lidar/ws", handlers.lidar_websocket),
            web.options("/{path:.*}", handlers.options),
        ]
    )
    return app
