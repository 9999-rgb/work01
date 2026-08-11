"""传感器流 FastAPI 路由：MJPEG 相机流、LiDAR WebSocket、健康检查。

从 aiohttp 迁移，行为与旧版 ``sensor_bridge/web_server.py`` 一致。
传感器数据只读，公开访问（loopback 绑定 + 数据本身非机密）。
"""

from __future__ import annotations

import asyncio
from typing import Annotated, Any

from fastapi import APIRouter, Depends, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response, StreamingResponse

from sensor_bridge.state import SensorStreamState

MJPEG_BOUNDARY = "xczs-camera-frame"


def get_sensor_state(request: Request) -> SensorStreamState:
    state = getattr(request.app.state, "sensor_state", None)
    if state is None:
        raise HTTPException(status_code=503, detail="传感器流未初始化。")
    return state


SensorStateDep = Annotated[SensorStreamState, Depends(get_sensor_state)]

router = APIRouter(tags=["传感器"])


@router.get(
    "/",
    summary="传感器服务信息",
    include_in_schema=False,
)
async def index() -> dict[str, Any]:
    return {
        "service": "xczs_web_sensor_stream",
        "endpoints": {
            "health": "/health",
            "camera_mjpeg": "/camera.mjpg",
            "camera_jpeg": "/camera.jpg",
            "lidar_json": "/lidar.json",
            "lidar_websocket": "/lidar/ws",
        },
    }


@router.get(
    "/health",
    summary="传感器健康检查",
    include_in_schema=False,
)
async def health(state: SensorStateDep) -> dict[str, Any]:
    return await asyncio.to_thread(state.health)


@router.get(
    "/camera.jpg",
    summary="最新相机帧（单帧 JPEG）",
)
async def camera_jpeg(state: SensorStateDep) -> Response:
    _sequence, jpeg, metadata = await asyncio.to_thread(state.camera_snapshot)
    if jpeg is None:
        return JSONResponse(
            status_code=503,
            content={"error": "相机帧不可用。"},
        )
    headers = {"Cache-Control": "no-store"}
    frame_id = (metadata or {}).get("frame_id", "")
    if frame_id:
        headers["X-Frame-Id"] = str(frame_id)
    return Response(content=jpeg, media_type="image/jpeg", headers=headers)


@router.get(
    "/camera.mjpg",
    summary="MJPEG 相机流",
)
async def camera_mjpeg(state: SensorStateDep, request: Request) -> StreamingResponse:
    async def frame_generator():
        last_sequence = -1
        while True:
            if await request.is_disconnected():
                return
            sequence, jpeg, _ = await asyncio.to_thread(state.camera_snapshot)
            if jpeg is not None and sequence != last_sequence:
                part = (
                    f"--{MJPEG_BOUNDARY}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n"
                    "\r\n"
                ).encode("ascii")
                yield part + jpeg + b"\r\n"
                last_sequence = sequence
            await asyncio.sleep(0.1)  # ~10 FPS

    return StreamingResponse(
        frame_generator(),
        media_type=f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY}",
        headers={"Cache-Control": "no-store", "Pragma": "no-cache"},
    )


@router.get(
    "/lidar.json",
    summary="最新 LiDAR 扫描（JSON）",
)
async def lidar_json(state: SensorStateDep) -> Response:
    _sequence, _payload, json_payload = await asyncio.to_thread(state.lidar_snapshot)
    if json_payload is None:
        return JSONResponse(
            status_code=503,
            content={"error": "LiDAR 扫描不可用。"},
        )
    return Response(
        content=json_payload,
        media_type="application/json",
        headers={"Cache-Control": "no-store"},
    )


@router.websocket("/lidar/ws")
async def lidar_websocket(websocket: WebSocket) -> None:
    state: SensorStreamState = websocket.app.state.sensor_state
    await websocket.accept()
    last_sequence = -1
    try:
        while True:
            sequence, _payload, json_payload = await asyncio.to_thread(
                state.lidar_snapshot
            )
            if json_payload is not None and sequence != last_sequence:
                await websocket.send_text(json_payload)
                last_sequence = sequence
            try:
                await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
            except asyncio.TimeoutError:
                pass
    except WebSocketDisconnect:
        return
    finally:
        await websocket.close()
