"""传感器流 FastAPI 路由：MJPEG 相机流、LiDAR WebSocket、健康检查。

从 aiohttp 迁移，行为与旧版 ``sensor_bridge/web_server.py`` 一致。
传感器数据只读，但与控制 API 共用鉴权。HTTP/MJPEG 可使用
Bearer Header 或 ``?token=``；浏览器 WebSocket 使用 ``?token=``。
"""

from __future__ import annotations

import asyncio
from typing import Annotated, Any
from urllib.parse import urlsplit

from fastapi import (
    APIRouter,
    Depends,
    HTTPException,
    Request,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import JSONResponse, Response, StreamingResponse

from app.auth.deps import ActiveTokenChecker, resolve_active_user
from app.database.engine import async_session
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
    "/sensors",
    summary="传感器服务信息",
    include_in_schema=False,
)
async def index() -> dict[str, Any]:
    return {
        "service": "xczs_web_sensor_stream",
        "endpoints": {
            "health": "/sensors/health",
            "camera_mjpeg": "/camera.mjpg",
            "camera_jpeg": "/camera.jpg",
            "lidar_json": "/lidar.json",
            "lidar_websocket": "/lidar/ws",
        },
    }


@router.get(
    "/sensors/health",
    summary="传感器健康检查",
)
async def health(request: Request, state: SensorStateDep) -> dict[str, Any]:
    result = await asyncio.to_thread(state.health)
    runtime = getattr(request.app.state, "sensor_runtime", None)
    runtime_health = getattr(runtime, "health", None)
    if callable(runtime_health):
        executor = await asyncio.to_thread(runtime_health)
        result = {**result, "runtime": executor}
        if executor.get("status") != "ok":
            result["status"] = "error"
    return result


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
    authorization = ActiveTokenChecker.from_request(request)

    async def frame_generator():
        last_sequence = -1
        while True:
            if (
                await request.is_disconnected()
                or not await authorization.is_valid()
            ):
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
    authorization = await _authorize_websocket(websocket)
    if authorization is None:
        return
    state: SensorStreamState | None = getattr(
        websocket.app.state,
        "sensor_state",
        None,
    )
    if state is None:
        await websocket.close(code=1013, reason="传感器流未初始化。")
        return
    await websocket.accept()
    last_sequence = -1
    try:
        while True:
            if not await authorization.is_valid():
                await websocket.close(code=4401, reason="认证已失效。")
                return
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
        try:
            await websocket.close()
        except RuntimeError:
            # 对端已完成 close handshake。
            pass


async def _authorize_websocket(
    websocket: WebSocket,
) -> ActiveTokenChecker | None:
    """在 WebSocket accept 前校验 Origin 与当前用户。"""
    allowed_origins = getattr(websocket.app.state, "allowed_origins", frozenset())
    origin = websocket.headers.get("origin")
    if not _websocket_origin_allowed(
        origin,
        websocket.headers.get("host"),
        allowed_origins,
    ):
        await websocket.close(code=4403, reason="Origin 不允许。")
        return None
    if not getattr(websocket.app.state, "auth_enabled", True):
        return ActiveTokenChecker(token=None, user_id=None, enabled=False)
    token = websocket.query_params.get("token")
    if not token:
        authorization = websocket.headers.get("authorization", "")
        scheme, _, credentials = authorization.partition(" ")
        if scheme.lower() == "bearer" and credentials.strip():
            token = credentials.strip()
    if not token:
        await websocket.close(code=4401, reason="缺少认证凭证。")
        return None
    try:
        async with async_session() as session:
            user = await resolve_active_user(token, session)
    except HTTPException:
        await websocket.close(code=4401, reason="token 无效或已过期。")
        return None
    return ActiveTokenChecker(
        token=token,
        user_id=user.id,
        enabled=True,
    )


def _websocket_origin_allowed(
    origin: str | None,
    host: str | None,
    allowed_origins: frozenset[str],
) -> bool:
    """允许显式配置的 Origin，以及严格与当前 HTTP Host 同源的页面。

    同源分支保持 ``run_all.sh`` 默认的 LAN 访问可用，又不需要事先
    枚举服务器的每个 IP/主机名。
    """
    if not origin or not host:
        return False
    value = origin.rstrip("/")
    if value in allowed_origins:
        return True
    parsed = urlsplit(value)
    return (
        parsed.scheme in {"http", "https"}
        and parsed.netloc == host
        and parsed.username is None
        and parsed.password is None
        and not parsed.path
        and not parsed.query
        and not parsed.fragment
    )
