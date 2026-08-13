"""录制管理路由：rosbag2 被动录制与清单查询。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter, Depends, Path, Query

from app.api.deps import ControlServerDep
from app.api.schemas import RecordingStartRequest
from app.api.validators import reject_json_body

router = APIRouter(tags=["录制回放"])


@router.get(
    "/recordings",
    summary="列出录制",
    description="返回所有保留的录制清单（不含数据路径）。",
)
async def recordings(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.recordings)


@router.get(
    "/recordings/{recording_id}",
    summary="录制详情",
    description="返回单个录制的清单与产物可用性。",
)
async def recording_detail(
    recording_id: Annotated[str, Path(description="录制 ID")],
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.recording_detail, recording_id)


@router.get(
    "/recordings/{recording_id}/timeline",
    summary="录制时间线",
    description="返回该录制有界长度的事件时间线。",
)
async def recording_timeline(
    recording_id: Annotated[str, Path(description="录制 ID")],
    server: ControlServerDep,
    offset: Annotated[int, Query(ge=0)] = 0,
    limit: Annotated[int, Query(ge=1, le=5000)] = 500,
) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.recording_timeline,
        recording_id,
        offset,
        limit,
    )


@router.post(
    "/recording/start",
    status_code=202,
    summary="开始录制",
    description="启动被动 rosbag2 捕获；录制期间实时控制保持可用。",
)
async def recording_start(
    body: RecordingStartRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.start_recording,
        body.name,
        body.include_sensors,
    )


@router.post(
    "/recording/stop",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="停止录制",
    description="停止 rosbag2 捕获并原子化收尾产物。",
)
async def recording_stop(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.stop_recording)
