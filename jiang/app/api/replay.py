"""回放路由：隔离数据回放与任务重演。"""

from __future__ import annotations

import asyncio
from typing import Any

from fastapi import APIRouter, Depends

from app.api.deps import ControlServerDep
from app.api.schemas import (
    DataPlaybackRateRequest,
    DataPlaybackStartRequest,
    TaskReplayStartRequest,
)
from app.api.validators import reject_json_body

router = APIRouter(tags=["录制回放"])


@router.get(
    "/replay/status",
    summary="回放状态",
    description="返回录制、隔离数据回放与任务重演的联合状态和只读互锁标记。",
)
async def replay_status(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.replay_status)


@router.post(
    "/replay/data/start",
    status_code=202,
    summary="开始隔离数据回放",
    description="""以只读模式启动命名空间隔离的 rosbag2 回放。

回放期间所有写操作被拒绝（409）。
""",
)
async def data_playback_start(
    body: DataPlaybackStartRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.start_data_playback, body.recording_id, body.rate)


@router.post(
    "/replay/data/pause",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="暂停数据回放",
)
async def data_playback_pause(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.pause_data_playback)


@router.post(
    "/replay/data/resume",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="恢复数据回放",
)
async def data_playback_resume(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.resume_data_playback)


@router.post(
    "/replay/data/rate",
    status_code=202,
    summary="设置回放倍速",
    description="在当前回放位置以新倍速重启隔离回放。",
)
async def data_playback_rate(
    body: DataPlaybackRateRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.set_data_playback_rate, body.rate)


@router.post(
    "/replay/task/start",
    status_code=202,
    summary="开始任务重演",
    description="通过当前控制栈重演已记录的场景（语义任务重放，非运动命令重放）。",
)
async def task_replay_start(
    body: TaskReplayStartRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.start_task_replay, body.recording_id)


@router.post(
    "/replay/cancel",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="取消回放",
    description="取消当前拥有的数据回放或任务重演。",
)
async def replay_cancel(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.cancel_replay)
