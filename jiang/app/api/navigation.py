"""Nav2 导航控制路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter, Depends

from app.api.deps import ControlServerDep
from app.api.schemas import NavigationModeRequest
from app.api.validators import reject_json_body

router = APIRouter(tags=["导航"])


@router.get(
    "/navigation/status",
    summary="导航状态",
    description="返回 Nav2 状态与当前位姿。",
)
async def navigation_status(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.navigation_status)


@router.post(
    "/navigation/mode",
    status_code=202,
    summary="切换手动 / Nav2 底盘路由",
    description="旧版导航模式切换；返回路由状态。",
)
async def navigation_mode(
    body: NavigationModeRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.set_navigation_mode, body.enabled)


@router.post(
    "/navigation/cancel",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="取消导航",
    description="取消当前 Nav2 导航目标。",
)
async def navigation_cancel(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.cancel_navigation)


@router.post(
    "/navigation/takeover",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="接管导航为手动控制",
    description="取消 Nav2 并将底盘路由器切到零速手动模式。",
)
async def navigation_takeover(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.takeover_navigation)
