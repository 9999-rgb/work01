"""场景切换路由。"""

from __future__ import annotations

import asyncio
from typing import Any

from fastapi import APIRouter

from app.api.deps import ControlServerDep
from app.api.schemas import SceneSwitchRequest

router = APIRouter(tags=["场景"])


@router.get(
    "/scenes",
    summary="场景目录与当前活动场景",
    description="返回 scenes.yaml 中的全部场景及其当前活动场景标识。",
)
async def scenes(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.scenes)


@router.get(
    "/scene/active",
    summary="当前活动场景",
    description="返回当前活动场景的完整规格。",
)
async def active_scene(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.active_scene)


@router.post(
    "/scene/switch",
    # 该端点同步执行完整切换并在响应中返回最终结果（status: switched /
    # unchanged / failed），不是"受理后异步处理"的 202 语义。
    status_code=200,
    summary="切换场景",
    description="切换 Gazebo 几何、Nav2 地图并重定位机器人；返回切换结果。",
)
async def switch_scene(
    body: SceneSwitchRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.switch_scene, body.name)
