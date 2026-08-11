"""机器人能力查询路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter

from app.api.deps import ControlServerDep

router = APIRouter(tags=["机器人"])


@router.get(
    "/robot/capabilities",
    summary="机器人能力与适配参数",
    description="返回当前机器人的 frame、手动关节分组、安全范围与话题。",
)
async def robot_capabilities(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.robot_capabilities)
