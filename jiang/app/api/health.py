"""健康检查路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter

from app.api.deps import ControlServerDep

router = APIRouter(tags=["健康检查"])


@router.get(
    "/health",
    summary="系统健康检查",
    description="返回网关、Nav2、控制柜与全局任务可用性。",
)
async def health(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.health)
