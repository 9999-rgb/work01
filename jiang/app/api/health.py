"""健康检查路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter, Request

from app.api.deps import ControlServerDep

router = APIRouter(tags=["健康检查"])


@router.get(
    "/health",
    summary="系统健康检查",
    description="返回网关、Nav2、控制柜与全局任务可用性。",
)
async def health(request: Request, server: ControlServerDep) -> dict[str, Any]:
    result = await asyncio.to_thread(server.health)
    # 应用层声明真实门禁策略；不要依赖可注入的 ControlServer/fake 后端，
    # 也避免浏览器根据 token 是否存在猜测迁移模式配置。
    return {
        **result,
        "auth_required": bool(getattr(request.app.state, "auth_enabled", True)),
    }
