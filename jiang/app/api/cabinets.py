"""控制柜查询路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter, Path

from app.api.deps import ControlServerDep

router = APIRouter(tags=["柜体管理"])


@router.get(
    "/cabinets",
    summary="列出控制柜实例",
    description="返回所有柜体实例及其计算后的导航工位。",
)
async def cabinets(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.cabinets)


@router.get(
    "/cabinets/{name}/controls",
    summary="单个控制柜的控件目录与实时状态",
    description="返回指定实例的 catalog、可操作性及逐控件物理状态。",
)
async def cabinet_controls(
    name: Annotated[str, Path(description="柜体实例名称")],
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.cabinet_controls, name)
