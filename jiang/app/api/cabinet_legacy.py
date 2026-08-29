"""旧版单柜兼容接口路由。

这些路由仅在配置单个柜体时可用（旧客户端兼容），多柜时返回 410。
"""

from __future__ import annotations

import asyncio
from typing import Any

from fastapi import APIRouter, Depends

from app.api.deps import ControlServerDep
from app.api.schemas import CabinetOperateRequest, CabinetPressRequest
from app.api.validators import reject_json_body

router = APIRouter(tags=["柜体管理"])


@router.get(
    "/cabinet/status",
    summary="单柜兼容：柜体状态",
    description="仅单柜配置时可用；多柜时返回 410。",
)
async def cabinet_status(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.cabinet_status)


@router.get(
    "/cabinet/controls",
    summary="单柜兼容：控件目录与状态",
    description="仅单柜配置时可用；多柜时返回 410。",
)
async def cabinet_controls(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.cabinet_controls)


@router.post(
    "/cabinet/press",
    status_code=202,
    summary="单柜兼容：按压按钮",
    description="直接操作按钮（不进入全局任务管理器）。",
)
async def cabinet_press(
    body: CabinetPressRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.press_cabinet_button,
        body.button_id,
        body.navigate_to_staging_pose,
    )


@router.post(
    "/cabinet/operate",
    status_code=202,
    summary="单柜兼容：操作控件",
    description="直接操作控件（不进入全局任务管理器）。",
)
async def cabinet_operate(
    body: CabinetOperateRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.operate_cabinet_control,
        body.control_id,
        body.command,
        body.target_state,
        body.target_position,
        body.navigate_to_staging_pose,
        body.force,
    )


@router.post(
    "/cabinet/cancel",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="单柜兼容：取消操作",
    description="取消当前柜体操作。",
)
async def cabinet_cancel(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.cancel_cabinet_operation)


@router.post(
    "/cabinet/reset",
    dependencies=[Depends(reject_json_body)],
    summary="单柜兼容：重置控件",
    description="重置当前柜体所有控件。",
)
async def cabinet_reset(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.reset_cabinet_controls)
