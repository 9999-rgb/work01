"""机器人能力查询路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter, Depends, status

from app.api.deps import ControlServerDep
from app.api.schemas import ToolsetSwitchRequest
from app.auth.deps import require_admin
from app.auth.models import User

router = APIRouter(tags=["机器人"])


@router.get(
    "/robot/capabilities",
    summary="机器人能力与适配参数",
    description="返回当前机器人的 frame、手动关节分组、安全范围与话题。",
)
async def robot_capabilities(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.robot_capabilities)


@router.get(
    "/robot/toolset/status",
    summary="运行中末端工具套装状态",
    description=(
        "返回 Gazebo 世界保活切换监督器的运行态。状态为 switching 时所有"
        "机器人运动写入均被网关锁定。"
    ),
)
async def toolset_status(server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.toolset_status)


@router.post(
    "/robot/toolset/switch",
    status_code=status.HTTP_202_ACCEPTED,
    summary="一键切换末端工具套装",
    description=(
        "异步切换 A/B 末端。Gazebo 世界和柜体模型不重启；机器人实体、"
        "ros2_control、MoveIt 与柜体操作子栈会安全替换。仅在目标完整就绪"
        "后才更新运行态与持久化选择。"
    ),
)
async def switch_toolset(
    body: ToolsetSwitchRequest,
    server: ControlServerDep,
    _admin: Annotated[User, Depends(require_admin)],
) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.request_toolset_switch,
        body.toolset,
        expected_generation=body.expected_generation,
    )
