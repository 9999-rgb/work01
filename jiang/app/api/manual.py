"""手动底盘与机械臂控制路由。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import APIRouter

from app.api.deps import ControlServerDep
from app.api.schemas import CmdVelRequest, JointTrajectoryRequest

router = APIRouter(tags=["手动控制"])


@router.post(
    "/cmd_vel",
    summary="发布底盘速度指令",
    description="向底盘发布一个手动速度目标，返回经过限幅后的值。",
)
async def cmd_vel(body: CmdVelRequest, server: ControlServerDep) -> dict[str, Any]:
    linear_y, angular_z = await asyncio.to_thread(
        server.publish_cmd_vel,
        body.linear_y,
        body.angular_z,
    )
    return {"status": "ok", "linear_y": linear_y, "angular_z": angular_z}


@router.post(
    "/joint_trajectory",
    summary="发布机械臂关节轨迹",
    description="发布一个手动关节目标，返回实际生效的目标位置。",
)
async def joint_trajectory(
    body: JointTrajectoryRequest,
    server: ControlServerDep,
) -> dict[str, Any]:
    positions = await asyncio.to_thread(
        server.publish_joint_trajectory,
        body.positions,
    )
    return {"status": "ok", "positions": positions}
