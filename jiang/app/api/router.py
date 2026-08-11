"""合并所有控制网关 API 子路由。"""

from __future__ import annotations

from fastapi import APIRouter

from app.api import (
    cabinet_legacy,
    cabinets,
    health,
    manual,
    navigation,
    recording,
    replay,
    robot,
    task,
)

api_router = APIRouter()

# 按领域挂载。路径不冲突时各 router 独立工作；若未来引入前缀，
# 可在此统一调整。
api_router.include_router(health.router)
api_router.include_router(cabinet_legacy.router)
api_router.include_router(cabinets.router)
api_router.include_router(manual.router)
api_router.include_router(navigation.router)
api_router.include_router(recording.router)
api_router.include_router(replay.router)
api_router.include_router(robot.router)
api_router.include_router(task.router)
