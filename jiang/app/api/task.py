"""任务操作路由：导航、操作、重置、状态查询、取消与 SSE 事件流。"""

from __future__ import annotations

import asyncio
from typing import Any, Annotated

from fastapi import (
    APIRouter,
    Depends,
    Header,
    HTTPException,
    Path,
    Request,
    Response,
    status,
)
from fastapi.responses import StreamingResponse
from starlette.background import BackgroundTask

from app.api.deps import ControlServerDep
from app.api.schemas import (
    NavigateRequest,
    OperateRequest,
    TaskResetRequest,
)
from app.api.sse_utils import stream_task_events
from app.api.validators import reject_json_body
from app.auth.deps import ActiveTokenChecker

router = APIRouter(tags=["任务操作"])


@router.post(
    "/task/navigate",
    status_code=202,
    summary="导航到控制柜",
    description="""将机器人导航到指定控制柜的操作工位。

- 传入 ``control_id`` 时使用该控件专用工位（robot_adapter.yaml 配置）
- 默认使用 cabinet_scene.yaml 的公共工位
- 异步执行：返回 202 + task_id，通过 GET /task/{id}/status 轮询
""",
)
async def task_navigate(body: NavigateRequest, server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.submit_navigation_task,
        body.cabinet,
        body.control_id,
    )


@router.post(
    "/task/operate",
    status_code=202,
    summary="操作控制柜控件",
    description="""提交一个非导航的柜体操作任务。绝不隐式导航。

- command：press / set_state / set_position / toggle
- force 省略时使用控件目录 default_force；必须为正数
- 异步执行：返回 202 + task_id
""",
)
async def task_operate(body: OperateRequest, server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(
        server.submit_operation_task,
        body.cabinet,
        body.control_id,
        body.command,
        body.target_state,
        body.target_position,
        body.force,
    )


@router.post(
    "/task/reset",
    status_code=202,
    summary="重置场景",
    description="把指定柜体的底座回到初始位姿、机械臂/夹爪回到默认位置、控件回到默认状态。",
)
async def task_reset(body: TaskResetRequest, server: ControlServerDep) -> dict[str, Any]:
    return await asyncio.to_thread(server.submit_reset_task, body.cabinet)


@router.get(
    "/task/{task_id}/status",
    summary="任务状态",
    description="轮询任务的保留快照。",
)
async def task_status(
    task_id: Annotated[str, Path(description="任务 ID")],
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.task_status, task_id)


@router.post(
    "/task/{task_id}/cancel",
    status_code=202,
    dependencies=[Depends(reject_json_body)],
    summary="取消任务",
    description="请求取消任务，不提前声明终态。",
)
async def task_cancel(
    task_id: Annotated[str, Path(description="任务 ID")],
    server: ControlServerDep,
) -> dict[str, Any]:
    return await asyncio.to_thread(server.cancel_task, task_id)


@router.get(
    "/task/events",
    summary="任务事件流（SSE）",
    description="""可重连的 Server-Sent Events 实时事件流。

- 通过 ``Last-Event-ID`` 头断线重连
- EventSource 无法携带 Header，鉴权 token 通过查询参数 ``?token=`` 传递
""",
)
async def task_events(
    request: Request,
    server: ControlServerDep,
    last_event_id: Annotated[
        str | None,
        Header(alias="Last-Event-ID", include_in_schema=False),
    ] = None,
) -> Response:
    _validate_last_event_id(last_event_id)
    # Subscribe before emitting HTTP 200 so shutdown/validation failures retain
    # their ordinary structured 4xx/5xx response contract.  StreamingResponse
    # invokes the background close even if the peer disconnects before the
    # body iterator starts; stream_task_events also closes in its own finally.
    # This factory only takes in-process locks and copies at most the bounded
    # event ring.  Keeping it synchronous avoids the cancellation gap where a
    # to_thread call could finish after the route task was gone and leak the
    # newly returned subscription.
    subscription = server.subscribe_task_events(last_event_id)
    return StreamingResponse(
        stream_task_events(
            subscription,
            request,
            authorization=ActiveTokenChecker.from_request(request),
        ),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
        background=BackgroundTask(subscription.close),
    )


def _validate_last_event_id(last_event_id: str | None) -> None:
    """复刻旧版对 Last-Event-ID 的校验：必须是非负整数。"""
    if last_event_id is None:
        return
    value = last_event_id.strip()
    if not value or not value.isascii() or not value.isdigit():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Last-Event-ID 必须是非负整数。",
        )
