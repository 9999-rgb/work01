"""任务历史查询路由：分页列表 / 详情 / 聚合摘要。

数据来自 ``app.tasks.store.TaskRecordStore``（SQLite ``task_records`` /
``task_progress_events``），经 ``asyncio.to_thread`` 调用同步 store。列表与
详情响应附带码→中文标签字段（``type_name`` 等）供前端直接展示；``labels``
映射在摘要接口一次性下发。

路由顺序：``/task/history``（列表）→ ``/task/history/summary`` →
``/task/history/{task_id}``（详情）。summary 必须先于 ``{task_id}`` 注册，
避免被当作 task_id 捕获。
"""

from __future__ import annotations

import asyncio
from typing import Annotated, Any

from fastapi import APIRouter, HTTPException, Query, status

from app.tasks import labels
from app.tasks.store import TaskRecordStore

router = APIRouter(tags=["任务历史"])

_PAGE_SIZE_MAX = 100
_DEFAULT_PAGE_SIZE = 20


def _store() -> TaskRecordStore:
    return TaskRecordStore()


def _with_labels(record: dict[str, Any]) -> dict[str, Any]:
    """给扁平记录补中文展示字段（未命中回退原文）。"""
    return {
        **record,
        "type_name": labels.type_name(record.get("type")),
        "status_name": labels.status_name(record.get("status")),
        "command_name": labels.command_name(record.get("command")),
        "failure_code_name": labels.failure_code_name(record.get("failure_code")),
        "phase_name": labels.phase_name(record.get("phase")),
    }


def _labels_map() -> dict[str, dict[str, str]]:
    return {
        "type": dict(labels.TYPE_LABELS),
        "status": dict(labels.STATUS_LABELS),
        "command": dict(labels.COMMAND_LABELS),
        "failure_code": dict(labels.FAILURE_CODE_LABELS),
    }


@router.get(
    "/task/history",
    summary="任务历史列表（分页）",
    description=(
        "分页查询 navigate / operate 任务记录；支持按类型、状态、柜体、控件、"
        "失败码与时间区间过滤。列表项含扁平索引列与中文展示字段。"
    ),
)
async def list_task_history(
    type: Annotated[
        str | None, Query(description="任务类型：navigate / operate")
    ] = None,
    status: Annotated[
        str | None, Query(description="任务状态：success / failed / canceled 等")
    ] = None,
    cabinet: Annotated[str | None, Query(description="柜体名")] = None,
    control_id: Annotated[str | None, Query(description="控件 ID")] = None,
    failure_code: Annotated[str | None, Query(description="失败码")] = None,
    since: Annotated[float | None, Query(description="创建时间下限（unix 秒）")] = None,
    until: Annotated[float | None, Query(description="创建时间上限（unix 秒）")] = None,
    page: Annotated[int, Query(ge=1, description="页码，从 1 起")] = 1,
    page_size: Annotated[
        int,
        Query(ge=1, le=_PAGE_SIZE_MAX, description="每页条数"),
    ] = _DEFAULT_PAGE_SIZE,
) -> dict[str, Any]:
    store = await asyncio.to_thread(_store)
    total, items = await asyncio.to_thread(
        store.list_records,
        task_type=type,
        status=status,
        cabinet=cabinet,
        control_id=control_id,
        failure_code=failure_code,
        since=since,
        until=until,
        page=page,
        page_size=page_size,
    )
    return {
        "items": [_with_labels(record) for record in items],
        "page": page,
        "page_size": page_size,
        "total": total,
        "total_pages": (total + page_size - 1) // page_size if total else 0,
    }


@router.get(
    "/task/history/summary",
    summary="任务历史聚合摘要",
    description=(
        "按类型 / 状态 / 失败码 / 柜体聚合计数 + 平均耗时；``labels`` 一次性"
        "下发码→中文映射供报表页渲染。"
    ),
)
async def task_history_summary() -> dict[str, Any]:
    store = await asyncio.to_thread(_store)
    data = await asyncio.to_thread(store.summary)
    data["labels"] = _labels_map()
    return data


@router.get(
    "/task/history/{task_id}",
    summary="任务详情",
    description="返回单条任务完整记录（含 request / result 载荷）与按 id 排序的进度明细。",
)
async def task_history_detail(task_id: str) -> dict[str, Any]:
    store = await asyncio.to_thread(_store)
    record = await asyncio.to_thread(store.get_record, task_id)
    if record is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"任务记录不存在：{task_id}",
        )
    events = await asyncio.to_thread(store.list_progress_events, task_id)
    return {
        "record": _with_labels(record),
        "progress_events": events,
    }
