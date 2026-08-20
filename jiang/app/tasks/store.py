"""SQLite 任务记录 store（同步 SQLAlchemy）。

复用 ``app.assets.store.SqlAssetStore`` 的模式：同步 ``Session``，共享按 URL
缓存的 engine / sessionmaker（同一数据库只建一个连接池），SQLite WAL +
busy_timeout 与异步引擎一致，Web 层经 ``asyncio.to_thread`` 调用。

写入（事件桥线程）与读取（Web，经 to_thread）共享同一 engine 池，WAL 支持
单写者 + 多读者并发。``control_names`` 是可选的 ``control_id → 中文名`` 映射，
生产由 ``control_server.py::main()`` 从活动控件目录加载，用于写入
``control_name``（未知回退 control_id）。

写入语义：主表 ``task_records`` 按 ``task_id`` upsert（终态快照）；明细表
``task_progress_events`` 对每个事件（task_accepted / 每个 ~1Hz task_progress /
task_completed）全量插一行，按 ``id`` 自增排序还原时间线。终态事件后按
``settings.task_record_retention`` 剪枝最旧记录并级联删除其明细。
"""

from __future__ import annotations

import logging
from typing import Any, Mapping, Optional

from sqlalchemy import create_engine, delete, func, select
from sqlalchemy.orm import Session, sessionmaker

from app.assets.store import (
    _ENGINES,
    _SESSION_FACTORIES,
    _set_sqlite_pragmas,
    _sync_database_url,
)
from app.config import settings
from app.database.base import Base
from app.tasks.models import TaskProgressEvent, TaskRecord

logger = logging.getLogger(__name__)

_SQLITE_TIMEOUT_SEC = 10

#: 参与记录的任务事件。
_TRACKED_EVENTS = frozenset({"task_accepted", "task_progress", "task_completed"})
#: 记录的任务类型（reset 不记录）。
_TRACKED_TYPES = frozenset({"navigate", "operate"})

#: 已建表的 URL 集合（与资产库的 engine 注册表配套，建表只执行一次）。
_TASK_SCHEMA_ENSURED: set[str] = set()


# ── 数值 / 字典收窄工具 ──────────────────────────────────────────────


def _as_float(value: object, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_float_or_none(value: object) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_str(value: object) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _dict_copy(value: object) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _dict_or_none(value: object) -> Optional[dict[str, Any]]:
    return dict(value) if isinstance(value, Mapping) else None


class TaskRecordStore:
    """SQLite 任务记录 store：实现 ``TaskRecordSink``（同步）。"""

    def __init__(
        self,
        database_url: Optional[str] = None,
        *,
        control_names: Optional[Mapping[str, str]] = None,
    ) -> None:
        url = _sync_database_url(database_url)
        if url not in _ENGINES:
            connect_args: dict = {}
            if url.startswith("sqlite"):
                connect_args = {
                    "check_same_thread": False,
                    "timeout": _SQLITE_TIMEOUT_SEC,
                }
            engine = create_engine(url, connect_args=connect_args)
            if url.startswith("sqlite"):
                _set_sqlite_pragmas(engine)
            _ENGINES[url] = engine
            _SESSION_FACTORIES[url] = sessionmaker(
                bind=engine, expire_on_commit=False
            )
        self._engine = _ENGINES[url]
        self._session_factory = _SESSION_FACTORIES[url]
        if url not in _TASK_SCHEMA_ENSURED:
            Base.metadata.create_all(
                self._engine,
                tables=[TaskRecord.__table__, TaskProgressEvent.__table__],
            )
            _TASK_SCHEMA_ENSURED.add(url)
        self._control_names: dict[str, str] = (
            dict(control_names) if control_names else {}
        )

    # ── TaskRecordSink 实现 ──────────────────────────────────────────

    def record_event(self, event: Mapping[str, Any]) -> None:
        """消费一个任务事件：过滤后 upsert 主表 + 全量追加明细。

        只在 ``task_accepted / task_progress / task_completed`` 且任务类型为
        navigate / operate 时动作；异常向上抛出，由事件桥调用方兜底记录。
        """
        if event.get("event") not in _TRACKED_EVENTS:
            return
        data = event.get("data")
        if not isinstance(data, Mapping):
            return
        task = data.get("task")
        if not isinstance(task, Mapping):
            return
        if str(task.get("type", "")) not in _TRACKED_TYPES:
            return
        if not str(task.get("task_id", "")).strip():
            return
        self._upsert_record(task)
        self._append_progress(event, task)
        if event.get("event") == "task_completed":
            self._enforce_retention()

    # ── 写入 ─────────────────────────────────────────────────────────

    def _upsert_record(self, task: Mapping[str, Any]) -> None:
        values = self._column_values(task)
        with self._session_factory() as session:
            existing = session.execute(
                select(TaskRecord).where(
                    TaskRecord.task_id == values["task_id"]
                )
            ).scalar_one_or_none()
            if existing is None:
                session.add(TaskRecord(**values))
            else:
                for key, value in values.items():
                    setattr(existing, key, value)
            session.commit()

    def _append_progress(
        self, event: Mapping[str, Any], task: Mapping[str, Any]
    ) -> None:
        timestamp = _as_float(event.get("timestamp"), task.get("updated_at", 0.0))
        created_at = _as_float(task.get("created_at"), 0.0)
        row = TaskProgressEvent(
            task_id=str(task.get("task_id", "")),
            sequence=self._sequence_int(event),
            event=str(event.get("event", "")),
            status=str(task.get("status", "") or ""),
            phase=str(task.get("phase", "") or ""),
            progress=_as_float(task.get("progress"), 0.0),
            message=str(task.get("message", "") or ""),
            business_data=_dict_or_none(task.get("business_data")),
            timestamp=timestamp,
            elapsed_seconds=(
                timestamp - created_at if created_at else None
            ),
        )
        with self._session_factory() as session:
            session.add(row)
            session.commit()

    def _enforce_retention(self) -> None:
        """终态后按条数剪枝：删除最旧超限记录及其进度明细。"""
        retention = int(settings.task_record_retention or 0)
        if retention <= 0:
            return
        with self._session_factory() as session:
            total = session.execute(
                select(func.count()).select_from(TaskRecord)
            ).scalar_one()
            if total <= retention:
                return
            excess = total - retention
            stale = session.execute(
                select(TaskRecord.task_id)
                .order_by(TaskRecord.id)
                .limit(excess)
            ).scalars().all()
            if not stale:
                return
            session.execute(
                delete(TaskProgressEvent).where(
                    TaskProgressEvent.task_id.in_(stale)
                )
            )
            session.execute(
                delete(TaskRecord).where(TaskRecord.task_id.in_(stale))
            )
            session.commit()

    # ── 查询 ─────────────────────────────────────────────────────────

    def list_records(
        self,
        *,
        task_type: Optional[str] = None,
        status: Optional[str] = None,
        cabinet: Optional[str] = None,
        control_id: Optional[str] = None,
        failure_code: Optional[str] = None,
        since: Optional[float] = None,
        until: Optional[float] = None,
        page: int = 1,
        page_size: int = 20,
    ) -> tuple[int, list[dict[str, Any]]]:
        """分页列出任务记录（倒序，不含大 JSON 载荷），返回 ``(total, items)``。"""
        conditions: list[Any] = []
        if task_type:
            conditions.append(TaskRecord.type == task_type)
        if status:
            conditions.append(TaskRecord.status == status)
        if cabinet:
            conditions.append(TaskRecord.cabinet == cabinet)
        if control_id:
            conditions.append(TaskRecord.control_id == control_id)
        if failure_code:
            conditions.append(TaskRecord.failure_code == failure_code)
        if since is not None:
            conditions.append(TaskRecord.created_at >= since)
        if until is not None:
            conditions.append(TaskRecord.created_at <= until)
        with self._session_factory() as session:
            total = session.execute(
                select(func.count())
                .select_from(TaskRecord)
                .where(*conditions)
            ).scalar_one()
            rows = session.execute(
                select(TaskRecord)
                .where(*conditions)
                .order_by(TaskRecord.id.desc())
                .offset((page - 1) * page_size)
                .limit(page_size)
            ).scalars().all()
        return total, [
            self._record_to_dict(row, include_payload=False) for row in rows
        ]

    def get_record(self, task_id: str) -> Optional[dict[str, Any]]:
        """按 task_id 取完整记录（含 request/result 等载荷）；不存在返回 None。"""
        with self._session_factory() as session:
            row = session.execute(
                select(TaskRecord).where(TaskRecord.task_id == task_id)
            ).scalar_one_or_none()
            if row is None:
                return None
            return self._record_to_dict(row, include_payload=True)

    def list_progress_events(self, task_id: str) -> list[dict[str, Any]]:
        """按 id 自增顺序返回任务的进度明细（~1Hz 全量）。"""
        with self._session_factory() as session:
            rows = session.execute(
                select(TaskProgressEvent)
                .where(TaskProgressEvent.task_id == task_id)
                .order_by(TaskProgressEvent.id)
            ).scalars().all()
        return [self._event_to_dict(row) for row in rows]

    def summary(self) -> dict[str, Any]:
        """聚合计数 + 平均耗时（按报表维度），供历史页 / 报表分析。"""
        with self._session_factory() as session:
            total = session.execute(
                select(func.count()).select_from(TaskRecord)
            ).scalar_one()
            by_type = self._counts_by(session, TaskRecord.type)
            by_status = self._counts_by(session, TaskRecord.status)
            by_failure_code = self._counts_by(
                session, TaskRecord.failure_code, non_null=True
            )
            by_cabinet = self._counts_by(
                session, TaskRecord.cabinet, non_null=True
            )
            avg_rows = session.execute(
                select(TaskRecord.type, func.avg(TaskRecord.duration_seconds))
                .where(TaskRecord.duration_seconds.is_not(None))
                .group_by(TaskRecord.type)
            ).all()
        return {
            "total": total,
            "by_type": by_type,
            "by_status": by_status,
            "by_failure_code": by_failure_code,
            "by_cabinet": by_cabinet,
            "avg_duration_seconds": {
                str(task_type): (
                    round(float(average), 3) if average is not None else None
                )
                for task_type, average in avg_rows
            },
        }

    # ── 映射 ─────────────────────────────────────────────────────────

    def _column_values(self, task: Mapping[str, Any]) -> dict[str, Any]:
        request = task.get("request")
        request = _dict_copy(request) if isinstance(request, Mapping) else {}
        cabinet = _optional_str(request.get("cabinet"))
        control_id = _optional_str(request.get("control_id"))
        return {
            "task_id": str(task.get("task_id", "")),
            "type": str(task.get("type", "")),
            "status": str(task.get("status", "") or ""),
            "phase": str(task.get("phase", "") or ""),
            "progress": _as_float(task.get("progress"), 0.0),
            "message": str(task.get("message", "") or ""),
            "cabinet": cabinet,
            "control_id": control_id,
            "control_name": self._control_name(control_id),
            "command": _optional_str(request.get("command")),
            "target_state": _optional_str(request.get("target_state")),
            "target_position": _as_float_or_none(request.get("target_position")),
            "force": _as_float_or_none(request.get("force")),
            "request": _dict_copy(request),
            "result": _dict_or_none(task.get("result")),
            "business_data": _dict_or_none(task.get("business_data")),
            "failure_code": _optional_str(task.get("failure_code")),
            "failure_reason": _optional_str(task.get("failure_reason")),
            "failure_details": _dict_or_none(task.get("failure_details")),
            "duration_seconds": _as_float_or_none(task.get("duration_seconds")),
            "created_at": _as_float(task.get("created_at"), 0.0),
            "started_at": _as_float_or_none(task.get("started_at")),
            "completed_at": _as_float_or_none(task.get("completed_at")),
            "updated_at": _as_float(task.get("updated_at"), 0.0),
        }

    def _control_name(self, control_id: Optional[str]) -> Optional[str]:
        """控件中文名：映射命中取 display_name，未知回退 control_id。"""
        if control_id is None:
            return None
        return self._control_names.get(control_id, control_id)

    @staticmethod
    def _sequence_int(event: Mapping[str, Any]) -> Optional[int]:
        value = event.get("sequence")
        if isinstance(value, int):
            return value
        if isinstance(value, str) and value.isdigit():
            return int(value)
        return None

    @staticmethod
    def _counts_by(
        session: Session,
        column: Any,
        *,
        non_null: bool = False,
    ) -> dict[str, int]:
        statement = select(column, func.count()).group_by(column)
        if non_null:
            statement = statement.where(column.is_not(None))
        rows = session.execute(statement).all()
        return {
            str(key): int(count) for key, count in rows if key is not None
        }

    @staticmethod
    def _record_to_dict(
        row: TaskRecord, *, include_payload: bool
    ) -> dict[str, Any]:
        record = {
            "task_id": row.task_id,
            "type": row.type,
            "status": row.status,
            "phase": row.phase,
            "progress": row.progress,
            "message": row.message,
            "cabinet": row.cabinet,
            "control_id": row.control_id,
            "control_name": row.control_name,
            "command": row.command,
            "target_state": row.target_state,
            "target_position": row.target_position,
            "force": row.force,
            "failure_code": row.failure_code,
            "failure_reason": row.failure_reason,
            "duration_seconds": row.duration_seconds,
            "created_at": row.created_at,
            "started_at": row.started_at,
            "completed_at": row.completed_at,
            "updated_at": row.updated_at,
        }
        if include_payload:
            record.update(
                {
                    "request": dict(row.request or {}),
                    "result": _dict_or_none(row.result),
                    "business_data": _dict_or_none(row.business_data),
                    "failure_details": _dict_or_none(row.failure_details),
                }
            )
        return record

    @staticmethod
    def _event_to_dict(row: TaskProgressEvent) -> dict[str, Any]:
        return {
            "id": row.id,
            "task_id": row.task_id,
            "sequence": row.sequence,
            "event": row.event,
            "status": row.status,
            "phase": row.phase,
            "progress": row.progress,
            "message": row.message,
            "business_data": _dict_or_none(row.business_data),
            "timestamp": row.timestamp,
            "elapsed_seconds": row.elapsed_seconds,
        }


__all__ = ["TaskRecordStore"]
