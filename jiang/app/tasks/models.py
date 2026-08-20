"""任务记录 ORM 模型。

与 ``app.assets.models`` 共用同一 ``Base`` 与 ``XCZS_DATABASE_URL``，落在同一
数据库（默认 SQLite ``xczs.db``），由 Alembic 统一迁移。

``TaskRecord`` 是每个 navigate / operate 任务一行（``task_id`` 唯一）的最终
快照，展开 request / result 里的常用字段为独立列（报表维度），同时保留完整
JSON 供明细展示。``TaskProgressEvent`` 是 ~1Hz 全量进度事件明细，按 ``id``
自增排序还原时间线。

时间戳统一用 Float（unix 秒）：与 TaskManager 快照及录制 manifest.json 一致，
无时区歧义、便于区间查询。
"""

from __future__ import annotations

from sqlalchemy import Float, JSON, String
from sqlalchemy.orm import Mapped, mapped_column

from app.database.base import Base


class TaskRecord(Base):
    __tablename__ = "task_records"

    id: Mapped[int] = mapped_column(primary_key=True)
    task_id: Mapped[str] = mapped_column(String(40), unique=True, index=True)
    type: Mapped[str] = mapped_column(String(16), index=True)
    status: Mapped[str] = mapped_column(String(16), index=True)
    phase: Mapped[str] = mapped_column(String(32), default="")
    progress: Mapped[float] = mapped_column(Float, default=0.0)
    message: Mapped[str] = mapped_column(String(512), default="")

    # 从 request 提取的报表维度。
    cabinet: Mapped[str | None] = mapped_column(String(64), nullable=True, index=True)
    control_id: Mapped[str | None] = mapped_column(String(64), nullable=True, index=True)
    control_name: Mapped[str | None] = mapped_column(String(64), nullable=True)
    command: Mapped[str | None] = mapped_column(String(16), nullable=True)
    target_state: Mapped[str | None] = mapped_column(String(64), nullable=True)
    target_position: Mapped[float | None] = mapped_column(Float, nullable=True)
    force: Mapped[float | None] = mapped_column(Float, nullable=True)

    # 完整载荷。
    request: Mapped[dict] = mapped_column(JSON, default=dict)
    result: Mapped[dict | None] = mapped_column(JSON, nullable=True)
    business_data: Mapped[dict | None] = mapped_column(JSON, nullable=True)

    failure_code: Mapped[str | None] = mapped_column(String(64), nullable=True, index=True)
    failure_reason: Mapped[str | None] = mapped_column(String(512), nullable=True)
    failure_details: Mapped[dict | None] = mapped_column(JSON, nullable=True)
    duration_seconds: Mapped[float | None] = mapped_column(Float, nullable=True)

    created_at: Mapped[float] = mapped_column(Float, index=True)
    started_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    completed_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    updated_at: Mapped[float] = mapped_column(Float, default=0.0)


class TaskProgressEvent(Base):
    __tablename__ = "task_progress_events"

    id: Mapped[int] = mapped_column(primary_key=True)
    task_id: Mapped[str] = mapped_column(String(40), index=True)
    # EventHub 全局序号：仅供参考 / 关联 SSE，不用于排序（可能溢出或跨任务不连续）。
    sequence: Mapped[int | None] = mapped_column(nullable=True)
    event: Mapped[str] = mapped_column(String(32))
    status: Mapped[str] = mapped_column(String(16), default="")
    phase: Mapped[str] = mapped_column(String(32), default="")
    progress: Mapped[float] = mapped_column(Float, default=0.0)
    message: Mapped[str] = mapped_column(String(512), default="")
    business_data: Mapped[dict | None] = mapped_column(JSON, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float, index=True)
    elapsed_seconds: Mapped[float | None] = mapped_column(Float, nullable=True)
