"""SQLite 资产目录 / 选择 store（同步 SQLAlchemy）。

复用与 auth 相同的 ORM 模型与数据库（``XCZS_DATABASE_URL``，默认 ``xczs.db``），
但资产库本身是同步的（文件复制 / 校验 + 被 CLI 与启动脚本共享），因此这里用
**同步** ``Session``。Web 层经 ``asyncio.to_thread`` 调用；CLI 与启动脚本直接
调用。这样目录与选择只有一份实现，避免同步 / 异步两套代码路径。

SQLite 连接参数（``check_same_thread=False``、WAL、busy_timeout）与
``app.database.engine`` 的异步引擎保持一致，保证二者可安全并发访问同一文件。
"""

from __future__ import annotations

import os
import threading
from typing import Optional

from sqlalchemy import create_engine, event, select
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session, sessionmaker

from app.assets.models import Asset, Selection
from app.config import settings
from app.database.migrations import migrate_database

from control_gateway.asset_library import (
    AssetNotFoundError,
    AssetRecord,
    AssetSelection,
)

_SQLITE_TIMEOUT_SEC = 10
_SQLITE_BUSY_TIMEOUT_MS = 5000
_SELECTION_ROW_ID = 1

#: 按连接串缓存引擎与会话工厂：engine 是进程级单例（每个 URL 一个连接池），
#: 且测试里每用例覆盖 ``XCZS_DATABASE_URL`` 时自然得到隔离的引擎。
_ENGINES: dict[str, Engine] = {}
_SESSION_FACTORIES: dict[str, sessionmaker] = {}
#: 已迁移的 URL 集合：迁移只随 store 首次创建执行一次。
_SCHEMA_ENSURED: set[str] = set()
# 同一进程内的并发首建必须共享一个迁移/engine 初始化临界区，避免创建多个
# 连接池后只保留最后一个引用。
_STORE_INIT_LOCK = threading.RLock()


def _sync_database_url(database_url: Optional[str] = None) -> str:
    """解析同步引擎连接串。

    优先显式参数，其次 ``XCZS_DATABASE_URL`` 环境变量，最后 ``settings``
    （与 auth 同库）。SQLite 异步驱动后缀 ``+aiosqlite`` 剥掉后交给同步驱动。
    """
    url = database_url or os.environ.get("XCZS_DATABASE_URL") or settings.database_url
    if url.startswith("sqlite+aiosqlite:///"):
        return url.replace("+aiosqlite", "", 1)
    return url


def _set_sqlite_pragmas(engine: Engine) -> None:
    """SQLite 连接级 PRAGMA，与异步引擎一致（WAL + busy_timeout + 外键）。"""

    @event.listens_for(engine, "connect")
    def _on_connect(dbapi_connection: object, _record: object) -> None:
        cursor = dbapi_connection.cursor()  # type: ignore[union-attr]
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute(f"PRAGMA busy_timeout={_SQLITE_BUSY_TIMEOUT_MS}")
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()


class SqlAssetStore:
    """SQLite 目录 + 选择 store，实现 ``AssetStore`` 协议（同步）。"""

    def __init__(self, database_url: Optional[str] = None) -> None:
        url = _sync_database_url(database_url)
        with _STORE_INIT_LOCK:
            if url not in _SCHEMA_ENSURED:
                migrate_database(url)
                _SCHEMA_ENSURED.add(url)
            if url not in _ENGINES:
                connect_args = {
                    "check_same_thread": False,
                    "timeout": _SQLITE_TIMEOUT_SEC,
                }
                engine = create_engine(url, connect_args=connect_args)
                _set_sqlite_pragmas(engine)
                _ENGINES[url] = engine
                _SESSION_FACTORIES[url] = sessionmaker(
                    bind=engine, expire_on_commit=False
                )
            self._engine = _ENGINES[url]
            self._session_factory = _SESSION_FACTORIES[url]

    # ── AssetStore 实现 ─────────────────────────────────────────────────

    def list_assets(self) -> list[AssetRecord]:
        with self._session_factory() as session:
            rows = session.execute(select(Asset).order_by(Asset.id)).scalars().all()
            return [self._record_from_model(row) for row in rows]

    def get_asset(self, kind: str, name: str) -> AssetRecord:
        with self._session_factory() as session:
            row = session.execute(
                select(Asset).where(Asset.kind == kind, Asset.name == name)
            ).scalar_one_or_none()
            if row is None:
                raise AssetNotFoundError(kind, name)
            return self._record_from_model(row)

    def put_asset(self, record: AssetRecord) -> None:
        with self._session_factory() as session:
            existing = session.execute(
                select(Asset).where(
                    Asset.kind == record.kind, Asset.name == record.name
                )
            ).scalar_one_or_none()
            if existing is None:
                session.add(self._model_from_record(record))
            else:
                # 原地更新（kind+name 是唯一键），避免 delete+insert 在同一次
                # flush 里因执行顺序触发 UNIQUE 约束冲突。
                existing.version = record.version
                existing.description = record.description
                existing.path = record.path
                existing.files = dict(record.files)
                existing.references = dict(record.references)
                existing.imported_at = record.imported_at
                existing.validated = record.validated
            session.commit()

    def delete_asset(self, kind: str, name: str) -> None:
        with self._session_factory() as session:
            existing = session.execute(
                select(Asset).where(Asset.kind == kind, Asset.name == name)
            ).scalar_one_or_none()
            if existing is not None:
                session.delete(existing)
                session.commit()

    def load_selection(self) -> AssetSelection:
        with self._session_factory() as session:
            row = session.get(Selection, _SELECTION_ROW_ID)
            if row is None:
                return AssetSelection()
            return AssetSelection(
                scene=row.scene,
                cabinet=row.cabinet,
                toolset=row.toolset,
            )

    def save_selection(self, selection: AssetSelection) -> None:
        with self._session_factory() as session:
            row = session.get(Selection, _SELECTION_ROW_ID)
            if row is None:
                row = Selection(id=_SELECTION_ROW_ID)
                session.add(row)
            row.scene = selection.scene
            row.cabinet = selection.cabinet
            row.toolset = selection.toolset
            session.commit()

    # ── 映射 ───────────────────────────────────────────────────────────

    @staticmethod
    def _record_from_model(row: Asset) -> AssetRecord:
        return AssetRecord(
            kind=row.kind,
            name=row.name,
            version=row.version,
            description=row.description or "",
            path=row.path,
            files=dict(row.files or {}),
            references=dict(row.references or {}),
            imported_at=row.imported_at or "",
            validated=bool(row.validated),
        )

    @staticmethod
    def _model_from_record(record: AssetRecord) -> Asset:
        return Asset(
            kind=record.kind,
            name=record.name,
            version=record.version,
            description=record.description,
            path=record.path,
            files=dict(record.files),
            references=dict(record.references),
            imported_at=record.imported_at,
            validated=record.validated,
        )


__all__ = ["SqlAssetStore"]
