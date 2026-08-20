"""SQLAlchemy 2.0 异步引擎与会话工厂。

- 默认 SQLite + aiosqlite，开启 WAL 模式 + busy_timeout，适配单机/单写者。
- 数据层与连接串解耦：把 ``XCZS_DATABASE_URL`` 换成 PostgreSQL 的
  ``postgresql+asyncpg://...`` 即可无痛迁移，模型与迁移脚本保持一致。
- SQLite 特有的连接参数（``check_same_thread``）仅对 SQLite 生效。
"""

from __future__ import annotations

from typing import AsyncIterator

from sqlalchemy import event
from sqlalchemy.ext.asyncio import (
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)
from sqlalchemy.engine import Engine

from app.config import settings

_SQLITE_TIMEOUT_SEC = 10
_SQLITE_BUSY_TIMEOUT_MS = 5000


def _is_sqlite(url: str) -> bool:
    return url.startswith("sqlite")


# 连接参数按方言区分：PostgreSQL (asyncpg) 不接受 SQLite 专有参数。
_connect_args: dict = {}
if _is_sqlite(settings.database_url):
    # aiosqlite 的 check_same_thread：FastAPI 的 session 可能跨线程复用
    # （事件循环线程池），保持 False 兼容。
    _connect_args = {
        "check_same_thread": False,
        "timeout": _SQLITE_TIMEOUT_SEC,
    }

engine = create_async_engine(
    settings.database_url,
    echo=False,
    connect_args=_connect_args,
    pool_pre_ping=True,
)


@event.listens_for(engine.sync_engine, "connect")
def _set_sqlite_pragma(dbapi_connection: object, _record: object) -> None:
    """SQLite 连接级 PRAGMA：WAL + busy_timeout。

    只对 SQLite 生效；PostgreSQL 连接不会触发。
    """
    if not _is_sqlite(settings.database_url):
        return
    cursor = dbapi_connection.cursor()  # type: ignore[union-attr]
    cursor.execute("PRAGMA journal_mode=WAL")
    cursor.execute(f"PRAGMA busy_timeout={_SQLITE_BUSY_TIMEOUT_MS}")
    cursor.execute("PRAGMA foreign_keys=ON")
    cursor.close()


async_session = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autoflush=False,
)


async def get_db() -> AsyncIterator[AsyncSession]:
    """FastAPI 依赖：为每个请求提供一个异步会话。

    依赖结束即关闭会话，交还连接池。
    """
    async with async_session() as session:
        yield session


async def init_database() -> None:
    """确保 SQLite 数据目录存在；创建所有表（非迁移路径的兜底）。

    正式建表使用 Alembic：``cd jiang && alembic upgrade head``。
    """
    if _is_sqlite(settings.database_url):
        path = settings.database_url.split("///", 1)[-1]
        from pathlib import Path

        Path(path).parent.mkdir(parents=True, exist_ok=True)
    # 幂等导入所有模型以注册到 metadata。
    from app.auth.models import User  # noqa: F401
    from app.assets.models import Asset, Selection  # noqa: F401
    from app.tasks.models import TaskProgressEvent, TaskRecord  # noqa: F401

    async with engine.begin() as conn:
        from app.database.base import Base

        await conn.run_sync(Base.metadata.create_all)
