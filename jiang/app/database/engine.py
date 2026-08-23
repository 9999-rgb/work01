"""SQLAlchemy 2.0 异步引擎与会话工厂。

当前运行时只支持文件型 SQLite + aiosqlite，开启 WAL 与 busy_timeout，适配
单机/单写者。配置层会在创建引擎前拒绝尚未完整适配的数据库方言。
"""

from __future__ import annotations

import asyncio
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


# aiosqlite 的 check_same_thread：FastAPI 的 session 可能跨线程复用
# （事件循环线程池），保持 False 兼容。
_connect_args: dict = {
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
    """SQLite 连接级 PRAGMA：WAL + busy_timeout + 外键。"""
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
    """在事件循环外执行唯一的 Alembic 启动迁移。"""
    from app.database.migrations import migrate_database

    await asyncio.to_thread(migrate_database, settings.database_url)
