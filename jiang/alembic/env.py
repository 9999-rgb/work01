"""Alembic 迁移环境：异步引擎 + 应用 metadata。

数据库连接串统一从 ``app.config.settings.database_url`` 读取，与运行时一致。
当前配置层只接受文件型 SQLite。
"""

from __future__ import annotations

import asyncio
from logging.config import fileConfig

from alembic import context
from sqlalchemy import pool
from sqlalchemy.engine import Connection
from sqlalchemy.ext.asyncio import async_engine_from_config

from app.config import settings
from app.database.base import Base

# 导入所有模型，注册到 metadata。
from app.auth.models import User  # noqa: F401
from app.assets.models import Asset, Selection  # noqa: F401
from app.tasks.models import TaskProgressEvent, TaskRecord  # noqa: F401

config = context.config

if config.config_file_name is not None:
    # 迁移也会在 FastAPI/CLI 进程内程序化执行；不得禁用应用已经配置好的
    # logger（fileConfig 默认会把它们全部 disabled）。
    fileConfig(config.config_file_name, disable_existing_loggers=False)

# 程序化迁移可显式传入测试/CLI 数据库；命令行调用仍使用应用配置。
database_url = str(config.attributes.get("database_url", settings.database_url))
config.set_main_option("sqlalchemy.url", database_url.replace("%", "%%"))

target_metadata = Base.metadata


def run_migrations_offline() -> None:
    """离线模式：不连接数据库，仅生成 SQL。"""
    url = config.get_main_option("sqlalchemy.url")
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
    )

    with context.begin_transaction():
        context.run_migrations()


def _run_migrations(connection: Connection) -> None:
    context.configure(connection=connection, target_metadata=target_metadata)
    with context.begin_transaction():
        context.run_migrations()


async def run_async_migrations() -> None:
    """在线模式：异步引擎。"""
    connectable = async_engine_from_config(
        config.get_section(config.config_ini_section, {}),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
        connect_args=(
            {"check_same_thread": False}
            if database_url.startswith("sqlite")
            else {}
        ),
    )

    try:
        async with connectable.connect() as connection:
            await connection.run_sync(_run_migrations)
    finally:
        await connectable.dispose()


def run_migrations_online() -> None:
    asyncio.run(run_async_migrations())


if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
