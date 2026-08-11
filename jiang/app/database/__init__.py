"""数据库层：SQLAlchemy 2.0 async engine + session factory。

数据层与连接串解耦：切换 SQLite → PostgreSQL 只改 ``XCZS_DATABASE_URL``。
"""

from .engine import async_session, engine, get_db, init_database
from .base import Base

__all__ = ["Base", "async_session", "engine", "get_db", "init_database"]
