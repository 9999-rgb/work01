"""资产目录与选择 ORM 模型。

与 ``app.auth.models.User`` 共用同一 ``Base`` 与 ``XCZS_DATABASE_URL``，因此
资产目录 / 选择与用户表落在同一数据库（默认 SQLite ``xczs.db``），由 Alembic
统一迁移。

``Asset`` 是导入资产目录的索引（对应 ``AssetRecord``）；``Selection`` 是单行
表（``id`` 固定为 1），对应当前运行的场景 / 柜体组合。
"""

from __future__ import annotations

from sqlalchemy import JSON, Boolean, String, UniqueConstraint
from sqlalchemy.orm import Mapped, mapped_column

from app.database.base import Base


class Asset(Base):
    __tablename__ = "assets"
    __table_args__ = (
        UniqueConstraint("kind", "name", name="uq_assets_kind_name"),
    )

    id: Mapped[int] = mapped_column(primary_key=True)
    kind: Mapped[str] = mapped_column(String(16), index=True)
    name: Mapped[str] = mapped_column(String(64))
    version: Mapped[str] = mapped_column(String(32))
    description: Mapped[str] = mapped_column(String(256), default="")
    path: Mapped[str] = mapped_column(String(256))
    files: Mapped[dict] = mapped_column(JSON)
    references: Mapped[dict] = mapped_column(JSON)
    imported_at: Mapped[str] = mapped_column(String(40), default="")
    validated: Mapped[bool] = mapped_column(Boolean, default=False)


class Selection(Base):
    __tablename__ = "selection"

    # 单行表：主键固定为 1，读写都取这一行。
    id: Mapped[int] = mapped_column(primary_key=True)
    scene: Mapped[str | None] = mapped_column(String(64), nullable=True)
    cabinet: Mapped[str | None] = mapped_column(String(64), nullable=True)
