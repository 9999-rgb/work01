"""用户 ORM 模型。

角色：``admin``（用户管理 + 全部操作）与 ``operator``（仅任务操作）。

注意：``role`` 目前仅是**建议性**元数据——鉴权只校验登录态，任务 API 并不
按角色拒绝请求；``admin``/``operator`` 的区分仅体现在用户管理端点的
``require_admin`` 依赖上。若未来需要真正的 RBAC 权限隔离，需在任务路由层
按角色显式授权。
"""

from __future__ import annotations

from datetime import datetime

from sqlalchemy import Boolean, DateTime, String, func
from sqlalchemy.orm import Mapped, mapped_column

from app.database.base import Base

ROLE_ADMIN = "admin"
ROLE_OPERATOR = "operator"
VALID_ROLES = frozenset({ROLE_ADMIN, ROLE_OPERATOR})


class User(Base):
    __tablename__ = "users"

    id: Mapped[int] = mapped_column(primary_key=True)
    username: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    hashed_password: Mapped[str] = mapped_column(String(128))
    role: Mapped[str] = mapped_column(String(16), default=ROLE_OPERATOR)
    is_active: Mapped[bool] = mapped_column(Boolean, default=True)
    created_at: Mapped[datetime] = mapped_column(
        server_default=func.now(),
    )
    updated_at: Mapped[datetime] = mapped_column(
        server_default=func.now(),
        onupdate=func.now(),
    )
