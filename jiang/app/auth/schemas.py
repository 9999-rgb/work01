"""用户认证与管理的 Pydantic v2 schema。"""

from __future__ import annotations

from datetime import datetime

from pydantic import BaseModel, ConfigDict, Field, field_validator

from app.auth.models import ROLE_ADMIN, ROLE_OPERATOR


class UserCreate(BaseModel):
    """创建用户请求。"""

    model_config = ConfigDict(extra="forbid")

    username: str = Field(
        min_length=3,
        max_length=64,
        description="用户名，3-64 字符",
        examples=["admin"],
    )
    password: str = Field(
        min_length=8,
        max_length=128,
        description="密码，至少 8 字符",
        examples=["changeme123"],
    )
    role: str = Field(
        default=ROLE_OPERATOR,
        description=f"角色：{ROLE_ADMIN} 或 {ROLE_OPERATOR}",
        examples=[ROLE_OPERATOR],
    )

    @field_validator("role")
    @classmethod
    def _validate_role(cls, value: str) -> str:
        if value not in {ROLE_ADMIN, ROLE_OPERATOR}:
            raise ValueError(f"role 必须是 {ROLE_ADMIN} 或 {ROLE_OPERATOR}")
        return value


class UserUpdate(BaseModel):
    """更新用户请求（管理员操作）。"""

    model_config = ConfigDict(extra="forbid")

    password: str | None = Field(
        default=None,
        min_length=8,
        max_length=128,
        description="新密码（可选）",
    )
    role: str | None = Field(
        default=None,
        description=f"新角色（可选）：{ROLE_ADMIN} 或 {ROLE_OPERATOR}",
    )
    is_active: bool | None = Field(
        default=None,
        description="是否启用；禁用后该用户无法登录",
    )

    @field_validator("role")
    @classmethod
    def _validate_role(cls, value: str | None) -> str | None:
        if value is not None and value not in {ROLE_ADMIN, ROLE_OPERATOR}:
            raise ValueError(f"role 必须是 {ROLE_ADMIN} 或 {ROLE_OPERATOR}")
        return value


class UserResponse(BaseModel):
    """用户信息（响应，不含密码）。"""

    model_config = ConfigDict(from_attributes=True)

    id: int
    username: str
    role: str
    is_active: bool
    created_at: datetime


class LoginRequest(BaseModel):
    """登录请求。"""

    model_config = ConfigDict(extra="forbid")

    username: str = Field(description="用户名", examples=["admin"])
    password: str = Field(description="密码", examples=["changeme123"])


class TokenResponse(BaseModel):
    """登录成功响应：JWT Bearer Token + 用户信息。"""

    access_token: str
    token_type: str = "bearer"
    expires_in: int = Field(description="token 有效期（秒）")
    user: UserResponse
