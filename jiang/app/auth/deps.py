"""认证依赖：从请求中解析当前用户与角色。"""

from __future__ import annotations

import logging
import math
import time
from typing import Annotated

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.auth.models import ROLE_ADMIN, User
from app.auth.service import InvalidTokenError, decode_access_token
from app.database.engine import async_session, get_db

logger = logging.getLogger(__name__)
STREAM_AUTH_RECHECK_SECONDS = 5.0
MIN_STREAM_AUTH_RECHECK_SECONDS = 0.05

_bearer = HTTPBearer(auto_error=False)


async def _resolve_user(
    credentials: HTTPAuthorizationCredentials | None,
    session: AsyncSession,
) -> User:
    if credentials is None or credentials.scheme.lower() != "bearer":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="缺少认证凭证。请在 Authorization: Bearer <token> 中携带登录 token。",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return await resolve_active_user(credentials.credentials, session)


async def resolve_active_user(token: str, session: AsyncSession) -> User:
    """校验 token 并从数据库重新解析启用状态的用户。

    不信任 JWT 中缓存的角色或启用状态，因此删除、禁用和角色
    变更在下一个请求立即生效。
    """
    try:
        payload = decode_access_token(token)
        user_id = int(payload["sub"])
        if user_id <= 0:
            raise ValueError("user id must be positive")
    except (InvalidTokenError, KeyError, TypeError, ValueError):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="token 无效或已过期。",
            headers={"WWW-Authenticate": "Bearer"},
        ) from None
    result = await session.execute(select(User).where(User.id == user_id))
    user = result.scalar_one_or_none()
    if user is None or not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="用户不存在或已被禁用。",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user


class ActiveTokenChecker:
    """为长连接定期重验 token 过期时间和数据库用户状态。"""

    def __init__(
        self,
        *,
        token: str | None,
        user_id: int | None,
        enabled: bool,
        initially_validated: bool = True,
        recheck_seconds: float = STREAM_AUTH_RECHECK_SECONDS,
    ) -> None:
        if (
            isinstance(recheck_seconds, bool)
            or not isinstance(recheck_seconds, (int, float))
            or not math.isfinite(float(recheck_seconds))
            or float(recheck_seconds) < 0.0
        ):
            raise ValueError(
                "recheck_seconds must be a finite nonnegative number."
            )
        self._token = token
        self._user_id = user_id
        self._enabled = enabled
        # A zero request still performs the first validation immediately, but
        # subsequent checks are rate-limited so an empty SSE/WS stream cannot
        # turn into a busy loop and database denial-of-service.
        self.recheck_seconds = max(
            MIN_STREAM_AUTH_RECHECK_SECONDS,
            float(recheck_seconds),
        )
        self._next_check = (
            time.monotonic() + self.recheck_seconds
            if initially_validated and enabled
            else 0.0
        )

    @classmethod
    def from_request(cls, request: Request) -> "ActiveTokenChecker":
        return cls(
            token=getattr(request.state, "auth_token", None),
            user_id=getattr(request.state, "auth_user_id", None),
            enabled=bool(getattr(request.app.state, "auth_enabled", True)),
        )

    async def is_valid(self) -> bool:
        if not self._enabled:
            return True
        now = time.monotonic()
        if now < self._next_check:
            return True
        if not self._token or self._user_id is None:
            return False
        self._next_check = now + self.recheck_seconds
        try:
            async with async_session() as session:
                user = await resolve_active_user(self._token, session)
        except HTTPException:
            return False
        except Exception:  # noqa: BLE001 - 长连接鉴权必须 fail closed
            logger.exception("长连接用户状态重验失败")
            return False
        return user.id == self._user_id


async def get_current_user(
    credentials: Annotated[HTTPAuthorizationCredentials | None, Depends(_bearer)],
    session: Annotated[AsyncSession, Depends(get_db)],
) -> User:
    """标准 Header 鉴权依赖（REST 端点）。"""
    return await _resolve_user(credentials, session)


async def require_admin(request: Request) -> User | None:
    """Require an administrator only when application authentication is on.

    ``XCZS_AUTH_ENABLED=false`` is documented as a complete compatibility
    switch, not merely as a way to bypass the middleware.  Resolve credentials
    lazily here so a deliberately database-free no-auth test/deployment does
    not create a needless session either.
    """
    if not bool(getattr(request.app.state, "auth_enabled", True)):
        return None
    credentials = await _bearer(request)  # type: ignore[arg-type]
    async with async_session() as session:
        user = await _resolve_user(credentials, session)
    if user.role != ROLE_ADMIN:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=f"需要角色 {ROLE_ADMIN}，当前为 {user.role}。",
        )
    return user
