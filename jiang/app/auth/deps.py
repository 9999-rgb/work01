"""认证依赖：从请求中解析当前用户与角色。"""

from __future__ import annotations

from typing import Annotated

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.auth.models import ROLE_ADMIN, User
from app.auth.service import InvalidTokenError, decode_access_token
from app.database.engine import get_db

_bearer = HTTPBearer(auto_error=False)


async def _sse_credentials(
    request: Request,
) -> HTTPAuthorizationCredentials | None:
    """Header 优先，回退查询参数 ``token=``（EventSource 场景）。

    EventSource 无法携带自定义 Header，SSE 端点用 ``?token=`` 传递凭证。
    """
    credentials = await _bearer(request)  # type: ignore[arg-type]
    if credentials is not None:
        return credentials
    token = request.query_params.get("token")
    if token:
        return HTTPAuthorizationCredentials(scheme="Bearer", credentials=token)
    return None


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
    try:
        payload = decode_access_token(credentials.credentials)
        user_id = int(payload["sub"])
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
        )
    return user


async def get_current_user(
    credentials: Annotated[HTTPAuthorizationCredentials | None, Depends(_bearer)],
    session: Annotated[AsyncSession, Depends(get_db)],
) -> User:
    """标准 Header 鉴权依赖（REST 端点）。"""
    return await _resolve_user(credentials, session)


async def get_current_user_sse(
    credentials: Annotated[
        HTTPAuthorizationCredentials | None,
        Depends(_sse_credentials),
    ],
    session: Annotated[AsyncSession, Depends(get_db)],
) -> User:
    """Header + 查询参数兜底鉴权依赖（SSE/事件流端点）。"""
    return await _resolve_user(credentials, session)


def require_role(*roles: str):
    """生成角色门禁依赖：当前用户必须是给定角色之一。"""

    async def _dependency(user: Annotated[User, Depends(get_current_user)]) -> User:
        if user.role not in roles:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail=f"需要角色 {', '.join(roles)} 之一，当前为 {user.role}。",
            )
        return user

    return _dependency


require_admin = require_role(ROLE_ADMIN)
