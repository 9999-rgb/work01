"""全局鉴权中间件。

策略：默认保护除公开白名单外的所有路径。
- 公开：/health、/auth/login、/docs、/redoc、/openapi.json、静态页面、
  传感器流（camera/lidar）、Zenoh SSE（/sse/*）。
- 受保护：控制网关的 REST 路由。凭证从 ``Authorization: Bearer`` 或
  SSE 查询参数 ``?token=`` 提取；仅校验 JWT 签名与过期（无状态）。
  角色门禁（admin）仍由各路由的 ``Depends(require_admin)`` 做完整数据库校验。

通过 ``XCZS_AUTH_ENABLED=false`` 可整体关闭（兼容迁移过渡期）。
"""

from __future__ import annotations

from typing import Any

from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.responses import Response

from app.auth.service import InvalidTokenError, decode_access_token
from app.config import settings

# 完全公开的精确路径。
_PUBLIC_PATHS: frozenset[str] = frozenset(
    {
        "/health",
        "/auth/login",
        "/docs",
        "/redoc",
        "/openapi.json",
        "/",
        "/monitor.html",
        "/favicon.ico",
    }
)

# 公开路径前缀（传感器流、Zenoh SSE、静态资源）。
_PUBLIC_PREFIXES: tuple[str, ...] = (
    "/sse/",
    "/camera",
    "/lidar",
    "/static/",
    "/monitor/",
)


def _is_public(path: str) -> bool:
    if path in _PUBLIC_PATHS:
        return True
    return any(path.startswith(prefix) for prefix in _PUBLIC_PREFIXES)


class AuthMiddleware(BaseHTTPMiddleware):
    """JWT 门禁中间件（幂等：已通过则直接放行）。"""

    async def dispatch(
        self,
        request: Request,
        call_next: RequestResponseEndpoint,
    ) -> Response:
        if not settings.auth_enabled or _is_public(request.url.path):
            return await call_next(request)
        token = self._extract_token(request)
        if token is None:
            return JSONResponse(
                status_code=401,
                content={"error": "缺少认证凭证。请登录后携带 Bearer token。"},
                headers={"WWW-Authenticate": "Bearer"},
            )
        try:
            payload = decode_access_token(token)
        except InvalidTokenError:
            return JSONResponse(
                status_code=401,
                content={"error": "token 无效或已过期。"},
                headers={"WWW-Authenticate": "Bearer"},
            )
        # 无状态门禁通过：把解析出的身份挂到 request.state 供依赖复用。
        request.state.auth_payload = payload
        return await call_next(request)

    @staticmethod
    def _extract_token(request: Request) -> str | None:
        authorization = request.headers.get("Authorization")
        if authorization:
            scheme, _, credentials = authorization.partition(" ")
            if scheme.strip().lower() == "bearer" and credentials.strip():
                return credentials.strip()
        # SSE/EventSource 无法携带 Header，支持查询参数 token=。
        query_token = request.query_params.get("token")
        if query_token:
            return query_token
        return None


def create_auth_middleware(app: Any) -> None:
    """把鉴权中间件挂到 app。"""
    app.add_middleware(AuthMiddleware)
