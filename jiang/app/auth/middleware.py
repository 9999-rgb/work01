"""全局鉴权中间件。

策略：默认保护除公开白名单外的所有路径。
- 公开：/health、/auth/login、/docs、/redoc、/openapi.json 与监控页。
- 受保护：控制网关的 REST 路由。凭证从 ``Authorization: Bearer`` 或
  SSE/传感器查询参数 ``?token=`` 提取。除校验 JWT 签名与过期外，
  每次请求都确认用户仍存在且已启用。

通过 ``XCZS_AUTH_ENABLED=false`` 可整体关闭（兼容迁移过渡期）。
"""

from __future__ import annotations

from typing import Any

from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.responses import Response

from app.auth.deps import resolve_active_user
from app.database.engine import async_session

_QUERY_TOKEN_PATHS: frozenset[str] = frozenset(
    {
        "/task/events",
        "/camera.mjpg",
    }
)

# 完全公开的精确路径。
_PUBLIC_PATHS: frozenset[str] = frozenset(
    {
        "/health",
        # 只读的运行态既不暴露控制能力，也供启动脚本在 A/B 热切换期间
        # 判断维护窗口；其鉴权策略与 /health 保持一致。
        "/robot/toolset/status",
        "/auth/login",
        "/docs",
        "/redoc",
        "/openapi.json",
        "/",
        "/monitor.html",
        # The page contains its own login flow, so it must be reachable before
        # a bearer token exists.  Its history APIs remain protected normally.
        "/history.html",
        "/favicon.ico",
    }
)


def _is_public(path: str) -> bool:
    return path in _PUBLIC_PATHS


class AuthMiddleware(BaseHTTPMiddleware):
    """JWT 门禁中间件（幂等：已通过则直接放行）。"""

    def __init__(self, app: Any, *, enabled: bool = True) -> None:
        super().__init__(app)
        self._enabled = enabled

    async def dispatch(
        self,
        request: Request,
        call_next: RequestResponseEndpoint,
    ) -> Response:
        if not self._enabled or _is_public(request.url.path):
            return await call_next(request)
        token = self._extract_token(request)
        if token is None:
            return JSONResponse(
                status_code=401,
                content={"error": "缺少认证凭证。请登录后携带 Bearer token。"},
                headers={"WWW-Authenticate": "Bearer"},
            )
        try:
            async with async_session() as session:
                user = await resolve_active_user(token, session)
        except HTTPException:
            return JSONResponse(
                status_code=401,
                content={"error": "token 无效或已过期。"},
                headers={"WWW-Authenticate": "Bearer"},
            )
        # 路由的角色依赖仍会在自己的 session 中查询完整 User；
        # 这里只缓存经验证的标量身份，不跨 session 传递 ORM 对象。
        request.state.auth_user_id = user.id
        request.state.auth_token = token
        return await call_next(request)

    @staticmethod
    def _extract_token(request: Request) -> str | None:
        authorization = request.headers.get("Authorization")
        if authorization:
            scheme, _, credentials = authorization.partition(" ")
            if scheme.strip().lower() == "bearer" and credentials.strip():
                return credentials.strip()
        # 只对浏览器无法携带 Header 的流式端点支持 token=。
        # 普通 REST/写控制禁止 URL 凭证，避免 token 进入访问日志和历史。
        path = request.url.path
        query_allowed = path in _QUERY_TOKEN_PATHS or path.startswith("/sse/")
        if query_allowed:
            query_token = request.query_params.get("token")
            if query_token:
                return query_token
        return None
