"""FastAPI 应用工厂。

统一承载：控制网关 REST API、任务 SSE、Zenoh SSE、传感器流（MJPEG/WebSocket）、
用户认证、静态页面（monitor.html）。默认单端口（8090）。

``create_app`` 支持注入 control_server / sensor_state / zenoh_source，便于
测试（fake 后端）与分段启用子系统。
"""

from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Query, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import get_redoc_html, get_swagger_ui_html
from fastapi.staticfiles import StaticFiles

from app.api.exceptions import register_exception_handlers
from app.api.router import api_router
from app.auth.bootstrap import bootstrap_admin
from app.auth.middleware import AuthMiddleware
from app.auth.router import router as auth_router
from app.config import settings
from app.database.base import Base  # noqa: F401  (注册所有模型)
from app.database.engine import async_session, engine, init_database

logger = logging.getLogger(__name__)

_DESCRIPTION = """XCZS 巡操机器人控制柜操作闭环仿真系统的 Web 控制网关。

## 认证
除 `/auth/login`、`/health`、传感器流与静态页面外，所有端点需要 JWT Bearer Token。
在 Swagger UI 右上角点击 **Authorize** 输入 token，或使用：
```
Authorization: Bearer <access_token>
```
SSE 端点 `/task/events` 因 EventSource 无法携带 Header，用 `?token=` 传参。

## 约定
- 异步操作返回 **202**，通过 `GET /task/{id}/status` 轮询结果。
- 录制与回放互斥：回放活跃时拒绝所有写操作（**409**）。
- 任务管理器全局互斥：同时最多一个活跃任务。
- 导航与操作分离：`/task/operate` 绝不隐式导航。
"""

_OPENAPI_TAGS = [
    {"name": "认证", "description": "登录、用户管理"},
    {"name": "健康检查", "description": "系统可用性"},
    {"name": "柜体管理", "description": "控制柜查看与操作"},
    {"name": "任务操作", "description": "导航、操作、重置任务"},
    {"name": "导航", "description": "Nav2 导航控制"},
    {"name": "手动控制", "description": "底盘和机械臂手动指令"},
    {"name": "录制回放", "description": "rosbag2 录制与数据/任务回放"},
    {"name": "机器人", "description": "机器人能力与适配参数"},
    {"name": "SSE 事件", "description": "Server-Sent Events 实时事件流"},
    {"name": "传感器", "description": "相机 MJPEG 流、LiDAR WebSocket"},
]


def create_app(
    *,
    control_server: Any = None,
    sensor_state: Any = None,
    zenoh_source: Any = None,
    sensor_runtime: Any = None,
    enable_db: bool = True,
    auth_enabled: bool | None = None,
    docs: bool = True,
    swagger_token: str | None = None,
    static_dir: str | Path | None = None,
) -> FastAPI:
    """创建 FastAPI 应用。

    参数：
    - ``control_server``: 测试时注入 fake 后端；生产由入口创建真实 ControlServer。
    - ``sensor_state`` / ``zenoh_source``: 可选注入，供测试用。
    - ``enable_db``: 是否初始化数据库（测试可关闭）。
    - ``auth_enabled``: 覆盖全局鉴权开关（None = 用 settings）。
    - ``docs``: 是否暴露 /docs、/redoc、/openapi.json。
    """
    effective_auth = settings.auth_enabled if auth_enabled is None else auth_enabled

    # 若配置了 swagger_token，则用自定义 /docs 保护 Swagger UI。
    # 参数未传入时回退读取 env（支持调用方测试注入）。
    _token = swagger_token if swagger_token is not None else settings.swagger_token
    protect_docs = bool(docs and _token)
    app = FastAPI(
        title="XCZS 巡操机器人控制 API",
        description=_DESCRIPTION,
        version="2.0.0",
        openapi_tags=_OPENAPI_TAGS,
        docs_url=None if protect_docs else ("/docs" if docs else None),
        redoc_url=None if protect_docs else ("/redoc" if docs else None),
        openapi_url="/openapi.json" if docs else None,
    )

    if protect_docs:
        @app.get("/docs", include_in_schema=False)
        async def _protected_swagger(
            token: str | None = Query(default=None),
        ) -> Any:
            if token != _token:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Swagger UI 需要 ?token= 访问。",
                )
            return get_swagger_ui_html(
                openapi_url="/openapi.json",
                title="XCZS API Docs",
            )

        @app.get("/redoc", include_in_schema=False)
        async def _protected_redoc(
            token: str | None = Query(default=None),
        ) -> Any:
            if token != _token:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="ReDoc 需要 ?token= 访问。",
                )
            return get_redoc_html(
                openapi_url="/openapi.json",
                title="XCZS API Docs",
            )

    # 应用状态：供路由依赖读取。
    app.state.control_server = control_server
    app.state.sensor_state = sensor_state
    app.state.sensor_runtime = sensor_runtime
    app.state.zenoh_source = zenoh_source
    app.state.auth_enabled = effective_auth

    # CORS：内网工具 + JWT 鉴权，允许任意 Origin（兼容不同主机访问）。
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=False,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["Content-Type", "Last-Event-ID", "Authorization"],
    )
    if effective_auth:
        app.add_middleware(AuthMiddleware)

    register_exception_handlers(app)

    app.include_router(auth_router)
    app.include_router(api_router)

    # 传感器与 Zenoh SSE 路由：state 未注入时端点返回 503。
    from app.sensors.router import router as sensor_router
    from app.sse.router import router as sse_router

    app.include_router(sensor_router)
    app.include_router(sse_router)

    # 静态页面（monitor.html）。路由未匹配时才回退到文件服务。
    if static_dir is not None:
        app.mount(
            "/",
            StaticFiles(directory=str(static_dir), html=True),
            name="static",
        )

    _attach_lifespan(
        app,
        control_server=control_server,
        sensor_runtime=sensor_runtime,
        zenoh_source=zenoh_source,
        enable_db=enable_db,
    )
    return app


def _attach_lifespan(
    app: FastAPI,
    *,
    control_server: Any,
    sensor_runtime: Any,
    zenoh_source: Any,
    enable_db: bool,
) -> None:
    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        if enable_db:
            await init_database()
            async with async_session() as session:
                await bootstrap_admin(session)

        server = getattr(_app.state, "control_server", None)
        # 真实 ControlServer 有 start/stop；测试 fake 通常没有（保持不动）。
        if server is not None and callable(getattr(server, "start", None)):
            await asyncio.to_thread(server.start, start_http=False)
            logger.info("ControlServer 已启动（HTTP 由 uvicorn 接管）")

        runtime = getattr(_app.state, "sensor_runtime", None)
        if runtime is not None and callable(getattr(runtime, "start", None)):
            runtime.start()
            logger.info("传感器 ROS 运行时已启动")

        yield

        if server is not None and callable(getattr(server, "stop", None)):
            try:
                await asyncio.to_thread(server.stop)
            except Exception:  # noqa: BLE001 - 关闭尽力而为
                logger.exception("ControlServer 停止时发生异常")

        if runtime is not None and callable(getattr(runtime, "stop", None)):
            try:
                runtime.stop()
            except Exception:  # noqa: BLE001 - 关闭尽力而为
                logger.exception("传感器运行时停止时发生异常")

        if zenoh_source is not None and callable(getattr(zenoh_source, "close", None)):
            try:
                zenoh_source.close()
            except Exception:  # noqa: BLE001 - 关闭尽力而为
                logger.exception("Zenoh 源关闭时发生异常")

        if enable_db:
            await engine.dispose()

    app.router.lifespan_context = lifespan
