"""控制网关 API 依赖：访问应用状态中的 ControlServer。"""

from __future__ import annotations

from typing import TYPE_CHECKING, Annotated

from fastapi import Depends, HTTPException, Request, status

if TYPE_CHECKING:
    from control_gateway.runner import ControlServer


def get_control_server(request: Request) -> "ControlServer":
    """从 ``app.state.control_server`` 获取 ControlServer。

    该实例由 FastAPI lifespan 启动，或在测试中注入 fake 后端。
    """
    server = getattr(request.app.state, "control_server", None)
    if server is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="控制网关未初始化。",
        )
    return server


ControlServerDep = Annotated["ControlServer", Depends(get_control_server)]
