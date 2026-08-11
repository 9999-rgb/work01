"""错误响应标准化。

保持旧版契约：错误响应为 ``{"error": "<消息>", ...details}``。
- ``ControlRequestError``（业务错误，含 status/details）→ 对应状态码。
- ``RequestValidationError``（Pydantic 422）→ 400 + 汇总消息。
- ``HTTPException`` → ``{"error": detail}``。
"""

from __future__ import annotations

from typing import Any

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException


def _error_payload(message: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    payload: dict[str, Any] = {"error": message}
    if details:
        payload.update(details)
    return payload


def register_exception_handlers(app: FastAPI) -> None:
    """把业务异常与校验异常统一映射为旧版 ``{"error": ...}`` 格式。

    ``ControlRequestError`` 定义在依赖 ROS 的模块中；此处惰性导入，
    使应用在未 source ROS 时也能导入（测试/文档场景）。
    """
    try:
        from control_gateway.ros_node import ControlRequestError
    except ModuleNotFoundError:
        ControlRequestError = None  # type: ignore[assignment]

    if ControlRequestError is not None:

        @app.exception_handler(ControlRequestError)  # type: ignore[arg-type]
        async def _control_request_error(
            request: Request,
            exc: ControlRequestError,  # type: ignore[name-defined]
        ) -> JSONResponse:
            del request
            status = getattr(exc, "status", 500)
            if isinstance(status, bool) or not isinstance(status, int):
                status = 500
            return JSONResponse(
                status_code=status,
                content=_error_payload(str(exc), getattr(exc, "details", None)),
            )

    @app.exception_handler(RequestValidationError)
    async def _validation_error(
        request: Request,
        exc: RequestValidationError,
    ) -> JSONResponse:
        del request
        messages = []
        for error in exc.errors():
            loc = ".".join(str(part) for part in error.get("loc", ()) if part != "body")
            message = error.get("msg", "invalid")
            messages.append(f"{loc}: {message}" if loc else message)
        return JSONResponse(
            status_code=400,
            content=_error_payload("；".join(messages)),
        )

    @app.exception_handler(StarletteHTTPException)
    async def _http_exception(
        request: Request,
        exc: StarletteHTTPException,
    ) -> JSONResponse:
        del request
        return JSONResponse(
            status_code=exc.status_code,
            content=_error_payload(str(exc.detail)),
            headers=getattr(exc, "headers", None),
        )
