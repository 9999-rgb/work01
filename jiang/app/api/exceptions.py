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


def _error_payload(
    message: str,
    details: dict[str, Any] | None = None,
    *,
    failure_code: str = "request_error",
    failure_reason: str | None = None,
) -> dict[str, Any]:
    """Return a stable error envelope while retaining legacy flat details."""
    # A backend may provide a more useful, operator-facing reason in either
    # the exception itself or its structured details.  Preserve that reason
    # instead of replacing it with ``str(exc)`` (which is often only a
    # generic wrapper such as "operation failed").  Empty/non-text values are
    # ignored so malformed details cannot make the response non-serializable.
    resolved_reason: str | None = None
    for candidate in (
        failure_reason,
        details.get("failure_reason") if isinstance(details, dict) else None,
    ):
        if candidate is None or isinstance(candidate, bool):
            continue
        try:
            candidate_text = str(candidate).strip()
        except Exception:  # noqa: BLE001 - defensive error-envelope path
            continue
        if candidate_text:
            resolved_reason = candidate_text
            break
    payload: dict[str, Any] = {
        "error": message,
        "failure_code": failure_code or "request_error",
        "failure_reason": resolved_reason or message,
    }
    if details:
        # Keep the old flat keys for existing clients, but never allow a
        # backend detail to overwrite the canonical error envelope.
        payload["details"] = dict(details)
        for key, value in details.items():
            if key not in payload:
                payload[key] = value
    return payload


def register_exception_handlers(app: FastAPI) -> None:
    """把业务异常与校验异常统一映射为旧版 ``{"error": ...}`` 格式。

    ``ControlRequestError`` 定义在依赖 ROS 的模块中；此处惰性导入，
    使应用在未 source ROS 时也能导入（测试/文档场景）。
    """
    try:
        from control_gateway.ros_node import ControlRequestError
    except (ImportError, ModuleNotFoundError):
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
            details = getattr(exc, "details", None)
            return JSONResponse(
                status_code=status,
                content=_error_payload(
                    str(exc),
                    details,
                    failure_code=str(
                        getattr(exc, "code", None) or "request_error"
                    ),
                    failure_reason=getattr(exc, "failure_reason", None),
                ),
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
            content=_error_payload(
                "；".join(messages),
                failure_code="invalid_request",
            ),
        )

    @app.exception_handler(StarletteHTTPException)
    async def _http_exception(
        request: Request,
        exc: StarletteHTTPException,
    ) -> JSONResponse:
        del request
        return JSONResponse(
            status_code=exc.status_code,
            content=_error_payload(
                str(exc.detail),
                failure_code=f"http_{exc.status_code}",
            ),
            headers=getattr(exc, "headers", None),
        )
