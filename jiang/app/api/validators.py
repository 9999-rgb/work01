"""共享的 Pydantic 字段校验器，复刻旧版 web_server.py 的语义。

- ``nonempty_string``：非空字符串，去首尾空白（等价旧 ``_nonempty_string``）。
- ``finite_number``：有限浮点数，拒绝 bool 与非有限值（等价旧 ``_finite_number``）。
"""

from __future__ import annotations

import json
import math

from fastapi import HTTPException, Request


def nonempty_string(value: str, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} 必须是非空字符串")
    return value.strip()


def finite_number(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} 必须是数字")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} 必须是数字") from None
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数字")
    return result


async def reject_json_body(request: Request) -> None:
    """拒绝携带非空 JSON 请求体的端点（旧版 ``_require_empty_body``）。

    - 空 body 或 ``{}`` → 放行
    - 非空对象 → 400
    - 非 application/json 且带 body → 415
    """
    body = await request.body()
    if not body:
        return
    content_type = request.headers.get("Content-Type", "")
    # Parameters such as ``charset=utf-8`` are valid, but the media type
    # itself must be exactly JSON.  A substring check would incorrectly
    # accept values such as ``text/application/json-evil``.
    media_type = content_type.partition(";")[0].strip().lower()
    if media_type != "application/json":
        raise HTTPException(status_code=415, detail="Content-Type 必须是 application/json。")
    try:
        data = json.loads(body)
    except (json.JSONDecodeError, UnicodeDecodeError):
        raise HTTPException(status_code=400, detail="JSON 请求体无效。") from None
    if not isinstance(data, dict) or data:
        raise HTTPException(status_code=400, detail="此端点不接受请求体参数。")
