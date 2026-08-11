"""SSE 编码与流式辅助工具。

复刻旧版 ``ControlHandler._encode_sse_event`` 的格式，保证前端兼容：
``id:`` / ``event:`` / ``data:`` 字段，空行分隔，JSON 紧凑序列化。
"""

from __future__ import annotations

import asyncio
import json
import queue
from collections.abc import AsyncIterator
from typing import Any

from fastapi import Request

from control_gateway.task_manager import EventHubClosed


def encode_sse_event(event: Any) -> bytes:
    """把一个任务事件 dict 编码为 SSE 帧字节。"""
    if not isinstance(event, dict):
        raise TypeError("任务事件必须是对象")
    event_id = str(event.get("id", "")).replace("\r", "").replace("\n", "")
    event_type = (
        str(event.get("event", "message"))
        .replace("\r", "")
        .replace("\n", "")
    )
    data = json.dumps(
        event.get("data", {}),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )
    fields = []
    if event_id:
        fields.append(f"id: {event_id}")
    fields.extend((f"event: {event_type}", f"data: {data}", "", ""))
    return "\n".join(fields).encode("utf-8")


async def stream_task_events(
    subscription: Any,
    request: Request,
    *,
    heartbeat_seconds: float = 15.0,
) -> AsyncIterator[bytes]:
    """把阻塞式事件订阅桥接到异步 SSE 流。

    ``subscription.get(timeout)`` 是阻塞调用（EventHub 基于线程 Condition），
    通过 ``asyncio.to_thread`` 放到线程池，避免阻塞事件循环。
    每个 SSE 客户端短期占有一个线程池线程，单操作员场景（1-2 个浏览器）足够。
    """
    try:
        while True:
            if await request.is_disconnected():
                return
            try:
                event = await asyncio.to_thread(
                    subscription.get,
                    timeout=heartbeat_seconds,
                )
            except queue.Empty:
                yield b": heartbeat\n\n"
                continue
            yield encode_sse_event(event)
    except EventHubClosed:
        return
    finally:
        subscription.close()
