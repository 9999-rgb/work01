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

from app.auth.deps import ActiveTokenChecker
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
    authorization: ActiveTokenChecker | None = None,
) -> AsyncIterator[bytes]:
    """把阻塞式事件订阅桥接到异步 SSE 流。

    EventHub 通过线程安全回调唤醒 asyncio Event；读取本身使用非阻塞
    ``get_nowait``。因此空闲 SSE 不占用 FastAPI 默认线程池，多个长连接也
    不会饿死依赖 ``asyncio.to_thread`` 的控制 API。
    """
    event_loop = asyncio.get_running_loop()
    wake_event = asyncio.Event()

    def notify() -> None:
        try:
            event_loop.call_soon_threadsafe(wake_event.set)
        except RuntimeError:
            # 应用关闭后事件循环可能已停止。
            return

    subscription.set_notify(notify)
    try:
        while True:
            if (
                await request.is_disconnected()
                or (
                    authorization is not None
                    and not await authorization.is_valid()
                )
            ):
                return
            poll_seconds = (
                min(heartbeat_seconds, authorization.recheck_seconds)
                if authorization is not None
                else heartbeat_seconds
            )
            try:
                event = subscription.get_nowait()
            except queue.Empty:
                # clear 后再检查一次，封闭“数据在 clear 前到达”的竞态。
                wake_event.clear()
                try:
                    event = subscription.get_nowait()
                except queue.Empty:
                    try:
                        await asyncio.wait_for(
                            wake_event.wait(),
                            timeout=poll_seconds,
                        )
                    except asyncio.TimeoutError:
                        yield b": heartbeat\n\n"
                    continue
            yield encode_sse_event(event)
    except EventHubClosed:
        return
    finally:
        subscription.set_notify(None)
        subscription.close()
