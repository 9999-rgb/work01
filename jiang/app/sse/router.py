"""Zenoh CDR → HTTP SSE 桥（FastAPI 版）。

从 ``sse_bridge.py`` 迁移，复用其 ``ZenohSource``（线程安全、按 key 订阅扇出）。
每个 SSE 连接注册一个 listener，用有界 asyncio Queue 和合并调度
保持浏览器端高频 ROS 2 话题不堆积。
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from collections.abc import AsyncIterator
from typing import Any

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse

from app.auth.deps import ActiveTokenChecker
from sse_bridge import ZenohSource
from zenoh_key import normalize_ros_key

MONITOR_UPDATE_INTERVAL_SECONDS = 1.0
HEARTBEAT_SECONDS = 15.0
router = APIRouter(tags=["SSE 事件"])


def _get_zenoh_source(request: Request) -> ZenohSource:
    source = getattr(request.app.state, "zenoh_source", None)
    if source is None:
        raise HTTPException(status_code=503, detail="Zenoh 桥未初始化。")
    return source


def _normalize_ros_key(key: str) -> str:
    """只允许单个确定的 ROS 话题 key，禁止 Zenoh 通配订阅。"""
    try:
        return normalize_ros_key(key)
    except ValueError as error:
        raise HTTPException(
            status_code=400,
            detail=str(error),
        ) from error


@router.get(
    "/sse/{key:path}",
    summary="Zenoh 话题 SSE 流",
    description="""订阅一个 Zenoh key 表达式并把最新值以 SSE 推送给浏览器。

- ``/sse/xczs/odom`` 订阅 ``xczs/odom``
- ``/sse/xczs/odom/json`` 后缀自动去除（后端自行 CDR 解码）
- 事件格式：``event:PUT`` + data 包装（key/value/encoding/timestamp）
""",
)
async def zenoh_sse(request: Request, key: str) -> StreamingResponse:
    source = _get_zenoh_source(request)
    key = _normalize_ros_key(key)
    authorization = ActiveTokenChecker.from_request(request)

    events: asyncio.Queue[tuple[str, str]] = asyncio.Queue(maxsize=1)
    event_loop = asyncio.get_running_loop()
    pending_lock = threading.Lock()
    pending_item: tuple[str, str] | None = None
    delivery_scheduled = False

    def deliver_latest() -> None:
        nonlocal delivery_scheduled, pending_item
        with pending_lock:
            item = pending_item
            pending_item = None
            delivery_scheduled = False
        if item is None:
            return
        if events.full():
            try:
                events.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            events.put_nowait(item)
        except asyncio.QueueFull:
            # 另一个并发回调已经放入了更新的数据。
            pass

    def on_data(topic: str, json_str: str) -> None:
        nonlocal delivery_scheduled, pending_item
        # Zenoh 回调位于中间件线程。投递回事件循环后始终只保留
        # 最新一条。delivery_scheduled 确保高频样本不会在事件循环
        # callback 队列中无界堆积。
        with pending_lock:
            pending_item = (topic, json_str)
            if delivery_scheduled:
                return
            delivery_scheduled = True
        try:
            event_loop.call_soon_threadsafe(deliver_latest)
        except RuntimeError:
            # 应用正在关闭，事件循环已停止。
            with pending_lock:
                delivery_scheduled = False
                pending_item = None
            return

    source.add_listener(key, on_data)

    async def stream() -> AsyncIterator[bytes]:
        next_send_time = 0.0
        try:
            while True:
                if (
                    await request.is_disconnected()
                    or not await authorization.is_valid()
                ):
                    return
                try:
                    latest_item = await asyncio.wait_for(
                        events.get(),
                        timeout=min(
                            HEARTBEAT_SECONDS,
                            authorization.recheck_seconds,
                        ),
                    )
                except asyncio.TimeoutError:
                    yield b":heartbeat\n\n"
                    continue
                remaining = next_send_time - time.monotonic()
                if remaining > 0.0:
                    await asyncio.sleep(remaining)
                    try:
                        latest_item = events.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                topic, json_str = latest_item
                ts = int(time.time() * 1000)
                try:
                    value = json.loads(json_str)
                except (TypeError, ValueError):
                    value = {"_raw": str(json_str)}
                sse_data = json.dumps(
                    {
                        "key": topic,
                        "value": value,
                        "encoding": "application/json",
                        "timestamp": str(ts),
                    },
                    ensure_ascii=False,
                )
                yield f"event:PUT\ndata:{sse_data}\n\n".encode("utf-8")
                next_send_time = time.monotonic() + MONITOR_UPDATE_INTERVAL_SECONDS
        finally:
            source.remove_listener(key, on_data)

    return StreamingResponse(
        stream(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "Connection": "keep-alive"},
    )
