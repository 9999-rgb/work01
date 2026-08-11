"""Zenoh CDR → HTTP SSE 桥（FastAPI 版）。

从 ``sse_bridge.py`` 迁移，复用其 ``ZenohSource``（线程安全、按 key 订阅扇出）。
每个 SSE 连接注册一个 listener，通过 ``threading.Queue(maxsize=1)`` 节流，
保持浏览器端高频 ROS 2 话题不堆积。
"""

from __future__ import annotations

import asyncio
import json
import time
from collections.abc import AsyncIterator
from queue import Empty, Queue
from typing import Any

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import StreamingResponse

from sse_bridge import ZenohSource

MONITOR_UPDATE_INTERVAL_SECONDS = 1.0
HEARTBEAT_SECONDS = 15.0

router = APIRouter(tags=["SSE 事件"])


def _get_zenoh_source(request: Request) -> ZenohSource:
    source = getattr(request.app.state, "zenoh_source", None)
    if source is None:
        raise HTTPException(status_code=503, detail="Zenoh 桥未初始化。")
    return source


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
    if not key:
        raise HTTPException(status_code=400, detail="缺少 key 表达式")
    if key.endswith("/json"):
        key = key[:-5]

    events: Queue[tuple[str, str]] = Queue(maxsize=1)

    def on_data(topic: str, json_str: str) -> None:
        try:
            events.get_nowait()
        except Empty:
            pass
        events.put_nowait((topic, json_str))

    source.add_listener(key, on_data)

    async def stream() -> AsyncIterator[bytes]:
        next_send_time = 0.0
        try:
            while True:
                if await request.is_disconnected():
                    return
                try:
                    latest_item = await asyncio.to_thread(
                        events.get,
                        timeout=HEARTBEAT_SECONDS,
                    )
                except Empty:
                    yield b":heartbeat\n\n"
                    continue
                remaining = next_send_time - time.monotonic()
                if remaining > 0.0:
                    await asyncio.sleep(remaining)
                    try:
                        latest_item = events.get_nowait()
                    except Empty:
                        pass
                topic, json_str = latest_item
                ts = int(time.time() * 1000)
                sse_data = json.dumps(
                    {
                        "key": topic,
                        "value": json.loads(json_str),
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
