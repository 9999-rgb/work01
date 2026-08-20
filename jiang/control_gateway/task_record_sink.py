"""任务记录 sink 协议（控制网关侧，不依赖 app）。

``ControlServer`` 只依赖这个 ``Protocol`` 来消费任务事件并持久化
navigate / operate 任务记录；生产实现是 ``app.tasks.store.TaskRecordStore``，
由应用入口（``control_server.py::main()``）注入。未注入时控制网关照常运行，
只是不落库。

写实现必须自行捕获异常 —— 事件桥线程不能被数据库故障拖垮。
"""

from __future__ import annotations

from typing import Any, Mapping, Protocol, runtime_checkable


@runtime_checkable
class TaskRecordSink(Protocol):
    """消费 TaskManager 事件，持久化 navigate / operate 任务记录。"""

    def record_event(self, event: Mapping[str, Any]) -> None:
        """消费一个任务事件。

        实现应过滤非 navigate / operate 任务，并对数据库异常做兜底日志，
        不得向上抛出。
        """


__all__ = ["TaskRecordSink"]
