"""Pure regression tests for fail-closed runtime toolset switching.

The toolset switch service only acknowledges admission.  These tests keep the
supervisor status deliberately stale so the gateway cannot accidentally rely
on topic delivery winning a race with a service response or timeout.
"""

from __future__ import annotations

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from typing import Any, Callable

import pytest


JIANG_ROOT = Path(__file__).resolve().parents[1]
if str(JIANG_ROOT) not in sys.path:
    sys.path.insert(0, str(JIANG_ROOT))

from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.ros_node import RosControlNode  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402


def _status(
    *,
    state: str = "ready",
    stage: str = "ready",
    ready: bool = True,
    active: str = "A",
    target: str | None = None,
    generation: int = 1,
) -> dict[str, Any]:
    return {
        "managed": True,
        "state": state,
        "stage": stage,
        "ready": ready,
        "active_toolset": active,
        "target_toolset": target,
        "generation": generation,
        "operation_id": None,
        "last_operation_id": None,
        "last_error": None,
        "message": state,
        "updated_at": 1.0,
    }


def _accepted(target: str = "B", generation: int = 2) -> dict[str, Any]:
    return {
        "accepted": True,
        "state": "switching",
        "active_toolset": "A",
        "target_toolset": target,
        "generation": generation,
        "message": "toolset switch accepted",
    }


class _GatewayNode:
    def __init__(self) -> None:
        self._status = _status()
        self.request_effect: (
            dict[str, Any]
            | BaseException
            | Callable[[str, int], dict[str, Any]]
        ) = _accepted()
        self.on_request: Callable[[], None] | None = None
        self.requests: list[tuple[str, int]] = []
        self.scene_requests: list[tuple[str, str]] = []
        self.reconfigured: list[tuple[str, ...]] = []
        self._lock = threading.RLock()

    def set_status(self, value: dict[str, Any]) -> None:
        with self._lock:
            self._status = dict(value)

    def toolset_switch_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    def request_toolset_switch(
        self,
        target: str,
        *,
        expected_generation: int = 0,
        scene: str = "",
        scenes_config: str = "",
    ) -> dict[str, Any]:
        self.requests.append((target, expected_generation))
        self.scene_requests.append((scene, scenes_config))
        if self.on_request is not None:
            self.on_request()
        effect = self.request_effect
        if isinstance(effect, BaseException):
            raise effect
        if callable(effect):
            return effect(target, expected_generation)
        return dict(effect)

    def reconfigure_manual_joints(self, joints: Any) -> None:
        self.reconfigured.append(tuple(joints))


def _server() -> tuple[ControlServer, _GatewayNode, list[str]]:
    server = object.__new__(ControlServer)
    node = _GatewayNode()
    persisted: list[str] = []

    server._node = node
    server._robot_adapter = SimpleNamespace(
        active_toolset="A",
        manual_joints=("a_tool_joint",),
    )
    server._toolset_runtime_lock = threading.RLock()
    server._toolset_transition_generation = None
    server._toolset_supervisor_required = True
    server._toolset_persistence_error = None
    server._toolset_persist_callback = persisted.append
    server._load_runtime_toolset_adapter = lambda toolset: SimpleNamespace(
        active_toolset=toolset,
        manual_joints=(f"{toolset.lower()}_tool_joint",),
    )
    server._validate_runtime_toolset = lambda _toolset: None
    server._ensure_backend_quiescent = lambda _operation: None
    server._quiesce_manual_outputs = lambda: None

    # Production always owns these request/task locks.  Supplying the minimal
    # collaborators here exercises the real admission and interlock methods.
    server._task_manager = SimpleNamespace(active_task_id=None)
    server._task_interlock_lock = threading.RLock()
    server._request_condition = threading.Condition()
    server._active_requests = 0
    server._stopping = False
    server._recording_manager = None
    server._task_replay = None
    return server, node, persisted


def _motion_lock_status(server: ControlServer) -> int | None:
    try:
        server._ensure_toolset_motion_ready()
    except ControlRequestError as error:
        return error.status
    return None


@pytest.mark.parametrize(
    "message",
    (
        "Toolset supervisor did not acknowledge the request in time.",
        "Toolset supervisor request failed: response transport failed",
    ),
)
def test_dispatched_timeout_or_response_error_keeps_old_ready_motion_locked(
    message: str,
) -> None:
    server, node, persisted = _server()
    locked_during_dispatch: list[int | None] = []
    node.on_request = lambda: locked_during_dispatch.append(
        _motion_lock_status(server)
    )
    node.request_effect = ControlRequestError(
        message,
        503,
        details={"admission_uncertain": True},
    )

    with pytest.raises(ControlRequestError) as raised:
        server.request_toolset_switch("B", expected_generation=1)

    assert raised.value.status == 503
    assert locked_during_dispatch == [409]
    assert _motion_lock_status(server) == 409
    assert server._robot_adapter.active_toolset == "A"
    assert node.reconfigured == []
    assert persisted == []


def test_definitely_unsent_request_releases_provisional_motion_lock() -> None:
    server, node, persisted = _server()
    locked_during_dispatch: list[int | None] = []
    node.on_request = lambda: locked_during_dispatch.append(
        _motion_lock_status(server)
    )
    node.request_effect = ControlRequestError(
        "The robot toolset supervisor service is unavailable.",
        503,
        details={"admission_uncertain": False},
    )

    with pytest.raises(ControlRequestError) as raised:
        server.request_toolset_switch("B", expected_generation=1)

    assert raised.value.status == 503
    assert locked_during_dispatch == [409]
    assert _motion_lock_status(server) is None
    assert server._robot_adapter.active_toolset == "A"
    assert node.reconfigured == []
    assert persisted == []


def test_explicit_unchanged_rejection_releases_provisional_motion_lock() -> None:
    server, node, persisted = _server()
    node.request_effect = ControlRequestError(
        "request was rejected",
        409,
        details={
            "admission_uncertain": False,
            "toolset_response": {
                "accepted": False,
                "state": "ready",
                "active_toolset": "A",
                "target_toolset": None,
                "generation": 1,
                "message": "request was rejected",
            },
        },
    )

    with pytest.raises(ControlRequestError):
        server.request_toolset_switch("B", expected_generation=1)

    assert _motion_lock_status(server) is None
    assert server._toolset_transition_generation is None
    assert node.reconfigured == []
    assert persisted == []


def test_rejection_reporting_an_external_transition_keeps_motion_locked() -> None:
    server, node, persisted = _server()
    node.request_effect = ControlRequestError(
        "another transition is in progress",
        409,
        details={
            "admission_uncertain": False,
            "toolset_response": {
                "accepted": False,
                "state": "switching",
                "active_toolset": "A",
                "target_toolset": "B",
                "generation": 2,
                "message": "another transition is in progress",
            },
        },
    )

    with pytest.raises(ControlRequestError):
        server.request_toolset_switch("B", expected_generation=1)

    assert _motion_lock_status(server) == 409
    assert server._toolset_transition_generation == 2
    assert node.reconfigured == []
    assert persisted == []


def test_accepted_request_with_lagged_status_keeps_motion_locked() -> None:
    server, node, persisted = _server()

    response = server.request_toolset_switch("B", expected_generation=1)

    assert response["accepted"] is True
    assert response["generation"] == 2
    assert response["toolset_status"]["generation"] == 1
    assert _motion_lock_status(server) == 409
    assert server._robot_adapter.active_toolset == "A"
    assert node.reconfigured == []
    assert persisted == []


def test_target_ready_reconfigures_and_persists_exactly_once() -> None:
    server, node, persisted = _server()
    server.request_toolset_switch("B", expected_generation=1)
    node.set_status(_status(active="B", generation=2))

    synced = server._sync_toolset_runtime()
    synced_again = server._sync_toolset_runtime()

    assert synced["gateway_synced"] is True
    assert synced_again["gateway_synced"] is True
    assert _motion_lock_status(server) is None
    assert server._robot_adapter.active_toolset == "B"
    assert node.reconfigured == [("b_tool_joint",)]
    assert persisted == ["B"]


def test_rollback_ready_keeps_old_adapter_without_persisting_target() -> None:
    server, node, persisted = _server()
    server.request_toolset_switch("B", expected_generation=1)
    node.set_status(
        _status(stage="rollback_ready", active="A", generation=2)
    )

    synced = server._sync_toolset_runtime()

    assert synced["gateway_synced"] is True
    assert _motion_lock_status(server) is None
    assert server._robot_adapter.active_toolset == "A"
    assert node.reconfigured == []
    assert persisted == []


def test_pending_transition_rejects_a_concurrent_second_request() -> None:
    server, node, _persisted = _server()
    first_entered = threading.Event()
    release_first = threading.Event()

    def blocking_accept(_target: str, _generation: int) -> dict[str, Any]:
        first_entered.set()
        assert release_first.wait(timeout=2.0)
        return _accepted()

    node.request_effect = blocking_accept
    outcomes: list[tuple[str, int]] = []
    outcomes_lock = threading.Lock()

    def request() -> None:
        try:
            result = server.request_toolset_switch("B", expected_generation=1)
            outcome = ("accepted", int(result["generation"]))
        except ControlRequestError as error:
            outcome = ("rejected", error.status)
        with outcomes_lock:
            outcomes.append(outcome)

    first = threading.Thread(target=request)
    second = threading.Thread(target=request)
    first.start()
    assert first_entered.wait(timeout=2.0)
    second.start()
    release_first.set()
    first.join(timeout=2.0)
    second.join(timeout=2.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert sorted(outcomes) == [("accepted", 2), ("rejected", 409)]
    assert node.requests == [("B", 1)]
    assert _motion_lock_status(server) == 409


class _NeverCompletesFuture:
    def add_done_callback(self, _callback: Callable[[Any], None]) -> None:
        return None


class _FailedFuture:
    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        callback(self)

    def result(self) -> Any:
        raise RuntimeError("response transport failed")


class _FutureClient:
    def __init__(self, future: Any) -> None:
        self.future = future

    def call_async(self, _request: Any) -> Any:
        return self.future


@pytest.mark.parametrize("future", (_NeverCompletesFuture(), _FailedFuture()))
def test_ros_request_marks_post_dispatch_failures_as_uncertain(
    future: Any,
) -> None:
    node = object.__new__(RosControlNode)
    node._toolset_switch_client = _FutureClient(future)
    node._service_ready = lambda _client: True

    with pytest.raises(ControlRequestError) as raised:
        node.request_toolset_switch("B", expected_generation=1, timeout_sec=0.001)

    assert raised.value.status == 503
    assert raised.value.details["admission_uncertain"] is True


def test_ros_request_marks_service_unavailable_as_definitely_unsent() -> None:
    node = object.__new__(RosControlNode)
    node._toolset_switch_client = object()
    node._service_ready = lambda _client: False

    with pytest.raises(ControlRequestError) as raised:
        node.request_toolset_switch("B", expected_generation=1)

    assert raised.value.status == 503
    assert raised.value.details["admission_uncertain"] is False


def test_switch_forwards_the_active_scene_override() -> None:
    """切套装时必须把**当前活动场景**与其 scenes.yaml 成对下发给监督器。

    2026-09-22 实测缺陷：场景切到电气夹层后再切套装，监督器按启动期烘死的
    ``scene:=generator_plant`` 重启子栈 → map_server 载入发电机层地图 → Nav2 在
    配置阶段卡住、永不 active → 就绪门 120 s 超时把套装**回滚回 B**（Web 上表现为
    "NAV2 未就绪 / 控制柜操作未就绪，过一会又自动变回套装 B"）。同一份错误地图还会
    让抽拉柜工位（夹层坐标）落进发电机图的占用带，导航被拒
    （``Navigation goal is inside an occupied map cell``）。

    路径必须**取自该场景所在的那份 scenes.yaml**：资产库场景是逐场景单文件，
    非资产场景在内置 catalog。传错会让子栈找不到该场景、启动即退出
    （实测 ``robot child exited before ready (code 1)``，回滚同样失败）。
    """
    server, node, _persisted = _server()
    server._builtin_scenes_config_path = "/repo/config/scenes.yaml"
    server._asset_scene_provider = SimpleNamespace(
        scene_config_path=lambda name: (
            f"/repo/jiang/data/assets/scene/{name}/scenes.yaml"
            if name == "electrical_mezzanine"
            else None
        )
    )

    # 资产库里的场景 → 用资产自己的 scenes.yaml。
    server._active_scene = "electrical_mezzanine"
    server.request_toolset_switch("B", expected_generation=1)
    assert node.scene_requests == [
        (
            "electrical_mezzanine",
            "/repo/jiang/data/assets/scene/electrical_mezzanine/scenes.yaml",
        )
    ]

    # 内置场景 → 用内置 catalog（每次切换都会留下待确认状态，故换一个 server）。
    server2, node2, _persisted2 = _server()
    server2._builtin_scenes_config_path = "/repo/config/scenes.yaml"
    server2._asset_scene_provider = SimpleNamespace(
        scene_config_path=lambda _name: None
    )
    server2._active_scene = "generator_plant"
    server2.request_toolset_switch("B", expected_generation=1)
    assert node2.scene_requests == [("generator_plant", "/repo/config/scenes.yaml")]
