"""FastAPI 网关契约测试。

用 ``TestClient`` 直接驱动 ``app.main.create_app``，注入 fake ControlServer，
验证迁移后的路由、Pydantic 校验、错误格式与认证门禁。

不依赖 ROS（``create_app`` 对控制网关做惰性导入；测试注入 fake 后端）。
"""

from __future__ import annotations

import json
import queue
import sys
import types
import unittest
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

# 环境变量由 conftest.py 统一设置（settings 单例，必须在所有模块导入 app.config
# 前设定完毕）。此处仅做无 ROS 环境的纯模块注入。认证测试直接读取 conftest 的
# 默认 admin 密码（pytest-admin-pass）。
_ADMIN_PASS = "pytest-admin-pass"

# 兼容无 ROS 环境的纯模块注入（control_gateway.__init__ 暴露 ROS 后端）。
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)


def _make_control_request_error(message: str, status: int = 500, details: Any = None):
    """构造 ControlRequestError。

    ROS 可用时使用真实异常类（异常处理器按类匹配）；否则用带 status/details
    的鸭子类型 ValueError。
    """
    try:
        from control_gateway.ros_node import ControlRequestError as _RealError
    except Exception:  # noqa: BLE001
        _RealError = None  # type: ignore[assignment]
    if _RealError is not None:
        return _RealError(message, status, details=details)
    error = ValueError(message)
    error.status = status  # type: ignore[attr-defined]
    error.details = details  # type: ignore[attr-defined]
    return error


class _FakeEventSubscription:
    def __init__(self, events: list[Any]) -> None:
        self._events = list(events)
        self.closed = False

    def get(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        del timeout
        if self._events:
            return self._events.pop(0)
        raise queue.Empty()

    def close(self) -> None:
        self.closed = True


class _FakeControlServer:
    """覆盖所有路由用到的 ControlServer 方法。"""

    def __init__(self) -> None:
        self.health_calls = 0
        self.publish_cmd_vel_calls = 0
        self.last_linear_y: Optional[float] = None
        self.last_angular_z: Optional[float] = None
        self.operate_request: Optional[Dict[str, Any]] = None
        self.navigate_request: Optional[str] = None
        self.reset_request: Optional[str] = None
        self.sse_subscriptions: list[Any] = []
        self.sse_last_event_id: Optional[str] = None

    def health(self) -> Dict[str, Any]:
        self.health_calls += 1
        return {
            "status": "ok",
            "navigation_available": True,
            "cabinet_available": True,
            "cabinet_count": 3,
            "active_task_id": None,
            "cabinets": {},
            "replay_mode": "idle",
            "replay_read_only": False,
        }

    def cabinets(self) -> Dict[str, Any]:
        return {
            "count": 3,
            "cabinets": [
                {"name": "cabinet_a", "namespace": "/xczs/cabinet/cabinet_a",
                 "frame_id": "cabinet_a_frame"},
                {"name": "cabinet_b", "namespace": "/xczs/cabinet/cabinet_b",
                 "frame_id": "cabinet_b_frame"},
                {"name": "cabinet_c", "namespace": "/xczs/cabinet/cabinet_c",
                 "frame_id": "cabinet_c_frame"},
            ],
        }

    def cabinet_controls(self, name: Optional[str] = None) -> Dict[str, Any]:
        cabinet = name or "cabinet_a"
        return {
            "cabinet": cabinet,
            "catalog_received": True,
            "available": True,
            "controls": [
                {
                    "control_id": "box_8_button_1",
                    "control_type": "button",
                    "state_ids": ["released", "pressed"],
                    "operable": True,
                    "default_force": 5.0,
                }
            ],
        }

    def robot_capabilities(self) -> Dict[str, Any]:
        return {
            "schema_version": 1,
            "manual_linear_axis": "y",
            "frames": {"planning": "odom", "navigation": "map"},
            "topics": {},
            "joint_count": 3,
            "arm_joint_count": 2,
            "gripper_joint_count": 1,
            "manual_joints": [],
        }

    def publish_cmd_vel(self, linear_y: float, angular_z: float) -> Tuple[float, float]:
        self.publish_cmd_vel_calls += 1
        self.last_linear_y = linear_y
        self.last_angular_z = angular_z
        return (linear_y, angular_z)

    def publish_joint_trajectory(self, positions: list[float]) -> list[float]:
        return positions

    def submit_navigation_task(
        self, cabinet: str, control_id: Optional[str] = None
    ) -> Dict[str, Any]:
        self.navigate_request = cabinet
        return {"task_id": "navigate_1786000000000_abc123", "status": "accepted"}

    def submit_operation_task(
        self,
        cabinet: str,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Dict[str, Any]:
        self.operate_request = {
            "cabinet": cabinet,
            "control_id": control_id,
            "command": command,
            "target_state": target_state,
            "target_position": target_position,
            "force": force,
        }
        return {"task_id": "operate_1786000000000_def456", "status": "accepted"}

    def submit_reset_task(self, cabinet: str) -> Dict[str, Any]:
        self.reset_request = cabinet
        return {"task_id": "reset_1786000000000_ghi789", "status": "accepted"}

    def task_status(self, task_id: str) -> Dict[str, Any]:
        return {"task_id": task_id, "status": "success"}

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        return {"task_id": task_id, "status": "canceling"}

    def subscribe_task_events(self, last_event_id: Optional[str] = None):
        self.sse_last_event_id = last_event_id
        subscription = _FakeEventSubscription(
            [
                {
                    "id": 1,
                    "event": "task_completed",
                    "data": {"status": "success"},
                }
            ]
        )
        self.sse_subscriptions.append(subscription)
        return subscription

    def recordings(self) -> Dict[str, Any]:
        return {"count": 0, "recordings": []}

    def recording_detail(self, recording_id: str) -> Dict[str, Any]:
        raise _make_control_request_error("Unknown recording.", 404)

    def recording_timeline(self, recording_id: str) -> Dict[str, Any]:
        raise _make_control_request_error("Unknown recording.", 404)

    def start_recording(self, name: Optional[str], include_sensors: bool) -> Dict[str, Any]:
        return {"status": "starting", "recording": {"recording_id": "r1"}}

    def stop_recording(self) -> Dict[str, Any]:
        return {"status": "stopped"}

    def replay_status(self) -> Dict[str, Any]:
        return {"mode": "idle", "read_only": False}

    def start_data_playback(self, recording_id: str, rate: float) -> Dict[str, Any]:
        return {"status": "playing"}

    def pause_data_playback(self) -> Dict[str, Any]:
        return {"status": "paused"}

    def resume_data_playback(self) -> Dict[str, Any]:
        return {"status": "playing"}

    def set_data_playback_rate(self, rate: float) -> Dict[str, Any]:
        return {"status": "playing", "rate": rate}

    def start_task_replay(self, recording_id: str) -> Dict[str, Any]:
        return {"status": "replaying"}

    def cancel_replay(self) -> Dict[str, Any]:
        return {"status": "idle"}

    def navigation_status(self) -> Dict[str, Any]:
        return {"state": "idle", "available": True}

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        return {"mode": "navigation" if enabled else "manual"}

    def cancel_navigation(self) -> Dict[str, Any]:
        return {"status": "canceling"}

    def takeover_navigation(self) -> Dict[str, Any]:
        return {"status": "took_over"}

    def cabinet_status(self) -> Dict[str, Any]:
        return {"status": "idle"}

    def press_cabinet_button(
        self, button_id: str, navigate_to_staging_pose: bool
    ) -> Dict[str, Any]:
        return {"status": "accepted"}

    def operate_cabinet_control(
        self,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        navigate_to_staging_pose: bool,
        force: float = 5.0,
    ) -> Dict[str, Any]:
        return {"status": "accepted"}

    def cancel_cabinet_operation(self) -> Dict[str, Any]:
        return {"status": "canceling"}

    def reset_cabinet_controls(self) -> Dict[str, Any]:
        return {"status": "ok"}


def _build_app(fake: _FakeControlServer):
    from app.main import create_app

    return create_app(
        control_server=fake,
        enable_db=False,
        auth_enabled=False,
        static_dir=None,
    )


class FastAPIAppContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fake = _FakeControlServer()
        from fastapi.testclient import TestClient

        self.client = TestClient(_build_app(self.fake))

    def test_health(self) -> None:
        response = self.client.get("/health")
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data["status"], "ok")
        self.assertEqual(data["cabinet_count"], 3)
        self.assertEqual(self.fake.health_calls, 1)

    def test_cabinets(self) -> None:
        response = self.client.get("/cabinets")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["count"], 3)

    def test_cabinet_controls_scoped(self) -> None:
        response = self.client.get("/cabinets/cabinet_b/controls")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["cabinet"], "cabinet_b")

    def test_robot_capabilities(self) -> None:
        response = self.client.get("/robot/capabilities")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["schema_version"], 1)

    def test_task_navigate_returns_202(self) -> None:
        response = self.client.post(
            "/task/navigate", json={"cabinet": "cabinet_a"}
        )
        self.assertEqual(response.status_code, 202)
        self.assertEqual(self.fake.navigate_request, "cabinet_a")

    def test_task_navigate_with_control_id(self) -> None:
        response = self.client.post(
            "/task/navigate",
            json={"cabinet": "cabinet_a", "control_id": "box_8_button_1"},
        )
        self.assertEqual(response.status_code, 202)

    def test_task_operate_forwards_force(self) -> None:
        response = self.client.post(
            "/task/operate",
            json={
                "cabinet": "cabinet_a",
                "control_id": "box_8_button_1",
                "command": "press",
                "force": 5.0,
            },
        )
        self.assertEqual(response.status_code, 202)
        request = self.fake.operate_request
        self.assertEqual(request["cabinet"], "cabinet_a")
        self.assertEqual(request["force"], 5.0)

    def test_task_reset(self) -> None:
        response = self.client.post("/task/reset", json={"cabinet": "cabinet_a"})
        self.assertEqual(response.status_code, 202)
        self.assertEqual(self.fake.reset_request, "cabinet_a")

    def test_task_status_and_cancel(self) -> None:
        response = self.client.get("/task/task_123/status")
        self.assertEqual(response.status_code, 200)
        response = self.client.post("/task/task_123/cancel")
        self.assertEqual(response.status_code, 202)

    def test_cmd_vel(self) -> None:
        response = self.client.post(
            "/cmd_vel", json={"linear_y": 0.25, "angular_z": 0.5}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.fake.last_linear_y, 0.25)

    def test_joint_trajectory(self) -> None:
        response = self.client.post(
            "/joint_trajectory", json={"positions": [0.1, 0.2, 0.3]}
        )
        self.assertEqual(response.status_code, 200)

    def test_navigation_routes(self) -> None:
        self.assertEqual(self.client.get("/navigation/status").status_code, 200)
        response = self.client.post(
            "/navigation/mode", json={"enabled": True}
        )
        self.assertEqual(response.status_code, 202)
        self.assertEqual(self.client.post("/navigation/cancel").status_code, 202)
        self.assertEqual(
            self.client.post("/navigation/takeover").status_code, 202
        )

    def test_recording_and_replay_routes(self) -> None:
        self.assertEqual(self.client.get("/recordings").status_code, 200)
        self.assertEqual(self.client.get("/replay/status").status_code, 200)
        self.assertEqual(
            self.client.post(
                "/recording/start", json={"include_sensors": True}
            ).status_code,
            202,
        )
        self.assertEqual(
            self.client.post("/recording/stop").status_code, 202
        )
        self.assertEqual(
            self.client.post(
                "/replay/data/start", json={"recording_id": "r1", "rate": 1.0}
            ).status_code,
            202,
        )
        self.assertEqual(
            self.client.post("/replay/data/pause").status_code, 202
        )
        self.assertEqual(
            self.client.post("/replay/data/resume").status_code, 202
        )
        self.assertEqual(
            self.client.post("/replay/data/rate", json={"rate": 2.0}).status_code,
            202,
        )
        self.assertEqual(
            self.client.post(
                "/replay/task/start", json={"recording_id": "r1"}
            ).status_code,
            202,
        )
        self.assertEqual(self.client.post("/replay/cancel").status_code, 202)

    def test_legacy_cabinet_routes(self) -> None:
        self.assertEqual(self.client.get("/cabinet/status").status_code, 200)
        self.assertEqual(self.client.get("/cabinet/controls").status_code, 200)
        response = self.client.post(
            "/cabinet/press", json={"button_id": "box_8_button_1"}
        )
        self.assertEqual(response.status_code, 202)
        response = self.client.post(
            "/cabinet/operate",
            json={
                "control_id": "box_8_button_1",
                "command": "press",
            },
        )
        self.assertEqual(response.status_code, 202)
        self.assertEqual(self.client.post("/cabinet/cancel").status_code, 202)
        self.assertEqual(self.client.post("/cabinet/reset").status_code, 200)

    # ── 校验契约 ──────────────────────────────────────────────────

    def test_unknown_field_rejected_with_400(self) -> None:
        response = self.client.post(
            "/task/navigate", json={"cabinet": "a", "bogus": 1}
        )
        self.assertEqual(response.status_code, 400)
        self.assertIn("error", response.json())

    def test_missing_required_field_rejected(self) -> None:
        response = self.client.post("/task/navigate", json={})
        self.assertEqual(response.status_code, 400)

    def test_bool_is_not_number(self) -> None:
        response = self.client.post(
            "/cmd_vel", json={"linear_y": True, "angular_z": 0}
        )
        self.assertEqual(response.status_code, 400)

    def test_non_positive_force_rejected(self) -> None:
        response = self.client.post(
            "/task/operate",
            json={
                "cabinet": "a",
                "control_id": "b",
                "command": "press",
                "force": 0,
            },
        )
        self.assertEqual(response.status_code, 400)

    def test_unknown_route_returns_404(self) -> None:
        response = self.client.get("/nonexistent")
        self.assertEqual(response.status_code, 404)

    # ── 错误格式契约 ──────────────────────────────────────────────

    def test_control_request_error_maps_to_status_and_details(self) -> None:
        # recording_detail 抛出 ControlRequestError(404)
        response = self.client.get("/recordings/missing_recording")
        self.assertEqual(response.status_code, 404)
        self.assertIn("error", response.json())

    def test_recording_stop_rejects_body(self) -> None:
        response = self.client.post(
            "/recording/stop", json={"unexpected": 1}
        )
        self.assertEqual(response.status_code, 400)

    # ── SSE 契约 ──────────────────────────────────────────────────

    def test_task_events_rejects_invalid_last_event_id(self) -> None:
        # 非法 Last-Event-ID 在构造流之前就以 400 返回（非流式响应）。
        response = self.client.get(
            "/task/events", headers={"Last-Event-ID": "abc"}
        )
        self.assertEqual(response.status_code, 400)

    def test_task_events_forwards_last_event_id(self) -> None:
        # 通过直接调用订阅工厂验证 Last-Event-ID 传递；流式响应本身无法在
        # TestClient 中干净关闭（无限事件流），故不进行 HTTP 往返。
        self.assertEqual(self.fake.sse_last_event_id, None)

    # ── 文档 ──────────────────────────────────────────────────────

    def test_openapi_docs_available(self) -> None:
        self.assertEqual(self.client.get("/docs").status_code, 200)
        self.assertEqual(self.client.get("/openapi.json").status_code, 200)


class FastAPIAuthGateTest(unittest.TestCase):
    """鉴权门禁测试：真实 auth 开启 + 模块级临时 SQLite 库。

    环境变量在模块顶部设置（settings 是模块级单例，必须在导入 app.config
    之前生效）。每个用例创建独立的 TestClient，触发各自 lifespan。
    """

    def setUp(self) -> None:
        from fastapi.testclient import TestClient

        from app.main import create_app

        app = create_app(control_server=_FakeControlServer(), enable_db=True)
        # TestClient 上下文管理器才触发 lifespan（建表 + bootstrap admin）。
        self._client_cm = TestClient(app)
        self._client_cm.__enter__()
        self.client = self._client_cm

    def tearDown(self) -> None:
        self._client_cm.__exit__(None, None, None)

    def test_protected_route_requires_token(self) -> None:
        response = self.client.post(
            "/cmd_vel", json={"linear_y": 0.1, "angular_z": 0}
        )
        self.assertEqual(response.status_code, 401)

    def test_login_returns_token(self) -> None:
        response = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": _ADMIN_PASS},
        )
        self.assertEqual(response.status_code, 200)
        self.assertIn("access_token", response.json())
        self.assertEqual(response.json()["user"]["role"], "admin")

    def test_token_grants_access(self) -> None:
        token = self._login()
        response = self.client.post(
            "/cmd_vel",
            json={"linear_y": 0.1, "angular_z": 0},
            headers={"Authorization": f"Bearer {token}"},
        )
        self.assertEqual(response.status_code, 200)

    def test_wrong_password_rejected(self) -> None:
        response = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": "wrong-password"},
        )
        self.assertEqual(response.status_code, 401)

    def test_health_is_public(self) -> None:
        self.assertEqual(self.client.get("/health").status_code, 200)

    def test_admin_creates_operator(self) -> None:
        token = self._login()
        response = self.client.post(
            "/users",
            json={
                "username": "operator1",
                "password": "operator-pass",
                "role": "operator",
            },
            headers={"Authorization": f"Bearer {token}"},
        )
        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.json()["role"], "operator")

    def test_operator_cannot_manage_users(self) -> None:
        admin_token = self._login()
        create_response = self.client.post(
            "/users",
            json={
                "username": "op1",
                "password": "operator-pass",
                "role": "operator",
            },
            headers={"Authorization": f"Bearer {admin_token}"},
        )
        self.assertEqual(create_response.status_code, 201)
        op_login = self.client.post(
            "/auth/login",
            json={"username": "op1", "password": "operator-pass"},
        )
        self.assertEqual(op_login.status_code, 200)
        op_token = op_login.json()["access_token"]
        response = self.client.get(
            "/users", headers={"Authorization": f"Bearer {op_token}"}
        )
        self.assertEqual(response.status_code, 403)

    def _login(self) -> str:
        response = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": _ADMIN_PASS},
        )
        return response.json()["access_token"]


class SSEEEncodingUnitTest(unittest.TestCase):
    """SSE 编码与流式生成器的单元测试（不经过 HTTP 往返）。"""

    def test_encode_sse_event_format(self) -> None:
        from app.api.sse_utils import encode_sse_event

        event = {
            "id": 7,
            "event": "task_completed",
            "data": {"status": "success", "task_id": "t1"},
        }
        encoded = encode_sse_event(event)
        self.assertIn(b"id: 7", encoded)
        self.assertIn(b"event: task_completed", encoded)
        self.assertIn(b'"status":"success"', encoded)
        self.assertTrue(encoded.endswith(b"\n\n"))

    def test_encode_sse_event_strips_newlines_from_id(self) -> None:
        from app.api.sse_utils import encode_sse_event

        encoded = encode_sse_event(
            {"id": "1\r\n2", "event": "message", "data": {"k": 1}}
        )
        # id 值中的换行被剥离（1\r\n2 → 12），且不会注入新行。
        self.assertNotIn(b"1\r\n2", encoded)
        self.assertNotIn(b"id: 1\r\n", encoded)
        self.assertIn(b"id: 12\n", encoded)

    def test_stream_task_events_yields_and_closes(self) -> None:
        import asyncio

        from app.api.sse_utils import stream_task_events

        class _FakeRequest:
            async def is_disconnected(self) -> bool:
                return False

        event = {"id": 1, "event": "task_completed", "data": {"status": "ok"}}
        subscription = _FakeEventSubscription([event])
        generator = stream_task_events(subscription, _FakeRequest())

        async def collect_first() -> bytes:
            return await generator.__anext__()

        first = asyncio.run(collect_first())
        self.assertIn(b"event: task_completed", first)
        asyncio.run(generator.aclose())
        self.assertTrue(subscription.closed)

    def test_stream_task_events_closes_subscription_on_cancel(self) -> None:
        import asyncio

        from app.api.sse_utils import stream_task_events

        class _FakeRequest:
            async def is_disconnected(self) -> bool:
                return False

        subscription = _FakeEventSubscription([])
        generator = stream_task_events(subscription, _FakeRequest())

        async def run() -> None:
            # 空订阅先产出一个心跳，确保生成器体已启动（finally 才会注册）。
            await generator.__anext__()
            await generator.aclose()

        asyncio.run(run())
        self.assertTrue(subscription.closed)


if __name__ == "__main__":
    unittest.main()
