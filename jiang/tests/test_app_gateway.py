"""FastAPI 网关契约测试。

用 ``TestClient`` 直接驱动 ``app.main.create_app``，注入 fake ControlServer，
验证迁移后的路由、Pydantic 校验、错误格式与认证门禁。

不依赖 ROS（``create_app`` 对控制网关做惰性导入；测试注入 fake 后端）。
"""

from __future__ import annotations

import json
import os
import queue
import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from unittest.mock import MagicMock, patch

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
        self._notify = None

    def get(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        del timeout
        if self._events:
            return self._events.pop(0)
        raise queue.Empty()

    def get_nowait(self) -> Dict[str, Any]:
        return self.get(timeout=0.0)

    def set_notify(self, callback) -> None:
        self._notify = callback
        if callback is not None and (self._events or self.closed):
            callback()

    def close(self) -> None:
        self.closed = True
        callback = self._notify
        self._notify = None
        if callback is not None:
            callback()


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


class _FakeSensorState:
    def health(self) -> Dict[str, Any]:
        return {
            "camera": {"ready": True, "age_seconds": 0.1},
            "lidar": {"ready": True, "age_seconds": 0.2},
        }

    def camera_snapshot(self):
        return 1, b"\xff\xd8\xff\xd9", {"frame_id": "camera_frame"}

    def lidar_snapshot(self):
        payload = {"ranges": [1.0], "sample_count": 1}
        return 1, payload, json.dumps(payload)


class _FakeZenohSource:
    def add_listener(self, key: str, callback: Any) -> None:
        del key, callback

    def remove_listener(self, key: str, callback: Any) -> None:
        del key, callback


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
        self.assertFalse(data["auth_required"])
        self.assertEqual(data["cabinet_count"], 3)
        self.assertEqual(self.fake.health_calls, 1)

    def test_sensor_health_has_a_distinct_path(self) -> None:
        from app.main import create_app
        from fastapi.testclient import TestClient

        app = create_app(
            control_server=self.fake,
            sensor_state=_FakeSensorState(),
            enable_db=False,
            auth_enabled=False,
        )
        client = TestClient(app)
        self.assertEqual(client.get("/health").json()["status"], "ok")
        sensor_health = client.get("/sensors/health")
        self.assertEqual(sensor_health.status_code, 200)
        self.assertTrue(sensor_health.json()["camera"]["ready"])
        self.assertEqual(
            client.get("/sensors").json()["endpoints"]["health"],
            "/sensors/health",
        )

    def test_sensor_health_surfaces_ros_runtime_fatal_state(self) -> None:
        from app.main import create_app
        from fastapi.testclient import TestClient

        class _FailedRuntime:
            @staticmethod
            def health() -> Dict[str, Any]:
                return {
                    "status": "error",
                    "healthy": False,
                    "thread_alive": False,
                    "error": {
                        "component": "sensor_ros_executor",
                        "type": "RuntimeError",
                        "message": "executor crashed",
                    },
                }

        app = create_app(
            control_server=self.fake,
            sensor_state=_FakeSensorState(),
            sensor_runtime=_FailedRuntime(),
            enable_db=False,
            auth_enabled=False,
        )
        response = TestClient(app).get("/sensors/health")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["status"], "error")
        self.assertFalse(response.json()["runtime"]["healthy"])
        self.assertEqual(
            response.json()["runtime"]["error"]["component"],
            "sensor_ros_executor",
        )

    def test_lidar_websocket_accepts_lan_same_origin(self) -> None:
        from app.main import create_app
        from fastapi.testclient import TestClient

        app = create_app(
            control_server=self.fake,
            sensor_state=_FakeSensorState(),
            enable_db=False,
            auth_enabled=False,
        )
        client = TestClient(app)
        with client.websocket_connect(
            "/lidar/ws",
            headers={
                "Host": "192.168.1.20:8090",
                "Origin": "http://192.168.1.20:8090",
            },
        ) as websocket:
            self.assertEqual(json.loads(websocket.receive_text())["sample_count"], 1)

    def test_cors_uses_configured_origin_and_all_api_methods(self) -> None:
        from app.main import create_app
        from fastapi.testclient import TestClient

        app = create_app(
            control_server=self.fake,
            enable_db=False,
            auth_enabled=False,
            allowed_origins=["https://console.example"],
        )
        client = TestClient(app)
        response = client.options(
            "/users/1",
            headers={
                "Origin": "https://console.example",
                "Access-Control-Request-Method": "PATCH",
                "Access-Control-Request-Headers": "Authorization",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            response.headers["access-control-allow-origin"],
            "https://console.example",
        )
        methods = response.headers["access-control-allow-methods"]
        self.assertIn("PATCH", methods)
        self.assertIn("DELETE", methods)

        denied = client.options(
            "/users/1",
            headers={
                "Origin": "https://attacker.example",
                "Access-Control-Request-Method": "DELETE",
            },
        )
        self.assertEqual(denied.status_code, 400)
        self.assertNotIn("access-control-allow-origin", denied.headers)

    def test_zenoh_sse_rejects_wildcard_key_over_http(self) -> None:
        from app.main import create_app
        from fastapi.testclient import TestClient

        app = create_app(
            control_server=self.fake,
            zenoh_source=_FakeZenohSource(),
            enable_db=False,
            auth_enabled=False,
        )
        response = TestClient(app).get("/sse/xczs/%2A")
        self.assertEqual(response.status_code, 400)
        self.assertIn("规范 ROS 话题", response.json()["error"])

    def test_static_allowlist_only_serves_monitor_page(self) -> None:
        from app.main import create_app
        from fastapi.testclient import TestClient

        with tempfile.TemporaryDirectory(prefix="xczs_static_test_") as tmp:
            static_dir = Path(tmp)
            (static_dir / "monitor.html").write_text(
                "<!doctype html><title>monitor</title>",
                encoding="utf-8",
            )
            (static_dir / "private.py").write_text(
                "SECRET = 'must-not-be-served'",
                encoding="utf-8",
            )
            app = create_app(
                control_server=self.fake,
                enable_db=False,
                auth_enabled=False,
                static_dir=static_dir,
            )
            client = TestClient(app)
            self.assertEqual(client.get("/").status_code, 200)
            monitor = client.get("/monitor.html")
            self.assertEqual(monitor.status_code, 200)
            self.assertEqual(monitor.headers["cache-control"], "no-store")
            self.assertEqual(monitor.headers["x-frame-options"], "DENY")
            self.assertIn(
                "frame-ancestors 'none'",
                monitor.headers["content-security-policy"],
            )
            self.assertEqual(client.get("/private.py").status_code, 404)

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

    def test_task_operate_enforces_command_specific_target_contract(
        self,
    ) -> None:
        base = {"cabinet": "cabinet_a", "control_id": "control_1"}
        valid_operations = (
            (
                {
                    **base,
                    "command": "set_position",
                    "target_position": 1.25,
                    "force": 6.0,
                },
                (None, 1.25),
            ),
            (
                {
                    **base,
                    "command": "set_state",
                    "target_state": "on",
                    "force": 6.0,
                },
                ("on", None),
            ),
            (
                {**base, "command": "toggle", "force": 6.0},
                (None, None),
            ),
        )
        for payload, expected_targets in valid_operations:
            with self.subTest(valid=payload):
                response = self.client.post("/task/operate", json=payload)
                self.assertEqual(response.status_code, 202, response.text)
                request = self.fake.operate_request
                self.assertEqual(
                    (request["target_state"], request["target_position"]),
                    expected_targets,
                )
                self.assertEqual(request["force"], 6.0)

        invalid_operations = (
            (
                {**base, "command": "set_position"},
                "target_position is required",
            ),
            (
                {
                    **base,
                    "command": "set_position",
                    "target_position": 1.0,
                    "target_state": "on",
                },
                "target_state is only valid for set_state",
            ),
            (
                {**base, "command": "set_state"},
                "target_state is required",
            ),
            (
                {
                    **base,
                    "command": "set_state",
                    "target_state": "on",
                    "target_position": 1.0,
                },
                "target_position is only valid for set_position",
            ),
            (
                {**base, "command": "press", "target_state": None},
                "not valid for press",
            ),
            (
                {**base, "command": "toggle", "target_position": None},
                "not valid for toggle",
            ),
            (
                {**base, "command": "press", "force": None},
                "force is required",
            ),
        )
        for payload, error_fragment in invalid_operations:
            with self.subTest(invalid=payload):
                response = self.client.post("/task/operate", json=payload)
                self.assertEqual(response.status_code, 400, response.text)
                self.assertIn(error_fragment, response.json()["error"])

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

    def test_external_boolean_and_command_fields_are_strict(self) -> None:
        requests = (
            ("/navigation/mode", {"enabled": "false"}),
            (
                "/cabinet/press",
                {
                    "button_id": "box_8_button_1",
                    "navigate_to_staging_pose": "false",
                },
            ),
            (
                "/cabinet/operate",
                {
                    "control_id": "box_8_button_1",
                    "command": "press",
                    "navigate_to_staging_pose": "false",
                },
            ),
            (
                "/cabinet/operate",
                {"control_id": "box_8_button_1", "command": True},
            ),
            ("/recording/start", {"include_sensors": "false"}),
        )
        for path, body in requests:
            with self.subTest(path=path, body=body):
                response = self.client.post(path, json=body)
                self.assertEqual(response.status_code, 400, response.text)

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

    def test_sensor_and_zenoh_streams_require_token(self) -> None:
        self.assertEqual(self.client.get("/sensors/health").status_code, 401)
        self.assertEqual(self.client.get("/camera.jpg").status_code, 401)
        self.assertEqual(self.client.get("/sse/xczs/odom").status_code, 401)

        from starlette.websockets import WebSocketDisconnect

        with self.assertRaises(WebSocketDisconnect) as raised:
            with self.client.websocket_connect(
                "/lidar/ws",
                headers={"Origin": "http://localhost:8090"},
            ):
                pass
        self.assertEqual(raised.exception.code, 4401)

    def test_auth_401_keeps_cors_headers(self) -> None:
        for authorization in (None, "Bearer invalid.jwt.token"):
            headers = {"Origin": "http://localhost:8090"}
            if authorization:
                headers["Authorization"] = authorization
            response = self.client.get("/robot/capabilities", headers=headers)
            self.assertEqual(response.status_code, 401)
            self.assertEqual(
                response.headers.get("access-control-allow-origin"),
                "http://localhost:8090",
            )

    def test_query_token_is_rejected_for_rest_control_routes(self) -> None:
        token = self._login()
        response = self.client.get(
            "/robot/capabilities",
            params={"token": token},
        )
        self.assertEqual(response.status_code, 401)

        header_response = self.client.get(
            "/robot/capabilities",
            headers={"Authorization": f"Bearer {token}"},
        )
        self.assertEqual(header_response.status_code, 200)

    def test_lidar_websocket_rejects_missing_and_untrusted_origin(self) -> None:
        from starlette.websockets import WebSocketDisconnect

        token = self._login()
        for headers in ({}, {"Origin": "https://attacker.example"}):
            with self.assertRaises(WebSocketDisconnect) as raised:
                with self.client.websocket_connect(
                    f"/lidar/ws?token={token}",
                    headers=headers,
                ):
                    pass
            self.assertEqual(raised.exception.code, 4403)

    def test_disabled_and_deleted_user_tokens_are_rejected(self) -> None:
        import asyncio

        from app.auth.deps import ActiveTokenChecker

        admin_token = self._login()
        headers = {"Authorization": f"Bearer {admin_token}"}

        disabled = self.client.post(
            "/users",
            json={
                "username": "disabled_token_user",
                "password": "operator-pass",
                "role": "operator",
            },
            headers=headers,
        )
        self.assertEqual(disabled.status_code, 201)
        disabled_token = self.client.post(
            "/auth/login",
            json={
                "username": "disabled_token_user",
                "password": "operator-pass",
            },
        ).json()["access_token"]
        self.assertEqual(
            self.client.patch(
                f"/users/{disabled.json()['id']}",
                json={"is_active": False},
                headers=headers,
            ).status_code,
            200,
        )
        self.assertEqual(
            self.client.get(
                "/robot/capabilities",
                headers={"Authorization": f"Bearer {disabled_token}"},
            ).status_code,
            401,
        )
        self.assertFalse(
            asyncio.run(
                ActiveTokenChecker(
                    token=disabled_token,
                    user_id=disabled.json()["id"],
                    enabled=True,
                    initially_validated=False,
                    recheck_seconds=0.0,
                ).is_valid()
            )
        )

        deleted = self.client.post(
            "/users",
            json={
                "username": "deleted_token_user",
                "password": "operator-pass",
                "role": "operator",
            },
            headers=headers,
        )
        self.assertEqual(deleted.status_code, 201)
        deleted_token = self.client.post(
            "/auth/login",
            json={
                "username": "deleted_token_user",
                "password": "operator-pass",
            },
        ).json()["access_token"]
        self.assertEqual(
            self.client.delete(
                f"/users/{deleted.json()['id']}",
                headers=headers,
            ).status_code,
            204,
        )
        self.assertEqual(
            self.client.get(
                "/robot/capabilities",
                headers={"Authorization": f"Bearer {deleted_token}"},
            ).status_code,
            401,
        )

    def test_last_admin_cannot_be_demoted_or_disabled(self) -> None:
        login = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": _ADMIN_PASS},
        )
        token = login.json()["access_token"]
        admin_id = login.json()["user"]["id"]
        headers = {"Authorization": f"Bearer {token}"}

        demotion = self.client.patch(
            f"/users/{admin_id}",
            json={"role": "operator"},
            headers=headers,
        )
        self.assertEqual(demotion.status_code, 409)
        disabled = self.client.patch(
            f"/users/{admin_id}",
            json={"is_active": False},
            headers=headers,
        )
        self.assertEqual(disabled.status_code, 409)

    def test_wrong_password_rejected(self) -> None:
        response = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": "wrong-password"},
        )
        self.assertEqual(response.status_code, 401)

    def test_health_is_public(self) -> None:
        response = self.client.get("/health")
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json()["auth_required"])

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

    def test_username_is_normalized_consistently_and_ambiguous_names_fail(
        self,
    ) -> None:
        token = self._login()
        headers = {"Authorization": f"Bearer {token}"}
        created = self.client.post(
            "/users",
            json={
                "username": "  normalized_user  ",
                "password": "operator-pass",
                "role": "operator",
            },
            headers=headers,
        )
        self.assertEqual(created.status_code, 201, created.text)
        self.assertEqual(created.json()["username"], "normalized_user")
        login = self.client.post(
            "/auth/login",
            json={
                "username": "  normalized_user  ",
                "password": "operator-pass",
            },
        )
        self.assertEqual(login.status_code, 200, login.text)

        for username in ("bad user", "bad\nuser", "bad\u200buser", "   "):
            with self.subTest(username=repr(username)):
                rejected = self.client.post(
                    "/users",
                    json={
                        "username": username,
                        "password": "operator-pass",
                        "role": "operator",
                    },
                    headers=headers,
                )
                self.assertEqual(rejected.status_code, 400, rejected.text)
                login_rejected = self.client.post(
                    "/auth/login",
                    json={
                        "username": username,
                        "password": "operator-pass",
                    },
                )
                self.assertEqual(
                    login_rejected.status_code,
                    400,
                    login_rejected.text,
                )

    def test_user_active_flag_rejects_string_coercion(self) -> None:
        token = self._login()
        headers = {"Authorization": f"Bearer {token}"}
        created = self.client.post(
            "/users",
            json={
                "username": "strict_active_user",
                "password": "operator-pass",
                "role": "operator",
            },
            headers=headers,
        )
        self.assertEqual(created.status_code, 201, created.text)
        rejected = self.client.patch(
            f"/users/{created.json()['id']}",
            json={"is_active": "false"},
            headers=headers,
        )
        self.assertEqual(rejected.status_code, 400, rejected.text)
        login = self.client.post(
            "/auth/login",
            json={
                "username": "strict_active_user",
                "password": "operator-pass",
            },
        )
        self.assertEqual(login.status_code, 200, login.text)

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


class SecurityConfigUnitTest(unittest.TestCase):
    def test_lifespan_cleans_constructed_subsystems_when_database_setup_fails(
        self,
    ) -> None:
        from fastapi.testclient import TestClient

        from app.main import create_app

        events = []

        class _Server:
            def stop(self) -> None:
                events.append("server_stop")

        class _Runtime:
            def stop(self) -> None:
                events.append("runtime_stop")

        class _Zenoh:
            def close(self) -> None:
                events.append("zenoh_close")

        app = create_app(
            control_server=_Server(),
            sensor_runtime=_Runtime(),
            zenoh_source=_Zenoh(),
            enable_db=True,
            auth_enabled=False,
        )
        with patch(
            "app.main.init_database",
            side_effect=RuntimeError("database setup failed"),
        ):
            with self.assertRaisesRegex(RuntimeError, "database setup failed"):
                with TestClient(app):
                    pass
        self.assertEqual(
            events,
            ["runtime_stop", "server_stop", "zenoh_close"],
        )

    def test_lifespan_rolls_back_started_subsystems_on_setup_failure(self) -> None:
        from fastapi.testclient import TestClient

        from app.main import create_app

        events = []

        class _Server:
            def start(self, *, start_http: bool) -> None:
                self.start_http = start_http
                events.append("server_start")

            def stop(self) -> None:
                events.append("server_stop")

        class _Runtime:
            def start(self) -> None:
                events.append("runtime_start")
                raise RuntimeError("sensor startup failed")

            def stop(self) -> None:
                events.append("runtime_stop")

        class _Zenoh:
            def close(self) -> None:
                events.append("zenoh_close")

        server = _Server()
        app = create_app(
            control_server=server,
            sensor_runtime=_Runtime(),
            zenoh_source=_Zenoh(),
            enable_db=False,
            auth_enabled=False,
        )
        with self.assertRaisesRegex(RuntimeError, "sensor startup failed"):
            with TestClient(app):
                pass
        self.assertFalse(server.start_http)
        self.assertEqual(
            events,
            [
                "server_start",
                "runtime_start",
                "runtime_stop",
                "server_stop",
                "zenoh_close",
            ],
        )

    def test_default_jwt_secret_is_random_and_not_the_known_legacy_value(self) -> None:
        from app.config import Settings

        with patch.dict(os.environ, {}, clear=True):
            first = Settings(_env_file=None).secret_key
            second = Settings(_env_file=None).secret_key
        self.assertGreaterEqual(len(first), 48)
        self.assertNotEqual(first, "dev-secret-change-me")
        self.assertNotEqual(first, second)

    def test_weak_jwt_configuration_is_rejected(self) -> None:
        from pydantic import ValidationError
        from app.config import Settings

        with self.assertRaises(ValidationError):
            Settings(secret_key="too-short", _env_file=None)
        with self.assertRaises(ValidationError):
            Settings(
                secret_key="a" * 32,
                jwt_algorithm="HS512",
                _env_file=None,
            )

    def test_zenoh_key_rejects_wildcards_and_malformed_segments(self) -> None:
        from fastapi import HTTPException
        from app.sse.router import _normalize_ros_key

        self.assertEqual(
            _normalize_ros_key("xczs/joint_states/json"),
            "xczs/joint_states",
        )
        for key in (
            "",
            "**",
            "xczs/*",
            "xczs/$router",
            "xczs/topic?query",
            "xczs/topic#fragment",
            "xczs//odom",
            "xczs/../odom",
        ):
            with self.subTest(key=key), self.assertRaises(HTTPException) as raised:
                _normalize_ros_key(key)
            self.assertEqual(raised.exception.status_code, 400)

    def test_lidar_websocket_origin_accepts_lan_same_origin_only(self) -> None:
        from app.sensors.router import _websocket_origin_allowed

        configured = frozenset({"http://localhost:8090"})
        self.assertTrue(
            _websocket_origin_allowed(
                "http://192.168.1.20:8090",
                "192.168.1.20:8090",
                configured,
            )
        )
        self.assertFalse(
            _websocket_origin_allowed(
                "https://attacker.example",
                "192.168.1.20:8090",
                configured,
            )
        )
        self.assertFalse(
            _websocket_origin_allowed(None, "192.168.1.20:8090", configured)
        )

    def test_control_server_origin_defaults_follow_selected_port(self) -> None:
        import argparse
        from control_server import _effective_allowed_origins

        args = argparse.Namespace(port=12345, allowed_origins=None)
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(
                _effective_allowed_origins(args),
                [
                    "http://localhost:12345",
                    "http://127.0.0.1:12345",
                ],
            )
        with patch.dict(
            os.environ,
            {"XCZS_ALLOWED_ORIGINS": "https://console.example"},
            clear=True,
        ):
            self.assertEqual(
                _effective_allowed_origins(args),
                ["https://console.example"],
            )
        args.allowed_origins = ["https://cli.example"]
        with patch.dict(
            os.environ,
            {"XCZS_ALLOWED_ORIGINS": "https://env.example"},
            clear=True,
        ):
            self.assertEqual(
                _effective_allowed_origins(args),
                ["https://cli.example"],
            )

    def test_executor_fatal_callback_requests_sigterm_only_once(self) -> None:
        import signal

        from control_server import _build_process_fatal_callback

        callback = _build_process_fatal_callback()
        with patch("control_server.os.kill") as kill:
            callback("control_ros_executor", RuntimeError("first"))
            callback("sensor_ros_executor", RuntimeError("second"))

        kill.assert_called_once_with(os.getpid(), signal.SIGTERM)

    def test_production_entrypoint_wires_one_fatal_callback_to_ros_runtimes(
        self,
    ) -> None:
        import argparse

        import control_server

        args = argparse.Namespace(
            host="127.0.0.1",
            port=18090,
            max_linear_speed=0.25,
            max_angular_speed=0.6,
            command_timeout=0.3,
            allowed_origins=None,
            cabinet_instances="instances.yaml",
            cabinet_scene="scene.yaml",
            cabinet_robot_adapter="adapter.yaml",
            cabinet_controls="controls.yaml",
            cabinet_pose="pose.yaml",
            robot_control="robot.yaml",
            recordings_root="recordings",
            camera_topic="/camera/image_raw",
            lidar_topic="/scan",
            zenoh="tcp/localhost:7447",
            no_sensors=False,
            no_sse=True,
        )
        parser = MagicMock()
        parser.parse_args.return_value = args
        fatal_callback = MagicMock()
        sensor_state = object()
        sensor_runtime = object()
        app = object()

        with (
            patch("control_server._parser", return_value=parser),
            patch(
                "control_server._build_process_fatal_callback",
                return_value=fatal_callback,
            ),
            patch(
                "control_server._build_sensor_subsystem",
                return_value=(sensor_state, sensor_runtime),
            ) as build_sensors,
            patch("control_gateway.ControlServer", create=True) as server_cls,
            patch("app.main.create_app", return_value=app),
            patch("control_server.uvicorn.run") as run,
        ):
            control_server.main()

        self.assertIs(
            server_cls.call_args.kwargs["fatal_callback"],
            fatal_callback,
        )
        build_sensors.assert_called_once_with(args, fatal_callback)
        self.assertIs(run.call_args.args[0], app)
        self.assertFalse(run.call_args.kwargs["access_log"])
        self.assertEqual(run.call_args.kwargs["log_level"], "warning")

    def test_bootstrap_recovers_without_modifying_existing_users(self) -> None:
        import asyncio

        from sqlalchemy import select
        from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine

        from app.auth.bootstrap import bootstrap_admin
        from app.auth.models import User
        from app.auth.service import hash_password
        from app.database.base import Base

        async def exercise() -> None:
            with tempfile.TemporaryDirectory(prefix="xczs_recovery_test_") as tmp:
                recovery_engine = create_async_engine(
                    f"sqlite+aiosqlite:///{Path(tmp) / 'recovery.db'}"
                )
                session_factory = async_sessionmaker(
                    recovery_engine,
                    expire_on_commit=False,
                )
                async with recovery_engine.begin() as connection:
                    await connection.run_sync(Base.metadata.create_all)
                async with session_factory() as session:
                    operator = User(
                        username="recovery_existing_operator",
                        hashed_password=hash_password("operator-pass"),
                        role="operator",
                        is_active=True,
                    )
                    disabled_admin = User(
                        username="recovery_disabled_admin",
                        hashed_password=hash_password("disabled-pass"),
                        role="admin",
                        is_active=False,
                    )
                    session.add_all([operator, disabled_admin])
                    await session.commit()

                    await bootstrap_admin(session)

                    result = await session.execute(
                        select(User).where(
                            User.username.in_(
                                {
                                    "recovery_existing_operator",
                                    "recovery_disabled_admin",
                                }
                            )
                        )
                    )
                    existing = {user.username: user for user in result.scalars()}
                    self.assertEqual(
                        existing["recovery_existing_operator"].role,
                        "operator",
                    )
                    self.assertFalse(
                        existing["recovery_disabled_admin"].is_active
                    )

                    active_result = await session.execute(
                        select(User).where(
                            User.role == "admin",
                            User.is_active.is_(True),
                        )
                    )
                    active_admins = list(active_result.scalars())
                    self.assertEqual(len(active_admins), 1)
                    self.assertTrue(
                        active_admins[0].username.startswith("recovery_admin")
                    )

                    recovery_id = active_admins[0].id
                    await bootstrap_admin(session)
                    active_result = await session.execute(
                        select(User).where(
                            User.role == "admin",
                            User.is_active.is_(True),
                        )
                    )
                    self.assertEqual(
                        [user.id for user in active_result.scalars()],
                        [recovery_id],
                    )
                await recovery_engine.dispose()

        asyncio.run(exercise())

    def test_concurrent_bootstrap_creates_exactly_one_active_admin(self) -> None:
        import asyncio

        from sqlalchemy import func, select
        from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine

        from app.auth.bootstrap import bootstrap_admin
        from app.auth.models import User
        from app.database.base import Base

        async def exercise() -> None:
            with tempfile.TemporaryDirectory(prefix="xczs_bootstrap_race_") as tmp:
                race_engine = create_async_engine(
                    f"sqlite+aiosqlite:///{Path(tmp) / 'race.db'}",
                    connect_args={"timeout": 5},
                )
                sessions = async_sessionmaker(
                    race_engine,
                    expire_on_commit=False,
                )
                async with race_engine.begin() as connection:
                    await connection.run_sync(Base.metadata.create_all)

                async def run_once() -> None:
                    async with sessions() as session:
                        await bootstrap_admin(session)

                await asyncio.gather(run_once(), run_once())
                async with sessions() as session:
                    result = await session.execute(
                        select(func.count())
                        .select_from(User)
                        .where(
                            User.role == "admin",
                            User.is_active.is_(True),
                        )
                    )
                    self.assertEqual(result.scalar_one(), 1)
                await race_engine.dispose()

        asyncio.run(exercise())

    def test_concurrent_admin_demotions_keep_one_active_admin(self) -> None:
        import asyncio

        from fastapi import HTTPException
        from sqlalchemy import func, select
        from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine

        from app.auth.models import User
        from app.auth.router import update_user
        from app.auth.schemas import UserUpdate, UserResponse
        from app.auth.service import hash_password
        from app.database.base import Base

        async def exercise() -> None:
            with tempfile.TemporaryDirectory(prefix="xczs_admin_race_") as tmp:
                race_engine = create_async_engine(
                    f"sqlite+aiosqlite:///{Path(tmp) / 'race.db'}",
                    connect_args={"timeout": 5},
                )
                sessions = async_sessionmaker(
                    race_engine,
                    expire_on_commit=False,
                )
                async with race_engine.begin() as connection:
                    await connection.run_sync(Base.metadata.create_all)
                async with sessions() as session:
                    first = User(
                        username="race_admin_1",
                        hashed_password=hash_password("admin-pass-1"),
                        role="admin",
                        is_active=True,
                    )
                    second = User(
                        username="race_admin_2",
                        hashed_password=hash_password("admin-pass-2"),
                        role="admin",
                        is_active=True,
                    )
                    session.add_all([first, second])
                    await session.commit()
                    ids = (first.id, second.id)

                async def demote(user_id: int):
                    async with sessions() as session:
                        return await update_user(
                            user_id,
                            UserUpdate(role="operator"),
                            session,
                            None,
                        )

                results = await asyncio.gather(
                    *(demote(user_id) for user_id in ids),
                    return_exceptions=True,
                )
                self.assertEqual(
                    sum(isinstance(value, UserResponse) for value in results),
                    1,
                )
                conflicts = [
                    value for value in results
                    if isinstance(value, HTTPException)
                ]
                self.assertEqual(len(conflicts), 1)
                self.assertEqual(conflicts[0].status_code, 409)
                async with sessions() as session:
                    result = await session.execute(
                        select(func.count())
                        .select_from(User)
                        .where(
                            User.role == "admin",
                            User.is_active.is_(True),
                        )
                    )
                    self.assertEqual(result.scalar_one(), 1)
                await race_engine.dispose()

        asyncio.run(exercise())


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
        generator = stream_task_events(
            subscription,
            _FakeRequest(),
            heartbeat_seconds=0.01,
        )

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
        generator = stream_task_events(
            subscription,
            _FakeRequest(),
            heartbeat_seconds=0.01,
        )

        async def run() -> None:
            # 空订阅先产出一个心跳，确保生成器体已启动（finally 才会注册）。
            await generator.__anext__()
            await generator.aclose()

        asyncio.run(run())
        self.assertTrue(subscription.closed)


if __name__ == "__main__":
    unittest.main()
