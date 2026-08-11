"""端到端集成测试：FastAPI TestClient 驱动完整应用（控制网关 + 认证 + 文档 + 静态页）。

覆盖所有核心路由、鉴权门禁、Pydantic 校验、角色权限与错误格式。
不依赖 ROS（注入 FakeControlServer）。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# 环境变量由 conftest.py 统一设置（settings 单例）。
# 默认 admin 密码: pytest-admin-pass
_ADMIN = "admin"
_ADMIN_PASS = "pytest-admin-pass"

from fastapi.testclient import TestClient


class FakeServer:
    """完整的 ControlServer mock，覆盖应用所有路由依赖的方法。"""

    def __init__(self) -> None:
        self._seq = 0
        self.navigate_requests: List[Tuple[str, Optional[str]]] = []
        self.operate_requests: List[Tuple[str, str, Any]] = []
        self.reset_requests: List[str] = []
        self.cmd_vel_calls: List[Tuple[float, float]] = []
        self.joint_calls: List[List[float]] = []
        self.recording_start_requests: List[Tuple[Optional[str], bool]] = []

    # ── 查询 ──────────────────────────────────────────────────────

    def health(self) -> Dict[str, Any]:
        return {
            "status": "ok",
            "navigation_available": True,
            "cabinet_available": True,
            "cabinet_count": 3,
            "cabinet_active": False,
            "active_task_id": None,
            "cabinets": {},
            "replay_mode": "idle",
            "replay_read_only": False,
        }

    def cabinets(self) -> Dict[str, Any]:
        return {
            "count": 3,
            "cabinets": [
                {
                    "name": "cabinet_a",
                    "namespace": "/xczs/cabinet/cabinet_a",
                    "frame_id": "cabinet_a_frame",
                    "pose": {
                        "x": 2.0, "y": 0.33, "z": 0.0,
                        "roll": 1.57, "pitch": 0.0, "yaw": -1.57,
                    },
                }
            ],
        }

    def cabinet_controls(self, name: Optional[str] = None) -> Dict[str, Any]:
        return {
            "cabinet": name or "cabinet_a",
            "catalog_received": True,
            "available": True,
            "operation_available": True,
            "reset_available": True,
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
            "manual_joints": [],
            "joint_count": 8,
            "arm_joint_count": 6,
            "gripper_joint_count": 2,
        }

    def navigation_status(self) -> Dict[str, Any]:
        return {"state": "idle", "available": True}

    def cabinet_status(self) -> Dict[str, Any]:
        return {"cabinet": "cabinet_a", "status": "idle"}

    def recordings(self) -> Dict[str, Any]:
        return {"count": 0, "recordings": []}

    def recording_detail(self, recording_id: str) -> Dict[str, Any]:
        from control_gateway.ros_node import ControlRequestError

        raise ControlRequestError("Unknown recording.", 404)

    def recording_timeline(self, recording_id: str) -> Dict[str, Any]:
        from control_gateway.ros_node import ControlRequestError

        raise ControlRequestError("Unknown recording.", 404)

    def replay_status(self) -> Dict[str, Any]:
        return {"mode": "idle", "read_only": False}

    # ── 手动控制 ──────────────────────────────────────────────────

    def publish_cmd_vel(
        self, linear_y: float, angular_z: float
    ) -> Tuple[float, float]:
        self.cmd_vel_calls.append((linear_y, angular_z))
        return (linear_y, angular_z)

    def publish_joint_trajectory(self, positions: List[float]) -> List[float]:
        self.joint_calls.append(positions)
        return positions

    # ── 任务 ──────────────────────────────────────────────────────

    def _new_id(self, prefix: str) -> str:
        self._seq += 1
        return f"{prefix}_{self._seq}"

    def submit_navigation_task(
        self, cabinet: str, control_id: Optional[str] = None
    ) -> Dict[str, Any]:
        self.navigate_requests.append((cabinet, control_id))
        return {"task_id": self._new_id("navigate"), "status": "accepted"}

    def submit_operation_task(
        self,
        cabinet: str,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Dict[str, Any]:
        self.operate_requests.append((cabinet, control_id, command))
        return {"task_id": self._new_id("operate"), "status": "accepted"}

    def submit_reset_task(self, cabinet: str) -> Dict[str, Any]:
        self.reset_requests.append(cabinet)
        return {"task_id": self._new_id("reset"), "status": "accepted"}

    def task_status(self, task_id: str) -> Dict[str, Any]:
        return {"task_id": task_id, "status": "success", "type": "navigate"}

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        return {"task_id": task_id, "status": "canceling"}

    def subscribe_task_events(self, last_event_id: Optional[str] = None):
        import queue

        class EmptySubscription:
            def get(self, timeout):
                raise queue.Empty

            def close(self):
                pass

        return EmptySubscription()

    # ── 导航 ──────────────────────────────────────────────────────

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        return {"mode": "navigation" if enabled else "manual"}

    def cancel_navigation(self) -> Dict[str, Any]:
        return {"status": "canceling"}

    def takeover_navigation(self) -> Dict[str, Any]:
        return {"status": "took_over"}

    # ── 旧版柜体兼容 ──────────────────────────────────────────────

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

    # ── 录制/回放 ─────────────────────────────────────────────────

    def start_recording(
        self, name: Optional[str], include_sensors: bool
    ) -> Dict[str, Any]:
        self.recording_start_requests.append((name, include_sensors))
        return {"status": "starting", "recording": {"recording_id": "r1"}}

    def stop_recording(self) -> Dict[str, Any]:
        return {"status": "stopped"}

    def start_data_playback(
        self, recording_id: str, rate: float
    ) -> Dict[str, Any]:
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


# ═══════════════════════════════════════════════════════════════════════


def _login(client: TestClient, username: str, password: str) -> str:
    r = client.post("/auth/login", json={"username": username, "password": password})
    assert r.status_code == 200, f"login failed: {r.status_code} {r.text}"
    body = r.json()
    assert "access_token" in body
    return body["access_token"]


def test_all_routes_and_auth() -> None:
    fake = FakeServer()
    from app.main import create_app

    app = create_app(control_server=fake, enable_db=True, static_dir=None)
    with TestClient(app) as c:
        # ═══ AUTH ═══
        token = _login(c, "admin", _ADMIN_PASS)
        user_info = c.post(
            "/auth/login",
            json={"username": "admin", "password": _ADMIN_PASS},
        ).json()
        assert user_info["user"]["role"] == "admin"
        assert user_info["user"]["username"] == "admin"
        assert user_info["token_type"] == "bearer"
        assert isinstance(user_info["expires_in"], int)

        H = {"Authorization": f"Bearer {token}"}

        # wrong password
        assert (
            c.post("/auth/login", json={"username": "admin", "password": "WRONG"}).status_code
            == 401
        )

        # ═══ PUBLIC ENDPOINTS ═══
        assert c.get("/health").status_code == 200
        assert c.get("/docs").status_code == 200
        assert c.get("/openapi.json").status_code == 200
        # 没有静态目录时不测试 monitor.html（create_app 的 static_dir=None）

        # ═══ UNAUTHORIZED ═══
        for path, method in [
            ("/cmd_vel", "post"),
            ("/task/navigate", "post"),
            ("/cabinets", "get"),
            ("/robot/capabilities", "get"),
            ("/navigation/status", "get"),
            ("/cabinet/status", "get"),
            ("/recordings", "get"),
            ("/replay/status", "get"),
        ]:
            fn = c.post if method == "post" else c.get
            body = {"cabinet": "a"} if method == "post" else None
            kwargs = {"json": body} if body else {}
            assert fn(path, **kwargs).status_code == 401, f"{method.upper()} {path} should be 401"

        # ═══ PROTECTED GET ROUTES ═══
        routes_get = {
            "/cabinets": 200,
            "/cabinets/cabinet_a/controls": 200,
            "/robot/capabilities": 200,
            "/navigation/status": 200,
            "/cabinet/status": 200,
            "/cabinet/controls": 200,
            "/recordings": 200,
            "/replay/status": 200,
        }
        for path, expected_status in routes_get.items():
            r = c.get(path, headers=H)
            assert r.status_code == expected_status, f"GET {path}: {r.status_code} {r.text}"

        # ═══ PROTECTED POST ROUTES ═══
        r = c.post("/task/navigate", json={"cabinet": "cabinet_a"}, headers=H)
        assert r.status_code == 202, f"navigate: {r.status_code} {r.text}"
        assert len(fake.navigate_requests) == 1

        r = c.post(
            "/task/navigate",
            json={"cabinet": "cabinet_a", "control_id": "box_8_button_1"},
            headers=H,
        )
        assert r.status_code == 202

        r = c.post(
            "/task/operate",
            json={
                "cabinet": "cabinet_a",
                "control_id": "box_8_button_1",
                "command": "press",
                "force": 5.0,
            },
            headers=H,
        )
        assert r.status_code == 202, f"operate: {r.status_code} {r.text}"
        assert len(fake.operate_requests) == 1

        r = c.post("/task/reset", json={"cabinet": "cabinet_a"}, headers=H)
        assert r.status_code == 202

        r = c.post("/cmd_vel", json={"linear_y": 0.25, "angular_z": 0.5}, headers=H)
        assert r.status_code == 200
        assert fake.cmd_vel_calls[-1] == (0.25, 0.5)

        r = c.post("/joint_trajectory", json={"positions": [0.1, 0.2, 0.3]}, headers=H)
        assert r.status_code == 200

        r = c.post("/navigation/mode", json={"enabled": True}, headers=H)
        assert r.status_code == 202

        for path, expected_status in [
            ("/navigation/cancel", 202),
            ("/navigation/takeover", 202),
            ("/cabinet/cancel", 202),
            ("/cabinet/reset", 200),
            ("/recording/stop", 202),
            ("/replay/data/pause", 202),
            ("/replay/data/resume", 202),
            ("/replay/cancel", 202),
        ]:
            r = c.post(path, headers=H)
            assert r.status_code == expected_status, f"POST {path}: {r.status_code} {r.text}"

        r = c.post(
            "/cabinet/press", json={"button_id": "box_8_button_1"}, headers=H
        )
        assert r.status_code == 202

        r = c.post(
            "/cabinet/operate",
            json={"control_id": "box_8_button_1", "command": "press"},
            headers=H,
        )
        assert r.status_code == 202

        r = c.post(
            "/recording/start", json={"include_sensors": True}, headers=H
        )
        assert r.status_code == 202

        r = c.post(
            "/replay/data/start",
            json={"recording_id": "r1", "rate": 2.0},
            headers=H,
        )
        assert r.status_code == 202

        r = c.post(
            "/replay/data/rate", json={"rate": 3.0}, headers=H
        )
        assert r.status_code == 202

        r = c.post(
            "/replay/task/start", json={"recording_id": "r1"}, headers=H
        )
        assert r.status_code == 202

        # ═══ VALIDATION (Pydantic + extra=forbid + StrictFloat) ═══
        r = c.post("/task/navigate", json={"cabinet": "a", "bogus": 1}, headers=H)
        assert r.status_code == 400, f"unknown field: {r.status_code} {r.text}"
        assert "error" in r.json()

        r = c.post(
            "/task/operate",
            json={"cabinet": "a", "control_id": "b", "command": "press", "force": True},
            headers=H,
        )
        assert r.status_code == 400, f"bool as force: {r.status_code} {r.text}"

        r = c.post(
            "/task/operate",
            json={"cabinet": "a", "control_id": "b", "command": "press", "force": 0},
            headers=H,
        )
        assert r.status_code == 400, f"zero force: {r.status_code} {r.text}"

        r = c.post(
            "/task/operate",
            json={"cabinet": "a", "control_id": "b", "command": "press", "force": -1},
            headers=H,
        )
        assert r.status_code == 400, f"negative force: {r.status_code} {r.text}"

        # missing cabinet → 400
        r = c.post("/task/navigate", json={}, headers=H)
        assert r.status_code == 400

        # recording_detail → 404 (ControlRequestError)
        r = c.get("/recordings/missing_id", headers=H)
        assert r.status_code == 404, f"missing recording: {r.status_code} {r.text}"

        # ═══ ADMIN USER CRUD ═══
        r = c.post(
            "/users",
            json={"username": "tester", "password": "test12345", "role": "operator"},
            headers=H,
        )
        assert r.status_code == 201, f"create user: {r.status_code} {r.text}"
        uid = r.json()["id"]

        r = c.get("/users", headers=H)
        assert r.status_code == 200
        assert len(r.json()) >= 2  # admin + tester

        r = c.get(f"/users/{uid}", headers=H)
        assert r.status_code == 200

        r = c.patch(f"/users/{uid}", json={"is_active": False}, headers=H)
        assert r.status_code == 200

        r = c.delete(f"/users/{uid}", headers=H)
        assert r.status_code == 204

        # duplicate username → 409
        r = c.post(
            "/users",
            json={"username": "admin", "password": "test12345", "role": "operator"},
            headers=H,
        )
        assert r.status_code == 409, f"duplicate user: {r.status_code}"

        # ═══ OPERATOR ROLE GATE ═══
        r = c.post(
            "/users",
            json={"username": "op999", "password": "test98765", "role": "operator"},
            headers=H,
        )
        assert r.status_code == 201

        op_token = _login(c, "op999", "test98765")
        op_h = {"Authorization": f"Bearer {op_token}"}

        # operator can operate
        assert (
            c.post("/task/navigate", json={"cabinet": "a"}, headers=op_h).status_code
            == 202
        )
        assert (
            c.post("/cmd_vel", json={"linear_y": 0.1, "angular_z": 0}, headers=op_h).status_code
            == 200
        )

        # operator cannot manage users
        assert (
            c.get("/users", headers=op_h).status_code == 403
        )

        # ═══ SSE 400 ON BAD Last-Event-ID ═══
        r = c.get(
            "/task/events",
            headers={"Last-Event-ID": "abc"},  # non-numeric
            params={"token": token},
        )
        assert r.status_code == 400

        # ═══ OPENAPI SCHEMA ═══
        schema = c.get("/openapi.json").json()
        paths = schema["paths"]
        assert len(paths) >= 30, f"openapi paths: {len(paths)}"
        # key routes must be present
        for p in [
            "/health",
            "/auth/login",
            "/users",
            "/cabinets",
            "/cabinets/{name}/controls",
            "/robot/capabilities",
            "/task/navigate",
            "/task/operate",
            "/task/reset",
            "/task/{task_id}/status",
            "/task/{task_id}/cancel",
            "/task/events",
            "/cmd_vel",
            "/joint_trajectory",
            "/navigation/status",
            "/navigation/mode",
            "/navigation/cancel",
            "/navigation/takeover",
            "/cabinet/status",
            "/cabinet/controls",
            "/cabinet/press",
            "/cabinet/operate",
            "/cabinet/cancel",
            "/cabinet/reset",
            "/recordings",
            "/recording/start",
            "/recording/stop",
            "/replay/status",
            "/replay/data/start",
            "/replay/data/pause",
            "/replay/data/resume",
            "/replay/data/rate",
            "/replay/task/start",
            "/replay/cancel",
        ]:
            assert p in paths, f"missing OpenAPI path: {p}"

        assert "securitySchemes" in schema.get("components", {})
        assert "HTTPBearer" in schema["components"]["securitySchemes"]

        # ═══ AUTH ME ═══
        r = c.get("/auth/me", headers=H)
        assert r.status_code == 200
        assert r.json()["username"] == "admin"


def test_docs_protection() -> None:
    """验证 swagger_token 参数保护 /docs。"""
    from app.main import create_app

    app = create_app(
        enable_db=False,
        static_dir=None,
        swagger_token="docs-secret",
    )
    with TestClient(app) as c:
        assert c.get("/docs").status_code == 403, "no token should 403"
        assert (
            c.get("/docs", params={"token": "docs-secret"}).status_code == 200
        ), "correct token should 200"
        assert (
            c.get("/docs", params={"token": "wrong"}).status_code == 403
        ), "wrong token should 403"
        assert (
            c.get("/openapi.json").status_code == 200
        ), "openapi.json always public"
