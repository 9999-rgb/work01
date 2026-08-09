"""HTTP contract tests for the browser control gateway."""

from __future__ import annotations

import json
import queue
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib.error import HTTPError
from urllib.request import Request, urlopen


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.task_manager import EventHubClosed  # noqa: E402
from control_gateway.web_server import ControlHandler  # noqa: E402


class _FakeConflictError(RuntimeError):
    def __init__(self, active_task_id: str) -> None:
        super().__init__("another task is active")
        self.status = 409
        self.details = {"active_task_id": active_task_id}


class _FakeEventSubscription:
    def __init__(self, events: list[Any]) -> None:
        self._events = list(events)
        self.closed = False

    def get(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        del timeout
        if not self._events:
            raise EventHubClosed("test stream ended")
        event = self._events.pop(0)
        if isinstance(event, BaseException):
            raise event
        return event

    def close(self) -> None:
        self.closed = True


class _FakeControlServer:
    def __init__(self) -> None:
        self.press_request: Optional[Tuple[str, bool]] = None
        self.operation_request: Optional[Dict[str, Any]] = None
        self.cancel_count = 0
        self.reset_count = 0
        self.takeover_count = 0
        self.joint_request: Optional[list[float]] = None
        self.requested_cabinet: Optional[str] = None
        self.navigation_task_request: Optional[str] = None
        self.operation_task_request: Optional[Dict[str, Any]] = None
        self.canceled_task_id: Optional[str] = None
        self.conflict_task_id: Optional[str] = None
        self.subscribed_last_event_id: Optional[str] = None
        self.last_subscription: Optional[_FakeEventSubscription] = None
        self.sse_events: list[Any] = []
        self.tasks: Dict[str, Dict[str, Any]] = {}
        self._task_sequence = 0
        self.expected_joint_count = 3

    def cabinets(self) -> Dict[str, Any]:
        return {
            "cabinets": [
                {
                    "name": "cabinet_a",
                    "namespace": "/xczs/cabinet/cabinet_a",
                    "frame_id": "cabinet_a_frame",
                },
                {
                    "name": "cabinet_b",
                    "namespace": "/xczs/cabinet/cabinet_b",
                    "frame_id": "cabinet_b_frame",
                },
            ]
        }

    def health(self) -> Dict[str, Any]:
        return {
            "status": "ok",
            "navigation_available": True,
            "cabinet_available": True,
            "cabinet_active": False,
        }

    def robot_capabilities(self) -> Dict[str, Any]:
        return {
            "schema_version": 1,
            "manual_linear_axis": "x",
            "frames": {
                "planning": "odom",
                "navigation": "map",
                "navigation_base": "base_link",
            },
            "topics": {
                "manual_cmd_vel": "/robot/manual_cmd_vel",
                "joint_state": "/robot/joint_states",
                "joint_trajectory": "/robot/joint_trajectory",
            },
            "joint_count": 3,
            "arm_joint_count": 2,
            "gripper_joint_count": 1,
            "manual_joints": [
                {
                    "name": "shoulder",
                    "group": "arm",
                    "min_position": -1.0,
                    "max_position": 1.0,
                    "default_position": 0.1,
                    "open_position": None,
                },
                {
                    "name": "elbow",
                    "group": "arm",
                    "min_position": -2.0,
                    "max_position": 2.0,
                    "default_position": 0.0,
                    "open_position": None,
                },
                {
                    "name": "finger",
                    "group": "gripper",
                    "min_position": 0.0,
                    "max_position": 0.4,
                    "default_position": 0.0,
                    "open_position": 0.4,
                },
            ],
        }

    def publish_joint_trajectory(
        self,
        positions: list[float],
    ) -> list[float]:
        if len(positions) != self.expected_joint_count:
            raise ControlRequestError(
                "positions must match the configured manual joint order."
            )
        self.joint_request = positions
        return positions

    def cabinet_status(self) -> Dict[str, Any]:
        return {
            "available": True,
            "operation_available": True,
            "legacy_button_available": True,
            "reset_available": True,
            "active": False,
            "state": "idle",
            "message": "No operation has been sent.",
            "control_id": "box_10_button_1",
            "control_type": 0,
            "type": 0,
            "current_position": 0.0,
            "current_state": "released",
            "command": None,
            "target_state": None,
            "target_position": None,
            "target": {"state": None, "position": None},
            "button_id": "box_10_button_1",
            "navigate_to_staging_pose": True,
            "phase": None,
            "progress": 0.0,
            "button_pressed": False,
            "button_travel": 0.0,
            "max_travel": None,
            "success": None,
            "error_code": None,
            "updated_at": 1.0,
            "button_state_updated_at": 1.0,
        }

    def cabinet_controls(
        self,
        cabinet: Optional[str] = None,
    ) -> Dict[str, Any]:
        if cabinet is not None and cabinet not in {"cabinet_a", "cabinet_b"}:
            raise ControlRequestError("Unknown cabinet.", 404)
        self.requested_cabinet = cabinet
        response = {
            "available": True,
            "operation_available": True,
            "legacy_button_available": True,
            "reset_available": True,
            "catalog_received": True,
            "source": "operator_catalog",
            "selected_control_id": "box_10_button_1",
            "controls": [
                {
                    "control_id": "box_10_button_1",
                    "display_name": "10 号模块红色按钮",
                    "control_type": 0,
                    "joint_name": "box_10_box_10_button_1",
                    "joint_state_topic": (
                        "/xczs/cabinet/box_10_button_1/joint_states"
                    ),
                    "pressed_topic": (
                        "/xczs/cabinet/box_10_button_1/pressed"
                    ),
                    "state_topic": "/xczs/cabinet/box_10_button_1/state",
                    "supported_commands": 1,
                    "unit": "m",
                    "min_position": 0.0,
                    "max_position": 0.008,
                    "state_ids": ["released", "pressed"],
                    "state_labels": ["已释放", "已按下"],
                    "state_positions": [0.0, 0.006],
                    "operable": True,
                    "current_position": 0.0,
                    "current_state": "released",
                    "button_pressed": False,
                    "button_travel": 0.0,
                    "button_state_updated_at": 1.0,
                },
                {
                    "control_id": "box_10_button_2",
                    "display_name": "10 号模块绿色按钮",
                    "control_type": 0,
                    "joint_name": "box_10_box_10_button_2",
                    "joint_state_topic": (
                        "/xczs/cabinet/box_10_button_2/joint_states"
                    ),
                    "pressed_topic": (
                        "/xczs/cabinet/box_10_button_2/pressed"
                    ),
                    "state_topic": "/xczs/cabinet/box_10_button_2/state",
                    "supported_commands": 1,
                    "unit": "m",
                    "min_position": 0.0,
                    "max_position": 0.008,
                    "state_ids": ["released", "pressed"],
                    "state_labels": ["已释放", "已按下"],
                    "state_positions": [0.0, 0.006],
                    "operable": True,
                    "current_position": 0.006,
                    "current_state": "pressed",
                    "button_pressed": True,
                    "button_travel": 0.006,
                    "button_state_updated_at": 2.0,
                },
                {
                    "control_id": "box_03_knob_1",
                    "display_name": "3 号模块旋钮",
                    "control_type": 1,
                    "state_topic": "/xczs/cabinet/box_03_knob_1/state",
                    "supported_commands": 2 | 4 | 8,
                    "unit": "rad",
                    "min_position": -3.14159,
                    "max_position": 3.14159,
                    "state_ids": ["left", "center", "right"],
                    "state_labels": ["左", "中", "右"],
                    "state_positions": [-1.5708, 0.0, 1.5708],
                    "operable": True,
                    "current_position": 0.0,
                    "current_state": "center",
                },
                {
                    "control_id": "box_07_switch_1",
                    "display_name": "7 号模块总开关",
                    "control_type": 2,
                    "state_topic": "/xczs/cabinet/box_07_switch_1/state",
                    "supported_commands": 2 | 8,
                    "unit": "rad",
                    "min_position": -0.5,
                    "max_position": 0.5,
                    "state_ids": ["off", "on"],
                    "state_labels": ["关", "开"],
                    "state_positions": [-0.5, 0.5],
                    "operable": True,
                    "current_position": -0.5,
                    "current_state": "off",
                },
                {
                    "control_id": "cabinet_door",
                    "display_name": "控制柜门",
                    "control_type": 3,
                    "state_topic": "/xczs/cabinet/cabinet_door/state",
                    "supported_commands": 2 | 8,
                    "unit": "rad",
                    "min_position": 0.0,
                    "max_position": 1.6,
                    "state_ids": ["closed", "open"],
                    "state_labels": ["关闭", "打开"],
                    "state_positions": [0.0, 1.57],
                    "operable": True,
                    "current_position": 0.0,
                    "current_state": "closed",
                },
            ],
        }
        if cabinet is not None:
            response["cabinet"] = cabinet
        return response

    def _new_task(self, task_type: str, request: Dict[str, Any]) -> Dict[str, Any]:
        if self.conflict_task_id is not None:
            raise _FakeConflictError(self.conflict_task_id)
        self._task_sequence += 1
        task_id = f"{task_type}_1700000000000_{self._task_sequence:06d}"
        task = {
            "task_id": task_id,
            "type": task_type,
            "status": "accepted",
            "request": request,
        }
        self.tasks[task_id] = task
        return task

    def submit_navigation_task(self, cabinet: str) -> Dict[str, Any]:
        self.navigation_task_request = cabinet
        return self._new_task("navigate", {"cabinet": cabinet})

    def submit_operation_task(
        self,
        cabinet: str,
        control_id: str,
        command: str,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Dict[str, Any]:
        self.operation_task_request = {
            "cabinet": cabinet,
            "control_id": control_id,
            "command": command,
            "target_state": target_state,
            "target_position": target_position,
            "force": force,
        }
        return self._new_task("operate", self.operation_task_request)

    def task_status(self, task_id: str) -> Dict[str, Any]:
        try:
            return self.tasks[task_id]
        except KeyError as error:
            raise ControlRequestError("Unknown task.", 404) from error

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        task = self.task_status(task_id)
        self.canceled_task_id = task_id
        task["status"] = "canceling"
        return task

    def subscribe_task_events(
        self,
        last_event_id: Optional[str],
    ) -> _FakeEventSubscription:
        self.subscribed_last_event_id = last_event_id
        self.last_subscription = _FakeEventSubscription(self.sse_events)
        return self.last_subscription

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        if button_id not in {
            "box_10_button_1",
            "box_10_button_2",
        }:
            raise ControlRequestError("Unsupported cabinet button.")
        self.press_request = (button_id, navigate_to_staging_pose)
        return {
            "status": "accepted",
            "button_id": button_id,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }

    def cancel_cabinet_button(self) -> Dict[str, Any]:
        self.cancel_count += 1
        return {"status": "canceling"}

    def operate_cabinet_control(
        self,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        if control_id not in {
            "box_10_button_1",
            "box_10_button_2",
            "box_03_knob_1",
            "box_07_switch_1",
            "cabinet_door",
        }:
            raise ControlRequestError("Unsupported cabinet control.")
        self.operation_request = {
            "control_id": control_id,
            "command": command,
            "target_state": target_state,
            "target_position": target_position,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }
        return {"status": "accepted", **self.operation_request}

    def cancel_cabinet_operation(self) -> Dict[str, Any]:
        return self.cancel_cabinet_button()

    def reset_cabinet_controls(self) -> Dict[str, Any]:
        self.reset_count += 1
        return {"status": "reset", "message": "reset complete"}

    def takeover_navigation(self) -> Dict[str, Any]:
        self.takeover_count += 1
        return {"status": "taking_over", "mode": True}


class ControlGatewayHttpTest(unittest.TestCase):
    """Exercise cabinet routes through a real local HTTP listener."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.control_server = _FakeControlServer()
        handler = type(
            "_TestControlHandler",
            (ControlHandler,),
            {
                "control_server": cls.control_server,
                "SSE_HEARTBEAT_SECONDS": 0.01,
                "allowed_origins": frozenset(
                    {"http://localhost:8080"}
                ),
            },
        )
        cls.http_server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        cls.http_server.daemon_threads = True
        cls.http_thread = threading.Thread(
            target=cls.http_server.serve_forever,
            daemon=True,
        )
        cls.http_thread.start()
        host, port = cls.http_server.server_address
        cls.base_url = f"http://{host}:{port}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.http_server.shutdown()
        cls.http_server.server_close()
        cls.http_thread.join(timeout=3.0)

    def setUp(self) -> None:
        self.control_server.press_request = None
        self.control_server.operation_request = None
        self.control_server.cancel_count = 0
        self.control_server.reset_count = 0
        self.control_server.takeover_count = 0
        self.control_server.joint_request = None
        self.control_server.requested_cabinet = None
        self.control_server.navigation_task_request = None
        self.control_server.operation_task_request = None
        self.control_server.canceled_task_id = None
        self.control_server.conflict_task_id = None
        self.control_server.subscribed_last_event_id = None
        self.control_server.last_subscription = None
        self.control_server.sse_events = []
        self.control_server.tasks = {}
        self.control_server._task_sequence = 0
        self.control_server.expected_joint_count = 3

    def request(
        self,
        path: str,
        method: str = "GET",
        body: Optional[Dict[str, Any]] = None,
        request_headers: Optional[Dict[str, str]] = None,
    ) -> Tuple[int, Dict[str, Any], Any]:
        data = None
        headers = dict(request_headers or {})
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = Request(
            self.base_url + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            response = urlopen(request, timeout=2.0)
        except HTTPError as error:
            response = error
        with response:
            payload = json.loads(response.read().decode("utf-8"))
            return response.status, payload, response.headers

    def test_health_and_cabinet_status(self) -> None:
        status, health, headers = self.request(
            "/health",
            request_headers={"Origin": "http://localhost:8080"},
        )
        self.assertEqual(200, status)
        self.assertTrue(health["cabinet_available"])
        self.assertFalse(health["cabinet_active"])
        self.assertNotIn("moveit_available", health)
        self.assertEqual(
            "http://localhost:8080",
            headers["Access-Control-Allow-Origin"],
        )

        status, cabinet, _ = self.request("/cabinet/status")
        self.assertEqual(200, status)
        self.assertEqual("idle", cabinet["state"])
        expected_fields = {
            "available",
            "operation_available",
            "reset_available",
            "active",
            "state",
            "message",
            "control_id",
            "type",
            "current_position",
            "current_state",
            "target",
            "command",
            "button_id",
            "navigate_to_staging_pose",
            "phase",
            "progress",
            "button_pressed",
            "button_travel",
            "max_travel",
            "success",
            "error_code",
            "updated_at",
            "button_state_updated_at",
        }
        self.assertTrue(expected_fields.issubset(cabinet))

    def test_untrusted_browser_origin_is_rejected_before_dispatch(self) -> None:
        status, payload, headers = self.request(
            "/health",
            request_headers={"Origin": "https://malicious.example"},
        )
        self.assertEqual(403, status)
        self.assertEqual("Request origin is not allowed.", payload["error"])
        self.assertNotIn("Access-Control-Allow-Origin", headers)

    def test_inventory_and_per_cabinet_catalog_routes(self) -> None:
        status, inventory, _ = self.request("/cabinets")
        self.assertEqual(200, status)
        self.assertEqual(
            ["cabinet_a", "cabinet_b"],
            [cabinet["name"] for cabinet in inventory["cabinets"]],
        )

        status, catalog, _ = self.request(
            "/cabinets/cabinet_a/controls"
        )
        self.assertEqual(200, status)
        self.assertEqual("cabinet_a", catalog["cabinet"])
        self.assertEqual("cabinet_a", self.control_server.requested_cabinet)

        status, response, _ = self.request(
            "/cabinets/unknown/controls"
        )
        self.assertEqual(404, status)
        self.assertEqual("Unknown cabinet.", response["error"])

    def test_task_navigation_status_and_cancel_routes(self) -> None:
        status, accepted, _ = self.request(
            "/task/navigate",
            "POST",
            {"cabinet": " cabinet_b "},
        )
        self.assertEqual(202, status)
        self.assertEqual("cabinet_b", self.control_server.navigation_task_request)
        self.assertRegex(
            accepted["task_id"],
            r"^navigate_[0-9]{13}_[a-z0-9]{6}$",
        )

        task_id = accepted["task_id"]
        status, task, _ = self.request(f"/task/{task_id}/status")
        self.assertEqual(200, status)
        self.assertEqual("accepted", task["status"])

        status, task, _ = self.request(
            f"/task/{task_id}/cancel",
            "POST",
            {},
        )
        self.assertEqual(202, status)
        self.assertEqual("canceling", task["status"])
        self.assertEqual(task_id, self.control_server.canceled_task_id)

    def test_arbitrary_coordinate_navigation_routes_are_not_exposed(
        self,
    ) -> None:
        status, response, _ = self.request("/navigation/map")
        self.assertEqual(404, status)
        self.assertEqual("not found", response["error"])

        status, response, _ = self.request(
            "/navigation/goal",
            "POST",
            {"x": 1.0, "y": 2.0, "yaw": 0.0},
        )
        self.assertEqual(404, status)
        self.assertEqual("not found", response["error"])

    def test_new_operation_route_defaults_and_forwards_force(self) -> None:
        status, accepted, _ = self.request(
            "/task/operate",
            "POST",
            {
                "cabinet": "cabinet_a",
                "control_id": "box_10_button_1",
                "command": "press",
            },
        )
        self.assertEqual(202, status)
        self.assertTrue(accepted["task_id"].startswith("operate_"))
        self.assertEqual(
            {
                "cabinet": "cabinet_a",
                "control_id": "box_10_button_1",
                "command": "press",
                "target_state": None,
                "target_position": None,
                "force": None,
            },
            self.control_server.operation_task_request,
        )

        status, _, _ = self.request(
            "/task/operate",
            "POST",
            {
                "cabinet": "cabinet_b",
                "control_id": "box_03_knob_1",
                "command": "set_position",
                "target_position": 1.5708,
                "force": 6.25,
            },
        )
        self.assertEqual(202, status)
        self.assertEqual(1.5708, self.control_server.operation_task_request[
            "target_position"
        ])
        self.assertEqual(
            6.25,
            self.control_server.operation_task_request["force"],
        )

        status, _, _ = self.request(
            "/task/operate",
            "POST",
            {
                "cabinet": "cabinet_a",
                "control_id": "box_07_switch_1",
                "command": "set_state",
                "target_state": " on ",
            },
        )
        self.assertEqual(202, status)
        self.assertEqual(
            "on",
            self.control_server.operation_task_request["target_state"],
        )

    def test_new_task_routes_reject_invalid_or_mixed_parameters(self) -> None:
        invalid_operations = [
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "box_10_button_1",
                    "command": "press",
                    "force": 0,
                },
                "greater than zero",
            ),
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "box_10_button_1",
                    "command": "press",
                    "force": float("nan"),
                },
                "finite",
            ),
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "box_10_button_1",
                    "command": "press",
                    "force": "5.0",
                },
                "must be a number",
            ),
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "box_10_button_1",
                    "command": "press",
                    "navigate_to_staging_pose": True,
                },
                "Unexpected request field",
            ),
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "box_03_knob_1",
                    "command": "set_position",
                },
                "target_position is required",
            ),
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "box_07_switch_1",
                    "command": "set_state",
                    "target_state": "",
                },
                "non-empty",
            ),
            (
                {
                    "cabinet": "cabinet_a",
                    "control_id": "cabinet_door",
                    "command": "toggle",
                    "target_state": "open",
                },
                "not valid for toggle",
            ),
        ]
        for body, error_fragment in invalid_operations:
            with self.subTest(body=body):
                status, response, _ = self.request(
                    "/task/operate",
                    "POST",
                    body,
                )
                self.assertEqual(400, status)
                self.assertIn(error_fragment, response["error"])

        status, response, _ = self.request(
            "/task/navigate",
            "POST",
            {"cabinet": "cabinet_a", "x": 1.0},
        )
        self.assertEqual(400, status)
        self.assertIn("Unexpected request field", response["error"])

    def test_task_conflict_includes_active_task_id(self) -> None:
        self.control_server.conflict_task_id = "operate_1700000000000_a1b2c3"
        status, response, _ = self.request(
            "/task/navigate",
            "POST",
            {"cabinet": "cabinet_a"},
        )
        self.assertEqual(409, status)
        self.assertEqual("another task is active", response["error"])
        self.assertEqual(
            "operate_1700000000000_a1b2c3",
            response["active_task_id"],
        )

    def test_task_sse_is_http_11_replayable_and_closes_subscription(self) -> None:
        self.control_server.sse_events = [
            queue.Empty(),
            {
                "id": "12",
                "event": "task_progress",
                "data": {
                    "task_id": "navigate_1700000000000_a1b2c3",
                    "phase": "driving",
                    "progress": 0.5,
                },
            },
            EventHubClosed("test stream ended"),
        ]
        request = Request(
            self.base_url + "/task/events",
            headers={
                "Accept": "text/event-stream",
                "Last-Event-ID": "11",
            },
        )
        with urlopen(request, timeout=2.0) as response:
            body = response.read().decode("utf-8")
            self.assertEqual(200, response.status)
            self.assertEqual(11, response.version)
            self.assertEqual(
                "text/event-stream; charset=utf-8",
                response.headers["Content-Type"],
            )
            self.assertEqual("no", response.headers["X-Accel-Buffering"])

        self.assertIn(": heartbeat\n\n", body)
        self.assertIn("id: 12\n", body)
        self.assertIn("event: task_progress\n", body)
        self.assertIn(
            'data: {"task_id":"navigate_1700000000000_a1b2c3",'
            '"phase":"driving","progress":0.5}\n\n',
            body,
        )
        self.assertEqual("11", self.control_server.subscribed_last_event_id)
        self.assertTrue(self.control_server.last_subscription.closed)

    def test_task_sse_rejects_invalid_last_event_id(self) -> None:
        request = Request(
            self.base_url + "/task/events",
            headers={"Last-Event-ID": "not-a-number"},
        )
        try:
            response = urlopen(request, timeout=2.0)
        except HTTPError as error:
            response = error
        with response:
            payload = json.loads(response.read().decode("utf-8"))
            self.assertEqual(400, response.status)
            self.assertIn("non-negative integer", payload["error"])
        self.assertIsNone(self.control_server.last_subscription)

    def test_direct_moveit_routes_are_not_exposed(self) -> None:
        status, response, _ = self.request("/motion/status")
        self.assertEqual(404, status)
        self.assertEqual("not found", response["error"])

        for path in ("/motion/named", "/motion/pose", "/motion/cancel"):
            with self.subTest(path=path):
                status, response, _ = self.request(path, "POST", {})
                self.assertEqual(404, status)
                self.assertEqual("not found", response["error"])

    def test_robot_capabilities_exposes_ordered_manual_contract(self) -> None:
        status, capabilities, _ = self.request("/robot/capabilities")

        self.assertEqual(200, status)
        self.assertEqual("x", capabilities["manual_linear_axis"])
        self.assertEqual("map", capabilities["frames"]["navigation"])
        self.assertEqual(3, capabilities["joint_count"])
        self.assertEqual(
            ["shoulder", "elbow", "finger"],
            [joint["name"] for joint in capabilities["manual_joints"]],
        )
        self.assertEqual(
            0.4,
            capabilities["manual_joints"][2]["open_position"],
        )

    def test_joint_trajectory_delegates_dynamic_length_to_backend(self) -> None:
        manual_positions = [0.1, -0.2, 0.3]
        status, response, _ = self.request(
            "/joint_trajectory",
            "POST",
            {"positions": manual_positions},
        )
        self.assertEqual(200, status)
        self.assertEqual(manual_positions, response["positions"])
        self.assertEqual(manual_positions, self.control_server.joint_request)

        for invalid in ([], "not-a-list", None):
            with self.subTest(value=invalid):
                status, response, _ = self.request(
                    "/joint_trajectory",
                    "POST",
                    {"positions": invalid},
                )
                self.assertEqual(400, status)
                self.assertIn("non-empty list", response["error"])

        for invalid in ([0.0, True], [0.0, float("nan")]):
            with self.subTest(value=invalid):
                status, response, _ = self.request(
                    "/joint_trajectory",
                    "POST",
                    {"positions": invalid},
                )
                self.assertEqual(400, status)
                self.assertIn("positions[1]", response["error"])

        status, response, _ = self.request(
            "/joint_trajectory",
            "POST",
            {"positions": [0.0, 0.0]},
        )
        self.assertEqual(400, status)
        self.assertIn("configured manual joint order", response["error"])

    def test_cabinet_controls_lists_all_control_types_and_states(self) -> None:
        status, catalog, _ = self.request("/cabinet/controls")

        self.assertEqual(200, status)
        self.assertTrue(catalog["catalog_received"])
        self.assertEqual("operator_catalog", catalog["source"])
        self.assertEqual(
            [
                "box_10_button_1",
                "box_10_button_2",
                "box_03_knob_1",
                "box_07_switch_1",
                "cabinet_door",
            ],
            [control["control_id"] for control in catalog["controls"]],
        )
        self.assertFalse(catalog["controls"][0]["button_pressed"])
        self.assertTrue(catalog["controls"][1]["button_pressed"])
        self.assertEqual(
            [0, 0, 1, 2, 3],
            [control["control_type"] for control in catalog["controls"]],
        )

    def test_operate_accepts_generic_commands_and_targets(self) -> None:
        cases = [
            (
                {
                    "control_id": "box_10_button_1",
                    "command": "press",
                    "navigate_to_staging_pose": True,
                },
                None,
                None,
            ),
            (
                {
                    "control_id": "box_03_knob_1",
                    "command": "set_position",
                    "target_position": 1.5708,
                    "navigate_to_staging_pose": False,
                },
                None,
                1.5708,
            ),
            (
                {
                    "control_id": "box_07_switch_1",
                    "command": "set_state",
                    "target_state": " on ",
                },
                "on",
                None,
            ),
            (
                {
                    "control_id": "cabinet_door",
                    "command": "toggle",
                },
                None,
                None,
            ),
        ]
        for body, expected_state, expected_position in cases:
            with self.subTest(control_id=body["control_id"]):
                status, response, _ = self.request(
                    "/cabinet/operate",
                    "POST",
                    body,
                )
                self.assertEqual(202, status)
                self.assertEqual("accepted", response["status"])
                self.assertEqual(
                    body["control_id"],
                    self.control_server.operation_request["control_id"],
                )
                self.assertEqual(
                    expected_state,
                    self.control_server.operation_request["target_state"],
                )
                self.assertEqual(
                    expected_position,
                    self.control_server.operation_request["target_position"],
                )

    def test_operate_rejects_malformed_generic_payloads(self) -> None:
        invalid_bodies = [
            ({"control_id": "", "command": "press"}, "control_id"),
            ({"control_id": "cabinet_door", "command": True}, "command"),
            (
                {
                    "control_id": "box_03_knob_1",
                    "command": "set_position",
                    "target_position": True,
                },
                "number",
            ),
            (
                {
                    "control_id": "box_07_switch_1",
                    "command": "set_state",
                    "target_state": " ",
                },
                "non-empty",
            ),
            (
                {
                    "control_id": "cabinet_door",
                    "command": "toggle",
                    "navigate_to_staging_pose": "false",
                },
                "boolean",
            ),
        ]
        for body, error_fragment in invalid_bodies:
            with self.subTest(body=body):
                status, response, _ = self.request(
                    "/cabinet/operate",
                    "POST",
                    body,
                )
                self.assertEqual(400, status)
                self.assertIn(error_fragment, response["error"])

    def test_reset_accepts_empty_body_and_rejects_options(self) -> None:
        status, response, _ = self.request(
            "/cabinet/reset",
            "POST",
            {},
        )
        self.assertEqual(200, status)
        self.assertEqual("reset", response["status"])
        self.assertEqual(1, self.control_server.reset_count)

        status, response, _ = self.request(
            "/cabinet/reset",
            "POST",
            {"control_id": "cabinet_door"},
        )
        self.assertEqual(400, status)
        self.assertIn("does not accept options", response["error"])
        self.assertEqual(1, self.control_server.reset_count)

    def test_press_and_cancel(self) -> None:
        status, response, _ = self.request(
            "/cabinet/press",
            "POST",
            {
                "button_id": " box_10_button_1 ",
                "navigate_to_staging_pose": True,
            },
        )
        self.assertEqual(202, status)
        self.assertEqual("accepted", response["status"])
        self.assertEqual(
            ("box_10_button_1", True),
            self.control_server.press_request,
        )

        status, response, _ = self.request(
            "/cabinet/cancel",
            "POST",
            {},
        )
        self.assertEqual(202, status)
        self.assertEqual("canceling", response["status"])
        self.assertEqual(1, self.control_server.cancel_count)

    def test_press_defaults_to_navigation(self) -> None:
        status, _, _ = self.request(
            "/cabinet/press",
            "POST",
            {"button_id": "box_10_button_1"},
        )
        self.assertEqual(202, status)
        self.assertEqual(
            ("box_10_button_1", True),
            self.control_server.press_request,
        )

    def test_press_accepts_second_catalog_button(self) -> None:
        status, response, _ = self.request(
            "/cabinet/press",
            "POST",
            {
                "button_id": "box_10_button_2",
                "navigate_to_staging_pose": False,
            },
        )

        self.assertEqual(202, status)
        self.assertEqual("box_10_button_2", response["button_id"])
        self.assertEqual(
            ("box_10_button_2", False),
            self.control_server.press_request,
        )

    def test_press_rejects_invalid_payload(self) -> None:
        status, response, _ = self.request(
            "/cabinet/press",
            "POST",
            {
                "button_id": "box_10_button_1",
                "navigate_to_staging_pose": "true",
            },
        )
        self.assertEqual(400, status)
        self.assertIn("must be a boolean", response["error"])
        self.assertIsNone(self.control_server.press_request)

        status, response, _ = self.request(
            "/cabinet/press",
            "POST",
            {
                "button_id": "unknown_button",
                "navigate_to_staging_pose": False,
            },
        )
        self.assertEqual(400, status)
        self.assertIn("Unsupported", response["error"])

    def test_navigation_takeover_accepts_an_empty_request(self) -> None:
        status, response, _ = self.request(
            "/navigation/takeover",
            "POST",
            {},
        )

        self.assertEqual(202, status)
        self.assertEqual("taking_over", response["status"])
        self.assertTrue(response["mode"])
        self.assertEqual(1, self.control_server.takeover_count)

    def test_navigation_takeover_rejects_a_velocity_payload(self) -> None:
        status, response, _ = self.request(
            "/navigation/takeover",
            "POST",
            {"linear_y": 0.2, "angular_z": 0.0},
        )

        self.assertEqual(400, status)
        self.assertIn("does not accept a velocity", response["error"])
        self.assertEqual(0, self.control_server.takeover_count)


if __name__ == "__main__":
    unittest.main()
