"""HTTP contract tests for the browser control gateway."""

from __future__ import annotations

import json
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
from control_gateway.web_server import ControlHandler  # noqa: E402


class _FakeControlServer:
    def __init__(self) -> None:
        self.press_request: Optional[Tuple[str, bool]] = None
        self.operation_request: Optional[Dict[str, Any]] = None
        self.cancel_count = 0
        self.reset_count = 0
        self.takeover_count = 0

    def health(self) -> Dict[str, Any]:
        return {
            "status": "ok",
            "navigation_available": True,
            "moveit_available": True,
            "cabinet_available": True,
            "cabinet_active": False,
        }

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

    def cabinet_controls(self) -> Dict[str, Any]:
        return {
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
            {"control_server": cls.control_server},
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

    def request(
        self,
        path: str,
        method: str = "GET",
        body: Optional[Dict[str, Any]] = None,
    ) -> Tuple[int, Dict[str, Any], Any]:
        data = None
        headers = {}
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
        status, health, headers = self.request("/health")
        self.assertEqual(200, status)
        self.assertTrue(health["cabinet_available"])
        self.assertFalse(health["cabinet_active"])
        self.assertEqual("*", headers["Access-Control-Allow-Origin"])

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
                    "target_position": 1.2,
                    "navigate_to_staging_pose": False,
                },
                None,
                1.2,
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
