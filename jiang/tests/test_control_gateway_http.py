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
        self.cancel_count = 0
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
            "active": False,
            "state": "idle",
            "message": "No operation has been sent.",
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
                    "button_pressed": True,
                    "button_travel": 0.006,
                    "button_state_updated_at": 2.0,
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
        self.control_server.cancel_count = 0
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
            "active",
            "state",
            "message",
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

    def test_cabinet_controls_lists_both_buttons_and_states(self) -> None:
        status, catalog, _ = self.request("/cabinet/controls")

        self.assertEqual(200, status)
        self.assertTrue(catalog["catalog_received"])
        self.assertEqual("operator_catalog", catalog["source"])
        self.assertEqual(
            ["box_10_button_1", "box_10_button_2"],
            [control["control_id"] for control in catalog["controls"]],
        )
        self.assertFalse(catalog["controls"][0]["button_pressed"])
        self.assertTrue(catalog["controls"][1]["button_pressed"])

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
