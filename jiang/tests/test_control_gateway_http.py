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

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        if button_id != "box_10_button_1":
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
