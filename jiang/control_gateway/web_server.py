"""Threaded HTTP API for manual and autonomous browser controls."""

from __future__ import annotations

import json
import math
from http.server import BaseHTTPRequestHandler
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

from .ros_node import ControlRequestError


class ControlHandler(BaseHTTPRequestHandler):
    """Serve validated JSON control requests with CORS support."""

    control_server: Any

    def do_OPTIONS(self) -> None:
        """Answer browser CORS preflight."""
        self._send_response(204, b"", "text/plain")

    def do_GET(self) -> None:
        """Serve health and autonomous-control status."""
        path = urlparse(self.path).path
        try:
            if path == "/health":
                self._send_json(200, self.control_server.health())
            elif path == "/navigation/status":
                self._send_json(
                    200,
                    self.control_server.navigation_status(),
                )
            elif path == "/navigation/map":
                self._send_json(
                    200,
                    self.control_server.navigation_map(),
                )
            elif path == "/motion/status":
                self._send_json(
                    200,
                    self.control_server.motion_status(),
                )
            elif path == "/cabinet/status":
                self._send_json(
                    200,
                    self.control_server.cabinet_status(),
                )
            elif path == "/cabinet/controls":
                self._send_json(
                    200,
                    self.control_server.cabinet_controls(),
                )
            else:
                self._send_json(404, {"error": "not found"})
        except ControlRequestError as error:
            self._send_json(error.status, {"error": str(error)})
        except Exception as error:  # noqa: BLE001
            self._send_json(500, {"error": str(error)})

    def do_POST(self) -> None:
        """Dispatch manual, Nav2 and MoveIt commands."""
        path = urlparse(self.path).path
        try:
            routes = {
                "/cmd_vel": self._handle_cmd_vel,
                "/joint_trajectory": self._handle_joint_trajectory,
                "/navigation/mode": self._handle_navigation_mode,
                "/navigation/goal": self._handle_navigation_goal,
                "/navigation/cancel": self._handle_navigation_cancel,
                "/navigation/takeover": self._handle_navigation_takeover,
                "/motion/named": self._handle_motion_named,
                "/motion/pose": self._handle_motion_pose,
                "/motion/cancel": self._handle_motion_cancel,
                "/cabinet/press": self._handle_cabinet_press,
                "/cabinet/cancel": self._handle_cabinet_cancel,
            }
            handler = routes.get(path)
            if handler is None:
                self._send_json(404, {"error": "not found"})
                return
            handler()
        except ControlRequestError as error:
            self._send_json(error.status, {"error": str(error)})
        except Exception as error:  # noqa: BLE001
            self._send_json(500, {"error": str(error)})

    def _handle_cmd_vel(self) -> None:
        body = self._required_body()
        linear_y = self._finite_number(body, "linear_y", 0.0)
        angular_z = self._finite_number(body, "angular_z", 0.0)
        linear_y, angular_z = self.control_server.publish_cmd_vel(
            linear_y,
            angular_z,
        )
        self._send_json(
            200,
            {
                "status": "ok",
                "linear_y": linear_y,
                "angular_z": angular_z,
            },
        )

    def _handle_joint_trajectory(self) -> None:
        body = self._required_body()
        positions = self._number_list(body, "positions", 8)
        positions = self.control_server.publish_joint_trajectory(positions)
        self._send_json(
            200,
            {"status": "ok", "positions": positions},
        )

    def _handle_navigation_mode(self) -> None:
        body = self._required_body()
        enabled = body.get("enabled")
        if not isinstance(enabled, bool):
            raise ControlRequestError("enabled must be a boolean.")
        self._send_json(
            202,
            self.control_server.set_navigation_mode(enabled),
        )

    def _handle_navigation_goal(self) -> None:
        body = self._required_body()
        x = self._finite_number(body, "x")
        y = self._finite_number(body, "y")
        yaw = self._finite_number(body, "yaw")
        self._send_json(
            202,
            self.control_server.send_navigation_goal(x, y, yaw),
        )

    def _handle_navigation_cancel(self) -> None:
        self._optional_body()
        self._send_json(
            202,
            self.control_server.cancel_navigation(),
        )

    def _handle_navigation_takeover(self) -> None:
        body = self._optional_body()
        if body:
            raise ControlRequestError(
                "navigation takeover does not accept a velocity; wait for "
                "manual_control_ready, then use /cmd_vel."
            )
        self._send_json(
            202,
            self.control_server.takeover_navigation(),
        )

    def _handle_motion_named(self) -> None:
        body = self._required_body()
        group = body.get("group")
        target = body.get("target")
        execute = body.get("execute", True)
        if not isinstance(group, str) or not isinstance(target, str):
            raise ControlRequestError(
                "group and target must be strings."
            )
        if not isinstance(execute, bool):
            raise ControlRequestError("execute must be a boolean.")
        self._send_json(
            202,
            self.control_server.send_named_motion(
                group,
                target,
                execute,
            ),
        )

    def _handle_motion_pose(self) -> None:
        body = self._required_body()
        frame_id = body.get("frame_id", "body")
        execute = body.get("execute", False)
        if not isinstance(frame_id, str):
            raise ControlRequestError("frame_id must be a string.")
        if not isinstance(execute, bool):
            raise ControlRequestError("execute must be a boolean.")
        position = self._number_list(body, "position", 3)
        orientation = self._number_list(body, "orientation", 4)
        self._send_json(
            202,
            self.control_server.send_pose_motion(
                frame_id,
                position,
                orientation,
                execute,
            ),
        )

    def _handle_motion_cancel(self) -> None:
        self._optional_body()
        self._send_json(
            202,
            self.control_server.cancel_motion(),
        )

    def _handle_cabinet_press(self) -> None:
        body = self._required_body()
        button_id = body.get("button_id")
        navigate_to_staging_pose = body.get(
            "navigate_to_staging_pose",
            True,
        )
        if not isinstance(button_id, str) or not button_id.strip():
            raise ControlRequestError(
                "button_id must be a non-empty string."
            )
        if not isinstance(navigate_to_staging_pose, bool):
            raise ControlRequestError(
                "navigate_to_staging_pose must be a boolean."
            )
        self._send_json(
            202,
            self.control_server.press_cabinet_button(
                button_id.strip(),
                navigate_to_staging_pose,
            ),
        )

    def _handle_cabinet_cancel(self) -> None:
        self._optional_body()
        self._send_json(
            202,
            self.control_server.cancel_cabinet_button(),
        )

    def _required_body(self) -> Dict[str, Any]:
        body = self._read_body(required=True)
        if body is None:
            raise ControlRequestError("JSON body is required.")
        return body

    def _optional_body(self) -> Optional[Dict[str, Any]]:
        return self._read_body(required=False)

    def _read_body(
        self,
        required: bool,
    ) -> Optional[Dict[str, Any]]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ControlRequestError("Invalid Content-Length.") from error
        if length == 0 and not required:
            return None
        if length <= 0 or length > 65536:
            raise ControlRequestError("Invalid Content-Length.")
        try:
            body = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError) as error:
            raise ControlRequestError(f"Invalid JSON: {error}") from error
        if not isinstance(body, dict):
            raise ControlRequestError("JSON body must be an object.")
        return body

    @staticmethod
    def _finite_number(
        body: Dict[str, Any],
        name: str,
        default: Optional[float] = None,
    ) -> float:
        value = body.get(name, default)
        if value is None:
            raise ControlRequestError(f"{name} is required.")
        try:
            value = float(value)
        except (TypeError, ValueError) as error:
            raise ControlRequestError(
                f"{name} must be a number."
            ) from error
        if not math.isfinite(value):
            raise ControlRequestError(f"{name} must be finite.")
        return value

    @classmethod
    def _number_list(
        cls,
        body: Dict[str, Any],
        name: str,
        expected_length: int,
    ) -> List[float]:
        values = body.get(name)
        if not isinstance(values, list) or len(values) != expected_length:
            raise ControlRequestError(
                f"{name} must contain {expected_length} numbers."
            )
        result = []
        for index, value in enumerate(values):
            result.append(
                cls._finite_number(
                    {f"{name}[{index}]": value},
                    f"{name}[{index}]",
                )
            )
        return result

    def _send_json(self, status: int, data: Any) -> None:
        body = json.dumps(
            data,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self._send_response(status, body, "application/json")

    def _send_response(
        self,
        status: int,
        body: bytes,
        content_type: str,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def log_message(self, format_string: str, *args: Any) -> None:
        """Disable BaseHTTPRequestHandler access logging."""
        del format_string, args
