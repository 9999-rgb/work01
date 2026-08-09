"""Threaded HTTP API for manual and autonomous browser controls."""

from __future__ import annotations

import json
import math
import queue
from http.server import BaseHTTPRequestHandler
from typing import Any, Dict, FrozenSet, Optional
from urllib.parse import unquote, urlparse

from .ros_node import ControlRequestError
from .task_manager import EventHubClosed


class ControlHandler(BaseHTTPRequestHandler):
    """Serve validated JSON control requests with CORS support."""

    protocol_version = "HTTP/1.1"
    SSE_HEARTBEAT_SECONDS = 15.0

    control_server: Any
    allowed_origins: FrozenSet[str] = frozenset(
        {
            "http://localhost:8080",
            "http://127.0.0.1:8080",
        }
    )

    def do_OPTIONS(self) -> None:
        """Answer browser CORS preflight."""
        if not self._allow_request_origin():
            return
        self._send_response(204, b"", "text/plain")

    def do_GET(self) -> None:
        """Serve health and autonomous-control status."""
        if not self._allow_request_origin():
            return
        path = urlparse(self.path).path
        try:
            if path == "/health":
                self._send_json(200, self.control_server.health())
            elif path == "/robot/capabilities":
                self._send_json(
                    200,
                    self.control_server.robot_capabilities(),
                )
            elif path == "/cabinets":
                self._send_json(200, self.control_server.cabinets())
            elif path == "/task/events":
                self._handle_task_events()
            elif (cabinet := self._cabinet_controls_path(path)) is not None:
                self._send_json(
                    200,
                    self.control_server.cabinet_controls(cabinet),
                )
            elif (task_id := self._task_status_path(path)) is not None:
                self._send_json(
                    200,
                    self.control_server.task_status(task_id),
                )
            elif path == "/navigation/status":
                self._send_json(
                    200,
                    self.control_server.navigation_status(),
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
        except Exception as error:  # noqa: BLE001
            self._send_exception(error)

    def do_POST(self) -> None:
        """Dispatch manual, Nav2 and cabinet commands."""
        if not self._allow_request_origin():
            return
        path = urlparse(self.path).path
        try:
            routes = {
                "/cmd_vel": self._handle_cmd_vel,
                "/joint_trajectory": self._handle_joint_trajectory,
                "/task/navigate": self._handle_task_navigate,
                "/task/operate": self._handle_task_operate,
                "/navigation/mode": self._handle_navigation_mode,
                "/navigation/cancel": self._handle_navigation_cancel,
                "/navigation/takeover": self._handle_navigation_takeover,
                "/cabinet/operate": self._handle_cabinet_operate,
                "/cabinet/press": self._handle_cabinet_press,
                "/cabinet/cancel": self._handle_cabinet_cancel,
                "/cabinet/reset": self._handle_cabinet_reset,
            }
            handler = routes.get(path)
            if handler is None:
                task_id = self._task_cancel_path(path)
                if task_id is not None:
                    self._handle_task_cancel(task_id)
                    return
                self._send_json(404, {"error": "not found"})
                return
            handler()
        except Exception as error:  # noqa: BLE001
            self._send_exception(error)

    def _handle_task_navigate(self) -> None:
        body = self._required_body()
        self._reject_unknown_fields(body, {"cabinet"})
        cabinet = self._nonempty_string(body, "cabinet")
        self._send_json(
            202,
            self.control_server.submit_navigation_task(cabinet),
        )

    def _handle_task_operate(self) -> None:
        body = self._required_body()
        self._reject_unknown_fields(
            body,
            {
                "cabinet",
                "control_id",
                "command",
                "target_state",
                "target_position",
                "force",
            },
        )
        cabinet = self._nonempty_string(body, "cabinet")
        control_id = self._nonempty_string(body, "control_id")
        command = self._nonempty_string(body, "command")
        if command not in {"press", "set_state", "set_position", "toggle"}:
            raise ControlRequestError(
                "command must be press, set_state, set_position or toggle."
            )

        target_state: Optional[str] = None
        target_position: Optional[float] = None
        if command == "set_state":
            target_state = self._nonempty_string(body, "target_state")
            if "target_position" in body:
                raise ControlRequestError(
                    "target_position is only valid for set_position."
                )
        elif command == "set_position":
            target_position = self._json_number(body, "target_position")
            if "target_state" in body:
                raise ControlRequestError(
                    "target_state is only valid for set_state."
                )
        elif "target_state" in body or "target_position" in body:
            raise ControlRequestError(
                "target_state and target_position are not valid for "
                f"{command}."
            )

        force = (
            self._json_number(body, "force")
            if "force" in body
            else None
        )
        if force is not None and force <= 0.0:
            raise ControlRequestError("force must be greater than zero.")
        self._send_json(
            202,
            self.control_server.submit_operation_task(
                cabinet,
                control_id,
                command,
                target_state,
                target_position,
                force,
            ),
        )

    def _handle_task_cancel(self, task_id: str) -> None:
        body = self._optional_body()
        if body:
            raise ControlRequestError("task cancel does not accept options.")
        self._send_json(202, self.control_server.cancel_task(task_id))

    def _handle_task_events(self) -> None:
        """Stream replayable task events without entering a request scope."""
        last_event_id = self.headers.get("Last-Event-ID")
        if last_event_id is not None:
            last_event_id = last_event_id.strip()
            if not last_event_id:
                last_event_id = None
            elif not last_event_id.isascii() or not last_event_id.isdigit():
                raise ControlRequestError(
                    "Last-Event-ID must be a non-negative integer."
                )

        subscription = self.control_server.subscribe_task_events(
            last_event_id
        )
        try:
            # Bound writes to a client that stopped reading. EventHub.close()
            # wakes subscription.get() during shutdown, and this timeout keeps
            # a blocked socket flush from leaving a daemon handler indefinitely.
            self.connection.settimeout(5.0)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self._send_cors_headers()
            self.end_headers()
            self.wfile.flush()

            while True:
                try:
                    event = subscription.get(
                        timeout=self.SSE_HEARTBEAT_SECONDS
                    )
                except queue.Empty:
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
                    continue
                self.wfile.write(self._encode_sse_event(event))
                self.wfile.flush()
        except EventHubClosed:
            return
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        finally:
            self.close_connection = True
            subscription.close()

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
        self._reject_unknown_fields(body, {"positions"})
        raw_positions = body.get("positions")
        if not isinstance(raw_positions, list) or not raw_positions:
            raise ControlRequestError(
                "positions must be a non-empty list of numbers."
            )
        positions = [
            self._finite_number(
                {f"positions[{index}]": value},
                f"positions[{index}]",
            )
            for index, value in enumerate(raw_positions)
        ]
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

    def _handle_cabinet_operate(self) -> None:
        body = self._required_body()
        control_id = body.get("control_id")
        command = body.get("command")
        target_state = body.get("target_state")
        target_position = (
            self._finite_number(body, "target_position")
            if "target_position" in body
            else None
        )
        navigate_to_staging_pose = body.get(
            "navigate_to_staging_pose",
            True,
        )
        if not isinstance(control_id, str) or not control_id.strip():
            raise ControlRequestError(
                "control_id must be a non-empty string."
            )
        if not (
            isinstance(command, str)
            or (isinstance(command, int) and not isinstance(command, bool))
        ):
            raise ControlRequestError(
                "command must be a string or integer command code."
            )
        if target_state is not None:
            if not isinstance(target_state, str) or not target_state.strip():
                raise ControlRequestError(
                    "target_state must be a non-empty string when provided."
                )
            target_state = target_state.strip()
        if not isinstance(navigate_to_staging_pose, bool):
            raise ControlRequestError(
                "navigate_to_staging_pose must be a boolean."
            )
        self._send_json(
            202,
            self.control_server.operate_cabinet_control(
                control_id.strip(),
                command,
                target_state,
                target_position,
                navigate_to_staging_pose,
            ),
        )

    def _handle_cabinet_cancel(self) -> None:
        self._optional_body()
        cancel = getattr(
            self.control_server,
            "cancel_cabinet_operation",
            self.control_server.cancel_cabinet_button,
        )
        self._send_json(
            202,
            cancel(),
        )

    def _handle_cabinet_reset(self) -> None:
        body = self._optional_body()
        if body:
            raise ControlRequestError("cabinet reset does not accept options.")
        self._send_json(
            200,
            self.control_server.reset_cabinet_controls(),
        )

    @staticmethod
    def _path_parameter(path: str, prefix: str, suffix: str) -> Optional[str]:
        if not path.startswith(prefix) or not path.endswith(suffix):
            return None
        end = len(path) - len(suffix) if suffix else None
        encoded = path[len(prefix):end]
        if not encoded or "/" in encoded:
            return None
        value = unquote(encoded)
        if not value or "/" in value or "\\" in value:
            return None
        return value

    @classmethod
    def _cabinet_controls_path(cls, path: str) -> Optional[str]:
        return cls._path_parameter(path, "/cabinets/", "/controls")

    @classmethod
    def _task_status_path(cls, path: str) -> Optional[str]:
        return cls._path_parameter(path, "/task/", "/status")

    @classmethod
    def _task_cancel_path(cls, path: str) -> Optional[str]:
        return cls._path_parameter(path, "/task/", "/cancel")

    @staticmethod
    def _nonempty_string(body: Dict[str, Any], name: str) -> str:
        value = body.get(name)
        if not isinstance(value, str) or not value.strip():
            raise ControlRequestError(f"{name} must be a non-empty string.")
        return value.strip()

    @staticmethod
    def _reject_unknown_fields(
        body: Dict[str, Any],
        allowed_fields: set[str],
    ) -> None:
        unknown_fields = sorted(set(body) - allowed_fields)
        if unknown_fields:
            raise ControlRequestError(
                "Unexpected request field(s): " + ", ".join(unknown_fields)
            )

    @staticmethod
    def _json_number(
        body: Dict[str, Any],
        name: str,
        default: Optional[float] = None,
    ) -> float:
        value = body.get(name, default)
        if value is None:
            raise ControlRequestError(f"{name} is required.")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ControlRequestError(f"{name} must be a number.")
        value = float(value)
        if not math.isfinite(value):
            raise ControlRequestError(f"{name} must be finite.")
        return value

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
        if isinstance(value, bool):
            raise ControlRequestError(f"{name} must be a number.")
        try:
            value = float(value)
        except (TypeError, ValueError) as error:
            raise ControlRequestError(
                f"{name} must be a number."
            ) from error
        if not math.isfinite(value):
            raise ControlRequestError(f"{name} must be finite.")
        return value

    def _send_json(self, status: int, data: Any) -> None:
        body = json.dumps(
            data,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self._send_response(status, body, "application/json")

    def _send_exception(self, error: Exception) -> None:
        status = getattr(error, "status", 500)
        if isinstance(status, bool) or not isinstance(status, int):
            status = 500
        payload = {"error": str(error)}
        details = getattr(error, "details", None)
        if isinstance(details, dict):
            payload.update(details)
        self._send_json(status, payload)

    @staticmethod
    def _encode_sse_event(event: Any) -> bytes:
        if not isinstance(event, dict):
            raise TypeError("Task event must be an object.")
        event_id = str(event.get("id", "")).replace("\r", "").replace("\n", "")
        event_type = (
            str(event.get("event", "message"))
            .replace("\r", "")
            .replace("\n", "")
        )
        data = json.dumps(
            event.get("data", {}),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        fields = []
        if event_id:
            fields.append(f"id: {event_id}")
        fields.extend((f"event: {event_type}", f"data: {data}", "", ""))
        return "\n".join(fields).encode("utf-8")

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
        self._send_cors_headers()
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header(
            "Access-Control-Allow-Headers",
            "Content-Type, Last-Event-ID",
        )
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _allow_request_origin(self) -> bool:
        origin = self.headers.get("Origin")
        if origin is None or origin in self.allowed_origins:
            return True
        body = json.dumps(
            {"error": "Request origin is not allowed."},
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(403)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Vary", "Origin")
        self.end_headers()
        self.wfile.write(body)
        return False

    def _send_cors_headers(self) -> None:
        origin = self.headers.get("Origin")
        if origin is not None and origin in self.allowed_origins:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")

    def log_message(self, format_string: str, *args: Any) -> None:
        """Disable BaseHTTPRequestHandler access logging."""
        del format_string, args
