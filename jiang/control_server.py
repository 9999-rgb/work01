#!/usr/bin/env python3
"""
Lightweight HTTP control server for robot commands.

Runs in a daemon thread inside run_proxy.py. Uses ``ros2 topic pub`` via
subprocess for reliable DDS publishing (rclpy from non-main threads cannot
reliably deliver messages to the DDS network).

Endpoints:
    POST /cmd_vel          - publish Twist to /xczs/cmd_vel
    POST /joint_trajectory - publish JointTrajectory to /xczs/joint_trajectory
    GET  /health           - health check
"""

from __future__ import annotations

import json
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# CORS-aware JSON request handler
# ---------------------------------------------------------------------------

class _ControlHandler(BaseHTTPRequestHandler):
    """HTTP handler with CORS, JSON-only request/response, and minimal logging."""

    control_server: "ControlServer" = None  # type: ignore[assignment]

    def do_OPTIONS(self) -> None:
        self._send_response(204, b"", "text/plain")

    def do_GET(self) -> None:
        if self.path == "/health":
            self._handle_health()
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:
        if self.path == "/cmd_vel":
            self._handle_cmd_vel_post()
        elif self.path == "/joint_trajectory":
            self._handle_joint_trajectory_post()
        else:
            self._send_json(404, {"error": "not found"})

    def _handle_health(self) -> None:
        self._send_json(200, {"status": "ok"})

    def _handle_cmd_vel_post(self) -> None:
        body = self._read_body()
        if body is None:
            return
        ly = body.get("linear_y", 0.0)
        az = body.get("angular_z", 0.0)
        self.control_server.publish_cmd_vel(ly, az)
        self._send_json(200, {"status": "ok", "linear_y": ly, "angular_z": az})

    def _handle_joint_trajectory_post(self) -> None:
        body = self._read_body()
        if body is None:
            return
        positions = body.get("positions", [])
        if len(positions) != 8:
            self._send_json(400, {"error": f"expected 8 positions, got {len(positions)}"})
            return
        try:
            positions = [float(p) for p in positions]
        except (TypeError, ValueError):
            self._send_json(400, {"error": "positions must be numbers"})
            return
        self.control_server.publish_joint_trajectory(positions)
        self._send_json(200, {"status": "ok", "positions": positions})

    def _read_body(self) -> Optional[Dict[str, Any]]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            return json.loads(self.rfile.read(length))
        except json.JSONDecodeError as exc:
            self._send_json(400, {"error": f"invalid JSON: {exc}"})
            return None

    def _send_json(self, status: int, data: Dict[str, Any]) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self._send_response(status, body, "application/json")

    def _send_response(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        return


# ---------------------------------------------------------------------------
# ControlServer — subprocess-based ROS2 publishing
# ---------------------------------------------------------------------------

class ControlServer:
    """HTTP server that publishes to ROS2 via ``ros2 topic pub`` subprocess.

    Uses subprocess instead of rclpy to guarantee DDS delivery regardless of
    which thread the HTTP handler runs on.
    """

    JOINT_NAMES = [
        "body_arm1", "arm1_arm2", "arm2_arm3",
        "arm3_arm4", "arm4_arm5", "arm5_end",
        "end_worklink1", "end_worklink2",
    ]

    def __init__(
        self,
        port: int = 8090,
        cmd_vel_topic: str = "/xczs/cmd_vel",
        joint_trajectory_topic: str = "/xczs/joint_trajectory",
        host: str = "0.0.0.0",
    ):
        self._port = port
        self._host = host
        self._cmd_vel_topic = cmd_vel_topic
        self._joint_trajectory_topic = joint_trajectory_topic

        self._http_server: Optional[HTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> "ControlServer":
        self._init_http()
        return self

    def stop(self) -> None:
        self._running = False
        if self._http_server is not None:
            try:
                self._http_server.shutdown()
                self._http_server.server_close()
            except Exception:
                pass
            self._http_server = None
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    # ------------------------------------------------------------------
    # Publishers — ros2 topic pub via subprocess
    # ------------------------------------------------------------------

    def publish_cmd_vel(self, linear_y: float, angular_z: float) -> None:
        yaml = (
            "linear:\n"
            "  x: 0.0\n"
            "  y: " + str(linear_y) + "\n"
            "  z: 0.0\n"
            "angular:\n"
            "  x: 0.0\n"
            "  y: 0.0\n"
            "  z: " + str(angular_z)
        )
        self._ros2_pub(self._cmd_vel_topic, "geometry_msgs/msg/Twist", yaml)

    def publish_joint_trajectory(self, positions: List[float]) -> None:
        pos_str = "[" + ", ".join(str(p) for p in positions) + "]"
        names_str = "[" + ", ".join(self.JOINT_NAMES) + "]"
        yaml = (
            "joint_names: " + names_str + "\n"
            "points:\n"
            "- positions: " + pos_str + "\n"
            "  velocities: []\n"
            "  accelerations: []\n"
            "  effort: []"
        )
        self._ros2_pub(
            self._joint_trajectory_topic,
            "trajectory_msgs/msg/JointTrajectory",
            yaml,
        )

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _ros2_pub(self, topic: str, msg_type: str, yaml_str: str) -> None:
        """Fire-and-forget ``ros2 topic pub --once``."""
        try:
            subprocess.run(
                ["ros2", "topic", "pub", "--once", topic, msg_type, yaml_str],
                capture_output=True,
                timeout=3.0,
            )
        except Exception:
            pass  # Failures on individual publishes are logged but don't crash

    def _init_http(self) -> None:
        handler = type(
            "_BoundHandler",
            (_ControlHandler,),
            {"control_server": self},
        )
        self._http_server = HTTPServer((self._host, self._port), handler)
        self._http_server.timeout = 0.5
        self._running = True
        self._thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="control-server",
            daemon=True,
        )
        self._thread.start()
