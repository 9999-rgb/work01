#!/usr/bin/env python3
"""HTTP-to-ROS 2 control gateway for the XCZS inspection robot."""

from __future__ import annotations

import argparse
import json
import math
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class _VelocityProfile:
    """One-dimensional jerk- and acceleration-limited velocity profile."""

    def __init__(self, max_acceleration: float, max_jerk: float) -> None:
        self._max_acceleration = max_acceleration
        self._max_jerk = max_jerk
        self.velocity = 0.0
        self.acceleration = 0.0

    def update(self, target: float, period: float) -> float:
        period = max(1.0e-4, min(period, 0.1))
        velocity_error = target - self.velocity
        desired_acceleration = max(
            -self._max_acceleration,
            min(self._max_acceleration, velocity_error / period),
        )
        acceleration_step = self._max_jerk * period
        self.acceleration += max(
            -acceleration_step,
            min(
                acceleration_step,
                desired_acceleration - self.acceleration,
            ),
        )
        next_velocity = self.velocity + self.acceleration * period

        if velocity_error == 0.0 or (
            velocity_error > 0.0 and next_velocity >= target
        ) or (
            velocity_error < 0.0 and next_velocity <= target
        ):
            self.velocity = target
            self.acceleration = 0.0
        else:
            self.velocity = next_velocity
        return self.velocity

    def reset(self) -> None:
        self.velocity = 0.0
        self.acceleration = 0.0


class _RosControlPublisher(Node):
    """Own the persistent ROS 2 publishers and the velocity watchdog."""

    JOINT_NAMES = [
        "body_arm1",
        "arm1_arm2",
        "arm2_arm3",
        "arm3_arm4",
        "arm4_arm5",
        "arm5_end",
        "end_worklink1",
        "end_worklink2",
    ]

    def __init__(
        self,
        cmd_vel_topic: str,
        joint_trajectory_topic: str,
        max_linear_speed: float,
        max_angular_speed: float,
        command_timeout: float,
        context: Context,
    ) -> None:
        super().__init__(
            "xczs_web_control_server",
            context=context,
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            joint_trajectory_topic,
            10,
        )
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._command_timeout = command_timeout
        self._linear_profile = _VelocityProfile(0.50, 2.00)
        self._angular_profile = _VelocityProfile(1.20, 4.80)
        self._target_linear_y = 0.0
        self._target_angular_z = 0.0
        self._last_command_time = time.monotonic()
        self._last_update_time = time.monotonic()
        self._pending_trajectory: Optional[JointTrajectory] = None
        self._pending_trajectory_repeats = 0
        self._lock = threading.Lock()
        self.create_timer(0.02, self._update)

    def set_base_target(
        self,
        linear_y: float,
        angular_z: float,
    ) -> Tuple[float, float]:
        linear_y = max(
            -self._max_linear_speed,
            min(self._max_linear_speed, linear_y),
        )
        angular_z = max(
            -self._max_angular_speed,
            min(self._max_angular_speed, angular_z),
        )
        with self._lock:
            self._target_linear_y = linear_y
            self._target_angular_z = angular_z
            self._last_command_time = time.monotonic()
        return linear_y, angular_z

    def set_joint_target(self, positions: List[float]) -> List[float]:
        safe_positions = [
            max(-2.80, min(2.80, value))
            for value in positions[:6]
        ]
        safe_positions.append(max(0.0, min(0.35, positions[6])))
        safe_positions.append(max(-0.35, min(0.0, positions[7])))

        trajectory = JointTrajectory()
        trajectory.header.frame_id = "world"
        trajectory.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = safe_positions
        trajectory.points.append(point)

        with self._lock:
            self._pending_trajectory = trajectory
            self._pending_trajectory_repeats = 6
        return safe_positions

    def emergency_stop(self) -> None:
        with self._lock:
            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
        self._cmd_vel_publisher.publish(Twist())

    def _update(self) -> None:
        now = time.monotonic()
        period = now - self._last_update_time
        self._last_update_time = now

        with self._lock:
            if now - self._last_command_time >= self._command_timeout:
                self._target_linear_y = 0.0
                self._target_angular_z = 0.0
            linear_y = self._linear_profile.update(
                self._target_linear_y,
                period,
            )
            angular_z = self._angular_profile.update(
                self._target_angular_z,
                period,
            )
            trajectory = self._pending_trajectory
            if self._pending_trajectory_repeats > 0:
                self._pending_trajectory_repeats -= 1
            else:
                trajectory = None
                self._pending_trajectory = None

        command = Twist()
        command.linear.y = linear_y
        command.angular.z = angular_z
        self._cmd_vel_publisher.publish(command)
        if trajectory is not None:
            self._trajectory_publisher.publish(trajectory)


class _ControlHandler(BaseHTTPRequestHandler):
    """Serve validated JSON control requests with CORS support."""

    control_server: "ControlServer"

    def do_OPTIONS(self) -> None:
        self._send_response(204, b"", "text/plain")

    def do_GET(self) -> None:
        if self.path == "/health":
            self._send_json(200, {"status": "ok"})
        else:
            self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:
        if self.path == "/cmd_vel":
            self._handle_cmd_vel()
        elif self.path == "/joint_trajectory":
            self._handle_joint_trajectory()
        else:
            self._send_json(404, {"error": "not found"})

    def _handle_cmd_vel(self) -> None:
        body = self._read_body()
        if body is None:
            return
        try:
            linear_y = float(body.get("linear_y", 0.0))
            angular_z = float(body.get("angular_z", 0.0))
        except (TypeError, ValueError):
            self._send_json(400, {"error": "velocities must be numbers"})
            return
        if not math.isfinite(linear_y) or not math.isfinite(angular_z):
            self._send_json(400, {"error": "velocities must be finite"})
            return

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
        body = self._read_body()
        if body is None:
            return
        positions = body.get("positions", [])
        if not isinstance(positions, list) or len(positions) != 8:
            actual_length = len(positions) if isinstance(positions, list) else 0
            self._send_json(
                400,
                {"error": f"expected 8 positions, got {actual_length}"},
            )
            return
        try:
            positions = [float(position) for position in positions]
        except (TypeError, ValueError):
            self._send_json(400, {"error": "positions must be numbers"})
            return
        if not all(math.isfinite(position) for position in positions):
            self._send_json(400, {"error": "positions must be finite"})
            return

        positions = self.control_server.publish_joint_trajectory(positions)
        self._send_json(
            200,
            {"status": "ok", "positions": positions},
        )

    def _read_body(self) -> Optional[Dict[str, Any]]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 65536:
                raise ValueError("invalid Content-Length")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("JSON body must be an object")
            return body
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            self._send_json(400, {"error": str(error)})
            return None

    def _send_json(self, status: int, data: Dict[str, Any]) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
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
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def log_message(self, format_string: str, *args: Any) -> None:
        del format_string, args


class ControlServer:
    """Run a persistent ROS 2 publisher behind a small HTTP API."""

    def __init__(
        self,
        port: int = 8090,
        cmd_vel_topic: str = "/xczs/manual_cmd_vel",
        joint_trajectory_topic: str = "/xczs/joint_trajectory",
        host: str = "127.0.0.1",
        max_linear_speed: float = 0.25,
        max_angular_speed: float = 0.60,
        command_timeout: float = 0.30,
    ) -> None:
        self._port = port
        self._host = host
        self._context = Context()
        rclpy.init(context=self._context)
        self._node = _RosControlPublisher(
            cmd_vel_topic,
            joint_trajectory_topic,
            max_linear_speed,
            max_angular_speed,
            command_timeout,
            self._context,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            name="web-control-ros-executor",
            daemon=True,
        )
        self._http_server: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None

    def start(self) -> "ControlServer":
        handler = type(
            "_BoundHandler",
            (_ControlHandler,),
            {"control_server": self},
        )
        self._http_server = ThreadingHTTPServer(
            (self._host, self._port),
            handler,
        )
        self._http_server.daemon_threads = True
        self._executor_thread.start()
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="web-control-http-server",
            daemon=True,
        )
        self._http_thread.start()
        return self

    def stop(self) -> None:
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        if self._http_thread is not None:
            self._http_thread.join(timeout=3.0)
            self._http_thread = None

        self._node.emergency_stop()
        self._executor.shutdown(timeout_sec=3.0)
        self._executor_thread.join(timeout=3.0)
        self._node.destroy_node()
        self._context.shutdown()

    def publish_cmd_vel(
        self,
        linear_y: float,
        angular_z: float,
    ) -> Tuple[float, float]:
        return self._node.set_base_target(linear_y, angular_z)

    def publish_joint_trajectory(
        self,
        positions: List[float],
    ) -> List[float]:
        return self._node.set_joint_target(positions)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="XCZS HTTP-to-ROS 2 control gateway",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--max-linear-speed", type=float, default=0.25)
    parser.add_argument("--max-angular-speed", type=float, default=0.60)
    parser.add_argument("--command-timeout", type=float, default=0.30)
    args = parser.parse_args()

    server = ControlServer(
        host=args.host,
        port=args.port,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        command_timeout=args.command_timeout,
    ).start()
    print(f"Control server: http://{args.host}:{args.port}")
    shutdown_event = threading.Event()

    def request_shutdown(
        signal_number: int,
        frame: object,
    ) -> None:
        del signal_number, frame
        shutdown_event.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        while not shutdown_event.wait(timeout=1.0):
            continue
    finally:
        server.stop()


if __name__ == "__main__":
    main()
