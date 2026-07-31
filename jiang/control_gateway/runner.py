"""Lifecycle wrapper for the ROS 2 and HTTP control gateway."""

from __future__ import annotations

import threading
from http.server import ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from .ros_node import RosControlNode
from .web_server import ControlHandler


class ControlServer:
    """Run the persistent ROS 2 control node behind a small HTTP API."""

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
        self._node = RosControlNode(
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
        """Start the ROS executor and threaded HTTP listener."""
        handler = type(
            "_BoundHandler",
            (ControlHandler,),
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
        """Stop HTTP, autonomous goals and the ROS executor."""
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        if self._http_thread is not None:
            self._http_thread.join(timeout=3.0)
            self._http_thread = None

        self._node.cancel_navigation(allow_idle=True)
        self._node.cancel_motion(allow_idle=True)
        self._node.emergency_stop()
        self._executor.shutdown(timeout_sec=3.0)
        self._executor_thread.join(timeout=3.0)
        self._node.destroy_node()
        self._context.shutdown()

    def health(self) -> Dict[str, Any]:
        """Return gateway and ROS action availability."""
        navigation = self._node.navigation_snapshot()
        motion = self._node.motion_snapshot()
        return {
            "status": "ok",
            "navigation_available": navigation["available"],
            "moveit_available": motion["available"],
        }

    def publish_cmd_vel(
        self,
        linear_y: float,
        angular_z: float,
    ) -> Tuple[float, float]:
        """Forward one manual base target."""
        return self._node.set_base_target(linear_y, angular_z)

    def publish_joint_trajectory(
        self,
        positions: List[float],
    ) -> List[float]:
        """Forward one manual joint target."""
        return self._node.set_joint_target(positions)

    def navigation_status(self) -> Dict[str, Any]:
        """Return Nav2 state and overlays."""
        return self._node.navigation_snapshot()

    def navigation_map(self) -> Dict[str, Any]:
        """Return the latest occupancy map."""
        return self._node.map_snapshot()

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        """Switch manual/Nav2 base routing."""
        return self._node.set_navigation_mode(enabled)

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Dict[str, Any]:
        """Send one map-frame Nav2 goal."""
        return self._node.send_navigation_goal(x, y, yaw)

    def cancel_navigation(self) -> Dict[str, Any]:
        """Cancel active Nav2 navigation."""
        return self._node.cancel_navigation()

    def motion_status(self) -> Dict[str, Any]:
        """Return MoveIt action state."""
        return self._node.motion_snapshot()

    def send_named_motion(
        self,
        group: str,
        target: str,
        execute: bool,
    ) -> Dict[str, Any]:
        """Send a named MoveIt goal."""
        return self._node.send_named_motion(group, target, execute)

    def send_pose_motion(
        self,
        frame_id: str,
        position: List[float],
        orientation: List[float],
        execute: bool,
    ) -> Dict[str, Any]:
        """Send an end-effector MoveIt goal."""
        return self._node.send_pose_motion(
            frame_id,
            position,
            orientation,
            execute,
        )

    def cancel_motion(self) -> Dict[str, Any]:
        """Cancel active MoveIt planning or execution."""
        return self._node.cancel_motion()
