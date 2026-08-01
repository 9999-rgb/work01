"""Lifecycle wrapper for the ROS 2 and HTTP control gateway."""

from __future__ import annotations

import threading
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from typing import Any, Dict, Iterator, List, Optional, Tuple

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from .ros_node import ControlRequestError
from .ros_node import RosControlNode
from .web_server import ControlHandler


CABINET_SHUTDOWN_TIMEOUT_SEC = 15.0


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
        self._request_condition = threading.Condition()
        self._active_requests = 0
        self._stopping = False

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
        with self._request_condition:
            if self._stopping:
                return
            self._stopping = True
            self._request_condition.notify_all()

        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        if self._http_thread is not None:
            self._http_thread.join(timeout=3.0)
            self._http_thread = None

        # Request handlers are daemon threads, so server_close() does not
        # join them.  Wait until every handler that entered the ROS gateway
        # before the stopping gate has left it.  Later handlers receive 503
        # without touching a node that is being torn down.
        with self._request_condition:
            while self._active_requests > 0:
                self._request_condition.wait()

        cabinet_cancel = self._node.cancel_cabinet_button(allow_idle=True)
        if (
            cabinet_cancel["status"] == "canceling"
            and not self._node.wait_for_cabinet_idle(
                timeout_sec=CABINET_SHUTDOWN_TIMEOUT_SEC
            )
        ):
            self._node.get_logger().warning(
                "Timed out waiting for the cabinet operation to cancel "
                "during Web gateway shutdown."
            )
        self._node.cancel_navigation(allow_idle=True)
        self._node.cancel_motion(allow_idle=True)
        self._node.emergency_stop()
        self._executor.shutdown(timeout_sec=3.0)
        self._executor_thread.join(timeout=3.0)
        self._node.destroy_node()
        self._context.shutdown()

    def health(self) -> Dict[str, Any]:
        """Return gateway and ROS action availability."""
        with self._request_scope():
            navigation = self._node.navigation_snapshot()
            motion = self._node.motion_snapshot()
            cabinet = self._node.cabinet_snapshot()
            return {
                "status": "ok",
                "navigation_available": navigation["available"],
                "moveit_available": motion["available"],
                "cabinet_available": cabinet["available"],
                "cabinet_active": cabinet["active"],
            }

    def publish_cmd_vel(
        self,
        linear_y: float,
        angular_z: float,
    ) -> Tuple[float, float]:
        """Forward one manual base target."""
        with self._request_scope():
            return self._node.set_base_target(linear_y, angular_z)

    def publish_joint_trajectory(
        self,
        positions: List[float],
    ) -> List[float]:
        """Forward one manual joint target."""
        with self._request_scope():
            return self._node.set_joint_target(positions)

    def navigation_status(self) -> Dict[str, Any]:
        """Return Nav2 state and overlays."""
        with self._request_scope():
            return self._node.navigation_snapshot()

    def navigation_map(self) -> Dict[str, Any]:
        """Return the latest occupancy map."""
        with self._request_scope():
            return self._node.map_snapshot()

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        """Switch manual/Nav2 base routing."""
        with self._request_scope():
            return self._node.set_navigation_mode(enabled)

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Dict[str, Any]:
        """Send one map-frame Nav2 goal."""
        with self._request_scope():
            return self._node.send_navigation_goal(x, y, yaw)

    def cancel_navigation(self) -> Dict[str, Any]:
        """Cancel active Nav2 navigation."""
        with self._request_scope():
            return self._node.cancel_navigation()

    def takeover_navigation(self) -> Dict[str, Any]:
        """Cancel Nav2 and switch the base router to zero-speed manual mode."""
        with self._request_scope():
            return self._node.takeover_navigation()

    def motion_status(self) -> Dict[str, Any]:
        """Return MoveIt action state."""
        with self._request_scope():
            return self._node.motion_snapshot()

    def send_named_motion(
        self,
        group: str,
        target: str,
        execute: bool,
    ) -> Dict[str, Any]:
        """Send a named MoveIt goal."""
        with self._request_scope():
            return self._node.send_named_motion(group, target, execute)

    def send_pose_motion(
        self,
        frame_id: str,
        position: List[float],
        orientation: List[float],
        execute: bool,
    ) -> Dict[str, Any]:
        """Send an end-effector MoveIt goal."""
        with self._request_scope():
            return self._node.send_pose_motion(
                frame_id,
                position,
                orientation,
                execute,
            )

    def cancel_motion(self) -> Dict[str, Any]:
        """Cancel active MoveIt planning or execution."""
        with self._request_scope():
            return self._node.cancel_motion()

    def cabinet_status(self) -> Dict[str, Any]:
        """Return cabinet-button action and physical button state."""
        with self._request_scope():
            return self._node.cabinet_snapshot()

    def cabinet_controls(self) -> Dict[str, Any]:
        """Return supported cabinet controls and their physical states."""
        with self._request_scope():
            return self._node.cabinet_controls_snapshot()

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        """Start one collision-checked cabinet-button operation."""
        with self._request_scope():
            return self._node.press_cabinet_button(
                button_id,
                navigate_to_staging_pose,
            )

    def cancel_cabinet_button(self) -> Dict[str, Any]:
        """Cancel the active cabinet-button operation."""
        with self._request_scope():
            return self._node.cancel_cabinet_button()

    @contextmanager
    def _request_scope(self) -> Iterator[None]:
        """Reject new API work during shutdown and drain entered calls."""
        with self._request_condition:
            if self._stopping:
                raise ControlRequestError(
                    "Web control server is stopping.",
                    503,
                )
            self._active_requests += 1
        try:
            yield
        finally:
            with self._request_condition:
                self._active_requests -= 1
                if self._active_requests == 0:
                    self._request_condition.notify_all()
